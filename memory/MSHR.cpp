#include "MSHR.h"
#include <cstdio>
mshr_entry mshr_entries[MSHR_ENTRY_SIZE];
table_entry mshr_table[MSHR_TABLE_SIZE];
uint32_t free_table[MSHR_TABLE_SIZE];

void MSHR::init()
{
    mshr_head = 0;
    mshr_tail = 0;
    table_head = 0;
    table_tail = 0;
    count_mshr = 0;
    count_table = 0;
    offset = 0;
    entry = 0;
    rdata = 0;
    done = false;
    wdata = 0;
    wdata_valid = false;
    state = MSHR_IDLE;

    for (int i = 0; i < MSHR_ENTRY_SIZE; i++)
    {
        mshr_entries[i].valid = false;
    }
    for (int i = 0; i < MSHR_TABLE_SIZE; i++)
    {
        mshr_table[i].valid = false;
        free_table[i] = i;
    }
}

void MSHR::comb_in()
{
    if (state == MSHR_WAIT_WRITE)
    {
        io.mem->control.en = true;
        io.mem->control.wen = wdata_valid;
        io.mem->control.wdata = wdata;
        uint32_t addr = get_addr(dcache_tag[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way], mshr_entries[mshr_head].index, 0);
        io.mem->control.addr = addr;
        io.mem->control.len = DCACHE_OFFSET_NUM - 1;
        io.mem->control.size = 0b10;
        io.mem->control.sel = 0b1111;
        io.mem->control.done = wdone&!io.mem->data.done;
        io.mem->control.last = offset == DCACHE_OFFSET_NUM - 1&wdone;
    }
    else if (state == MSHR_WAIT_READ)
    {
        io.mem->control.en = true;
        io.mem->control.wen = false;
        uint32_t addr = get_addr(mshr_entries[mshr_head].tag, mshr_entries[mshr_head].index, 0);
        io.mem->control.addr = addr;
        io.mem->control.wdata = 0;
        io.mem->control.len = DCACHE_OFFSET_NUM - 1;
        io.mem->control.size = 0b10;
        io.mem->control.sel = 0b1111;
        io.mem->control.done = false;
        io.mem->control.last = false;
    }
    else
    {
        io.mem->control.en = false;
        io.mem->control.wen = false;
        io.mem->control.addr = 0;
        io.mem->control.wdata = 0;
        io.mem->control.len = 0;
        io.mem->control.size = 0;
        io.mem->control.sel = 0;
        io.mem->control.done = false;
        io.mem->control.last = false;
    }
}

void MSHR::comb_out()
{
    io.control->ready = count_mshr < MSHR_ENTRY_SIZE & count_table < MSHR_TABLE_SIZE;

    io.cpu->valid = done == 1 || done == 2;
    io.cpu->wr = done == 2;
    io.cpu->data = rdata;
    io.cpu->tag = rtag;
    io.cpu->preg = rpreg;
    io.cpu->rob_idx = rrob_idx;
    io.cpu->page_fault = false;
}

void MSHR::seq()
{
    if (count_mshr < MSHR_ENTRY_SIZE && count_table < MSHR_TABLE_SIZE &&!io.control->flush)
    {
        uint32_t hit_way;
        bool st_done=false;
        bool ld_done=false;
        if(hit_check(io.dcache_st->index, io.dcache_st->tag,hit_way)&&io.dcache_st->valid&&state==MSHR_IDLE){
            //命中直接写回
            st_done=true;
            rdata=0;
            write_cache_data(io.dcache_st->index, hit_way, io.dcache_st->offset, io.dcache_st->wdata, io.dcache_st->wstrb);
            rpreg=io.dcache_st->preg;
            rrob_idx=io.dcache_st->rob_idx;
            rtag=io.dcache_st->tag;
            if (DCACHE_LOG)
            printf("MSHR direct write cache data addr:0x%08x wdata:0x%08x wstrb:%02x index:%d offset:%d way:%d\n", io.dcache_st->addr, dcache_data[io.dcache_st->index][hit_way][io.dcache_st->offset], io.dcache_st->wstrb, io.dcache_st->index, io.dcache_st->offset, hit_way);
        }
        else if (io.dcache_st->valid && state == MSHR_IDLE)
        {
            st_done=false;
            uint32_t entry = find_entry(io.dcache_st->tag, io.dcache_st->index);
            if (entry == MSHR_ENTRY_SIZE)
            {
                entry = mshr_tail;
                add_entry(io.dcache_st->tag, io.dcache_st->index, io.dcache_st->way,io.dcache_st->dirty,io.dcache_st->paddr);
            }
            add_table_entry(1, entry, io.dcache_st->offset,io.dcache_st->wdata,io.dcache_st->wstrb,io.dcache_st->tag,io.dcache_st->preg,io.dcache_st->rob_idx);
        }
        else st_done=false;

        if(hit_check(io.dcache_ld->index, io.dcache_ld->tag,hit_way)&&io.dcache_ld->valid&&state==MSHR_IDLE){
            //命中直接读出
            ld_done=true;
            rdata=read_cache_data(io.dcache_ld->index, hit_way, io.dcache_ld->offset);
            rpreg=io.dcache_ld->preg;
            rrob_idx=io.dcache_ld->rob_idx;
            rtag=io.dcache_ld->tag;
            if (DCACHE_LOG)
            printf("MSHR direct read cache data addr:0x%08x rdata:0x%08x index:%d offset:%d way:%d\n", io.dcache_ld->addr, rdata, io.dcache_ld->index, io.dcache_ld->offset, hit_way);
        }
        else if (io.dcache_ld->valid && state == MSHR_IDLE)
        {
            ld_done=false;
            uint32_t entry = find_entry(io.dcache_ld->tag, io.dcache_ld->index);
            if (entry == MSHR_ENTRY_SIZE)
            {
                entry = mshr_tail;
                add_entry(io.dcache_ld->tag, io.dcache_ld->index, io.dcache_ld->way,io.dcache_ld->dirty,io.dcache_ld->paddr);
            }
            add_table_entry(0, entry, io.dcache_ld->offset,0,0,io.dcache_ld->tag,io.dcache_ld->preg,io.dcache_ld->rob_idx);
        }
        else ld_done=false;

        if(ld_done){
            done=1;
        }
        else if(st_done){
            done=2;
        }
        else {
            done=0;
        }
    }

    if (io.control->flush)//优化
    {
        uint32_t imax=0;
        uint32_t index=0;
        for(int i=0;i<MSHR_TABLE_SIZE;i++){
            if(mshr_table[i].valid&&mshr_table[i].type==0&&mshr_table[i].priority>imax){
                imax=mshr_table[i].priority;
                index=i;
            }
        }
        if(imax!=0){
            if(mshr_head!=(mshr_table[index].entry)){
                mshr_entries[mshr_table[index].entry].valid=false;
                count_mshr--;
            }
            mshr_table[index].valid=false;
            add_free(index);
        }
    }
    if (state == MSHR_WAIT_WRITE)
    {
        wdata_valid = true;
        write_cache_line(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, offset, wdata,wdone, io.mem->data.done, io.mem->data.last);
        if (DCACHE_LOG)
        printf("MSHR write back data addr:0x%08x wdata:0x%08x index:%d offset:%d way:%d\n", get_addr(dcache_tag[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way], mshr_entries[mshr_head].index, offset), wdata, mshr_entries[mshr_head].index, offset, mshr_entries[mshr_head].way);
        if(io.mem->data.last){
            dcache_dirty[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way] = 0;
            // dcache_valid[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way] = 0;
            dcache_issued[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way] = 0;
        }
    }
    else if (state == MSHR_WAIT_READ)
    {
        wdata_valid = false;
        read_cache_line(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, offset, io.mem->data.data, io.mem->data.done, io.mem->data.last);

        if(io.mem->data.last){
            dcache_valid[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way] = 1;
            dcache_tag[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way] = mshr_entries[mshr_head].tag;
        }
    }
    else if(state == MSHR_TRAN){
        done=0;
        wdata=0;
        wdata_valid = false;
        uint32_t index=0;
        uint32_t imin=MSHR_TABLE_SIZE+1;
        for(int i=0;i<MSHR_TABLE_SIZE;i++){
            if(mshr_table[i].valid&&mshr_table[i].entry==mshr_head&&mshr_table[i].priority<=imin){
                imin=mshr_table[i].priority;
                index=i;
            }
        }
        if(imin!=MSHR_TABLE_SIZE+1){
            if(mshr_table[index].type==0){
                rdata=read_cache_data(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, mshr_table[index].offset);
                rpreg=mshr_table[index].preg;
                rrob_idx=mshr_table[index].rob_idx;
                rtag=mshr_table[index].tag;
            }
            else{
                write_cache_data(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, mshr_table[index].offset, mshr_table[index].wdata, mshr_table[index].wstrb);
                rdata=0;
                rpreg=mshr_table[index].preg;
                rrob_idx=mshr_table[index].rob_idx;
                rtag=mshr_table[index].tag;
                if(DCACHE_LOG)
                printf("MSHR write cache data addr:0x%08x wdata:0x%08x wstrb:%02x index:%d offset:%d way:%d\n", get_addr(mshr_entries[mshr_head].tag, mshr_entries[mshr_head].index, mshr_table[index].offset), dcache_data[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way][mshr_table[index].offset], 0x0f, mshr_entries[mshr_head].index, mshr_table[index].offset, mshr_entries[mshr_head].way);
            }
            done=mshr_table[index].type+1;
            add_free(index);
        }
        else{
            done=0;
            offset=0;
            mshr_entries[mshr_head].valid=false;
            mshr_head=(mshr_head+1)%MSHR_ENTRY_SIZE;
            count_mshr--;
        }
    }
    else {
        wdata_valid = false;
    }

    if (state == MSHR_IDLE)
    {
        if (mshr_entries[mshr_head].valid)
        {
            if (mshr_entries[mshr_head].dirty == 0)
            {
                state = MSHR_WAIT_READ;
            }
            else
            {
                state = MSHR_WAIT_WRITE;
                dcache_valid[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way] = 0;
                dcache_issued[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way] = 1;
            }
        }
        else {
            if(mshr_head != mshr_tail){
                mshr_head=(mshr_head+1)%MSHR_ENTRY_SIZE;
            }
        }
    }
    else if (state == MSHR_WAIT_WRITE)
    {
        if (io.mem->data.last)
        {
            if (mshr_entries[mshr_head].valid)
            {
                state = MSHR_WAIT_READ;
            }
            
        }
    }
    else if (state == MSHR_WAIT_READ)
    {
        if (io.mem->data.last)
        {
            state = MSHR_TRAN;
        }
    }
    else if(state==MSHR_TRAN){
        if(done==0){
            state=MSHR_IDLE;
        }
    }
}

uint32_t MSHR::find_entry(uint32_t tag, uint32_t index)
{
    for (int i = 0; i != MSHR_ENTRY_SIZE; i++)
    {
        if (mshr_entries[i].valid && mshr_entries[i].tag == tag && mshr_entries[i].index == index)
        {
            return i;
        }
    }
    return MSHR_ENTRY_SIZE;
}

void MSHR::add_entry(uint32_t tag, uint32_t index, uint32_t way,bool dirty,uint32_t paddr)
{
    mshr_entries[mshr_tail].valid = true;
    mshr_entries[mshr_tail].tag = tag;
    mshr_entries[mshr_tail].index = index;
    mshr_entries[mshr_tail].way = way;
    mshr_entries[mshr_tail].dirty = dirty;
    mshr_entries[mshr_tail].paddr = paddr;
    mshr_tail = (mshr_tail + 1) % MSHR_ENTRY_SIZE; 
    count_mshr++;
}
void MSHR::add_table_entry(bool type, uint32_t entry, uint32_t offset_table,uint32_t wdata,uint8_t wstrb,uint32_t tag,uint32_t preg,uint32_t rob_idx)
{
    uint32_t index=free_table[table_head];
    table_head = (table_head + 1) % MSHR_TABLE_SIZE;
    mshr_table[index].valid = true;
    mshr_table[index].entry = entry;
    mshr_table[index].type = type;
    mshr_table[index].offset = offset_table;
    mshr_table[index].wdata = wdata;
    mshr_table[index].wstrb = wstrb;
    mshr_table[index].priority = count_table+1;
    mshr_table[index].tag = tag;
    mshr_table[index].preg = preg;
    mshr_table[index].rob_idx = rob_idx;
    count_table++;
}
void MSHR::add_free(uint32_t entry)
{
    free_table[table_tail] = entry;
    mshr_table[entry].valid=false;
    table_tail = (table_tail + 1) % MSHR_TABLE_SIZE;
    count_table--;
}