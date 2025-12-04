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
    io.control->ld_out = done == 1;
    io.control->st_out = done == 2;

    if(state == MSHR_TRAN){
        io.cpu->valid = done == 1 || done == 2;
        io.cpu->wr = done == 2;
        io.cpu->data = rdata;
        io.cpu->uop = ruop;
    }
    else{
        io.cpu->valid = false;
        io.cpu->wr = false;
        io.cpu->addr = 0;
        io.cpu->data = 0;
    }
}

void MSHR::seq()
{
    if (count_mshr < MSHR_ENTRY_SIZE && count_table < MSHR_TABLE_SIZE)
    {
        if (io.dcache_st->valid)
        {
            uint32_t entry = find_entry(io.dcache_st->tag, io.dcache_st->index);
            if (entry == MSHR_ENTRY_SIZE)
            {
                entry = mshr_tail;
                add_entry(io.dcache_st->tag, io.dcache_st->index, io.dcache_st->way,io.dcache_st->dirty,io.dcache_st->paddr);
            }
            else {
                mshr_entries[entry].count++;
            }
            add_table_entry(1, entry, io.dcache_st->offset,io.dcache_st->wdata,io.dcache_st->wstrb,io.dcache_st->uop);
        }
        if (io.dcache_ld->valid)
        {
            uint32_t entry = find_entry(io.dcache_ld->tag, io.dcache_ld->index);
            if (entry == MSHR_ENTRY_SIZE)
            {
                entry = mshr_tail;
                add_entry(io.dcache_ld->tag, io.dcache_ld->index, io.dcache_ld->way,io.dcache_ld->dirty,io.dcache_ld->paddr);
            }
            else {
                mshr_entries[entry].count++;
            }
            add_table_entry(0, entry, io.dcache_ld->offset,0,0,io.dcache_ld->uop);
        }
    }
    if (io.control->flush)
    {
        for(int i=0;i<MSHR_TABLE_SIZE;i++){
            if(mshr_table[i].valid&&mshr_table[i].type==0){
                add_free(i);
            }
        }
    }
    if(io.control->mispred){
        for(int i=0;i<MSHR_TABLE_SIZE;i++){
            if(mshr_table[i].valid){
                if(io.control->br_mask & (1 << mshr_table[i].uop.tag)){
                    add_free(i);
                }
                
            }
        }
    }
    if (state == MSHR_WAIT_WRITE)
    {
        write_cache_line(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, offset, wdata,wdone, io.mem->data.done, io.mem->data.last);
    }
    else if (state == MSHR_WAIT_READ)
    {
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
        uint32_t index=MSHR_TABLE_SIZE;
        for(int i=0;i<MSHR_TABLE_SIZE;i++){
            if(mshr_table[i].valid&&mshr_table[i].entry==mshr_head){
                index=i;
                break;
            }
        }
        if(index!=MSHR_TABLE_SIZE){
            if(mshr_table[index].type==0){
                rdata=read_cache_data(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, mshr_table[index].offset);
                ruop = mshr_table[index].uop;
            }
            else{
                write_cache_data(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, mshr_table[index].offset, mshr_table[index].wdata, mshr_table[index].wstrb);
                rdata=0;
                ruop = mshr_table[index].uop;
                // if(DCACHE_LOG)
                // printf("MSHR write cache data addr:0x%08x wdata:0x%08x wstrb:%02x index:%d offset:%d way:%d\n", get_addr(mshr_entries[mshr_head].tag, mshr_entries[mshr_head].index, mshr_table[index].offset), dcache_data[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way][mshr_table[index].offset], 0x0f, mshr_entries[mshr_head].index, mshr_table[index].offset, mshr_entries[mshr_head].way);
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
            if(mshr_entries[mshr_head].count == 0){
                mshr_entries[mshr_head].valid=false;
                if(mshr_head != mshr_tail){
                    mshr_head=(mshr_head+1)%MSHR_ENTRY_SIZE;
                }
            }
            else if (mshr_entries[mshr_head].dirty == 0)
            {
                state = MSHR_WAIT_READ;
            }
            else
            {
                state = MSHR_WAIT_WRITE;
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
    if(DCACHE_LOG){
        printf("MSHR state:%d mshr_head:%d mshr_tail:%d count_mshr:%d count_table:%d offset:%d entry:%d done:%d wdone:%d wdonelast:%d wdata_valid:%d io.control->flush:%d\n", state, mshr_head, mshr_tail, count_mshr, count_table, offset, entry, done, wdone, wdonelast, wdata_valid, io.control->flush);
        for(int i=0;i<MSHR_ENTRY_SIZE;i++){
            printf("MSHR entry[%d] valid:%d tag:0x%08x index:%d way:%d dirty:%d paddr:0x%08x\n", i, mshr_entries[i].valid, mshr_entries[i].tag, mshr_entries[i].index, mshr_entries[i].way, mshr_entries[i].dirty, mshr_entries[i].paddr);
        }
        for(int i=0;i<MSHR_TABLE_SIZE;i++){
            printf("MSHR table[%d] valid:%d entry:%d type:%d offset:%d wdata:0x%08x wstrb:%02x tag:0x%08x preg:%d rob_idx:%d\n", i, mshr_table[i].valid, mshr_table[i].entry, mshr_table[i].type, mshr_table[i].offset, mshr_table[i].wdata, mshr_table[i].wstrb, mshr_table[i].uop.tag, mshr_table[i].uop.dest_preg, mshr_table[i].uop.rob_idx);
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
    mshr_entries[mshr_tail].count = 1;
    mshr_tail = (mshr_tail + 1) % MSHR_ENTRY_SIZE; 
    count_mshr++;
}
void MSHR::add_table_entry(bool type, uint32_t entry, uint32_t offset_table,uint32_t wdata,uint8_t wstrb,Inst_uop uop)
{
    uint32_t index=free_table[table_head];
    table_head = (table_head + 1) % MSHR_TABLE_SIZE;
    mshr_table[index].valid = true;
    mshr_table[index].entry = entry;
    mshr_table[index].type = type;
    mshr_table[index].offset = offset_table;
    mshr_table[index].wdata = wdata;
    mshr_table[index].wstrb = wstrb;
    mshr_table[index].uop = uop;
   
    count_table++;
}
void MSHR::add_free(uint32_t entry)
{
    free_table[table_tail] = entry;
    mshr_table[entry].valid=false;
    mshr_entries[mshr_table[entry].entry].count--;
    table_tail = (table_tail + 1) % MSHR_TABLE_SIZE;
    count_table--;
}