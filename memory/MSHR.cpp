#include "MSHR.h"
#include <cstdio>
mshr_entry mshr_entries[MSHR_ENTRY_SIZE];
table_entry mshr_table[MSHR_TABLE_SIZE];
uint32_t free_table[MSHR_TABLE_SIZE];
enum MSHR_STATE mshr_state = MSHR_IDLE;

uint32_t mshr_head;
void MSHR::init()
{
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
    mshr_state = MSHR_IDLE;

    for (int i = 0; i < MSHR_ENTRY_SIZE; i++)
    {
        mshr_entries[i].valid = false;
    }
    for (int i = 0; i < MSHR_TABLE_SIZE; i++)
    {
        mshr_table[i].valid = false;
    }
}

void MSHR::comb_in()
{
    if (mshr_state == MSHR_WAIT_WRITE)
    {
        io.mem->control.en = true;
        io.mem->control.wen = true;
        io.mem->control.wdata = wdata;
        io.mem->control.addr = mshr_entries[mshr_head].paddr;
        io.mem->control.len = DCACHE_OFFSET_NUM - 1;
        io.mem->control.size = 0b10;
        io.mem->control.sel = 0b1111;
        io.mem->control.done = wdone & !io.mem->data.done;
        io.mem->control.last = offset == DCACHE_OFFSET_NUM - 1 & wdone;
    }
    else if (mshr_state == MSHR_WAIT_READ)
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
void MSHR::comb_ready()
{

    io.control->ready = count_mshr < MSHR_ENTRY_SIZE & count_table < MSHR_TABLE_SIZE - 1;
}
void MSHR::comb_out()
{
    io.control->ld_out = done == 1;
    io.control->st_out = done == 2;

    if (DCACHE_LOG)
    {
        printf("MSHR_state:%d done:%d\n", mshr_state, done);
    }
    if (mshr_state == MSHR_TRAN)
    {
        io.cpu->valid = done == 1 || done == 2;
        io.cpu->wr = done == 2;
        io.cpu->data = rdata;
        io.cpu->addr = waddr;
        io.cpu->uop = ruop;
    }
    else
    {
        io.cpu->valid = false;
        io.cpu->wr = false;
        io.cpu->addr = 0;
        io.cpu->data = 0;
    }

    if (DCACHE_LOG)
    {
        printf("io.cpu->valid:%d wr:%d addr:0x%08x data:0x%08x rob_idx:%d preg:%d\n", io.cpu->valid, io.cpu->wr, io.cpu->addr, io.cpu->data, io.cpu->uop.rob_idx, io.cpu->uop.dest_preg);
    }
}

void MSHR::seq()
{
    if (count_mshr < MSHR_ENTRY_SIZE && count_table < MSHR_TABLE_SIZE - 1)
    {
        if (io.dcache_st->valid)
        {
            uint32_t entry = find_entry(io.dcache_st->tag, io.dcache_st->index, io.dcache_st->way, io.dcache_st->dirty);
            
            if (entry == MSHR_ENTRY_SIZE)
            {
                entry = mshr_tail;change_bias(io.dcache_st->index, io.dcache_st->way);
                add_entry(io.dcache_st->tag, io.dcache_st->index, io.dcache_st->way, io.dcache_st->dirty, io.dcache_st->paddr);
            }
            else
            {
                mshr_entries[entry].count++;
            }
            add_table_entry(1, entry, io.dcache_st->offset, io.dcache_st->wdata, io.dcache_st->wstrb, io.dcache_st->uop);
        }
        if (io.dcache_ld->valid)
        {
            if(DCACHE_LOG)
                printf("MSHR seq receive ld addr:0x%08x tag:0x%08x index:%d way:%d offset:%d dirty:%d preg:%02d rob_idx:%02d mispred:%d\n", io.dcache_ld->addr, io.dcache_ld->tag, io.dcache_ld->index, io.dcache_ld->way, io.dcache_ld->offset, io.dcache_ld->dirty, io.dcache_ld->uop.dest_preg, io.dcache_ld->uop.rob_idx, io.dcache_ld->mispred);
            uint32_t entry = find_entry(io.dcache_ld->tag, io.dcache_ld->index, io.dcache_ld->way, io.dcache_ld->dirty);
            
            if (entry == MSHR_ENTRY_SIZE)
            {
                entry = mshr_tail;change_bias(io.dcache_ld->index, io.dcache_ld->way);
                add_entry(io.dcache_ld->tag, io.dcache_ld->index, io.dcache_ld->way, io.dcache_ld->dirty, io.dcache_ld->paddr);
            }
            else
            {
                mshr_entries[entry].count++;
            }

            if(io.dcache_ld->mispred)
                add_table_entry(2, entry, io.dcache_ld->offset, 0, 0, io.dcache_ld->uop);
            else
                add_table_entry(0, entry, io.dcache_ld->offset, 0, 0, io.dcache_ld->uop);
        }
    }
    if (io.control->mispred)
    {
        for (int i = 0; i < MSHR_TABLE_SIZE; i++)
        {
            if (mshr_table[i].valid && mshr_table[i].type == 0)
            {
                if (io.control->br_mask & (1 << mshr_table[i].uop.tag))
                {
                    add_free(i);
                }
            }
        }
    }
    if (io.control->flush)
    {
        for (int i = 0; i < MSHR_TABLE_SIZE; i++)
        {
            if (mshr_table[i].valid && mshr_table[i].type == 0)
            {
                add_free(i);
            }
        }
    }
    if (mshr_state == MSHR_WAIT_WRITE)
    {

        write_cache_line(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, offset, wdata, wdone, io.mem->data.done, io.mem->data.last);
    }
    else if (mshr_state == MSHR_WAIT_READ)
    {
        read_cache_line(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, offset, io.mem->data.data, io.mem->data.done, io.mem->data.last);
    }
    else if (mshr_state == MSHR_TRAN)
    {
        done = 0;
        wdata = 0;
        uint32_t index = MSHR_TABLE_SIZE;

        for (int i = 0; i < count_table; i++)
        {
            uint32_t table_index = (table_head + i) % MSHR_TABLE_SIZE;
            if (mshr_table[table_index].valid && mshr_table[table_index].entry == mshr_head)
            {
                index = table_index;
                break;
            }
        }
        // if (DCACHE_LOG)
        // {
        //     printf("MSHR TRAN table_index search result:%d\n", index);
        // }
        if (index != MSHR_TABLE_SIZE)
        {
            // if (DCACHE_LOG)
            //     printf("MSHR TRAN table_index:%d entry:%d type:%d offset:%d wdata:0x%08x wstrb:%02x tag:0x%08x preg:%d rob_idx:%d\n", index, mshr_table[index].entry, mshr_table[index].type, mshr_table[index].offset, mshr_table[index].wdata, mshr_table[index].wstrb, mshr_table[index].uop.tag, mshr_table[index].uop.dest_preg, mshr_table[index].uop.rob_idx);
            if (mshr_table[index].type == 0)
            {
                rdata = read_cache_data(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, mshr_table[index].offset);
                waddr = get_addr(mshr_entries[mshr_head].tag, mshr_entries[mshr_head].index, mshr_table[index].offset);
                ruop = mshr_table[index].uop;
            }
            else
            {
                write_cache_data(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way, mshr_table[index].offset, mshr_table[index].wdata, mshr_table[index].wstrb);
                broadcast_mshr(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way,mshr_entries[mshr_head].tag);
                rdata = 0;
                ruop = mshr_table[index].uop;
                waddr = get_addr(mshr_entries[mshr_head].tag, mshr_entries[mshr_head].index, mshr_table[index].offset);
                if (DCACHE_LOG)
                    printf("MSHR write cache data table_index:%d addr:0x%08x wdata:0x%08x wstrb:%02x index:%d offset:%d way:%d\n", index, get_addr(mshr_entries[mshr_head].tag, mshr_entries[mshr_head].index, mshr_table[index].offset), dcache_data[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way][mshr_table[index].offset], 0x0f, mshr_entries[mshr_head].index, mshr_table[index].offset, mshr_entries[mshr_head].way);
            }
            done = mshr_table[index].type + 1;
            add_free(index);
            if (index == table_head)
            {
                table_head = (table_head + 1) % MSHR_TABLE_SIZE;

                count_table--;
            }
        }
        else
        {
            done = 0;
            offset = 0;
            mshr_entries[mshr_head].valid = false;
            updatelru(mshr_entries[mshr_head].index, mshr_entries[mshr_head].way);
            if (!mshr_entries[mshr_head].bias)
            {
                // If the entry was found by a load that came later, keep it valid
                dcache_valid[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way] = 1;
                dcache_issued[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way] = 0;
                // if(DCACHE_LOG)printf("Change Tag:index:%d way:%d dcache_tag:0x%08x change_tag:0x%08x\n",mshr_entries[mshr_head].index,mshr_entries[mshr_head].way,dcache_tag[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way],mshr_entries[mshr_head].tag);
                dcache_tag[mshr_entries[mshr_head].index][mshr_entries[mshr_head].way] = mshr_entries[mshr_head].tag;
            }
            else
            {
                mshr_entries[mshr_head].bias = 0;
            }
            mshr_head = (mshr_head + 1) % MSHR_ENTRY_SIZE;
            count_mshr--;
        }
    }
    else
    {
        wdata_valid = false;
    }
    if (mshr_table[table_head].valid == false && count_table > 0)
    {
        table_head = (table_head + 1) % MSHR_TABLE_SIZE;
        count_table--;
    }
    if (mshr_state == MSHR_IDLE)
    {
        if (mshr_entries[mshr_head].valid)
        {
            if (mshr_entries[mshr_head].count == 0 && mshr_entries[mshr_head].dirty == 0)
            {
                mshr_entries[mshr_head].valid = false;
                if (count_mshr > 0)
                {
                    mshr_head = (mshr_head + 1) % MSHR_ENTRY_SIZE;
                    count_mshr--;
                }
            }
            else if (mshr_entries[mshr_head].dirty == 0)
            {
                mshr_state = MSHR_WAIT_READ;
            }
            else
            {
                mshr_state = MSHR_WAIT_WRITE;
            }
        }
        else
        {
            if (mshr_head != mshr_tail)
            {
                mshr_head = (mshr_head + 1) % MSHR_ENTRY_SIZE;
            }
        }
    }
    else if (mshr_state == MSHR_WAIT_WRITE)
    {
        if (io.mem->data.last)
        {
            if (mshr_entries[mshr_head].valid)
            {
                mshr_state = MSHR_WAIT_READ;
            }
        }
    }
    else if (mshr_state == MSHR_WAIT_READ)
    {
        if (io.mem->data.last)
        {
            mshr_state = MSHR_TRAN;
        }
    }
    else if (mshr_state == MSHR_TRAN)
    {
        if (done == 0)
        {
            mshr_state = MSHR_IDLE;
        }
    }
    if (DCACHE_LOG)
    {
        printf("MSHR mshr_state:%d mshr_head:%d mshr_tail:%d count_mshr:%d table_head:%d table_tail:%d count_table:%d offset:%d entry:%d done:%d wdone:%d wdonelast:%d wdata_valid:%d io.control->flush:%d io.control->mispred:%d\n", mshr_state, mshr_head, mshr_tail, count_mshr, table_head, table_tail, count_table, offset, entry, done, wdone, wdonelast, wdata_valid, io.control->flush, io.control->mispred);
        for (int i = 0; i < MSHR_ENTRY_SIZE; i++)
        {
            printf("MSHR entry[%d] valid:%d tag:0x%08x index:%d way:%d dirty:%d paddr:0x%08x bias:%d\n", i, mshr_entries[i].valid, mshr_entries[i].tag, mshr_entries[i].index, mshr_entries[i].way, mshr_entries[i].dirty, mshr_entries[i].paddr, mshr_entries[i].bias);
        }
        for (int i = 0; i < MSHR_TABLE_SIZE; i++)
        {
            printf("MSHR table[%d] valid:%d entry:%d type:%d offset:%d wdata:0x%08x wstrb:%02x tag:0x%08x preg:%d rob_idx:%d\n", i, mshr_table[i].valid, mshr_table[i].entry, mshr_table[i].type, mshr_table[i].offset, mshr_table[i].wdata, mshr_table[i].wstrb, mshr_table[i].uop.tag, mshr_table[i].uop.dest_preg, mshr_table[i].uop.rob_idx);
        }
    }
}
void MSHR::broadcast_mshr(uint32_t index, uint32_t way,uint32_t tag)
{

    for (int i = 1; i < count_mshr; i++)
    {
        uint32_t mshr_index = (mshr_head + i) % MSHR_ENTRY_SIZE;
        if (mshr_entries[mshr_index].index == index && mshr_entries[mshr_index].way == way && mshr_entries[mshr_index].valid)
        {
            mshr_entries[mshr_index].dirty = 1;
            mshr_entries[mshr_index].paddr = get_addr(tag,index,0);
        }
    }
}
uint32_t MSHR::find_entry(uint32_t tag, uint32_t index, uint32_t way, uint32_t dirty)
{
    for (int i = 1; i < count_mshr; i++)
    {
        uint32_t mshr_index = (mshr_head + i) % MSHR_ENTRY_SIZE;
        if (mshr_entries[mshr_index].tag == tag && mshr_entries[mshr_index].index == index && mshr_entries[mshr_index].way == way && mshr_entries[mshr_index].valid)
        {
            mshr_entries[mshr_index].dirty |= dirty;
            // if (DCACHE_LOG)
            //     printf("MSHR find_entry hit tag:0x%08x index:%d way:%d dirty:%d at mshr_index:%d\n", tag, index, way, dirty, mshr_index);
            return mshr_index;
        }
    }
    if (mshr_entries[mshr_head].tag == tag && mshr_entries[mshr_head].index == index && mshr_entries[mshr_head].way == way && mshr_entries[mshr_head].valid)
    {
        if(dirty == 0 )return mshr_head;
        else if(dirty==1 && mshr_state != MSHR_TRAN && mshr_state != MSHR_WAIT_READ){
            // if (DCACHE_LOG)
            //     printf("MSHR find_entry hit tag:0x%08x index:%d way:%d dirty:%d at mshr_index:%d\n", tag, index, way, dirty, mshr_head);
            return mshr_head;
        }
        mshr_entries[mshr_head].bias = 1;
    }

    return MSHR_ENTRY_SIZE;
}

void MSHR::add_entry(uint32_t tag, uint32_t index, uint32_t way, bool dirty, uint32_t paddr)
{
    mshr_entries[mshr_tail].valid = true;
    mshr_entries[mshr_tail].tag = tag;
    mshr_entries[mshr_tail].index = index;
    mshr_entries[mshr_tail].way = way;
    mshr_entries[mshr_tail].dirty = dirty;
    mshr_entries[mshr_tail].paddr = paddr;
    mshr_entries[mshr_tail].bias = 0;
    mshr_entries[mshr_tail].count = 1;
    mshr_tail = (mshr_tail + 1) % MSHR_ENTRY_SIZE;
    if (DCACHE_LOG)
        printf("MSHR add_entry tag:0x%08x index:%d way:%d dirty:%d paddr:0x%08x\n", tag, index, way, dirty, paddr);
    count_mshr++;
}
void MSHR::add_table_entry(uint32_t type, uint32_t entry, uint32_t offset_table, uint32_t wdata, uint8_t wstrb, Inst_uop uop)
{
    mshr_table[table_tail].valid = true;
    mshr_table[table_tail].entry = entry;
    mshr_table[table_tail].type = type;
    mshr_table[table_tail].offset = offset_table;
    mshr_table[table_tail].wdata = wdata;
    mshr_table[table_tail].wstrb = wstrb;
    mshr_table[table_tail].uop = uop;
    if (DCACHE_LOG)
        printf("MSHR add_table_entry index:%d type:%d entry:%d offset:%d wdata:0x%08x wstrb:%02x tag:0x%08x preg:%d rob_idx:%d\n", table_tail, type, entry, offset_table, wdata, wstrb, uop.tag, uop.dest_preg, uop.rob_idx);
    table_tail = (table_tail + 1) % MSHR_TABLE_SIZE;
    count_table++;
}
void MSHR::add_free(uint32_t entry)
{
    mshr_table[entry].valid = false;
    mshr_entries[mshr_table[entry].entry].count--;
}
bool find_mshr_table(uint32_t addr, uint32_t &hit_way)
{
    for (int i = 0; i < MSHR_ENTRY_SIZE; i++)
    {
        if (mshr_entries[i].valid && mshr_entries[i].paddr == addr && (i != mshr_head || (i == mshr_head && mshr_state != MSHR_TRAN && mshr_state != MSHR_WAIT_READ)))
        {
            hit_way = mshr_entries[i].way;
            return true;
        }
    }
    return false;
}
void MSHR::change_bias(uint32_t index, uint32_t way)
{
    for (int i = 0; i<count_mshr; i++)
    {
        uint32_t mshr_index = (mshr_head + i) % MSHR_ENTRY_SIZE;
        if (mshr_entries[mshr_index].valid && mshr_entries[mshr_index].index == index && mshr_entries[mshr_index].way == way)
        {
            mshr_entries[mshr_index].bias = 1;
        }
    }
}