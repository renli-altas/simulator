#include "MSHR.h"
void MSHR::init()
{
    mshr_head = 0;
    mshr_tail = 0;
    table_head = 0;
    table_tail = 0;
    count = 0;
}

void MSHR::comb_in()
{
    if(state == WAIT_WRITE){
        io.mem->control.en = true;
        io.mem->control.wen = true;
        uint32_t addr = get_addr(mshr_entries[mshr_table[table_head].entry].tag,mshr_entries[mshr_table[table_head].entry].index,0);
        io.mem->control.addr = addr;
        io.mem->control.wdata = rdata;
        io.mem->control.len = DCACHE_OFFSET_NUM -1;
        io.mem->control.size = 0b10;
        io.mem->control.sel = 0b1111;
    }
    else if(state == WAIT_READ){
        io.mem->control.en = true;
        io.mem->control.wen = false;
        uint32_t addr = get_addr(mshr_entries[mshr_table[table_head].entry].tag,mshr_entries[mshr_table[table_head].entry].index,0);
        io.mem->control.addr = addr;
        io.mem->control.wdata = 0;
        io.mem->control.len = DCACHE_OFFSET_NUM -1;
        io.mem->control.size = 0b10;
        io.mem->control.sel = 0b1111;
    }
}

void MSHR::comb_out1()
{
    io.control->ready = count < MSHR_ENTRY_SIZE;
}




void MSHR::seq()
{

    if (count < MSHR_ENTRY_SIZE)
    {
        if (io.dcache_st->valid)
        {
            uint32_t entry = find_entry(io.dcache_st->tag, io.dcache_st->index);
            if (entry != MSHR_ENTRY_SIZE)
            {
                if (!find_entry(io.dcache_st->tag, io.dcache_st->index))
                {
                    add_entry(io.dcache_st->tag, io.dcache_st->index, io.dcache_st->way);
                }
            }
            add_table_entry(1, entry, io.dcache_st->dirty);
            count++;
        }
        if (io.dcache_ld->valid)
        {
            uint32_t entry = find_entry(io.dcache_ld->tag, io.dcache_ld->index);
            if (entry != MSHR_ENTRY_SIZE)
            {
                add_table_entry(0, entry, io.dcache_ld->dirty);
            }
            add_table_entry(0, entry, io.dcache_ld->dirty);
            count++;
        }
    }

    if(state == WAIT_WRITE){
        write_cache_line(mshr_entries[entry].index,mshr_entries[entry].way,offset,rdata,io.mem->data.done,io.mem->data.last);
    }
    else if(state == WAIT_READ){
        read_cache_line(mshr_entries[entry].index,mshr_entries[entry].way,offset,rdata,io.mem->data.done,io.mem->data.last);
    }

    if (state == IDLE)
    {
        if (mshr_table[table_head].valid)
        {
            if (mshr_table[table_head].dirty == 0)
            {
                state = WAIT_READ;
            }
            else
            {
                state = WAIT_WRITE;
            }
        }
    }
    else if (state == WAIT_WRITE)
    {
        if (io.mem->data.last)
        {
            state = WAIT_READ;
        }
    }
    else if (state == WAIT_READ)
    {
        if (io.mem->data.last)
        {
            state = IDLE;
        }
    }
}

uint32_t MSHR::find_entry(uint32_t tag, uint32_t index)
{
    for (int i = mshr_head; i != mshr_tail; i = (i + 1) % MSHR_ENTRY_SIZE)
    {
        if (mshr_entries[i].valid && mshr_entries[i].tag == tag && mshr_entries[i].index == index)
        {
            return i;
        }
    }
    return MSHR_ENTRY_SIZE;
}

void MSHR::add_entry(uint32_t tag, uint32_t index, uint32_t way)
{
    mshr_entries[mshr_tail].valid = true;
    mshr_entries[mshr_tail].tag = tag;
    mshr_entries[mshr_tail].index = index;
    mshr_entries[mshr_tail].way = way;
    mshr_entries[mshr_tail].issued = false;
    mshr_tail = (mshr_tail + 1) % MSHR_ENTRY_SIZE;
}
void MSHR::add_table_entry(bool type, uint32_t entry, bool dirty)
{
    mshr_table[table_tail].valid = true;
    mshr_table[table_tail].entry = entry;
    mshr_table[table_tail].type = type;
    mshr_table[table_tail].dirty = dirty;
    table_tail = (table_tail + 1) % MSHR_TABLE_SIZE;
}