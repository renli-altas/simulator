#include "Dcache.h"
#include <cstdio>

void Dcache::init()
{
    hit_ld = false;
    hit_st = false;

    hit_way_ld = -1;
    hit_way_st = -1;
}

void Dcache::comb()
{
    get_addr_info(io.cpu_ld->addr, tag_ld, index_ld, offset_ld);
    get_addr_info(io.cpu_st->addr, tag_st, index_st, offset_st);

    if (hit_ld == true)
    {
        io.cpu_ld->data_ok = true;
        io.cpu_ld->rdata = read_cache_line(index_ld, hit_way_ld, offset_ld);
        // io.mem_ld->control.en = false;
        // io.mem_ld->control.wen = false;
        // io.mem_ld->control.addr = 0;
        // io.mem_ld->control.wdata = 0;
        // io.mem_ld->control.sel = 0;
        // io.mem_ld->control.len = 0;
        // io.mem_ld->control.size = 0;
        // io.mem_ld->control.last = true;
    }
    else
    {

    }

    if (hit_st == true)
    {
        io.cpu_st->data_ok = true;
        write_cache_line(index_st, hit_way_st, offset_st, io.cpu_st->wdata, io.cpu_st->wstrb);
        // io.mem_st->control.en = false;
        // io.mem_st->control.wen = false;
        // io.mem_st->control.addr = 0;
        // io.mem_st->control.wdata = 0;
        // io.mem_st->control.sel = 0;
        // io.mem_st->control.len = 0;
        // io.mem_st->control.size = 0;
        // io.mem_st->control.last = true;
    }
    else
    {

    }
}
void Dcache::seq()
{
    if (io.cpu_ld->req == true&&state==DCACHE_IDLE)
    {
        hit_ld = hit_check(index_ld, tag_ld, hit_way_ld);
        updatelru(index_ld);
        if (hit_ld)
        {
            uselru(index_ld, hit_way_ld);
            hit_num++;
        }
    }

    if (io.cpu_st->req == true&&state==DCACHE_IDLE)
    {
        hit_st = hit_check(index_st, tag_st, hit_way_st);
        updatelru(index_st);
        if (hit_st)
        {
            uselru(index_st, hit_way_st);
            hit_num++;
        }
    }
}
