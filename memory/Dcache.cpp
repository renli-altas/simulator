#include "Dcache.h"
#include <cstdio>

void Dcache::init()
{
    hit_ld = false;
    hit_st = false;

    hit_way_ld = -1;
    hit_way_st = -1;

    state_ld = DCACHE_IDLE;
    state_st = DCACHE_IDLE;

    hit_num = 0;
    miss_num = 0;

    offset_id_ld = 0;
    offset_id_st = 0;


}

void Dcache::comb()
{
    if(state_ld==DCACHE_IDLE&&io.cpu_ld->req==true){
        get_addr_info(io.cpu_ld->addr, tag_ld, index_ld, offset_ld);
    }
    if(state_st==DCACHE_IDLE&&io.cpu_st->req==true){
        get_addr_info(io.cpu_st->addr, tag_st, index_st, offset_st);
    }

    if (hit_st == true)
    {
        io.cpu_st->data_ok = true;
        transfer_zero(io.mem_st);
    }
    else
    {
        if (state_st == DCACHE_WRITE)
        {
            /* code */
        }
        else if(state_st==DCACHE_READ){
            transfer_data(io.cpu_st, io.mem_st);
            io.cpu_st->data_ok = io.mem_st->data.last;
        }else {
            transfer_zero(io.mem_st);
            io.cpu_st->data_ok = false;
        }
    }

    if (hit_ld == true)
    {
        io.cpu_ld->data_ok = true;
        transfer_zero(io.mem_ld);
    }
    else
    {
        if(state_ld==DCACHE_WRITE){
            /* code */
        }
        else if(state_ld==DCACHE_READ){
            transfer_data(io.cpu_ld, io.mem_ld);
            io.cpu_ld->data_ok = io.mem_ld->data.last;
        }
        else {
            transfer_zero(io.mem_ld);
            io.cpu_ld->data_ok = false;
        }
    }
}
void Dcache::seq()
{

    if (io.cpu_ld->req == true&&state_ld==DCACHE_IDLE)
    {
        hit_ld = hit_check(index_ld, tag_ld, hit_way_ld);
    }
    if (io.cpu_st->req == true&&state_st==DCACHE_IDLE)
    {
        hit_st = hit_check(index_st, tag_st, hit_way_st);
    }

    if(hit_ld && io.cpu_ld->req==true && state_ld==DCACHE_IDLE){
        updatelru(index_ld);
        uselru(index_ld, hit_way_ld);
        rdata = read_cache_line(index_ld, hit_way_ld, offset_ld);
        hit_num++;
    }else if(!hit_ld && io.cpu_ld->req==true && state_ld==DCACHE_IDLE){
        miss_deal(index_ld, hit_way_ld, tag_ld, hit_way_ld, dirty_writeback_ld);
        miss_num++;
    }

    if(hit_st && io.cpu_st->req==true && state_st==DCACHE_IDLE){
        updatelru(index_st);
        uselru(index_st, hit_way_st);
        write_cache_line(index_st, hit_way_st, offset_st, io.cpu_st->wdata, io.cpu_st->wstrb);
        hit_num++;
    }else if(!hit_st && io.cpu_st->req==true && state_st==DCACHE_IDLE){
        miss_deal(index_st, hit_way_st, tag_st, hit_way_st, dirty_writeback_st);
        miss_num++;
    }

    change_state(state_ld,io.cpu_ld->req,hit_ld,io.mem_ld->data.last,dirty_writeback_ld);
    change_state(state_st,io.cpu_st->req,hit_st,io.mem_st->data.last,dirty_writeback_st);

    if(state_ld == DCACHE_READ){
        transfer_cache_line(index_ld, hit_way_ld, offset_id_ld, io.mem_ld->data.data, io.mem_ld->data.done, io.mem_ld->data.last);
    }
    if(state_st == DCACHE_READ){
        transfer_cache_line(index_st, hit_way_st, offset_id_st, io.mem_st->data.data, io.mem_st->data.done, io.mem_st->data.last);
    }
}
