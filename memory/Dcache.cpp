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

    dirty_writeback_ld = false;
    dirty_writeback_st = false;

}

void Dcache::comb_in()
{
    if (state_st == DCACHE_WAIT)
    {
        transfer_data(io.mshr_st, io.cpu_st, tag_st, offset_st, index_st,hit_way_st,dirty_writeback_st,paddr_st,io.mshr_control->ready);
    }
    else
    {
        transfer_zero(io.mshr_st);
    }

    if (state_ld == DCACHE_WAIT)
    {
        transfer_data(io.mshr_ld, io.cpu_ld, tag_ld, offset_ld, index_ld,hit_way_ld,dirty_writeback_ld,paddr_ld,io.mshr_control->ready);
    }
    else
    {
        transfer_zero(io.mshr_ld);
    }

    io.mshr_control->flush=io.control->flush;
}
//================================================================================
void Dcache::comb_out()
{
    if (hit_st == true && io.cpu_st->req)
    {
        io.cpu_st->data_ok = true;
    }
    else
    {
        if (state_st == DCACHE_WAIT && io.mshr_st->done)
        {
            io.cpu_st->data_ok = true;
        }
        else
        {
            io.cpu_st->data_ok = false;
        }
    }

    if (hit_ld == true && req_ld_reg && !io.control->flush)
    {
        io.cpu_ld->rdata = rdata;
        io.cpu_ld->data_ok = true;
    }
    else
    {
        if (state_ld == DCACHE_WAIT && io.mshr_ld->done)
        {
            if (!io.control->flush)
            {
                io.cpu_ld->rdata = io.mshr_ld->rdata;
                io.cpu_ld->data_ok = true;
            }
            else
            {
                io.cpu_ld->data_ok = false;
                io.cpu_ld->rdata = 0;
            }
        }
        else
        {
            io.cpu_ld->data_ok = false;
            io.cpu_ld->rdata = 0;
        }
    }
}
//================================================================================
void Dcache::seq()
{
    if (state_ld == DCACHE_IDLE && io.cpu_ld->req == true)
    {
        get_addr_info(io.cpu_ld->addr, tag_ld, index_ld, offset_ld);
    }
    if (state_st == DCACHE_IDLE && io.cpu_st->req == true)
    {
        get_addr_info(io.cpu_st->addr, tag_st, index_st, offset_st);
        if(tag_st == 0x002020cf && index_st==0x23 && offset_st==0x2){
            printf("Dcache store req addr:0x%08x tag:0x%08x index:0x%02x offset:0x%02x wdata:0x%08x wstrb:0x%02x\n", io.cpu_st->addr, tag_st, index_st, offset_st, io.cpu_st->wdata, io.cpu_st->wstrb);
        }
    }

    //================================================================================
    if (io.cpu_ld->req == true && state_ld == DCACHE_IDLE)
    {
        hit_ld = hit_check(index_ld, tag_ld, hit_way_ld);
    }
    else if (io.cpu_ld->req == false)
    {
        hit_ld = false;
    }

    if (io.cpu_st->req == true && state_st == DCACHE_IDLE)
    {
        hit_st = hit_check(index_st, tag_st, hit_way_st);
    }
    else if (io.cpu_st->req == false)
    {
        hit_st = false;
    }

    //================================================================================
    if (hit_ld && io.cpu_ld->req == true && state_ld == DCACHE_IDLE && !io.cpu_ld->data_ok)
    {
        updatelru(index_ld, hit_way_ld);
        rdata = read_cache_data(index_ld, hit_way_ld, offset_ld);
        hit_num++;
    }
    else if (!hit_ld && io.cpu_ld->req == true && state_ld == DCACHE_IDLE)
    {
        miss_deal(index_ld, hit_way_ld, tag_ld, dirty_writeback_ld,paddr_ld);
        miss_num++;
    }


    

    if (hit_st && io.cpu_st->req == true && state_st == DCACHE_IDLE)
    {
        updatelru(index_st, hit_way_st);
        write_cache_data(index_st, hit_way_st, offset_st, io.cpu_st->wdata, io.cpu_st->wstrb);
        if (DCACHE_LOG)
        {
            printf("write cache data addr:0x%08x wdata:0x%08x wstrb:%02x index_st:%d offset_st:%d way_st:%d\n", io.cpu_st->addr, dcache_data[index_st][hit_way_st][offset_st], io.cpu_st->wstrb, index_st, offset_st, hit_way_st);
        }
        hit_num++;
    }
    else if (!hit_st && io.cpu_st->req == true && state_st == DCACHE_IDLE)
    {
        miss_deal(index_st, hit_way_st, tag_st, dirty_writeback_st,paddr_st);
        miss_num++;
    }

    if (io.cpu_ld->req == false || io.cpu_ld->data_ok == true)
    {
        req_ld_reg = false;
    }
    else
    {
        req_ld_reg = true;
    }
    // if(io.cpu_st->req==true&&io.cpu_st->addr==0x808b6c00){
    //     printf("Dcache store req addr:0x%08x wdata:0x%08x wstrb:%02x hit:%d data_ok:%d\n", io.cpu_st->addr, io.cpu_st->wdata, io.cpu_st->wstrb, hit_st, io.cpu_st->data_ok);
    // }
        
    if (DCACHE_LOG)
    {
        printf("\n\n");
        printf("========== Dcache Status ==========\n");
        printf("Dcache state_ld:%d state_st:%d flush:%d rdata:%08x\n", state_ld, state_st, io.control->flush, rdata);
        printf("ld req:%d addr:0x%08x tag:0x%08x index:0x%02x offset:0x%02x hit:%d hit_way:%2d dirty_writeback:%d \n", io.cpu_ld->req, io.cpu_ld->addr, tag_ld, index_ld, offset_ld, hit_ld, hit_way_ld, dirty_writeback_ld);
        printf("st req:%d addr:0x%08x tag:0x%08x index:0x%02x offset:0x%02x hit:%d hit_way:%2d dirty_writeback:%d \n", io.cpu_st->req, io.cpu_st->addr, tag_st, index_st, offset_st, hit_st, hit_way_st, dirty_writeback_st);
        printf("ld rdata:0x%08x data_ok:%d\n", io.cpu_ld->rdata, io.cpu_ld->data_ok);
        printf("st wdata:0x%08x data_ok:%d\n", io.cpu_st->wdata, io.cpu_st->data_ok);
        printf("Dcache hit num:%d miss num:%d\n\n", hit_num, miss_num);
    }
    //================================================================================
    change_state(state_ld, io.cpu_ld->req & !io.control->flush, hit_ld, io.mshr_ld->done, io.control->flush);
    change_state(state_st, io.cpu_st->req , hit_st, io.mshr_st->done, false);
}