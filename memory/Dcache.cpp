#include "Dcache.h"
#include <cstdio>

void Dcache::init()
{
    hit_ld1 = false;
    hit_st1 = false;

    hit_way_ld1 = -1;
    hit_way_st1 = -1;

    hit_ld2 = false;
    hit_st2 = false;

    hit_way_ld2 = -1;
    hit_way_st2 = -1;

    hit_num = 0;
    miss_num = 0;

    dirty_writeback_ld = false;
    dirty_writeback_st = false;

}

void Dcache::comb_in()
{
    if (io2.cpu_ld_in->req == true && io2.mshr_control->ready == true&&io1.control->flush==false)
    {
        if(hit_ld2 == true){
            transfer_zero(io2.mshr_ld);

        }
        else
        {
            transfer_data(io2.mshr_ld, io2.cpu_ld_in, tag_ld2, offset_ld2, index_ld2,hit_way_ld2,dirty_writeback_ld,paddr_ld,io2.mshr_control->ready);
        }
    }
    else
    {
        transfer_zero(io2.mshr_ld);
    }

    if (io2.cpu_st_in->req == true && io2.mshr_control->ready == true&&io1.control->flush==false)
    {
        if(hit_st2 == true){
            transfer_zero(io2.mshr_st);

        }
        else
        {
            transfer_data(io2.mshr_st, io2.cpu_st_in, tag_st2, offset_st2, index_st2,hit_way_st2,dirty_writeback_st,paddr_st,io2.mshr_control->ready);
        }
    }else
    {
        transfer_zero(io2.mshr_st);
    }

    io2.mshr_control->flush=io2.control->flush;
}
//================================================================================
void Dcache::comb_out()
{
    if(io2.)
    io2.cpu_st_out->valid = hit_st2 == true && io2.cpu_st_in->req&&!io1.control->flush;
    io2.cpu_st_out->wr = true;
    io2.cpu_st_out->data = 0;
    io2.cpu_st_out->tag = io2.cpu_st_in->tag;
    io2.cpu_st_out->preg = io2.cpu_st_in->preg;
    io2.cpu_st_out->rob_idx = io2.cpu_st_in->rob_idx;
    io2.cpu_st_out->page_fault = false;

    io2.cpu_ld_out->valid = (hit_ld2 == true && io2.cpu_ld_in->req && !io1.control->flush)||(io2.cpu_ld_in->req && io2.cpu_ld_in->page_fault && !io1.control->flush);
    io2.cpu_ld_out->wr = false;
    io2.cpu_ld_out->data = rdata;
    io2.cpu_ld_out->tag = io2.cpu_ld_in->tag;
    io2.cpu_ld_out->preg = io2.cpu_ld_in->preg;
    io2.cpu_ld_out->rob_idx = io2.cpu_ld_in->rob_idx;
    io2.cpu_ld_out->page_fault = io2.cpu_ld_in->page_fault;
    
    io1.cpu_ld_in->ready = io2.mshr_control->ready;
    io2.cpu_st_in->ready = io2.mshr_control->ready;
}
//================================================================================
void Dcache::seq()
{
    if (io1.cpu_ld_in->req == true&&io1.cpu_ld_in->page_fault==false)
    {
        get_addr_info(io1.cpu_ld_in->addr, tag_ld1, index_ld1, offset_ld1);
        hit_ld1 = hit_check(index_ld1, tag_ld1, hit_way_ld1);
    }else{
        hit_ld1 = false;
    }
    if (io1.cpu_st_in->req == true)
    {
        get_addr_info(io1.cpu_st_in->addr, tag_st1, index_st1, offset_st1);
        hit_st1 = hit_check(index_st1, tag_st1, hit_way_st1);
    }else{
        hit_st1 = false;
    }


    if (io2.mshr_control->ready==true&&io1.control->flush==false)
    {
        io2 = io1;
        tag_ld2 = tag_ld1;
        index_ld2 = index_ld1;
        offset_ld2 = offset_ld1;

        tag_st2 = tag_st1;
        index_st2 = index_st1;
        offset_st2 = offset_st1;

        hit_ld2 = hit_ld1;
        hit_st2 = hit_st1;
        hit_way_ld2 = hit_way_ld1;
        hit_way_st2 = hit_way_st1;
    }
    else if(io1.control->flush==true){
        io2.cpu_ld_in->req = false;
        io2.cpu_st_in->req = false;
    }
    
    

    //================================================================================
    if (io2.cpu_ld_in->req == true)
    {
        if(io2.cpu_ld_in->page_fault==true){
            rdata = io2.cpu_ld_in->rdata;
        }
        else if(hit_ld2==false){
            miss_deal(index_ld2, hit_way_ld2, tag_ld2, dirty_writeback_ld,paddr_ld);
            miss_num++;
        }
        else
        {
            updatelru(index_ld2, hit_way_ld2);
            rdata = read_cache_data(index_ld2, hit_way_ld2, offset_ld2);
            hit_num++;
            if (DCACHE_LOG)
            {
                printf("read cache data addr:0x%08x rdata:0x%08x index_ld:%d offset_ld:%d way_ld:%d\n", io2.cpu_ld_in->addr, rdata, index_ld2, offset_ld2, hit_way_ld2);
            }
        }
    }
    else
    {
        rdata = 0;
    }

    if (io2.cpu_st_in->req == true)
    {
        if(hit_st2==true){
            updatelru(index_st2, hit_way_st2);
            write_cache_data(index_st2, hit_way_st2, offset_st2, io2.cpu_st_in->wdata, io2.cpu_st_in->wstrb);
            hit_num++;
            if (DCACHE_LOG)
            {
                printf("write cache data addr:0x%08x wdata:0x%08x wstrb:%02x index_st:%d offset_st:%d way_st:%d\n", io2.cpu_st_in->addr, dcache_data[index_st2][hit_way_st2][offset_st2], io2.cpu_st_in->wstrb, index_st2, offset_st2, hit_way_st2);
            }
        }
        else {
            miss_deal(index_st2, hit_way_st2, tag_st2, dirty_writeback_st,paddr_st);
            miss_num++;
        }
    }
    else 
    {
        
    }
}