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
    if (cpu_ld_in->req == true && io.mshr_control->ready == true&&control->flush==false)
    {
        if(hit_ld == true){
            transfer_zero(io.mshr_ld);
        }
        else
        {
            transfer_data(io.mshr_ld, cpu_ld_in, tag_ld, offset_ld, index_ld,hit_way_ld,dirty_writeback_ld,paddr_ld,io.mshr_control->ready);
        }
    }
    else
    {
        transfer_zero(io.mshr_ld);
    }

    if (cpu_st_in->req == true && io.mshr_control->ready == true&&control->flush==false)
    {
        if(hit_st == true){
            transfer_zero(io.mshr_st);

        }
        else
        {
            transfer_data(io.mshr_st, cpu_st_in, tag_st, offset_st, index_st,hit_way_st,dirty_writeback_st,paddr_st,io.mshr_control->ready);
        }
    }else
    {
        transfer_zero(io.mshr_st);
    }

    io.mshr_control->flush=io.control->flush;
}
//================================================================================
void Dcache::comb_out()
{
    if(io.mshr_control->st_out){
        io.cpu_st_out->valid = true;
        io.cpu_st_out->data = 0;
        io.cpu_st_out->wr = true;
        io.cpu_st_out->tag = cpu_st_in->tag;
        io.cpu_st_out->preg = cpu_st_in->preg;
        io.cpu_st_out->rob_idx = cpu_st_in->rob_idx;
        io.cpu_st_out->page_fault = false;
    }
    io.cpu_st_out->valid = hit_st == true && io.cpu_st_in->req&&!io.control->flush&&!io.mshr_control->st_out;
    io.cpu_st_out->wr = true;
    io.cpu_st_out->data = 0;
    io.cpu_st_out->tag = io.cpu_st_in->tag;
    io.cpu_st_out->preg = io.cpu_st_in->preg;
    io.cpu_st_out->rob_idx = io.cpu_st_in->rob_idx;
    io.cpu_st_out->page_fault = false;

    io.cpu_ld_out->valid = io.cpu_ld_in->req && !io.control->flush &&
                           (hit_ld == true || io.cpu_ld_in->page_fault) &&
                           !io.mshr_control->ld_out;
    io.cpu_ld_out->wr = false;
    io.cpu_ld_out->data = data_ld;
    io.cpu_ld_out->tag = io.cpu_ld_in->tag;
    io.cpu_ld_out->preg = io.cpu_ld_in->preg;
    io.cpu_ld_out->rob_idx = io.cpu_ld_in->rob_idx;
    io.cpu_ld_out->page_fault = io.cpu_ld_in->page_fault;
    
    io.cpu_ld_in->ready = io.mshr_control->ready && ((hit_ld == true|| io.cpu_ld_in->page_fault) && !io.mshr_control->ld_out);
    io.cpu_st_in->ready = io.mshr_control->ready && (hit_st == true && !io.mshr_control->st_out);
}
void Dcache::seq()
{
//================================================================================
    if (io.cpu_ld_in->req == true&&io.cpu_ld_in->page_fault==false)
    {
        uint32_t tag_ld;
        uint32_t index_ld;
        uint32_t offset_ld;
        get_addr_info(io.cpu_ld_in->addr, tag_ld, index_ld, offset_ld);
        tag_and_data_read(index_ld, tag_ld_way, data_ld_way);
    }
    if (io1.cpu_st_in->req == true)
    {
        uint32_t tag_st;
        uint32_t index_st;
        uint32_t offset_st;
        get_addr_info(io.cpu_st_in->addr, tag_st, index_st, offset_st);
        tag_and_data_read(index_st, tag_st_way, data_st_way);
    }
//================================================================================
    if (io.mshr_control->ready==true&&io.control->flush==false)
    {
        cpu_ld_in = !io.mshr_control->ld_out?io.cpu_ld_in: cpu_ld_in;
        cpu_st_in = !io.mshr_control->st_out?io.cpu_st_in: cpu_st_in;
    }
    else if(io.control->flush==true){
        cpu_ld_in->req = false;
        cpu_st_in->req = false;
    }    
//================================================================================
    if (cpu_ld_in->req == true)
    {
        if(cpu_ld_in->page_fault==true){
            data_ld = cpu_ld_in->rdata;
        }
        else {
            uint32_t tag_ld;
            uint32_t index_ld;
            uint32_t offset_ld;
            hit_ld = false;
            uint32_t hit_way_ld = -1;
            get_addr_info(cpu_ld_in->addr, tag_ld, index_ld, offset_ld);
            hit_check(index_ld, tag_ld, offset_ld, tag_ld_way, data_ld_way, hit_ld,data_ld, hit_way_ld);
            if(hit_ld==true){
                hit_num++;
                updatelru(index_ld, hit_way_ld);
                if (DCACHE_LOG)
                {
                    printf("read cache data addr:0x%08x rdata:0x%08x index_ld:%d offset_ld:%d way_ld:%d\n", cpu_ld_in->addr, data_ld, index_ld, offset_ld, hit_way_ld);
                }
            }
            else
            {
                hit_way_ld = getlru(index_ld);
                miss_deal(index_ld, hit_way_ld, tag_ld, dirty_writeback_ld,paddr_ld);
                miss_num++;
            }
        }
    }
    else
    {
        data_ld = 0;
    }

    if (cpu_st_in->req == true)
    {
        uint32_t tag_st;
        uint32_t index_st;
        uint32_t offset_st;
        hit_st = false;
        uint32_t hit_way_st = -1;
        get_addr_info(cpu_st_in->addr, tag_st, index_st, offset_st);
        hit_check(index_st, tag_st, offset_st, tag_st_way, data_st_way, hit_st, data_st, hit_way_st);
        if(hit_st==true){
            updatelru(index_st, hit_way_st);
            write_cache_data(index_st, hit_way_st, offset_st, cpu_st_in->wdata, cpu_st_in->wstrb);
            hit_num++;
            if (DCACHE_LOG)
            {
                printf("write cache data addr:0x%08x wdata:0x%08x wstrb:%02x index_st:%d offset_st:%d way_st:%d\n", cpu_st_in->addr, dcache_data[index_st][hit_way_st][offset_st], cpu_st_in->wstrb, index_st, offset_st, hit_way_st);
            }
        }
        else {
            hit_way_st = getlru(index_st);
            miss_deal(index_st, hit_way_st, tag_st, dirty_writeback_st,paddr_st);
            miss_num++;
        }
    }
    else 
    {
        
    }
}

void Dcache::output(Mem_OUT*out,bool valid,bool wr,uint32_t data,uint32_t tag,uint32_t preg,uint32_t rob_idx,bool page_fault)
{
    out->valid = valid;
    out->wr = wr;
    out->data = data;
    out->tag = tag;
    out->preg = preg;
    out->rob_idx = rob_idx;
    out->page_fault = page_fault;
}