#include "Dcache.h"
#include <cstdio>

void Dcache::init()
{

    hit_num = 0;
    miss_num = 0;

    dirty_writeback_ld = false;
    dirty_writeback_st = false;
}

void Dcache::comb_in()
{
    if (cpu_ld_in.req == true && io.mshr_control->ready == true && io.control->flush == false)
    {
        if (hit_ld == true)
        {
            transfer_zero(io.mshr_ld);
        }
        else
        {
            transfer_data(io.mshr_ld, cpu_ld_in, tag_ld, offset_ld, index_ld, hit_way_ld, dirty_writeback_ld, paddr_ld, io.mshr_control->ready);
        }
    }
    else
    {
        transfer_zero(io.mshr_ld);
    }

    if (cpu_st_in.req == true && io.mshr_control->ready == true && io.control->flush == false)
    {
        if (hit_st == true)
        {
            transfer_zero(io.mshr_st);
        }
        else
        {
            transfer_data(io.mshr_st, cpu_st_in, tag_st, offset_st, index_st, hit_way_st, dirty_writeback_st, paddr_st, io.mshr_control->ready);
        }
    }
    else
    {
        transfer_zero(io.mshr_st);
    }

    io.mshr_control->flush = io.control->flush;
}
//================================================================================
void Dcache::comb_out()
{
    if (io.mshr_control->st_out)
    {
        output(io.cpu_st_out, true, true, 0, io.mshr_out->addr,io.mshr_out->fun3, io.mshr_out->size, io.mshr_out->offset, io.mshr_out->tag, io.mshr_out->preg, io.mshr_out->rob_idx, false);
    }
    else
    {
        output(io.cpu_st_out, hit_st == true && io.cpu_st_in->req && !io.control->flush, true, 0, cpu_st_in.addr,cpu_st_in.fun3, cpu_st_in.size, cpu_st_in.offset, cpu_st_in.tag, cpu_st_in.preg, cpu_st_in.rob_idx, cpu_st_in.page_fault);
    }

    if (io.mshr_control->ld_out)
    {
        output(io.cpu_ld_out, true, false, io.mshr_out->data,io.mshr_out->addr, io.mshr_out->fun3, io.mshr_out->size, io.mshr_out->offset, io.mshr_out->tag, io.mshr_out->preg, io.mshr_out->rob_idx, false);
    }
    else
    {
        output(io.cpu_ld_out, (hit_ld == true || io.cpu_ld_in->page_fault) && io.cpu_ld_in->req && !io.control->flush, false, data_ld, cpu_ld_in.addr, cpu_ld_in.fun3, cpu_ld_in.size, cpu_ld_in.offset, cpu_ld_in.tag, cpu_ld_in.preg, cpu_ld_in.rob_idx, cpu_ld_in.page_fault);
    }

    io.cpu_ld_in->ready = io.mshr_control->ready && ((hit_ld == false && !io.cpu_ld_in->page_fault) || (hit_ld == true && !io.mshr_control->ld_out) || (io.cpu_ld_in->page_fault && !io.mshr_control->ld_out));
    io.cpu_st_in->ready = io.mshr_control->ready && (hit_st == false || (hit_st == true && !io.mshr_control->st_out));

    if (DCACHE_LOG)
    {
        printf("Dcache cpu_ld_in_ready:%d cpu_st_in_ready:%d\n", io.cpu_ld_in->ready, io.cpu_st_in->ready);
        printf("Dcache hit_ld:%d hit_st:%d st_out:%d ld_out:%d\n", hit_ld, hit_st, io.mshr_control->st_out, io.mshr_control->ld_out);
    }
}
void Dcache::seq()
{
    //================================================================================
    if (io.cpu_ld_in->req == true && io.cpu_ld_in->page_fault == false)
    {
        uint32_t tag_ld_tmp;
        uint32_t index_ld_tmp;
        uint32_t offset_ld_tmp;
        get_addr_info(io.cpu_ld_in->addr, tag_ld_tmp, index_ld_tmp, offset_ld_tmp);
        tag_and_data_read(index_ld_tmp, tag_ld_way, data_ld_way);
    }
    if (io.cpu_st_in->req == true)
    {
        uint32_t tag_st_tmp;
        uint32_t index_st_tmp;
        uint32_t offset_st_tmp;
        get_addr_info(io.cpu_st_in->addr, tag_st_tmp, index_st_tmp, offset_st_tmp);
        tag_and_data_read(index_st_tmp, tag_st_way, data_st_way);
    }
    //================================================================================
    if (io.mshr_control->ready == true && io.control->flush == false)
    {
        cpu_ld_in = !io.mshr_control->ld_out ? (*io.cpu_ld_in) : cpu_ld_in;
        cpu_st_in = !io.mshr_control->st_out ? (*io.cpu_st_in) : cpu_st_in;
    }
    else if (io.control->flush == true)
    {
        cpu_ld_in.req = false;
        cpu_st_in.req = false;
    }
    //================================================================================
    if (cpu_ld_in.req == true)
    {
        if (cpu_ld_in.page_fault == true)
        {
            data_ld = cpu_ld_in.rdata;
        }
        else
        {

            hit_ld = false;
            hit_way_ld = -1;
            get_addr_info(cpu_ld_in.addr, tag_ld, index_ld, offset_ld);
            hit_check(index_ld, tag_ld, offset_ld, tag_ld_way, data_ld_way, hit_ld, data_ld, hit_way_ld);
            if (hit_ld == true)
            {
                hit_num++;
                updatelru(index_ld, hit_way_ld);
                if (DCACHE_LOG)
                {
                    printf("read cache data addr:0x%08x rdata:0x%08x index_ld:%d offset_ld:%d way_ld:%d\n", cpu_ld_in.addr, data_ld, index_ld, offset_ld, hit_way_ld);
                }
            }
            else
            {
                hit_way_ld = getlru(index_ld);
                miss_deal(index_ld, hit_way_ld, tag_ld, dirty_writeback_ld, paddr_ld);
                miss_num++;
            }
        }
    }
    else
    {
        data_ld = 0;
    }

    if (cpu_st_in.req == true)
    {

        hit_st = false;
        hit_way_st = -1;
        get_addr_info(cpu_st_in.addr, tag_st, index_st, offset_st);
        hit_check(index_st, tag_st, offset_st, tag_st_way, data_st_way, hit_st, data_st, hit_way_st);
        if (hit_st == true)
        {
            updatelru(index_st, hit_way_st);
            write_cache_data(index_st, hit_way_st, offset_st, cpu_st_in.wdata, cpu_st_in.wstrb);
            hit_num++;
            if (DCACHE_LOG)
            {
                printf("write cache data addr:0x%08x wdata:0x%08x wstrb:%02x index_st:%d offset_st:%d way_st:%d\n", cpu_st_in.addr, dcache_data[index_st][hit_way_st][offset_st], cpu_st_in.wstrb, index_st, offset_st, hit_way_st);
            }
        }
        else
        {
            hit_way_st = getlru(index_st);
            miss_deal(index_st, hit_way_st, tag_st, dirty_writeback_st, paddr_st);
            miss_num++;
        }
    }
    else
    {
    }

    if(DCACHE_LOG)
    {
        printf("Dcache seq io.cpu_ld_in.req:%d io.cpu_ld_in.page_fault:%d io.cpu_st_in.req:%d\n", io.cpu_ld_in->req, io.cpu_ld_in->page_fault, io.cpu_st_in->req);
        printf("Dcache seq cpu_ld_in.req:%d cpu_ld_in.page_fault:%d cpu_st_in.req:%d\n", cpu_ld_in.req, cpu_ld_in.page_fault, cpu_st_in.req);
        printf("Dcache seq io.cpu_lad_in.addr:0x%08x io.cpu_st_in.addr:0x%08x\n", io.cpu_ld_in->addr, io.cpu_st_in->addr);
        printf("Dcache seq cpu_ld_in.addr:0x%08x cpu_st_in.addr:0x%08x\n", cpu_ld_in.addr, cpu_st_in.addr);
        printf("Dcache seq hit_ld:%d hit_way_ld:%d hit_st:%d hit_way_st:%d\n", hit_ld, hit_way_ld, hit_st, hit_way_st);
        printf("Dcache hit_num:%d miss_num:%d\n", hit_num, miss_num);
    }
}

void Dcache::output(Mem_OUT *out, bool valid, bool wr, uint32_t data, uint32_t addr, uint32_t fun3,uint32_t size, uint32_t offset_load, uint32_t tag, uint32_t preg, uint32_t rob_idx, bool page_fault)
{
    out->valid = valid;
    out->wr = wr;
    out->data = data;
    out->addr = addr;
    out->fun3 = fun3;
    out->size = size;
    out->offset = offset_load;
    out->tag = tag;
    out->preg = preg;
    out->rob_idx = rob_idx;
    out->page_fault = page_fault;
    printf("Dcache output valid:%d wr:%d data:0x%08x addr:%x fun3:0x%08x size:%d offset:%d tag:0x%08x preg:%d rob_idx:%d page_fault:%d\n", valid, wr, data, addr, fun3, size, offset_load, tag, preg, rob_idx, page_fault);
}