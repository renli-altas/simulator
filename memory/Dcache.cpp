#include "Dcache.h"
#include <cstdio>
#include <cstring>
void Dcache::init()
{

    hit_num = 0;
    miss_num = 0;

    dirty_writeback_ld = false;
    dirty_writeback_st = false;
}

void Dcache::comb_in()
{
    if (cpu_ld_in.req == true && io.mshr_control->ready == true && io.control->flush == false && !mispred_pc)
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

    if (cpu_st_in.req == true && io.mshr_control->ready == true)
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
    io.mshr_control->mispred = io.control->mispred;
    io.mshr_control->br_mask = io.control->br_mask;
}

void Dcache::comb_hit(){
    
    mispred_pc = (io.control->br_mask & (1 << io.cpu_ld_in->uop.tag))&io.control->mispred;
    if (cpu_ld_in.req == true)
    {
        if (cpu_ld_in.uop.page_fault_load == true)
        {
            data_ld = cpu_ld_in.rdata;
        }
        else
        {
            hit_ld = false;
            hit_way_ld = -1;
            get_addr_info(cpu_ld_in.addr, tag_ld, index_ld, offset_ld);
            hit_check(index_ld, tag_ld, offset_ld, tag_ld_way, data_ld_way, hit_ld, data_ld, hit_way_ld);
            if(!hit_ld){
                hit_way_ld = getlru(index_ld);
            }
        }
    }
    else
    {
        hit_ld = false;
        data_ld = 0;
    }
    if (cpu_st_in.req == true)
    {

        hit_st = false;
        hit_way_st = -1;
        get_addr_info(cpu_st_in.addr, tag_st, index_st, offset_st);
        hit_check(index_st, tag_st, offset_st, tag_st_way, data_st_way, hit_st, data_st, hit_way_st);
        if(!hit_st)
        {
            hit_way_st = getlru(index_st);
        }
    }
    else
    {
        hit_st = false;
    }
}
//================================================================================
void Dcache::comb_out()
{
    if (io.mshr_control->st_out)
    {
        output(io.cpu_st_out, true, true, 0, io.mshr_out->addr, io.mshr_out->uop);
    }
    else
    {
        output(io.cpu_st_out, hit_st == true && cpu_st_in.req, true, 0, cpu_st_in.addr, cpu_st_in.uop);
    }

    if (io.mshr_control->ld_out)
    {
        output(io.cpu_ld_out, true, false, io.mshr_out->data, io.mshr_out->addr, io.mshr_out->uop);
    }
    else
    {
        output(io.cpu_ld_out, (hit_ld == true || cpu_ld_in.uop.page_fault_load) && cpu_ld_in.req && !io.control->flush && !mispred_pc, false, data_ld, cpu_ld_in.addr, cpu_ld_in.uop);
    }
    io.cpu_ld_in->ready = io.mshr_control->ready && ((hit_ld == false && !io.cpu_ld_in->uop.page_fault_load) || (hit_ld == true && !io.mshr_control->ld_out) || (io.cpu_ld_in->uop.page_fault_load && !io.mshr_control->ld_out));
    io.cpu_st_in->ready = io.mshr_control->ready && (hit_st == false || (hit_st == true && !io.mshr_control->st_out));
}
void Dcache::seq()
{
    //================================================================================
    if (io.cpu_ld_in->req == true && io.cpu_ld_in->uop.page_fault_load == false)
    {
        uint32_t tag_ld_tmp;
        uint32_t index_ld_tmp;
        uint32_t offset_ld_tmp;
        get_addr_info(io.cpu_ld_in->addr, tag_ld_tmp, index_ld_tmp, offset_ld_tmp);
        tag_and_data_read(index_ld_tmp, tag_ld_way_1, data_ld_way_1);

    }
    if (io.cpu_st_in->req == true)
    {
        uint32_t tag_st_tmp;
        uint32_t index_st_tmp;
        uint32_t offset_st_tmp;
        get_addr_info(io.cpu_st_in->addr, tag_st_tmp, index_st_tmp, offset_st_tmp);
        tag_and_data_read(index_st_tmp, tag_st_way_1, data_st_way_1);
    }
    //================================================================================
    if(cpu_ld_in.req == true&&cpu_ld_in.uop.page_fault_load==false){
        if(hit_ld){
            hit_num++;
            updatelru(index_ld, hit_way_ld);
            if (DCACHE_LOG)
            {
                printf("read cache data addr:0x%08x rdata:0x%08x index_ld:%d offset_ld:%d way_ld:%d\n", cpu_ld_in.addr, data_ld, index_ld, offset_ld, hit_way_ld);
            }
        }
        else{
            miss_deal(index_ld, hit_way_ld, tag_ld, dirty_writeback_ld, paddr_ld);
            miss_num++;
        }
    }
    if(cpu_st_in.req == true){
        if(hit_st){
            hit_num++;
            updatelru(index_st, hit_way_st);
            write_cache_data(index_st, hit_way_st, offset_st, cpu_st_in.wdata, cpu_st_in.wstrb);
            if (DCACHE_LOG)
            {
                printf("write cache data addr:0x%08x wdata:0x%08x wstrb:%02x index_st:%d offset_st:%d way_st:%d\n", cpu_st_in.addr, dcache_data[index_st][hit_way_st][offset_st], cpu_st_in.wstrb, index_st, offset_st, hit_way_st);
            }
        }
        else{
            hit_way_st = getlru(index_st);
            miss_deal(index_st, hit_way_st, tag_st, dirty_writeback_st, paddr_st);
            miss_num++;
        }
    }
    //================================================================================

    
    

    if (DCACHE_LOG)
    {
        printf("\n\n============================ Dcache Input ============================\n");
        printf("Dcache Input io.cpu_ld_in->req:%d inst:0x%08x addr:0x%08x page_fault:%d      preg:%02d rob_idx:%02d \n", io.cpu_ld_in->req, io.cpu_ld_in->uop.instruction, io.cpu_ld_in->addr, io.cpu_ld_in->uop.page_fault_load, io.cpu_ld_in->uop.dest_preg, io.cpu_ld_in->uop.rob_idx);
        printf("Dcache Input io.cpu_st_in->req:%d inst:0x%08x addr:0x%08x data:0x%08x   preg:%02d rob_idx:%02d\n", io.cpu_st_in->req, io.cpu_st_in->uop.instruction, io.cpu_st_in->addr, io.cpu_st_in->wdata, io.cpu_st_in->uop.dest_preg, io.cpu_st_in->uop.rob_idx);
        printf("Dcache Input cpu_ld_in.req:%d     inst:0x%08x addr:0x%08x page_fault:%d      preg:%02d rob_idx:%02d tag:0x%08x index:%d offset:%d\n", cpu_ld_in.req, cpu_ld_in.uop.instruction, cpu_ld_in.addr, cpu_ld_in.uop.page_fault_load, cpu_ld_in.uop.dest_preg, cpu_ld_in.uop.rob_idx, tag_ld, index_ld, offset_ld);
        printf("Dcache Input cpu_st_in.req:%d     inst:0x%08x addr:0x%08x data:0x%08x   preg:%02d rob_idx:%02d tag:0x%08x index:%d offset:%d\n", cpu_st_in.req, cpu_st_in.uop.instruction, cpu_st_in.addr, cpu_st_in.wdata, cpu_st_in.uop.dest_preg, cpu_st_in.uop.rob_idx, tag_st, index_st, offset_st);
        printf("============================ Dcache Output ============================\n");
        printf("Dcache Output io.cpu_ld_out->valid:%d inst:0x%08x addr:0x%08x data:0x%08x preg:%02d rob_idx:%02d\n", io.cpu_ld_out->valid, io.cpu_ld_out->uop.instruction, io.cpu_ld_out->addr, io.cpu_ld_out->data, io.cpu_ld_out->uop.dest_preg, io.cpu_ld_out->uop.rob_idx);
        printf("Dcache Output io.cpu_st_out->valid:%d inst:0x%08x addr:0x%08x                 preg:%02d rob_idx:%02d\n", io.cpu_st_out->valid, io.cpu_st_out->uop.instruction, io.cpu_st_out->addr, io.cpu_st_out->uop.dest_preg, io.cpu_st_out->uop.rob_idx);
        printf("Dcache Output io.mshr_out->valid:%d   inst:0x%08x addr:0x%08x data:0x%08x preg:%02d rob_idx:%02d\n", io.mshr_out->valid, io.mshr_out->uop.instruction, io.mshr_out->addr, io.mshr_out->data, io.mshr_out->uop.dest_preg, io.mshr_out->uop.rob_idx);
        printf("Dcache Stats io.mshr_control->ld_out:%d io.mshr_control->st_out:%d\n", io.mshr_control->ld_out, io.mshr_control->st_out);
        printf("============================ Dcache to MSHR ============================\n");
        printf("Dcache to MSHR io.mshr_ld->valid:%d addr:0x%08x wr:%d preg:%02d rob_idx:%02d\n", io.mshr_ld->valid, io.mshr_ld->addr, io.mshr_ld->wr, io.mshr_ld->uop.dest_preg, io.mshr_ld->uop.rob_idx);
        printf("Dcache to MSHR io.mshr_st->valid:%d addr:0x%08x wr:%d preg:%02d rob_idx:%02d\n", io.mshr_st->valid, io.mshr_st->addr, io.mshr_st->wr, io.mshr_st->uop.dest_preg, io.mshr_st->uop.rob_idx);
        printf("============================ Dcache Stats ============================\n");
        printf("Dcache Stats hit_ld:%d hit_way_ld:%d dirty_writeback_ld:%d paddr_ld:0x%08x\n", hit_ld, hit_way_ld, dirty_writeback_ld, paddr_ld);
        printf("Dcache Stats hit_st:%d hit_way_st:%d dirty_writeback_st:%d paddr_st:0x%08x\n", hit_st, hit_way_st, dirty_writeback_st, paddr_st);
        printf("Dcache Stats io.mshr_control->ready:%d io.control->flush:%d io.control->mispred:%d mispred_pc:%d\n", io.mshr_control->ready, io.control->flush, io.control->mispred, mispred_pc);
        printf("Dcache stats cpu_ld_in.ready:%d cpu_st_in.ready:%d\n", io.cpu_ld_in->ready, io.cpu_st_in->ready);
        printf("Dcache Stats hit_num:%d miss_num:%d\n\n\n", hit_num, miss_num);
    }
    //================================================================================
    if (io.mshr_control->ready == true && io.control->flush == false )
    {
        if (((!io.mshr_control->ld_out&&hit_ld)||!hit_ld)&&!mispred_pc)
        {
            cpu_ld_in = (*io.cpu_ld_in);
            memcpy(tag_ld_way, tag_ld_way_1, sizeof(tag_ld_way));
            memcpy(data_ld_way, data_ld_way_1, sizeof(data_ld_way));
        }
        if ((!io.mshr_control->st_out&&hit_st)||!hit_st)
        {
            cpu_st_in = (*io.cpu_st_in);
            memcpy(tag_st_way, tag_st_way_1, sizeof(tag_st_way));
            memcpy(data_st_way, data_st_way_1, sizeof(data_st_way));
        }
    }
    else if (io.control->flush == true || mispred_pc)
    {
        cpu_ld_in.req = false;
        memset(tag_ld_way, 0, sizeof(tag_ld_way));
        memset(data_ld_way, 0, sizeof(data_ld_way));
    }
}

void Dcache::output(Mem_OUT *out, bool valid, bool wr, uint32_t data, uint32_t addr, Inst_uop uop)
{
    out->valid = valid;
    out->wr = wr;
    out->data = data;
    out->addr = addr;
    out->uop = uop;
}