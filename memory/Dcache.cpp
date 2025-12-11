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
    // if(DCACHE_LOG){
    //     printf("mshr control ready:%d\n",io.mshr_control->ready);
    // }
    if (cpu_ld_in.req == true && io.mshr_control->ready == true  && !uncache_access)
    {
        if (hit_ld == true)
        {
            transfer_zero(io.mshr_ld);
        }
        else
        {
            transfer_data(io.mshr_ld, cpu_ld_in, tag_ld, offset_ld, index_ld, hit_way_ld, dirty_writeback_ld, paddr_ld, io.mshr_control->ready,mispred_reg);
            dcache_dirty[index_ld][hit_way_ld]=0;
            mispred_reg = false;
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
            dcache_dirty[index_st][hit_way_st]=0;
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

void Dcache::comb_hit()
{

    mispred_1 = (io.control->br_mask & (1 << io.cpu_ld_in->uop.tag)) && io.control->mispred;
    mispred_2 = (io.control->br_mask & (1 << cpu_ld_in.uop.tag)) && io.control->mispred;
    uncache_access = (cpu_ld_in.uop.page_fault_load == true) || (cpu_ld_in.addr == 0x1fd0e00) || (cpu_ld_in.addr == 0x1fd0e04);
    if (cpu_ld_in.req == true)
    {
        if (uncache_access == true)
        {
            data_ld = cpu_ld_in.wdata;
            hit_ld = false;
        }
        else
        {
            hit_ld = false;
            hit_way_ld = -1;
            get_addr_info(cpu_ld_in.addr, tag_ld, index_ld, offset_ld);
            hit_check(index_ld, tag_ld, tag_ld_way, hit_ld, hit_way_ld);
            if (!hit_ld)
            {
                if(hit_way_ld==-1)hit_way_ld = getlru(index_ld);
                miss_deal(index_ld, hit_way_ld, tag_ld, dirty_writeback_ld, paddr_ld, tag_ld_way);
                miss_num++;
            }
            else
            {
                hit_num++;
                mispred_reg=false;
                updatelru(index_ld, hit_way_ld);
                data_ld = read_cache_data_pipeline(hit_way_ld, data_ld_way);
                if (DCACHE_LOG)
                {
                    printf("read cache data addr:0x%08x rdata:0x%08x dcache_data:0x%08x index_ld:%d offset_ld:%d way_ld:%d\n", cpu_ld_in.addr, data_ld, dcache_data[index_ld][hit_way_ld][offset_ld], index_ld, offset_ld, hit_way_ld);
                }
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
        hit_check(index_st, tag_st, tag_st_way, hit_st, hit_way_st);
        if (!hit_st)
        {
            if(hit_way_st==-1)hit_way_st = getlru(index_st);
            miss_deal(index_st, hit_way_st, tag_st, dirty_writeback_st, paddr_st, tag_st_way);
            miss_num++;
        }
        else
        {
            hit_num++;
            updatelru(index_st, hit_way_st);
            write_cache_data_pipeline(index_st, hit_way_st, offset_st, data_st_way, cpu_st_in.wdata, cpu_st_in.wstrb);
            if (index_st == index_ld && hit_way_st == hit_way_ld && offset_st == offset_ld)
            {
                write_cache_data_load(hit_way_ld, data_st_way, data_ld_way, cpu_st_in.wdata, cpu_st_in.wstrb);
            }
            if (DCACHE_LOG)
            {
                printf("write cache data:%08x addr:0x%08x wdata:0x%08x wstrb:%02x index_st:%d offset_st:%d way_st:%d\n",io.cpu_st_in->wdata, cpu_st_in.addr, dcache_data[index_st][hit_way_st][offset_st], cpu_st_in.wstrb, index_st, offset_st, hit_way_st);
            }
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
        output(io.cpu_ld_out, (hit_ld == true || uncache_access) && cpu_ld_in.req && !io.control->flush && !mispred_2, false, data_ld, cpu_ld_in.addr, cpu_ld_in.uop);
    }
    io.cpu_ld_in->ready = (io.mshr_control->ready && 
                          ((hit_ld == false && !io.cpu_ld_in->uop.page_fault_load) || 
                          (hit_ld == true && !io.mshr_control->ld_out) || 
                          (io.cpu_ld_in->uop.page_fault_load && !io.mshr_control->ld_out)))||
                          (!io.mshr_control->ready && hit_ld && !io.mshr_control->ld_out);
    io.cpu_st_in->ready = (io.mshr_control->ready && (hit_st == false || (hit_st == true && !io.mshr_control->st_out)))||
                          (!io.mshr_control->ready && hit_st && !io.mshr_control->st_out);
}
uint32_t old_cache_data_debug = 0;
uint32_t old_cache_dirty_debug=0;
uint32_t old_cache_tag_debug=0;
void Dcache::seq()
{
    //================================================================================
    if (io.cpu_ld_in->req == true && io.cpu_ld_in->uop.page_fault_load == false)
    {
        get_addr_info(io.cpu_ld_in->addr, tag_ld_tmp, index_ld_tmp, offset_ld_tmp);
        tag_and_data_read(index_ld_tmp, offset_ld_tmp, tag_ld_way_1, data_ld_way_1);

        
    }
    if (io.cpu_st_in->req == true)
    {
        uint32_t tag_st_tmp;
        uint32_t index_st_tmp;
        uint32_t offset_st_tmp;
        get_addr_info(io.cpu_st_in->addr, tag_st_tmp, index_st_tmp, offset_st_tmp);
        tag_and_data_read(index_st_tmp, offset_st_tmp, tag_st_way_1, data_st_way_1);
    }

    uint32_t out_index = 0;
    uint32_t out_way = 0;
    uint32_t out_offset = 3;
    // if(DEBUG&&old_cache_data_debug != dcache_data[out_index][out_way][out_offset]){
    //     old_cache_data_debug = dcache_data[out_index][out_way][out_offset];
    //     printf("Debug Dcache seq sim_time:%lld dcache_data[%d][%d][%d]:0x%08x data_tag:0x%08x data_valid:%d data_issued:%d dcache_dirty:%d\n",sim_time,out_index,out_way,out_offset, old_cache_data_debug, dcache_tag[out_index][out_way], dcache_valid[out_index][out_way], dcache_issued[out_index][out_way], dcache_dirty[out_index][out_way]);
    // }
    if(DEBUG&&io.cpu_st_in->req == true && io.cpu_st_in->addr &0xfffffffc ==DEBUG_ADDR){
        printf("Debug Dcache seq 2 sim_time:%lld cpu_st_in addr:0x%08x wdata:0x%08x wstrb:%02x preg:%d rob_idx:%d\n",sim_time, io.cpu_st_in->addr, io.cpu_st_in->wdata, io.cpu_st_in->wstrb, io.cpu_st_in->uop.dest_preg, io.cpu_st_in->uop.rob_idx);
    }
    if (DCACHE_LOG)
    {
        printf("\n\n");
        for (int i = 0; i < DCACHE_WAY_NUM; i++)
        {
            printf("dcache_data[%d][%d][%d]:0x%08x dcache_valid[%d][%d]:%d dcache_tag[%d][%d]:0x%08x dcache_lru[%d][%d]:%d dcache_issued[%d][%d]:%d dcache_dirty[%d][%d]:%d addr:0x%08x\n", out_index, i, out_offset, dcache_data[out_index][i][out_offset], out_index, i, dcache_valid[out_index][i], out_index, i, dcache_tag[out_index][i], out_index, i, dcache_lru[out_index][i], out_index, i, dcache_issued[out_index][i], out_index, i, dcache_dirty[out_index][i],get_addr(dcache_tag[out_index][i], out_index, out_offset));
        }
        // printf("dcache_data[53][1][3]:0x%08x dcache_valid[45][1]:%d dcache_tag[45][1]:0x%08x\n", dcache_data[45][1][3], dcache_valid[45][1], dcache_tag[45][1]);

        printf("============================ Dcache Input ============================\n");
        printf("Dcache Input io.cpu_ld_in->req:%d inst:0x%08x addr:0x%08x page_fault:%d      preg:%02d rob_idx:%02d inst_idx:%ld tag_ld_tmp:0x%08x index_ld_tmp:%d offset_ld_tmp:%d\n", io.cpu_ld_in->req, io.cpu_ld_in->uop.instruction, io.cpu_ld_in->addr, io.cpu_ld_in->uop.page_fault_load, io.cpu_ld_in->uop.dest_preg, io.cpu_ld_in->uop.rob_idx, io.cpu_ld_in->uop.inst_idx, tag_ld_tmp, index_ld_tmp, offset_ld_tmp);
        printf("Dcache Input io.cpu_st_in->req:%d                 addr:0x%08x data:0x%08x   preg:%02d rob_idx:%02d\n", io.cpu_st_in->req, io.cpu_st_in->addr, io.cpu_st_in->wdata, io.cpu_st_in->uop.dest_preg, io.cpu_st_in->uop.rob_idx);
        printf("Dcache Input cpu_ld_in.req:%d     inst:0x%08x addr:0x%08x page_fault:%d      preg:%02d rob_idx:%02d tag:0x%08x index:%d offset:%d mispred:%d\n", cpu_ld_in.req, cpu_ld_in.uop.instruction, cpu_ld_in.addr, cpu_ld_in.uop.page_fault_load, cpu_ld_in.uop.dest_preg, cpu_ld_in.uop.rob_idx, tag_ld, index_ld, offset_ld,mispred_reg);
        printf("Dcache Input cpu_st_in.req:%d                     addr:0x%08x data:0x%08x   preg:%02d rob_idx:%02d tag:0x%08x index:%d offset:%d\n", cpu_st_in.req, cpu_st_in.addr, cpu_st_in.wdata, cpu_st_in.uop.dest_preg, cpu_st_in.uop.rob_idx, tag_st, index_st, offset_st);
        printf("============================ Dcache Output ============================\n");
        printf("Dcache Output io.cpu_ld_out->valid:%d inst:0x%08x addr:0x%08x data:0x%08x preg:%02d rob_idx:%02d\n", io.cpu_ld_out->valid, io.cpu_ld_out->uop.instruction, io.cpu_ld_out->addr, io.cpu_ld_out->data, io.cpu_ld_out->uop.dest_preg, io.cpu_ld_out->uop.rob_idx);
        printf("Dcache Output io.cpu_st_out->valid:%d inst:0x%08x addr:0x%08x                 preg:%02d rob_idx:%02d\n", io.cpu_st_out->valid, io.cpu_st_out->uop.instruction, io.cpu_st_out->addr, io.cpu_st_out->uop.dest_preg, io.cpu_st_out->uop.rob_idx);
        printf("Dcache Output io.mshr_out->valid:%d   inst:0x%08x addr:0x%08x data:0x%08x preg:%02d rob_idx:%02d\n", io.mshr_out->valid, io.mshr_out->uop.instruction, io.mshr_out->addr, io.mshr_out->data, io.mshr_out->uop.dest_preg, io.mshr_out->uop.rob_idx);
        printf("Dcache Stats io.mshr_control->ld_out:%d io.mshr_control->st_out:%d\n", io.mshr_control->ld_out, io.mshr_control->st_out);
        printf("============================ Dcache to MSHR ============================\n");
        printf("Dcache to MSHR io.mshr_ld->valid:%d addr:0x%08x paddr:0x%08x index:%d way:%d offset:%d dirty:%d wr:%d preg:%02d rob_idx:%02d mispred:%d\n", io.mshr_ld->valid, io.mshr_ld->addr, io.mshr_ld->paddr, io.mshr_ld->index, io.mshr_ld->way, io.mshr_ld->offset, io.mshr_ld->dirty, io.mshr_ld->wr, io.mshr_ld->uop.dest_preg, io.mshr_ld->uop.rob_idx, io.mshr_ld->mispred);
        printf("Dcache to MSHR io.mshr_st->valid:%d addr:0x%08x paddr:0x%08x index:%d way:%d offset:%d dirty:%d wr:%d preg:%02d rob_idx:%02d\n", io.mshr_st->valid, io.mshr_st->addr, io.mshr_st->paddr, io.mshr_st->index, io.mshr_st->way, io.mshr_st->offset, io.mshr_st->dirty, io.mshr_st->wr, io.mshr_st->uop.dest_preg, io.mshr_st->uop.rob_idx);
        printf("============================ Dcache Stats ============================\n");
        printf("Dcache Stats hit_ld:%d hit_way_ld:%d dirty_writeback_ld:%d paddr_ld:0x%08x\n", hit_ld, hit_way_ld, dirty_writeback_ld, paddr_ld);
        printf("Dcache Stats hit_st:%d hit_way_st:%d dirty_writeback_st:%d paddr_st:0x%08x\n", hit_st, hit_way_st, dirty_writeback_st, paddr_st);
        printf("Dcache Stats io.mshr_control->ready:%d io.control->flush:%d io.control->mispred:%d mispred_1:%d mispred_2:%d\n", io.mshr_control->ready, io.control->flush, io.control->mispred, mispred_1, mispred_2);
        printf("Dcache stats cpu_ld_in.ready:%d cpu_st_in.ready:%d\n", io.cpu_ld_in->ready, io.cpu_st_in->ready);
        printf("Dcache Stats hit_num:%d miss_num:%d\n\n\n", hit_num, miss_num);
    }
    // ================================================================================
    bool ld_out = (!io.mshr_control->ld_out & hit_ld) | (!hit_ld);
    bool st_out = (!io.mshr_control->st_out & hit_st) | (!hit_st);
    if(DCACHE_LOG)
        printf("io.mshr_control->ready:%d ld_out:%d st_out:%d\n", io.mshr_control->ready, ld_out, st_out);
    
    if (io.mshr_control->ready == true && ld_out)
    {
        if (mispred_1 || io.control->flush == true)
        {
            cpu_ld_in.req = false;
            memset(tag_ld_way, 0, sizeof(tag_ld_way));
            memset(data_ld_way, 0, sizeof(data_ld_way));
        }
        else
        {
            cpu_ld_in = (*io.cpu_ld_in);
            memcpy(tag_ld_way, tag_ld_way_1, sizeof(tag_ld_way));
            memcpy(data_ld_way, data_ld_way_1, sizeof(data_ld_way));
        }
    }
    else if(io.mshr_control->ready == false && hit_ld && !io.mshr_control->ld_out){
        cpu_ld_in = (*io.cpu_ld_in);
        memcpy(tag_ld_way, tag_ld_way_1, sizeof(tag_ld_way));
        memcpy(data_ld_way, data_ld_way_1, sizeof(data_ld_way));

    }else if(!hit_ld && mispred_2&&cpu_ld_in.req){ 
        mispred_reg = true;
        if(DCACHE_LOG){
            printf("Dcache seq mispred_2 true cpu_ld_in.req:%d cpu_ld_in.addr:0x%08x\n", cpu_ld_in.req, cpu_ld_in.addr);
        }
    }
    else if(hit_ld && mispred_2&&cpu_ld_in.req){ 
        cpu_ld_in.req = false;
        memset(tag_ld_way, 0, sizeof(tag_ld_way));
        memset(data_ld_way, 0, sizeof(data_ld_way));
        if(DCACHE_LOG){
            printf("Dcache seq mispred_3 true cpu_ld_in.req:%d cpu_ld_in.addr:0x%08x\n", cpu_ld_in.req, cpu_ld_in.addr);
        }

    }
    else{
        for(int i=0;i<DCACHE_WAY_NUM;i++){
            data_ld_way[i] = dcache_data[index_ld][i][offset_ld];
        }
    }

    if (io.mshr_control->ready == true && st_out)
    {
        cpu_st_in = (*io.cpu_st_in);
        memcpy(tag_st_way, tag_st_way_1, sizeof(tag_st_way));
        memcpy(data_st_way, data_st_way_1, sizeof(data_st_way));
    }else if(io.mshr_control->ready == false && hit_st && !io.mshr_control->st_out){
        cpu_st_in = (*io.cpu_st_in);
        memcpy(tag_st_way, tag_st_way_1, sizeof(tag_st_way));
        memcpy(data_st_way, data_st_way_1, sizeof(data_st_way));
    }else{
        for(int i=0;i<DCACHE_WAY_NUM;i++){
            data_st_way[i] = dcache_data[index_st][i][offset_st];
        }
    }
    if(DCACHE_LOG){
        printf("Dcache seq end cpu_ld_in.req:%d cpu_ld_in.addr:0x%08x cpu_st_in.req:%d cpu_st_in.addr:0x%08x\n", cpu_ld_in.req, cpu_ld_in.addr, cpu_st_in.req, cpu_st_in.addr);
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