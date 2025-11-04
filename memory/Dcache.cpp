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

    dirty_writeback_ld = false;
    dirty_writeback_st = false;

    flush_flag_ld = false;
    flush_flag_st = false;
}

void Dcache::comb()
{

    if (hit_st == true)
    {
        io.cpu_st->data_ok = true;
        transfer_zero(io.mem_st);
    }
    else
    {
        if (state_st == DCACHE_WRITE)
        {
            write_data(io.mem_st, write_data_st, io.cpu_st->addr&0xfffffffc, offset_id_st);
        }
        else if(state_st==DCACHE_READ){
            read_data(io.mem_st, io.cpu_st->addr&0xfffffffc, offset_id_st);
            if(!flush_flag_st)io.cpu_st->data_ok = io.mem_st->data.last;
            else io.cpu_st->data_ok = false;
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
            write_data(io.mem_ld, write_data_ld, io.cpu_ld->addr&0xfffffffc, offset_id_ld);
        }
        else if(state_ld==DCACHE_READ){
            read_data(io.mem_ld, io.cpu_ld->addr&0xfffffffc, offset_id_ld);
            if(!flush_flag_ld)io.cpu_ld->data_ok = io.mem_ld->data.last;
            else io.cpu_ld->data_ok = false;
            if(offset_ld==offset_id_ld)io.cpu_ld->rdata = rdata;
            else io.cpu_ld->rdata = 0;
        }
        else {
            transfer_zero(io.mem_ld);
            io.cpu_ld->data_ok = false;
        }
    }
}
void Dcache::seq()
{   
    if(!io.control->flush){
        
        if(state_ld==DCACHE_IDLE&&io.cpu_ld->req==true){
            get_addr_info(io.cpu_ld->addr, tag_ld, index_ld, offset_ld);
        }
        if(state_st==DCACHE_IDLE&&io.cpu_st->req==true){
            get_addr_info(io.cpu_st->addr, tag_st, index_st, offset_st);
        }

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
    }
    
    if(state_ld!=DCACHE_IDLE){
        flush_flag_ld = io.control->flush;
    }else{
        flush_flag_ld = false;
    }
    if(state_st!=DCACHE_IDLE){
        flush_flag_st = io.control->flush;
    }else{
        flush_flag_st = false;
    }

    change_state(state_ld,io.cpu_ld->req&!io.control->flush,hit_ld,io.mem_ld->data.last,dirty_writeback_ld,flush_flag_ld);
    change_state(state_st,io.cpu_st->req&!io.control->flush,hit_st,io.mem_st->data.last,dirty_writeback_st,flush_flag_st);

    

    if(state_ld == DCACHE_WRITE){
        write_cache_line(index_ld, hit_way_ld, offset_id_ld, write_data_ld, io.mem_ld->data.done, io.mem_ld->data.last);
        if(io.mem_ld->data.last){
            dirty_writeback_ld = false;
            dcache_tag[index_ld][hit_way_ld] = tag_ld;
            dcache_valid[index_ld][hit_way_ld] = false;
            dcache_dirty[index_ld][hit_way_ld] = false;
        }
    }else if(state_ld == DCACHE_READ){
        printf("state_ld READ done:%d last:%d\n",io.mem_ld->data.done,io.mem_ld->data.last);
        read_cache_line(index_ld, hit_way_ld, offset_id_ld, io.mem_ld->data.data, io.mem_ld->data.done, io.mem_ld->data.last);
        if(io.mem_ld->data.last){
            dcache_valid[index_ld][hit_way_ld] = true;
        }
    }

    if(state_st == DCACHE_WRITE){
        write_cache_line(index_st, hit_way_st, offset_id_st, write_data_st, io.mem_st->data.done, io.mem_st->data.last);

        if(io.mem_st->data.last){
            dirty_writeback_st = false;
            dcache_tag[index_st][hit_way_st] = tag_st;
            dcache_valid[index_st][hit_way_st] = false;
            dcache_dirty[index_st][hit_way_st] = false;
        }

    }else if(state_st == DCACHE_READ){
        read_cache_line(index_st, hit_way_st, offset_id_st, io.mem_st->data.data, io.mem_st->data.done, io.mem_st->data.last);
        if(io.mem_st->data.last){
            dcache_valid[index_st][hit_way_st] = true;
        }
    }

    if(DCACHE_LOG){
        printf("\n\nDcache state_ld:%d state_st:%d flush_ld:%d flush_st:%d flush:%d\n",state_ld,state_st,flush_flag_ld,flush_flag_st,io.control->flush);
        printf("ld req:%d addr:0x%08x tag:0x%08x index:0x%02x offset:0x%02x hit:%d hit_way:%2d dirty_writeback:%d control->flush:%d offset_id_ld:%d\n",io.cpu_ld->req,io.cpu_ld->addr,tag_ld,index_ld,offset_ld,hit_ld,hit_way_ld,dirty_writeback_ld,flush_flag_ld,offset_id_ld);
        printf("st req:%d addr:0x%08x tag:0x%08x index:0x%02x offset:0x%02x hit:%d hit_way:%2d dirty_writeback:%d control->flush:%d offset_id_st:%d\n",io.cpu_st->req,io.cpu_st->addr,tag_st,index_st,offset_st,hit_st,hit_way_st,dirty_writeback_st,flush_flag_st,offset_id_st);
        printf("ld rdata:0x%08x data_ok:%d\n",io.cpu_ld->rdata,io.cpu_ld->data_ok);
        printf("st data_ok:%d\n",io.cpu_st->data_ok);
        printf("Dcache hit num:%d miss num:%d\n",hit_num,miss_num);
    }
}