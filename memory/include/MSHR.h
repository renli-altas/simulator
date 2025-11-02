#pragma once

#include "IO.h"
#include <config.h>
#include "Dcache_Config.h"

const int MSHR_SIZE = 8;

typedef struct{
    bool valid;
    bool wr; 
    uint32_t addr;
    uint32_t wdata;
    uint8_t wstrb;
}mshr_entry;

enum MSHR_STATE{
    IDLE,
    WAIT_READ,
    WAIT_WRITE, 
};

mshr_entry mshr_entries[MSHR_SIZE];


class MSHR_IO
{
public:
    MSHR_DCACHE* cpu_ld;
    MSHR_DCACHE* cpu_st;
    EXMem_IO* mem;

};
class MSHR
{
public:
    MSHR_IO io;
    
    uint32_t point_mshr_new;
    uint32_t point_mshr_handle;
    bool ld_allocated;
    bool st_allocated;

    uint32_t tag_inst;
    uint32_t index_inst;
    uint32_t offset_inst;

    MSHR_STATE handle_state;
    uint32_t lru_way;
    uint32_t lru_way_addr;
    uint32_t write_index;
    uint32_t read_index;
    uint32_t offset_data;

    void add_ld_entry();
    void add_st_entry();
    void handle_mshr(); 
    void deal_cache();
    void deal_mshr_entries();

    void read_mem();
    void write_mem();

    bool check_load();

    void init();
    void comb();
    void seq();
};

