#pragma once

#include "IO.h"
#include <config.h>
#include "Dcache_Utils.h"

const int MSHR_ENTRY_SIZE = 8;
const int MSHR_TABLE_SIZE = 16;

typedef struct{
    bool valid;
    uint32_t tag;
    uint32_t index;
    uint32_t way;
    uint32_t paddr;
    bool dirty;
    uint32_t count;
}mshr_entry;

typedef struct{
    bool valid;
    uint32_t entry;
    bool type;
    uint32_t offset;
    uint32_t wdata;
    uint8_t wstrb;
    
    Inst_uop uop;
}table_entry;

enum MSHR_STATE{
    MSHR_IDLE,
    MSHR_WAIT_WRITE, 
    MSHR_WAIT_READ,
    MSHR_TRAN
};

extern mshr_entry mshr_entries[MSHR_ENTRY_SIZE];
extern table_entry mshr_table[MSHR_TABLE_SIZE];
extern uint32_t free_table[MSHR_TABLE_SIZE];

class MSHR_IO
{
public:
    MSHR_INFO* dcache_ld;
    MSHR_INFO* dcache_st;
    Cache_Mshr* control;
    EXMem_IO* mem;
    Mem_OUT* cpu;
};
class MSHR
{
public:
    MSHR_IO io;
    
    uint32_t mshr_head;
    uint32_t mshr_tail;
    uint32_t table_head;
    uint32_t table_tail;
    uint32_t count_mshr;
    uint32_t count_table;
    uint32_t offset;
    uint32_t entry;

    uint32_t rdata;
    Inst_uop ruop;
    bool wdone;
    bool wdonelast;
    uint32_t wdata;
    bool wdata_valid;
    uint32_t done;
    
    enum MSHR_STATE state;

    void init();
    void comb_in();
    void comb_out();
    void seq();
    uint32_t find_entry(uint32_t tag,uint32_t index);
    void add_entry(uint32_t tag,uint32_t index,uint32_t way,bool dirty,uint32_t paddr);
    void add_free(uint32_t entry);
    void add_table_entry(bool type,uint32_t entry,uint32_t offset_table,uint32_t wdata,uint8_t wstrb,Inst_uop uop);
};

