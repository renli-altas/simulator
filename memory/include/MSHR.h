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
    bool issued;
}mshr_entry;

typedef struct{
    bool valid;
    uint32_t entry;
    bool type;
    bool dirty;
    uint32_t offset;
}table_entry;

enum MSHR_STATE{
    IDLE,
    WAIT_READ,
    WAIT_WRITE, 
};

mshr_entry mshr_entries[MSHR_ENTRY_SIZE];
table_entry mshr_table[MSHR_TABLE_SIZE];


class MSHR_IO
{
public:
    MSHR_INFO* dcache_ld;
    MSHR_INFO* dcache_st;
    Cache_Mshr* control;
    EXMem_IO* mem;
};
class MSHR
{
public:
    MSHR_IO io;
    
    uint32_t mshr_head;
    uint32_t mshr_tail;
    uint32_t table_head;
    uint32_t table_tail;
    uint32_t count;
    uint32_t offset;
    uint32_t entry;

    uint32_t rdata;


    enum MSHR_STATE state;

    void init();
    void comb_in();
    void comb_out1();
    void comb_out2();
    void seq();
    uint32_t find_entry(uint32_t tag,uint32_t index);
    void add_entry(uint32_t tag,uint32_t index,uint32_t way);
    void add_table_entry(bool type,uint32_t entry,bool dirty);
};

