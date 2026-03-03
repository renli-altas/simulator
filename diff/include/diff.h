#pragma once
#include "PhysMemory.h"
#include "ref.h"
#include "config.h"
#include <cstdint>

enum { DIFFTEST_TO_DUT, DIFFTEST_TO_REF };

extern CPU_state dut_cpu;
class SimContext;

void init_difftest(int);
void init_diff_ckpt(CPU_state ckpt_state, uint32_t *ckpt_memory);
void get_state(CPU_state &dut_state, uint8_t &privilege, uint32_t *dut_memory);
void difftest_step(bool);
void difftest_skip();
