#include "PhysMemory.h"

// 物理内存指针——由 main() 通过 calloc() 分配，由所有 cache/LSU 模块共享。
uint32_t *p_memory = nullptr;
