#ifndef TEST_MMIO_H
#define TEST_MMIO_H
#include <stdint.h>
uint32_t mmio_read_32(uintptr_t addr);
void mmio_write_32(uintptr_t addr, uint32_t value);
#endif
