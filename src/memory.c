#include "memory.h"

uint8_t memRead8(union Memory* memory, uint16_t address) {
    return memory->internal_ram[address];
}

uint16_t memRead16(union Memory* memory, uint16_t address) {
    return (memory->internal_ram[address + 1] << 8) | memory->internal_ram[address];
}

void memWrite8(union Memory* memory, uint16_t address, uint8_t data) {
    memory->internal_ram[address] = data;
}

void memWrite16(union Memory* memory, uint16_t address, uint16_t data) {
    memory->internal_ram[address] = data & 0x00FF;
    memory->internal_ram[address + 1] = (data & 0xFF00) >> 8;
}
