#include "memory.h"

uint8_t memRead8(struct Memory* memory, uint16_t address) {
    return memory->raw[address];
}

uint16_t memRead16(struct Memory* memory, uint16_t address) {
    return (memory->raw[address + 1] << 8) | memory->raw[address];
}

void memWrite8(struct Memory* memory, uint16_t address, uint8_t data) {
    memory->raw[address] = data;
}

void memWrite16(struct Memory* memory, uint16_t address, uint16_t data) {
    memory->raw[address] = data & 0x00FF;
    memory->raw[address + 1] = (data & 0xFF00) >> 8;
}
