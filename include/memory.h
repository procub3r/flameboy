#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>
#include "types.h"

uint8_t memRead8(struct Memory* memory, uint16_t address);
uint16_t memRead16(struct Memory* memory, uint16_t address);
void memWrite8(struct Memory* memory, uint16_t address, uint8_t data);
void memWrite16(struct Memory* memory, uint16_t address, uint16_t data);

#endif
