#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

struct Registers {
    union {
        uint8_t raw[8];
        struct {
            union {
                uint16_t af;
                struct { uint8_t f, a; };
            };
            union {
                uint16_t bc;
                struct { uint8_t c, b; };
            };
            union {
                uint16_t de;
                struct { uint8_t e, d; };
            };
            union {
                uint16_t hl;
                struct { uint8_t l, h; };
            };
        };
    };
    uint16_t sp, pc;
};

struct CPU {
    struct Registers regs;
    uint8_t current_opcode;
};

// Memory is just a raw contiguous array of bytes for now,
// so that I can test out CPU instructions
struct Memory {
    uint8_t raw[512];
};

struct Console {
    struct CPU cpu;
    struct Memory memory;
};

struct Instruction {
    char* name;
    uint8_t size, cycles, cycles_onbranch;
    void(*execute)(struct Console* console);
    uint8_t arg1, arg2;
};

#endif
