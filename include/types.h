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

enum Flag { CARRY = 4, HALF_CARRY, SUBTRACT, ZERO };

struct CPU {
    struct Registers regs;
    uint8_t current_opcode;
};

union Memory {
    uint8_t raw[0x10000];
    struct {
        union {
            uint8_t rombanks[0x8000];
            struct {
                uint8_t rombank[0x4000];
                uint8_t rombank_switchable[0x4000];
            };
        };
        uint8_t video_ram[0x2000];
        uint8_t external_ram[0x2000];
        uint8_t internal_ram[0x2000];
        uint8_t echo_ram[0x1E00];
        uint8_t oam[0xA0]; // Object Attribute Memory
        uint8_t unusable[0x60];
        uint8_t ioports[0x80]; // TODO: Implement this as a union
        uint8_t high_internal_ram[0x7F];
        uint8_t interrupt_enable_register;
    };
};

struct Console {
    struct CPU cpu;
    union Memory memory;
};

struct Instruction {
    char* name;
    uint8_t size, cycles, cycles_onbranch;
    void(*execute)(struct Console* console);
    uint8_t arg1, arg2;
};

#endif
