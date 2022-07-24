#include <stddef.h>
#include <stdio.h>
#include "types.h"
#include "memory.h"
#include "cpu.h"

// Macro to get offset of a member from struct CPU
#define O(member) offsetof(struct CPU, member)

// Macro to get a register given an offset into struct CPU
#define REG8(offset) (*(uint8_t*)((uint8_t*)&console->cpu + offset))
#define REG16(offset) (*(uint16_t*)((uint8_t*)&console->cpu + offset))

// Macro to get the current instance of struct Instruction
#define INSTRUCTION instructions[console->cpu.current_opcode]

// Macro to access the program counter easily
#define PC console->cpu.regs.pc

void setFlag(struct CPU* cpu, enum Flag flag) {
    cpu->regs.f |= 1 << flag;
}

void clearFlag(struct CPU* cpu, enum Flag flag) {
    cpu->regs.f &= ~(1 << flag);
}

void nop(struct Console* console) {} // Do nothing
void inc(struct Console* console) {}
void dec(struct Console* console) {}
void rlca(struct Console* console) {}
void add(struct Console* console) {}
void rrca(struct Console* console) {}
void stop(struct Console* console) {}
void rla(struct Console* console) {}
void jr(struct Console* console) {}
void rra(struct Console* console) {}
void daa(struct Console* console) {}
void cpl(struct Console* console) {}
void scf(struct Console* console) {}
void ccf(struct Console* console) {}
void halt(struct Console* console) {}
void adc(struct Console* console) {}
void sub(struct Console* console) {}
void sbc(struct Console* console) {}
void and(struct Console* console) {}
void xor(struct Console* console) {}
void or(struct Console* console) {}
void cp(struct Console* console) {}
void ret(struct Console* console) {}
void pop(struct Console* console) {}
void jp(struct Console* console) {}
void call(struct Console* console) {}
void push(struct Console* console) {}
void rst(struct Console* console) {}
void prefix(struct Console* console) {}
void unused(struct Console* console) {}
void reti(struct Console* console) {}
void di(struct Console* console) {}
void ei(struct Console* console) {}

static struct Instruction instructions[256];

void ld(struct Console* console) {}

void ld_read8(struct Console* console) {
    REG8(INSTRUCTION.arg1) = memRead8(&console->memory, REG16(INSTRUCTION.arg2));
}

void ld_read8_incsrc(struct Console* console) {
    REG8(INSTRUCTION.arg1) = memRead8(&console->memory, REG16(INSTRUCTION.arg2)++);
}

void ld_read8_decsrc(struct Console* console) {
    REG8(INSTRUCTION.arg1) = memRead8(&console->memory, REG16(INSTRUCTION.arg2)--);
}

void ld_read8_mem_offset(struct Console* console) {
    uint16_t address = 0xFF00 | memRead8(&console->memory, PC);
    console->cpu.regs.a = memRead8(&console->memory, address);
}

void ld_read_c_offset(struct Console* console) {
    uint16_t address = 0xFF00 | console->cpu.regs.c;
    console->cpu.regs.a = memRead8(&console->memory, address);
}

void ld_write8(struct Console* console) {
    memWrite8(&console->memory, REG16(INSTRUCTION.arg1), REG8(INSTRUCTION.arg2));
}

void ld_write8_incdest(struct Console* console) {
    memWrite8(&console->memory, REG16(INSTRUCTION.arg1)++, REG8(INSTRUCTION.arg2));
}

void ld_write8_decdest(struct Console* console) {
    memWrite8(&console->memory, REG16(INSTRUCTION.arg1)--, REG8(INSTRUCTION.arg2));
}

void ld_write8_mem_offset(struct Console* console) {
    uint16_t address = 0xFF00 | memRead8(&console->memory, PC);
    memWrite8(&console->memory, address, console->cpu.regs.a);
}

void ld_write8_c_offset(struct Console* console) {
    uint16_t address = 0xFF00 | console->cpu.regs.c;
    memWrite8(&console->memory, address, console->cpu.regs.a);
}

void ld_read16(struct Console* console) {
    REG16(INSTRUCTION.arg1) = memRead16(&console->memory, PC);
}

void ld_write16(struct Console* console) {
    memWrite16(&console->memory, REG16(INSTRUCTION.arg1), REG16(INSTRUCTION.arg2));
}

void ld_mem16(struct Console* console) {
    memWrite16(&console->memory, REG16(INSTRUCTION.arg1), memRead16(&console->memory, REG16(INSTRUCTION.arg2)));
}

void ld_reg8(struct Console* console) {
    REG8(INSTRUCTION.arg1) = REG8(INSTRUCTION.arg2);
}

void ld_reg16(struct Console* console) {
    REG16(INSTRUCTION.arg1) = REG16(INSTRUCTION.arg2);
}

void ld_reg16_src_offset(struct Console* console) {
    uint8_t src = REG16(INSTRUCTION.arg2);
    int8_t offset = (int8_t)memRead8(&console->memory, PC);

    if ((src & 0x0F) + (offset & 0x0F) > 0x0F) setFlag(&console->cpu, HALF_CARRY);
    else clearFlag(&console->cpu, HALF_CARRY);

    if ((src & 0xFF) + (offset & 0xFF) > 0xFF) setFlag(&console->cpu, CARRY);
    else clearFlag(&console->cpu, CARRY);

    REG16(INSTRUCTION.arg1) = src + offset;
}

void cpuExecuteOpcode(struct Console* console, uint8_t opcode) {
    console->cpu.current_opcode = opcode;
    instructions[opcode].execute(console);
}

static struct Instruction instructions[256] = {
    // { name, size, cycles, cycles_onbranch, execute, operand1, operand2 }
    { "NOP", 1, 4, 0, nop, 0, 0 },
    { "LD BC,u16", 3, 12, 0, ld_read16, O(regs.bc), 0 },
    { "LD (BC),A", 1, 8, 0, ld_write8, O(regs.bc), O(regs.a) },
    { "INC BC", 1, 8, 0, inc, 0, 0 },
    { "INC B", 1, 4, 0, inc, 0, 0 },
    { "DEC B", 1, 4, 0, dec, 0, 0 },
    { "LD B,u8", 2, 8, 0, ld_read8, O(regs.b), O(regs.pc) },
    { "RLCA", 1, 4, 0, rlca, 0, 0 },
    { "LD (u16),SP", 3, 20, 0, ld_write16, O(regs.pc), O(regs.sp) },
    { "ADD HL,BC", 1, 8, 0, add, 0, 0 },
    { "LD A,(BC)", 1, 8, 0, ld_read8, O(regs.a), O(regs.bc) },
    { "DEC BC", 1, 8, 0, dec, 0, 0 },
    { "INC C", 1, 4, 0, inc, 0, 0 },
    { "DEC C", 1, 4, 0, dec, 0, 0 },
    { "LD C,u8", 2, 8, 0, ld_read8, O(regs.c), O(regs.pc) },
    { "RRCA", 1, 4, 0, rrca, 0, 0 },
    { "STOP", 1, 4, 0, stop, 0, 0 },
    { "LD DE,u16", 3, 12, 0, ld_read16, O(regs.de), 0 },
    { "LD (DE),A", 1, 8, 0, ld_write8, O(regs.de), O(regs.a) },
    { "INC DE", 1, 8, 0, inc, 0, 0 },
    { "INC D", 1, 4, 0, inc, 0, 0 },
    { "DEC D", 1, 4, 0, dec, 0, 0 },
    { "LD D,u8", 2, 8, 0, ld_read8, O(regs.d), O(regs.pc) },
    { "RLA", 1, 4, 0, rla, 0, 0 },
    { "JR i8", 2, 12, 0, jr, 0, 0 },
    { "ADD HL,DE", 1, 8, 0, add, 0, 0 },
    { "LD A,(DE)", 1, 8, 0, ld_read8, O(regs.a), O(regs.de) },
    { "DEC DE", 1, 8, 0, dec, 0, 0 },
    { "INC E", 1, 4, 0, inc, 0, 0 },
    { "DEC E", 1, 4, 0, dec, 0, 0 },
    { "LD E,u8", 2, 8, 0, ld_read8, O(regs.e), O(regs.pc) },
    { "RRA", 1, 4, 0, rra, 0, 0 },
    { "JR NZ,i8", 2, 8, 12, jr, 0, 0 },
    { "LD HL,u16", 3, 12, 0, ld_read16, O(regs.hl), 0 },
    { "LD (HL+),A", 1, 8, 0, ld_write8_incdest, O(regs.hl), O(regs.a) },
    { "INC HL", 1, 8, 0, inc, 0, 0 },
    { "INC H", 1, 4, 0, inc, 0, 0 },
    { "DEC H", 1, 4, 0, dec, 0, 0 },
    { "LD H,u8", 2, 8, 0, ld_read8, O(regs.h), O(regs.pc) },
    { "DAA", 1, 4, 0, daa, 0, 0 },
    { "JR Z,i8", 2, 8, 12, jr, 0, 0 },
    { "ADD HL,HL", 1, 8, 0, add, 0, 0 },
    { "LD A,(HL+)", 1, 8, 0, ld_read8_incsrc, O(regs.a), O(regs.hl) },
    { "DEC HL", 1, 8, 0, dec, 0, 0 },
    { "INC L", 1, 4, 0, inc, 0, 0 },
    { "DEC L", 1, 4, 0, dec, 0, 0 },
    { "LD L,u8", 2, 8, 0, ld_read8, O(regs.l), O(regs.pc) },
    { "CPL", 1, 4, 0, cpl, 0, 0 },
    { "JR NC,i8", 2, 8, 12, jr, 0, 0 },
    { "LD SP,u16", 3, 12, 0, ld_read16, O(regs.sp), 0 },
    { "LD (HL-),A", 1, 8, 0, ld_write8_decdest, O(regs.hl), O(regs.a) },
    { "INC SP", 1, 8, 0, inc, 0, 0 },
    { "INC (HL)", 1, 12, 0, inc, 0, 0 },
    { "DEC (HL)", 1, 12, 0, dec, 0, 0 },
    { "LD (HL),u8", 2, 12, 0, ld_mem16, O(regs.hl), O(regs.pc) },
    { "SCF", 1, 4, 0, scf, 0, 0 },
    { "JR C,i8", 2, 8, 12, jr, 0, 0 },
    { "ADD HL,SP", 1, 8, 0, add, 0, 0 },
    { "LD A,(HL-)", 1, 8, 0, ld_read8_decsrc, O(regs.a), O(regs.hl) },
    { "DEC SP", 1, 8, 0, dec, 0, 0 },
    { "INC A", 1, 4, 0, inc, 0, 0 },
    { "DEC A", 1, 4, 0, dec, 0, 0 },
    { "LD A,u8", 2, 8, 0, ld_read8, O(regs.a), O(regs.pc) },
    { "CCF", 1, 4, 0, ccf, 0, 0 },
    { "LD B,B", 1, 4, 0, ld_reg8, O(regs.b), O(regs.b) },
    { "LD B,C", 1, 4, 0, ld_reg8, O(regs.b), O(regs.c) },
    { "LD B,D", 1, 4, 0, ld_reg8, O(regs.b), O(regs.d) },
    { "LD B,E", 1, 4, 0, ld_reg8, O(regs.b), O(regs.e) },
    { "LD B,H", 1, 4, 0, ld_reg8, O(regs.b), O(regs.h) },
    { "LD B,L", 1, 4, 0, ld_reg8, O(regs.b), O(regs.l) },
    { "LD B,(HL)", 1, 8, 0, ld_read8, O(regs.b), O(regs.hl) },
    { "LD B,A", 1, 4, 0, ld_reg8, O(regs.b), O(regs.a) },
    { "LD C,B", 1, 4, 0, ld_reg8, O(regs.c), O(regs.b) },
    { "LD C,C", 1, 4, 0, ld_reg8, O(regs.c), O(regs.c) },
    { "LD C,D", 1, 4, 0, ld_reg8, O(regs.c), O(regs.d) },
    { "LD C,E", 1, 4, 0, ld_reg8, O(regs.c), O(regs.e) },
    { "LD C,H", 1, 4, 0, ld_reg8, O(regs.c), O(regs.h) },
    { "LD C,L", 1, 4, 0, ld_reg8, O(regs.c), O(regs.l) },
    { "LD C,(HL)", 1, 8, 0, ld_read8, O(regs.c), O(regs.hl) },
    { "LD C,A", 1, 4, 0, ld_reg8, O(regs.c), O(regs.a) },
    { "LD D,B", 1, 4, 0, ld_reg8, O(regs.d), O(regs.b) },
    { "LD D,C", 1, 4, 0, ld_reg8, O(regs.d), O(regs.c) },
    { "LD D,D", 1, 4, 0, ld_reg8, O(regs.d), O(regs.d) },
    { "LD D,E", 1, 4, 0, ld_reg8, O(regs.d), O(regs.e) },
    { "LD D,H", 1, 4, 0, ld_reg8, O(regs.d), O(regs.h) },
    { "LD D,L", 1, 4, 0, ld_reg8, O(regs.d), O(regs.l) },
    { "LD D,(HL)", 1, 8, 0, ld_read8, O(regs.d), O(regs.hl) },
    { "LD D,A", 1, 4, 0, ld_reg8, O(regs.d), O(regs.a) },
    { "LD E,B", 1, 4, 0, ld_reg8, O(regs.e), O(regs.b) },
    { "LD E,C", 1, 4, 0, ld_reg8, O(regs.e), O(regs.c) },
    { "LD E,D", 1, 4, 0, ld_reg8, O(regs.e), O(regs.d) },
    { "LD E,E", 1, 4, 0, ld_reg8, O(regs.e), O(regs.e) },
    { "LD E,H", 1, 4, 0, ld_reg8, O(regs.e), O(regs.h) },
    { "LD E,L", 1, 4, 0, ld_reg8, O(regs.e), O(regs.l) },
    { "LD E,(HL)", 1, 8, 0, ld_read8, O(regs.e), O(regs.hl) },
    { "LD E,A", 1, 4, 0, ld_reg8, O(regs.e), O(regs.a) },
    { "LD H,B", 1, 4, 0, ld_reg8, O(regs.h), O(regs.b) },
    { "LD H,C", 1, 4, 0, ld_reg8, O(regs.h), O(regs.c) },
    { "LD H,D", 1, 4, 0, ld_reg8, O(regs.h), O(regs.d) },
    { "LD H,E", 1, 4, 0, ld_reg8, O(regs.h), O(regs.e) },
    { "LD H,H", 1, 4, 0, ld_reg8, O(regs.h), O(regs.h) },
    { "LD H,L", 1, 4, 0, ld_reg8, O(regs.h), O(regs.l) },
    { "LD H,(HL)", 1, 8, 0, ld_read8, O(regs.h), O(regs.hl) },
    { "LD H,A", 1, 4, 0, ld_reg8, O(regs.h), O(regs.a) },
    { "LD L,B", 1, 4, 0, ld_reg8, O(regs.l), O(regs.b) },
    { "LD L,C", 1, 4, 0, ld_reg8, O(regs.l), O(regs.c) },
    { "LD L,D", 1, 4, 0, ld_reg8, O(regs.l), O(regs.d) },
    { "LD L,E", 1, 4, 0, ld_reg8, O(regs.l), O(regs.e) },
    { "LD L,H", 1, 4, 0, ld_reg8, O(regs.l), O(regs.h) },
    { "LD L,L", 1, 4, 0, ld_reg8, O(regs.l), O(regs.l) },
    { "LD L,(HL)", 1, 8, 0, ld_read8, O(regs.l), O(regs.hl) },
    { "LD L,A", 1, 4, 0, ld_reg8, O(regs.l), O(regs.a) },
    { "LD (HL),B", 1, 8, 0, ld_write8, O(regs.hl), O(regs.b) },
    { "LD (HL),C", 1, 8, 0, ld_write8, O(regs.hl), O(regs.c) },
    { "LD (HL),D", 1, 8, 0, ld_write8, O(regs.hl), O(regs.d) },
    { "LD (HL),E", 1, 8, 0, ld_write8, O(regs.hl), O(regs.e) },
    { "LD (HL),H", 1, 8, 0, ld_write8, O(regs.hl), O(regs.h) },
    { "LD (HL),L", 1, 8, 0, ld_write8, O(regs.hl), O(regs.l) },
    { "HALT", 1, 4, 0, halt, 0, 0 },
    { "LD (HL),A", 1, 8, 0, ld_write8, O(regs.hl), O(regs.a) },
    { "LD A,B", 1, 4, 0, ld_reg8, O(regs.a), O(regs.b) },
    { "LD A,C", 1, 4, 0, ld_reg8, O(regs.a), O(regs.c) },
    { "LD A,D", 1, 4, 0, ld_reg8, O(regs.a), O(regs.d) },
    { "LD A,E", 1, 4, 0, ld_reg8, O(regs.a), O(regs.e) },
    { "LD A,H", 1, 4, 0, ld_reg8, O(regs.a), O(regs.h) },
    { "LD A,L", 1, 4, 0, ld_reg8, O(regs.a), O(regs.l) },
    { "LD A,(HL)", 1, 8, 0, ld_read8, O(regs.a), O(regs.hl) },
    { "LD A,A", 1, 4, 0, ld_reg8, O(regs.a), O(regs.a) },
    { "ADD A,B", 1, 4, 0, add, 0, 0 },
    { "ADD A,C", 1, 4, 0, add, 0, 0 },
    { "ADD A,D", 1, 4, 0, add, 0, 0 },
    { "ADD A,E", 1, 4, 0, add, 0, 0 },
    { "ADD A,H", 1, 4, 0, add, 0, 0 },
    { "ADD A,L", 1, 4, 0, add, 0, 0 },
    { "ADD A,(HL)", 1, 8, 0, add, 0, 0 },
    { "ADD A,A", 1, 4, 0, add, 0, 0 },
    { "ADC A,B", 1, 4, 0, adc, 0, 0 },
    { "ADC A,C", 1, 4, 0, adc, 0, 0 },
    { "ADC A,D", 1, 4, 0, adc, 0, 0 },
    { "ADC A,E", 1, 4, 0, adc, 0, 0 },
    { "ADC A,H", 1, 4, 0, adc, 0, 0 },
    { "ADC A,L", 1, 4, 0, adc, 0, 0 },
    { "ADC A,(HL)", 1, 8, 0, adc, 0, 0 },
    { "ADC A,A", 1, 4, 0, adc, 0, 0 },
    { "SUB A,B", 1, 4, 0, sub, 0, 0 },
    { "SUB A,C", 1, 4, 0, sub, 0, 0 },
    { "SUB A,D", 1, 4, 0, sub, 0, 0 },
    { "SUB A,E", 1, 4, 0, sub, 0, 0 },
    { "SUB A,H", 1, 4, 0, sub, 0, 0 },
    { "SUB A,L", 1, 4, 0, sub, 0, 0 },
    { "SUB A,(HL)", 1, 8, 0, sub, 0, 0 },
    { "SUB A,A", 1, 4, 0, sub, 0, 0 },
    { "SBC A,B", 1, 4, 0, sbc, 0, 0 },
    { "SBC A,C", 1, 4, 0, sbc, 0, 0 },
    { "SBC A,D", 1, 4, 0, sbc, 0, 0 },
    { "SBC A,E", 1, 4, 0, sbc, 0, 0 },
    { "SBC A,H", 1, 4, 0, sbc, 0, 0 },
    { "SBC A,L", 1, 4, 0, sbc, 0, 0 },
    { "SBC A,(HL)", 1, 8, 0, sbc, 0, 0 },
    { "SBC A,A", 1, 4, 0, sbc, 0, 0 },
    { "AND A,B", 1, 4, 0, and, 0, 0 },
    { "AND A,C", 1, 4, 0, and, 0, 0 },
    { "AND A,D", 1, 4, 0, and, 0, 0 },
    { "AND A,E", 1, 4, 0, and, 0, 0 },
    { "AND A,H", 1, 4, 0, and, 0, 0 },
    { "AND A,L", 1, 4, 0, and, 0, 0 },
    { "AND A,(HL)", 1, 8, 0, and, 0, 0 },
    { "AND A,A", 1, 4, 0, and, 0, 0 },
    { "XOR A,B", 1, 4, 0, xor, 0, 0 },
    { "XOR A,C", 1, 4, 0, xor, 0, 0 },
    { "XOR A,D", 1, 4, 0, xor, 0, 0 },
    { "XOR A,E", 1, 4, 0, xor, 0, 0 },
    { "XOR A,H", 1, 4, 0, xor, 0, 0 },
    { "XOR A,L", 1, 4, 0, xor, 0, 0 },
    { "XOR A,(HL)", 1, 8, 0, xor, 0, 0 },
    { "XOR A,A", 1, 4, 0, xor, 0, 0 },
    { "OR A,B", 1, 4, 0, or, 0, 0 },
    { "OR A,C", 1, 4, 0, or, 0, 0 },
    { "OR A,D", 1, 4, 0, or, 0, 0 },
    { "OR A,E", 1, 4, 0, or, 0, 0 },
    { "OR A,H", 1, 4, 0, or, 0, 0 },
    { "OR A,L", 1, 4, 0, or, 0, 0 },
    { "OR A,(HL)", 1, 8, 0, or, 0, 0 },
    { "OR A,A", 1, 4, 0, or, 0, 0 },
    { "CP A,B", 1, 4, 0, cp, 0, 0 },
    { "CP A,C", 1, 4, 0, cp, 0, 0 },
    { "CP A,D", 1, 4, 0, cp, 0, 0 },
    { "CP A,E", 1, 4, 0, cp, 0, 0 },
    { "CP A,H", 1, 4, 0, cp, 0, 0 },
    { "CP A,L", 1, 4, 0, cp, 0, 0 },
    { "CP A,(HL)", 1, 8, 0, cp, 0, 0 },
    { "CP A,A", 1, 4, 0, cp, 0, 0 },
    { "RET NZ", 1, 8, 20, ret, 0, 0 },
    { "POP BC", 1, 12, 0, pop, 0, 0 },
    { "JP NZ,u16", 3, 12, 16, jp, 0, 0 },
    { "JP u16", 3, 16, 0, jp, 0, 0 },
    { "CALL NZ,u16", 3, 12, 24, call, 0, 0 },
    { "PUSH BC", 1, 16, 0, push, 0, 0 },
    { "ADD A,u8", 2, 8, 0, add, 0, 0 },
    { "RST 00h", 1, 16, 0, rst, 0, 0 },
    { "RET Z", 1, 8, 20, ret, 0, 0 },
    { "RET", 1, 16, 0, ret, 0, 0 },
    { "JP Z,u16", 3, 12, 16, jp, 0, 0 },
    { "PREFIX CB", 1, 4, 0, prefix, 0, 0 },
    { "CALL Z,u16", 3, 12, 24, call, 0, 0 },
    { "CALL u16", 3, 24, 0, call, 0, 0 },
    { "ADC A,u8", 2, 8, 0, adc, 0, 0 },
    { "RST 08h", 1, 16, 0, rst, 0, 0 },
    { "RET NC", 1, 8, 20, ret, 0, 0 },
    { "POP DE", 1, 12, 0, pop, 0, 0 },
    { "JP NC,u16", 3, 12, 16, jp, 0, 0 },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "CALL NC,u16", 3, 12, 24, call, 0, 0 },
    { "PUSH DE", 1, 16, 0, push, 0, 0 },
    { "SUB A,u8", 2, 8, 0, sub, 0, 0 },
    { "RST 10h", 1, 16, 0, rst, 0, 0 },
    { "RET C", 1, 8, 20, ret, 0, 0 },
    { "RETI", 1, 16, 0, reti, 0, 0 },
    { "JP C,u16", 3, 12, 16, jp, 0, 0 },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "CALL C,u16", 3, 12, 24, call, 0, 0 },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "SBC A,u8", 2, 8, 0, sbc, 0, 0 },
    { "RST 18h", 1, 16, 0, rst, 0, 0 },
    { "LD (FF00+u8),A", 2, 12, 0, ld_write8_mem_offset, 0, 0 },
    { "POP HL", 1, 12, 0, pop, 0, 0 },
    { "LD (FF00+C),A", 1, 8, 0, ld_write8_c_offset, 0, 0 },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "PUSH HL", 1, 16, 0, push, 0, 0 },
    { "AND A,u8", 2, 8, 0, and, 0, 0 },
    { "RST 20h", 1, 16, 0, rst, 0, 0 },
    { "ADD SP,i8", 2, 16, 0, add, 0, 0 },
    { "JP HL", 1, 4, 0, jp, 0, 0 },
    { "LD (u16),A", 3, 16, 0, ld_write8, O(regs.pc), O(regs.a) },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "XOR A,u8", 2, 8, 0, xor, 0, 0 },
    { "RST 28h", 1, 16, 0, rst, 0, 0 },
    { "LD A,(FF00+u8)", 2, 12, 0, ld_read8_mem_offset, 0, 0 },
    { "POP AF", 1, 12, 0, pop, 0, 0 },
    { "LD A,(FF00+C)", 1, 8, 0, ld_read_c_offset, 0, 0 },
    { "DI", 1, 4, 0, di, 0, 0 },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "PUSH AF", 1, 16, 0, push, 0, 0 },
    { "OR A,u8", 2, 8, 0, or, 0, 0 },
    { "RST 30h", 1, 16, 0, rst, 0, 0 },
    { "LD HL,SP+i8", 2, 12, 0, ld_reg16_src_offset, O(regs.hl), O(regs.sp) },
    { "LD SP,HL", 1, 8, 0, ld_reg16, O(regs.sp), O(regs.hl) },
    { "LD A,(u16)", 3, 16, 0, ld_read8, O(regs.a), O(regs.pc) },
    { "EI", 1, 4, 0, ei, 0, 0 },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "UNUSED", 1, 0, 0, unused, 0, 0 },
    { "CP A,u8", 2, 8, 0, cp, 0, 0 },
    { "RST 38h", 1, 16, 0, rst, 0, 0 },
};
