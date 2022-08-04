#include <stdio.h>
#include "types.h"
#include "cpu.h"

struct Console console;

int main() {
    // Hardcode memory with opcodes for testing
    console.memory.internal_ram[0] = 0x01; // #1: LD BC, U16
    console.memory.internal_ram[1] = 0x36; //     U16.lo
    console.memory.internal_ram[2] = 0x13; //     U16.hi
    console.memory.internal_ram[3] = 0x03; // #2: INC BC

    printf("B: %02x; C: %02x\n", console.cpu.regs.b, console.cpu.regs.c);
    cpuExecuteOpcode(&console);
    printf("B: %02x; C: %02x\n", console.cpu.regs.b, console.cpu.regs.c);
    cpuExecuteOpcode(&console);
    printf("B: %02x; C: %02x\n", console.cpu.regs.b, console.cpu.regs.c);
}
