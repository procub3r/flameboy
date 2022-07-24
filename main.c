#include <stdio.h>
#include "types.h"
#include "cpu.h"

struct Console console;

int main() {
    // console.cpu.regs.pc is 0.
    // It doesn't change atm.
    console.memory.raw[0] = 0x37;
    console.memory.raw[1] = 0x13;
    printf("HL Before: %04x\n", console.cpu.regs.hl);
    cpuExecuteOpcode(&console, 0x21); // LD HL,u16
    printf("HL After:  %04x\n", console.cpu.regs.hl);
}
