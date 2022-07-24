#include <stdio.h>
#include "types.h"
#include "cpu.h"

struct Console console;

int main() {
    // console.cpu.regs.hl is 0
    // because it is uninitialized.
    console.memory.raw[0] = 0x38;
    console.memory.raw[1] = 0x13;
    printf("Memory before: %04x\n", ((uint16_t*)&console.memory.raw)[0]);
    cpuExecuteOpcode(&console, 0x35); // DEC (HL)
    printf("Memory after:  %04x\n", ((uint16_t*)&console.memory.raw)[0]);
}
