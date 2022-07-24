#include <stdio.h>
#include "types.h"
#include "cpu.h"

struct Console console;

int main() {
    cpuExecuteOpcode(&console, 1);
}
