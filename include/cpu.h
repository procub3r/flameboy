#ifndef CPU_H
#define CPU_H

#include "types.h"

void setFlag(struct CPU* cpu, enum Flag flag);
void clearFlag(struct CPU* cpu, enum Flag flag);

void cpuExecuteOpcode(struct Console* console);

#endif
