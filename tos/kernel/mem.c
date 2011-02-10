
#include <kernel.h>


void poke_b (MEM_ADDR addr, BYTE value)
{
    *((BYTE*) addr) = value;
}

void poke_w (MEM_ADDR addr, WORD value)
{
    *((WORD*) addr) = value;
}

void poke_l (MEM_ADDR addr, LONG value)
{
    *((LONG*) addr) = value;
}

BYTE peek_b (MEM_ADDR addr)
{
    return *((BYTE*) addr);
}

WORD peek_w (MEM_ADDR addr)
{
    return *((WORD*) addr);
}

LONG peek_l (MEM_ADDR addr)
{
    return *((LONG*) addr);
}


