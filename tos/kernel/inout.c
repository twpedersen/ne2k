#include <kernel.h>

#define inouts(cmd, port, mem, n)	\
	__asm__ __volatile__ (	\
			"cld\n\t"	\
			"rep\n\t"	\
			cmd	\
			:	\
			: "c" (n), "d" (port), "D" (mem)	\
			: "cc");	\


unsigned char inportb (unsigned short port)
{
    unsigned char _v;

    asm ("inb %w1,%0" : "=a" (_v) : "Nd" (port));
    return _v;
}


void outportb (unsigned short port, unsigned char value)
{
    asm ("outb %b0,%w1" : : "a" (value), "Nd" (port));
}

void insb(unsigned short port, void *dst, unsigned short n)
{
	inouts("insb", port, dst, n);
}

void insw(unsigned short port, void *dst, unsigned short n)
{
	inouts("insw", port, dst, n);
}

void outsb(void *src, unsigned short port, unsigned short n)
{
	inouts("outsb", port, src, n);
}

void outsw(void *src, unsigned short port, unsigned short n)
{
	inouts("outsw", port, src, n);
}
