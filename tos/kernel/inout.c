
#include <kernel.h>

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

void inportw(unsigned short port, void *dst, unsigned short n)
{
	asm ("movl %w0, %%edx\n\t"
		 "movl %1, %%edi\n\t"
		 "movl %w2, %%ecx\n\t"
		 "rep insw"
		 :
		 : "g" (port), "g" (dst), "g" (n)
		 : "memory", "%edx", "%edi", "%ecx");
}
