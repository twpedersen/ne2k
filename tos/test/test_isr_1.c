
#include <kernel.h>
#include <test.h>



/*
 * This is the first test to check Interrupts. What essentially
 * happens is that an ISR (Interrupt Service Routine) is installed
 * that pokes into the video memory at a certain location. The ISR
 * is hooked with the timer interrupt so that this video update
 * happens in certain intervals. Meanwhile, the main program (or
 * rather the boot process) continues to run doing the same thing
 * (poking to the video memory; but at another location).
 */
MEM_ADDR screen_offset_for_timer_isr = 0xb8000 + 3 * 160 + 2 * 11;
int counter = 10;

void timer_isr ()
{
    /*
     *		push	%eax		; Push context
     *          push    %ecx
     *          push    %edx
     *          push    %ebx
     *          push    %ebp
     *          push    %esi
     *          push    %edi
     */
    asm ("push %eax");
    asm ("push %ecx");
    asm ("push %edx");
    asm ("push %ebx");
    asm ("push %ebp");
    asm ("push %esi");
    asm ("push %edi");
    if (!counter--) {
	counter = 20;
	poke_b(screen_offset_for_timer_isr,
	       peek_b(screen_offset_for_timer_isr) + 1);
        check_sum ++;
    }
    
    /*
     *		movb	$0x20,%al	; Reset interrupt controller
     *		outb	%al,$0x20
     *		pop	%edi	        ; Restore previously saved context
     *          pop     %esi
     *          pop     %ebp
     *          pop     %ebx
     *          pop     %edx
     *          pop     %ecx
     *          pop     %eax
     *		iret			; Return to interrupted program
     */
    asm ("movb  $0x20,%al");
    asm ("outb  %al,$0x20");
    asm ("pop	%edi");
    asm ("pop   %esi");
    asm ("pop   %ebp");
    asm ("pop   %ebx");
    asm ("pop   %edx");
    asm ("pop   %ecx");
    asm ("pop   %eax");
    asm ("iret");
}


void test_isr_1()
{
    MEM_ADDR screen_offset_for_boot_proc = 0xb8000 + 4 * 160 + 2 * 11;
    volatile int flag;
    
    test_reset();
    check_sum = 0;
    init_interrupts();
    DISABLE_INTR(flag);
    init_idt_entry(TIMER_IRQ, timer_isr);
    kprintf("=== test_isr_1 === \n");
    kprintf("This test will take a while.\n\n");
    kprintf("Timer ISR: A\n");
    kprintf("Boot proc: Z\n");
    ENABLE_INTR(flag);

    int i;
    for (i = 1; i < 700000; i++)
	poke_b(screen_offset_for_boot_proc,
	       peek_b(screen_offset_for_boot_proc) + 1);

    if (check_sum == 0)
	test_failed(70);
}

