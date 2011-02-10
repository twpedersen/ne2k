
#include <kernel.h>
#include <test.h>



/*
 * This test tests the wait_for_interrupt() function. The boot process
 * creates another process. This process enters a loop. For
 * each interation it calls wait_for_interrupt (TIMER_IRQ) to cause
 * a short delay before the next interation.
 */
void isr_process(PROCESS self, PARAM param)
{
    int ticks;
    int i;
    unsigned char* screen_base;
    
    kprintf("Process: %s\n", self->name);
    kprintf("ABCDEF");

    /* screen_base points to the beginning of the string "ABCDEF" */
    screen_base = (unsigned char*) 0xb8000 + 4 * 80 * 2;
    i = 0;
    int j;
    for (j = 0; j < 30; j ++) {
	ticks = 3;
	while (ticks--)
	    wait_for_interrupt(TIMER_IRQ);
	*(screen_base + i * 2) = *(screen_base + i * 2) + 1;
	i++;
	if (i == 6)
	    i = 0;
        check_sum ++;
    }
    return_to_boot();
}

void test_isr_3 ()
{
    test_reset();
    check_sum = 0;
    int check_2 = 0;

    kprintf("=== test_isr_3 === \n");
    kprintf("This test will take a while.\n\n");
    init_interrupts();
    create_process(isr_process, 5, 0, "ISR process");
    resign();

    int i;
    int j = 0;
    unsigned char* screen_base;
    screen_base = (unsigned char*) 0xb8000 + 7 * 80 * 2;

    kprintf("\n\nBoot process:\n"); 
    kprintf("ABCDEF");

    PROCESS isr_pro = find_process_by_name("ISR process");
    for (i = 0; i < 600000; i++) {
	if (isr_pro->state == STATE_INTR_BLOCKED)
	    check_2++;

	*(screen_base + j * 2) = *(screen_base + j * 2) + 1;
	j++;
	if (j == 6)
	    j = 0;
    }

    if (check_2 == 0)
       test_failed(72);
    if (check_sum <= 1)
       test_failed(73);
}
