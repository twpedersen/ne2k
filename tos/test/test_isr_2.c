
#include <kernel.h>
#include <test.h>



/*
 * This test finally makes use of preemptive scheduling. Two processes
 * run concurrently. Every process will be given a certain time slot.
 * When the time slot is used up, the other process gets a chance to
 * run. Note that the two processes do not call resign().
 */

unsigned int test_isr_2_check_sum;

void test_isr_2_process_1(PROCESS self, PARAM param)
{
    /* screen_offset points to the "A" of process 1 */
    MEM_ADDR screen_offset = 0xb8000 + 4 * 160 + 2 * 11;

    while (42) {
        if (check_sum == 80000)
	    return_to_boot();   
        check_sum ++;
	poke_b(screen_offset, peek_b(screen_offset) + 1);
    }
}

void test_isr_2_process_2(PROCESS self, PARAM param)
{
    /* screen_offset points to the "Z" of process 2 */
    MEM_ADDR screen_offset = 0xb8000 + 5 * 160 + 2 * 11;
    
    while (42) {
        if (test_isr_2_check_sum == 80000)
	    return_to_boot();
        test_isr_2_check_sum ++;
	poke_b(screen_offset, peek_b(screen_offset) + 1);
    }
}

void test_isr_2()
{
    test_reset();
    check_sum = 0;
    test_isr_2_check_sum = 0;
    init_interrupts();
    kprintf("=== test_isr_2 === \n");
    kprintf("This test will take a while.\n\n\n");
    kprintf("Process 1: A\n");
    kprintf("Process 2: Z\n");
    create_process(test_isr_2_process_1, 5, 0, "Process 1");
    create_process(test_isr_2_process_2, 5, 0, "Process 2");
    resign();
    if (check_sum == 0 || test_isr_2_check_sum == 0)
	test_failed(71);
}

