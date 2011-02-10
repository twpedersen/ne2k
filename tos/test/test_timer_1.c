
#include <kernel.h>
#include <test.h>

/*
 * The following test program tests the timer process. 
 * 1. Test_timer_1_process_B has a for loop in which it does a slow animation
 *    of some characters on the screen by using timer service.
 * 2. Test_timer_1_process_A has an infinite in which it does not use timer
 *    service and does a fast animation of characters.
 * 3. The test checks if process_A's animation is faster than process_B's 
 *    annimation.
 */



void test_timer_1_process_A(PROCESS self, PARAM param)
{
    unsigned char* screen_base;
    int            i;

    kprintf("%s: \n", self->name);
    kprintf("ABCDEF\n\n");
    print_all_processes(kernel_window);
    /* screen_base points to the beginning of the string "ABCDEF" */
    screen_base = (unsigned char*) 0xb8000 + 7 * 80 * 2;
    i = 0;
    while (42) {
	*(screen_base + i * 2) = *(screen_base + i * 2) + 1;
	i++;
	if (i == 6)
	    i = 0;

        check_sum ++;
        if (check_sum > 100)
           test_result = 0;
    }

    return_to_boot();
}

void test_timer_1_process_B(PROCESS self, PARAM param)
{
    Timer_Message  msg;
    unsigned char* screen_base;
    int            i;

    msg.num_of_ticks = 20;
    kprintf("%s: \n", self->name);
    kprintf("ABCDEF\n\n");
    /* screen_base points to the beginning of the string "ABCDEF" */
    screen_base = (unsigned char*) 0xb8000 + 4 * 80 * 2;
    i = 0;
    int j;
    for (j = 0; j < 10; j++) {
        send(timer_port, &msg);
	*(screen_base + i * 2) = *(screen_base + i * 2) + 1;
	i++;
	if (i == 6)
	    i = 0;
    }

    return_to_boot();
}
 
void test_timer_1()
{
    check_sum = 0;

    test_reset();

    init_interrupts();
    init_null_process();
    init_timer();

    kprintf("=== test_timer_1 ===\n");
    kprintf("This test will take a while.\n\n");

    create_process(test_timer_1_process_A, 5, 0, "Process A");
    create_process(test_timer_1_process_B, 6, 0, "Process B");

    test_result = 1;
    resign();

    if (test_result == 1)
	test_failed(80);
}
