
#include <kernel.h>
#include <test.h>



void test_dispatcher_3_process_a (PROCESS self, PARAM param)
{
    kprintf("Process: %s\n\n", self->name);
    kprintf("Param: %d\n", param);
    print_all_processes(kernel_window);
    test_failed(22);
}

void test_dispatcher_3_process_b (PROCESS self, PARAM param)
{
    kprintf("\nProcess: %s\n\n", self->name);
    print_all_processes(kernel_window);
    check_sum += 2;
    return_to_boot();
}


/*
 * This tests creates two new processes; both with different priorities.
 * When doing a resign(), the one with the higher priority
 * (test_process_b()) should get executed.
 */
void test_dispatcher_3()
{
    test_reset();
    create_process(test_dispatcher_3_process_a, 5, 42, "Test process A");
    kprintf("Created process A\n");
    create_process(test_dispatcher_3_process_b, 7, 0, "Test process B");
    kprintf("Created process B\n");

    check_sum = 0;
    resign();

    if (check_sum == 0) 
       test_failed(21);
}

