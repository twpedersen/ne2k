
#include <kernel.h>
#include <test.h>



void test_dispatcher_6_process_a(PROCESS self, PARAM param)
{
    kprintf("Process: %s\n\n", self->name);
    kprintf("Param: %d\n", param);
    print_all_processes(kernel_window);

    check_sum += 1; 
    resign();

    check_sum += 2;
    kprintf("\nBack to process: %s.\n\n", self->name);
    return_to_boot();
}

/*
 * Create a new process with a higher priority.
 * The execution sequence is as following:
 *   1. Boot process does a resign().
 *   2. The test process gets executed.
 *   3. The test process does a resign().
 *   4. Since there is no process of higher or the same priority, the test
 *      process gets exectued again. 
 * This tests the double-linked list structure of the ready_queue. (When there  
 * is only one process on the ready queue and there is no other process whose
 * priority is higher, ,its "pre" and "next" should point
 * to itself.)
 */
void test_dispatcher_6()
{
    test_reset();
    create_process(test_dispatcher_6_process_a, 5, 42, "Test process A");

    check_sum = 0;
    resign();

    if (check_sum == 0)
       test_failed(21);

    if (check_sum == 1)
       test_failed(24);
}

