
#include <kernel.h>
#include <test.h>



void test_dispatcher_5_process_d(PROCESS self, PARAM param)
{
    kprintf("Process: %s\n\n", self->name);
    print_all_processes(kernel_window);
    kprintf("\n");

    if (check_sum != 2)
       test_failed(23);

    check_sum += 1;
    resign();
    test_failed(24);
}

void test_dispatcher_5_process_e(PROCESS self, PARAM param)
{
    kprintf("\nProcess: %s\n\n", self->name);
    if (check_sum != 0) 
       test_failed(23);

    check_sum += 2;

    resign();
    kprintf("Back to %s", self->name);

    if (check_sum != 3)
       test_failed(24);

    check_sum += 4;
    return_to_boot();
}

/*
 * This test creates two processes with the same priority. Doing
 * a resign() in the main process should continue execution in
 * test_process_e(). When this process does a resign(), execution
 * should resume in test_process_d(). Then the execution should
 * be passed back to test_process_e(). This basically tests Round-
 * Robin of ready processes.
 */
void test_dispatcher_5()
{
    test_reset();
    create_process(test_dispatcher_5_process_e, 5, 0, "Test process E");
    kprintf("Created process E\n");
    create_process(test_dispatcher_5_process_d, 5, 0, "Test process D");
    kprintf("Created process D\n");

    check_sum = 0;
    resign();

    if(check_sum == 0)
       test_failed(21);
    if(check_sum != 7)
       test_failed(25); 
}

