
#include <kernel.h>
#include <test.h>


void test_dispatcher_1_process_a(PROCESS self, PARAM param)
{
    kprintf("Process: %s\n\n", self->name);
    kprintf("Param: %d\n", param);
    print_all_processes(kernel_window);
    return_to_boot();
}

/*
 * Create a new process and at it to the ready queue. There should
 * be two ready processes, but since we don't do a resign(), no
 * context switch happens.
 */
void test_dispatcher_1()
{
    test_reset();
    create_process(test_dispatcher_1_process_a, 5, 42, "Test process A");
    print_all_processes(kernel_window);

    check_process(boot_name, STATE_READY, TRUE);
    check_process("Test process A", STATE_READY, TRUE);
    check_num_proc_on_ready_queue(2);
    if (test_result != 0)
       test_failed (test_result);
}

