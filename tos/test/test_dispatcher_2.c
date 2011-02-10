
#include <kernel.h>
#include <test.h>



void test_dispatcher_2_process_a (PROCESS self, PARAM param)
{
    kprintf("Process: %s\n\n", self->name);
    kprintf("Param: %d\n\n", param);

    print_all_processes(kernel_window);

    check_process(boot_name, STATE_READY, TRUE);
    check_process("Test process A", STATE_READY, TRUE);
    check_num_proc_on_ready_queue(2);
 
    if (test_result != 0) 
       test_failed(test_result);

    if (param != 42)
       test_failed(20);

    check_sum += 1;
    return_to_boot();
}

/*
 * Create a new process with a higher priority. When doing a resign()
 * this new process should get scheduled so that execution continues
 * in test_process_a()
 */
void test_dispatcher_2()
{
    test_reset();
    create_process(test_dispatcher_2_process_a, 5, 42, "Test process A");

    check_process(boot_name, STATE_READY, TRUE);
    check_process("Test process A", STATE_READY, TRUE);
    check_num_proc_on_ready_queue(2);
    if (test_result != 0) 
       test_failed(test_result);

    check_sum = 0; 
    resign();
    if (check_sum == 0)
       test_failed(21);
}

