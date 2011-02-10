
#include <kernel.h>
#include <test.h>



void test_create_process_3_process_a (PROCESS self, PARAM param)
{
    /*
     * Since we don't do a context switch yet, this code will actually
     * not be executed.
     */
    kprintf("Process: %s\n\n", self->name);
    return_to_boot();
}

/*
 * Explicitly create a new process. We don't do a context switch just yet.
 * The new process should be in STATE_READY.
 */
void test_create_process_3() 
{
    test_reset();

    create_process(test_create_process_3_process_a, 5, 42, "Test process A");
    kprintf("Created Test Process A. \n\n");
    print_all_processes(kernel_window);

    check_create_process(boot_name, 1, NULL, 0);
    if (test_result != 0)
       test_failed(test_result);

    check_create_process("Test process A", 5,
			 test_create_process_3_process_a, 42);
    if (test_result != 0)
       test_failed(test_result);

    check_num_of_pcb_entries(2);
    if (test_result != 0)
       test_failed(test_result);

    check_process(boot_name, STATE_READY, TRUE);
    if (test_result != 0)
       test_failed(test_result);

    check_process("Test process A", STATE_READY, TRUE);
    if (test_result != 0)
       test_failed(test_result);

    check_num_proc_on_ready_queue(2);
    if (test_result != 0)
       test_failed(test_result);
}

