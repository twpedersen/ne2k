
#include <kernel.h>
#include <test.h>

/*
 * Tests if init_process() cleans up the pcb array and if init_dispatcher(). 
 * cleans up the ready queue. 
 */
void test_create_process_2() 
{
    test_reset();
    init_process();
    init_dispatcher();
    kprintf("Called init_process() and init_dispatcher().\n\n");

    print_all_processes(kernel_window);

    //check if the boot process is still initialized correctly and 
    //if there is only one used pcb entry and one process on ready queue.
    check_create_process(boot_name, 1, NULL, 0);
    if (test_result != 0) 
       test_failed(test_result);

    check_num_of_pcb_entries(1);
    if (test_result != 0)
       test_failed(test_result);

    check_process(boot_name, STATE_READY, TRUE);
    if (test_result != 0)
       test_failed(test_result);

    check_num_proc_on_ready_queue(1);
    if (test_result != 0)
       test_failed(test_result);
}

