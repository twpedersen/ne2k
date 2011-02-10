#include <kernel.h>
#include <test.h>

/*
 * We don't explicitly create a new process, but because of init_process()
 * and init_dispatcher(), the main thread should be initialized as a process
 * and be added to the ready queue.  
 * This also test print_all_processes() 
 */
void test_create_process_1() 
{
    test_reset();
    print_all_processes(kernel_window);

    //check if the boot process is initialized correctly.
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

