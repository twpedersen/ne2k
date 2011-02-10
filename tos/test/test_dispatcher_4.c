
#include <kernel.h>
#include <test.h>



void test_dispatcher_4_process_a(PROCESS self, PARAM param)
{
    kprintf("Process: %s\n\n", self->name);
    kprintf("Param: %d\n", param);
    print_all_processes(kernel_window);

    if (check_sum != 2)
       test_failed(22);

    check_sum += 1; 
    return_to_boot();
}


void test_dispatcher_4_process_c(PROCESS self, PARAM param)
{
    kprintf("\nProcess: %s\n\n", self->name);
    print_all_processes(kernel_window);
    kprintf("\n");

    if (check_sum != 0)
       test_failed(22);

    check_sum += 2;
    remove_ready_queue(active_proc);
    check_num_proc_on_ready_queue(2);
    if (test_result != 0)
       test_failed(test_result);

    resign();

    test_failed(26); 
}

/*
 * Creates two new processes with different priorities. When the main
 * thread calls resign(), execution should continue with test_process_c()
 * This process then removes itself from the ready queue and calls resign()
 * again. Execution should then continue in test_process_a()
 */
void test_dispatcher_4()
{
    test_reset();
    create_process(test_dispatcher_4_process_a, 5, 42, "Test process A");
    kprintf("Created process A\n");
    create_process(test_dispatcher_4_process_c, 7, 0, "Test process C");
    kprintf("Created process C\n");

    check_sum = 0;
    resign();
    if (check_sum == 0)
       test_failed(21); 

    if (check_sum != 3)
       test_failed(22); 
}
