
#include <kernel.h>
#include <test.h>



void test_create_process_5_process_a(PROCESS self, PARAM param)
{
    kprintf("Process: %s\n\n", self->name);
    return_to_boot();
}

void remove_ready_queue_by_name(char* name)
{
   //find the process in the PCB array
   PROCESS this_process = find_process_by_name(name); 
   
   //remove the process by calling remove_ready_queue
   remove_ready_queue(this_process);
}

 /* Explicitly create three new processes of the same priority. 
 *  Then remove them from ready queue.
 *  This tests remove_ready_queue.
 */
void test_create_process_5() 
{
    test_reset();
    create_process(test_create_process_5_process_a, 5, 42, "Test process A");
    kprintf("Created Test Process A. \n");
    create_process (test_create_process_5_process_a, 5, 42, "Test process B");
    kprintf("Created Test Process B. \n");
    create_process (test_create_process_5_process_a, 5, 42, "Test process C");
    kprintf("Created Test Process C.\n\n");
    print_all_processes(kernel_window);

    check_num_of_pcb_entries(4);
    if (test_result != 0) 
       test_failed(test_result);

    check_num_proc_on_ready_queue(4);
    check_process(boot_name, STATE_READY, TRUE);
    check_process("Test process A", STATE_READY, TRUE);
    check_process("Test process B", STATE_READY, TRUE);
    check_process("Test process C", STATE_READY, TRUE);
    if (test_result != 0)
       test_failed(test_result);

    //remove one process
    remove_ready_queue_by_name("Test process A");
    kprintf("\nRemoved Test Process A.\n");
    check_num_of_pcb_entries(4);
    if (test_result != 0)
       test_failed(test_result);

    check_num_proc_on_ready_queue(3);
    check_process(boot_name, STATE_READY, TRUE);
    check_process("Test process A", STATE_READY, FALSE);
    check_process("Test process B", STATE_READY, TRUE);
    check_process("Test process C", STATE_READY, TRUE);
    if (test_result != 0) 
       test_failed(test_result);

    //remove another process
    remove_ready_queue_by_name("Test process B");
    kprintf("Removed Test Process B.\n");
    check_num_proc_on_ready_queue(2);
    check_process(boot_name, STATE_READY, TRUE);
    check_process("Test process A", STATE_READY, FALSE);
    check_process("Test process B", STATE_READY, FALSE);
    check_process("Test process C", STATE_READY, TRUE);
    if (test_result != 0) 
       test_failed(test_result);

    //remove the third created process
    remove_ready_queue_by_name("Test process C");
    kprintf("Removed Test Process C.\n");
    check_num_proc_on_ready_queue(1);
    check_process(boot_name, STATE_READY, TRUE);
    check_process("Test process A", STATE_READY, FALSE);
    check_process("Test process B", STATE_READY, FALSE);
    check_process("Test process C", STATE_READY, FALSE);
    if (test_result != 0)
       test_failed(test_result);
}

