
#include <kernel.h>
#include <test.h>



void check_test_create_process_4();

void test_create_process_4_process_a(PROCESS self, PARAM param)
{
    kprintf("Process: %s\n\n", self->name);
    return_to_boot();
}


void test_create_process_4_process_b(PROCESS self, PARAM param)
{
    kprintf("Process: %s\n\n", self->name);
    return_to_boot();
}

void test_create_process_4_process_c(PROCESS self, PARAM param)
{
    kprintf("Process: %s\n\n", self->name);
    return_to_boot();
}

 /* Explicitly create three new processes of the same priority. 
 *  This tests add_ready_queue.
 */
void test_create_process_4() 
{
    test_reset();
    create_process(test_create_process_4_process_a, 1, 42, "Test process A");
    kprintf("Created Test Process A. \n");
    create_process(test_create_process_4_process_b, 1, 42, "Test process B");
    kprintf("Created Test Process B. \n");
    create_process(test_create_process_4_process_b, 1, 42, "Test process C");
    kprintf("Created Test Process C. \n\n");
    print_all_processes(kernel_window);

    check_test_create_process_4();
}

void check_test_create_process_4() {
    //check the four processes's pcb entries
    check_create_process(boot_name, 1, NULL, 0);
    check_create_process("Test process A", 1,
			 test_create_process_4_process_a, 42);
    check_create_process("Test process B", 1,
			 test_create_process_4_process_b, 42);
    check_create_process("Test process C", 1,
			 test_create_process_4_process_b, 42);
    check_num_of_pcb_entries(4);
    if (test_result != 0) 
       test_failed(test_result);
   
    //check number of processes on ready queue
    check_num_proc_on_ready_queue(4);
    if (test_result != 0) 
       test_failed(test_result);
 
    //check the ready queue. make sure the processes are in the right order. 
    //(Processes created later should be after processes that are created 
    //earlier.)
    PROCESS first_process = ready_queue[1];
    if (string_compare(first_process->name, "Boot process") == 0 ||
        string_compare(first_process->next->name, "Test process A") == 0 ||
        string_compare(first_process->next->next->name, "Test process B") == 0 ||
        string_compare(first_process->next->next->next->name, "Test process C") == 0) 
       test_failed(17);
}

