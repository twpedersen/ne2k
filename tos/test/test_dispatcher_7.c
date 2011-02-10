
#include <kernel.h>
#include <test.h>



void test_dispatcher_7_process_d(PROCESS self, PARAM param)
{
    kprintf("%s\n", self->name);
    if (check_sum != 3)
       test_failed(24);

    check_sum += 4;
    resign();
    kprintf("Back to process %s \n", self->name);
    if (check_sum != 15)
       test_failed(24);

    check_sum += 16;
    resign();
    kprintf("Back to process %s again\n\n", self->name);
    if (check_sum != 31)
       test_failed(24);
    
    check_sum += 32;
    return_to_boot();
}

void test_dispatcher_7_process_e(PROCESS self, PARAM param)
{
    kprintf("%s\n", self->name);
    if (check_sum != 1)
       test_failed(23);

    check_sum += 2;
    remove_ready_queue(self);
    check_num_proc_on_ready_queue(3);
    if (test_result != 0)
       test_failed(test_result);

    resign();
    test_failed(26);
}

void test_dispatcher_7_process_f(PROCESS self, PARAM param)
{
    kprintf("%s\n", self->name);
    if (check_sum != 0)
       test_failed(23);
    
    check_sum += 1;
    resign();
    kprintf("Back to %s \n", self->name);
    if (check_sum != 7)
       test_failed(24);
    
    check_sum += 8;
    remove_ready_queue(self);
    check_num_proc_on_ready_queue(2);
    if (test_result != 0)
       test_failed(test_result);
 
    resign();
    test_failed(26);
}

/*
 * This test creates three processes with the same priority.
 * The execution sequence is as following:
 * 1. After the boot process resign(),  process F (test_process_f)is executed.
 * 2. Process F resign, process E (test_process_e) is then executed. 
 * 3. Process E remove itself from the ready queue and then resign, process D
 *    (test_process_d) is then executed.
 * 4. Process D resign(), process F is then executed.
 * 5. Process F remove itself from ready queue and then resign. Process D is
 *    the next to be executed since Process E is off ready queue.    
 * 6. Process D resign, the next to be executed is still process D since both
 *    process F and E are off ready queue. 
 * 
 * The execution sequence should be: boot -> F -> E -> D -> F -> D -> D
*/
void test_dispatcher_7()
{
    test_reset();
    create_process(test_dispatcher_7_process_f, 5, 0, "Test process F");
    kprintf("Created process F\n");
    create_process(test_dispatcher_7_process_e, 5, 0, "Test process E");
    kprintf("Created process E\n");
    create_process(test_dispatcher_7_process_d, 5, 0, "Test process D");
    kprintf("Created process D\n");
    kprintf("\n");

    check_num_proc_on_ready_queue(4);
    if (test_result != 0)
       test_failed(test_result);

    check_sum = 0;
    resign();
    if(check_sum == 0)
       test_failed(21);

    if (check_sum != 63)
       test_failed(25);
}

