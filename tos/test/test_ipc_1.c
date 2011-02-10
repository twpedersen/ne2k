
#include <kernel.h>
#include <test.h>



void test_ipc_1_process(PROCESS self, PARAM param)
{
    //since we do not do a ipc or resgin, this process will not get executed
    PROCESS sender;
    int* data;
    print_all_processes(kernel_window);

    data = (int*) receive (&sender);
    print_all_processes(kernel_window);
}


/*
 * This test checks if a port is created correctly for a new process.
 */
void test_ipc_1()
{
    test_reset();

    PORT new_port;
    new_port = create_process (test_ipc_1_process, 5, 0, "Receiver");
    kprintf("A new process with a port is created.");

    check_port(new_port, "Receiver", TRUE);
    if (test_result != 0)
       test_failed(test_result);
}

