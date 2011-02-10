
#include <kernel.h>
#include <test.h>



void test_ipc_3_sender_process(PROCESS self, PARAM param)
{
    PORT receiver_port = (PORT) param;
    int data1= 42;
    int data2= 24;

    check_sum += 1;
   
    /* 
     * send the first message
     */
 
    check_process("Receiver", STATE_RECEIVE_BLOCKED, FALSE);
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(48); 
    }
 
    kprintf("%s: sending a message using send()...\n",
	    self->name);
    send(receiver_port, &data1);

    if (check_sum != 3)
        test_failed(49);
    check_sum += 4;

    kprintf("%s: received = %d\n", self->name, data1);

    if (data1 != 11)
	test_failed(41);

    /* 
     * send the second message
     */
    kprintf("%s: sending a message using message()...\n",
	    self->name);
    message(receiver_port, &data2);
    if (check_sum != 15)
        test_failed(54); 

    kprintf("%s: woken up from message()\n", self->name);
    return_to_boot();
}

void test_ipc_3_receiver_process(PROCESS self, PARAM param)
{
    PROCESS sender;
    int* data;

    check_sum = 0; 
   
    /*
     * receive the first message
     */
    check_process("Sender", STATE_READY, TRUE);
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: receiving the first message...\n", self->name);
    data = (int*) receive (&sender);

    if (check_sum != 1)
	test_failed(47);

    kprintf("%s: received a message from %s, parameter = %d\n",
            self->name, sender->name, *data);
    check_sum += 2;

    check_process("Receiver", STATE_READY, TRUE);
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(53);
    }

    // Sender should now be REPLY_BLOCKED and off ready queue
    check_process("Sender", STATE_REPLY_BLOCKED, FALSE);
    if (test_result == 13) {
	print_all_processes(kernel_window);
	test_failed(50);
    }
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(51);
    }

    if (*data != 42)
	test_failed(41);

    *data = 11;
    reply(sender);

    /*
     * receive the sencond message
     */
 
    kprintf("%s: receiving the second message...\n", self->name);
    data = (int*) receive(&sender);
    if (check_sum != 7)
	test_failed(47);

    kprintf("%s: received a message from %s, parameter = %d\n",
            self->name, sender->name, *data);

    check_sum += 8;

    // Sender should now be STATE_READY and on ready queue
    check_process("Sender", STATE_READY, TRUE);
    if (test_result == 13) {
	print_all_processes(kernel_window);
	test_failed(55);
    }
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(57);
    }

    if (*data == 11)
	test_failed(44);
    if (*data != 24)
	test_failed(41);
    
    /* We do the third receive. Since there is no message pending,
     * this process will become RECEIVE_BLOCKED so that the sender
     * will continue to run. */
    data = (int*) receive(&sender);
    test_failed(47);
}


/*
 * This test creates a sender and a receiver process. The receiver process
 * has the higher priority and is scheduled first. 
 * The execution sequence is as follow:
 * 1. The receiver executes a receive() and becomes RECEIVE_BLOCKED. 
 * 2. The sender gets executed and does a send(). The message is immediately
 *    delivered, unblocking the receiver and making the sender REPLY_BLOCKED.
 * 3. The receiver is executed. It does a receive and becomes RECEIVE_BLOCKED
 *    again.  
 * 4. The sender gets executed and does a message(). The message is immediately
 *    delivered, unblocking the receiver. The sender is still STATE_READY.
 * 5. The receiver gets the execution again. 
 * This test send() and message() in the case that the receiver is  
 * ready to receive. It also test receive() in the case that there is no 
 * messages pending. 
 */
void test_ipc_3()
{
    PORT new_port;

    test_reset();
    new_port = create_process (test_ipc_3_receiver_process, 6, 0, "Receiver");
    create_process(test_ipc_3_sender_process, 5, (PARAM) new_port, "Sender");
    resign();

    kprintf("Back to boot.\n");  
    if (check_sum == 1 || check_sum == 7)
	test_failed(52);
}
