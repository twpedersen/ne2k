
#include <kernel.h>
#include <test.h>



void test_ipc_2_sender_process(PROCESS self, PARAM param)
{
    PORT receiver_port = (PORT) param;
    int data1 = 42;
    int data2 = 24;
    check_sum = 0;

    /*
    * send first message
    */

    check_process("Receiver", STATE_READY, TRUE);
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: sending a message using send()...\n",
	    self->name);
    send(receiver_port, &data1);

    if (check_sum != 1)
       test_failed(37);

    kprintf("%s: received data = %d\n", self->name, data1);
 
    if (data1 != 11)
       test_failed(42);

    /*
     *  send second message
     */
    check_sum += 2;
    check_process("Sender", STATE_READY, TRUE);
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(39);
    }
    check_process("Receiver", STATE_READY, TRUE);
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: sending a message using message()...\n",
	    self->name);
    message(receiver_port, &data2);

    if (check_sum != 7)
	test_failed(37);

    kprintf("%s: woken up from message().\n", self->name);

    check_process("Receiver", STATE_READY, TRUE);
    if (test_result != 0)
	test_failed(test_result);

    check_sum += 8;
    return_to_boot();
}

void test_ipc_2_receiver_process (PROCESS self, PARAM param)
{
    PROCESS sender;
    int* data;
   
    /*
     * receiving the first message
     */
    check_sum += 1;

    // Sender should now be SEND_BLOCKED and off read queue
    check_process("Sender", STATE_SEND_BLOCKED, FALSE);
    if (test_result == 13) {    
	print_all_processes(kernel_window);
	test_failed(35);
    }
    if (test_result == 14) {
	print_all_processes(kernel_window);
	test_failed(36);
    }

    kprintf("%s: receiving first message...\n", self->name);
    data = (int*) receive (&sender);

    // check the processes
    // Sender should now be REPLY_BLOCKED and off read queue
    check_process("Sender", STATE_REPLY_BLOCKED, FALSE);
    if (test_result == 13) {
	print_all_processes(kernel_window);
	test_failed(38);
    }
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: received a message from %s, parameter = %d\n",
	    self->name, sender->name, *data);

    if (*data != 42)
	test_failed(41); 
    *data = 11;

    kprintf("%s: replying to %s.\n", self->name, sender->name);
    reply(sender);

    if (check_sum != 3)
	test_failed(40);

    /*
     * receiving the second message
     */
    check_sum += 4;

    // Sender should now be MESSAGE_BLOCKED and off read queue
    check_process("Sender", STATE_MESSAGE_BLOCKED, FALSE);
    if (test_result == 13) {    
	print_all_processes(kernel_window);
	test_failed(43);
    }
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(36);
    }

    kprintf("%s: receiving second message...\n", self->name);
    data = (int*) receive (&sender);

    if (*data == 11)
	test_failed(44); //the first message is received again
    
    // Sender should now be STATE_READY and on read queue
    check_process("Sender", STATE_READY, TRUE);
    if (test_result == 13) {
	print_all_processes(kernel_window);
	test_failed(45);
    }
    if (test_result != 0) { 
	print_all_processes(kernel_window); 
	test_failed(46);
    }

    kprintf("%s: received a message from %s, parameter = %d\n",
	    self->name, sender->name, *data);

    if (*data != 24)
	test_failed(41);

    resign();
    test_failed(46);
}


/*
 * This test creates a sender and a receiver process. The sender process
 * has the higher priority and will be scheduled first.
 * The execution sequence is as follow:
 * 1. The sender executes a send(). Since the receiver is not RECEIVE_BLOCKED,
 *    the sender will be SEND_BLOCKED.
 * 2. Execution resumes with the receiver. The receiver executes a receive(),
 *    which will return immediately, and change the sender to state
 *    REPLY_BLOCKED. 
 * 3. The receivers does a reply(), and put the sender back on the ready queue.
 *    The resign() in the reply() will therefore transfer the control back to
 *    the sender.
 * 4. The sender executes a message(). Since the receiver is not
 *    RECEIVE_BLOCKED, the sender will be MESSAGE_BLOCKED.
 * 5. Execution resumes with the receiver. The receiver executes a receive(),
 *    which will return immediately, and change the sender to STATE_READY.
 * 6. The receiver does a resign() and pass the execution back to the sender.
 * This test send() and message() in the case that the receiver is not 
 * ready to receive. It also test receive() in the case that there are messages
 * pending. 
 */ 
void test_ipc_2 ()
{
    PORT new_port;

    test_reset();
    new_port = create_process (test_ipc_2_receiver_process, 5, 0, "Receiver");
    create_process (test_ipc_2_sender_process, 6, (PARAM) new_port, "Sender");

    check_num_proc_on_ready_queue(3);
    check_process("Sender", STATE_READY, TRUE);
    check_process("Receiver", STATE_READY, TRUE);
    if (test_result != 0)
       test_failed(test_result);

    resign();
}

