
#include <kernel.h>
#include <test.h>



void test_ipc_5_first_sender_process(PROCESS self, PARAM param)
{
    PORT receiver_port = (PORT) param;
    int data = 1;
  
    check_process("Sender 3", STATE_MESSAGE_BLOCKED, FALSE); 
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(43);
    }
    check_process("Receiver", STATE_RECEIVE_BLOCKED, FALSE); 
    check_process("Sender 2", STATE_READY, TRUE); 
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: sending a message to the second port using send()...\n", 
	    self->name); 
    send(receiver_port, &data);

    kprintf("%s: received = %d\n", self->name, data);

    if (data != 11)
        test_failed(39);

    check_process("Receiver", STATE_RECEIVE_BLOCKED, FALSE); 
    check_process("Sender 3", STATE_MESSAGE_BLOCKED, FALSE); 
    check_process("Sender 2", STATE_READY, TRUE);

    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    return_to_boot();
}

void test_ipc_5_second_sender_process(PROCESS self, PARAM param)
{
    PORT receiver_port = (PORT) param;
    int data = 2;
   
    check_process("Receiver", STATE_RECEIVE_BLOCKED, FALSE); 
    check_process("Sender 3", STATE_MESSAGE_BLOCKED, FALSE); 
    check_process("Sender 1", STATE_REPLY_BLOCKED, FALSE); 

    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: sending a message to the second port using message()...\n", 
            self->name); 
    message(receiver_port, &data);
    test_failed(54);
}

void test_ipc_5_third_sender_process(PROCESS self, PARAM param)
{
    PORT receiver_port = (PORT) param;
    int data = 3;
    
    check_process("Receiver", STATE_RECEIVE_BLOCKED, FALSE); 
    check_process("Sender 2", STATE_READY, TRUE); 
    check_process("Sender 1", STATE_READY, TRUE); 

    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: sending a message to the first port using message()...\n", 
            self->name); 
    message(receiver_port, &data);
    test_failed(37);
}

void test_ipc_5_receiver_process(PROCESS self, PARAM param)
{
    PORT    receiver_port1;
    PORT    receiver_port2;
    PROCESS sender1;
    PROCESS sender2;
    int* data1;
    int* data2;

    /* Close the default port and create a new port. Here we make
     * sure that close_port() and create_port() works.
     */
    receiver_port1 = self->first_port;
    kprintf("%s: closing the first port...\n", self->name);
    close_port(receiver_port1);

    check_port(receiver_port1, self->name, FALSE);
    if (test_result != 0)
	test_failed(test_result);

    receiver_port2 = create_port();
    check_port(receiver_port2, self->name, TRUE);
    if (test_result != 0)
	test_failed(test_result);

    create_process(test_ipc_5_first_sender_process, 6,
		   (PARAM) receiver_port2, "Sender 1");
    create_process(test_ipc_5_second_sender_process, 5,
		   (PARAM) receiver_port2, "Sender 2");
    create_process(test_ipc_5_third_sender_process, 7,
		   (PARAM) receiver_port1, "Sender 3");

    /*
     * receive first message
     */
    kprintf("%s: receiving first message...\n", self->name);
    data1 = (int*) receive(&sender1);

    kprintf("%s: received a message from %s, parameter = %d.\n",
	    self->name, sender1->name, *data1);

    if (string_compare(sender1->name, "Sender 3") == 1)
        test_failed(60);
    if (string_compare(sender1->name, "Sender 1") != 1)
	test_failed(58);
    
    check_process("Sender 3", STATE_MESSAGE_BLOCKED, FALSE); 
    check_process("Sender 2", STATE_READY, TRUE); 
    check_process("Sender 1", STATE_REPLY_BLOCKED, FALSE); 

    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    /*
     * receive seconde message
     */
    kprintf("%s: receiving second message...\n", self->name);
    data2 = (int*) receive(&sender2);

    kprintf("%s: received a message from %s, parameter = %d.\n",
	    self->name, sender2->name, *data2);

    if (string_compare(sender2->name, "Sender 3") == 1)
	test_failed(60);
    if (string_compare(sender2->name, "Sender 1") == 1)
	test_failed(44);
    if (string_compare(sender2->name, "Sender 2") != 1)
	test_failed(58);

    check_process("Sender 3", STATE_MESSAGE_BLOCKED, FALSE); 
    check_process("Sender 2", STATE_READY, TRUE); 
    check_process("Sender 1", STATE_REPLY_BLOCKED, FALSE); 

    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: replying %s...\n", self->name, sender1->name);
    *data1 = 11;
    reply(sender1);

    /* We do the third receive. Since there is no message pending,
     * this process will become RECEIVE_BLOCKED so that the sender
     * will continue to run. */
    data1 = (int*) receive(&sender1);
    test_failed(47);
}


/*
 * This test creates one receiver process and three sender processes.
 * Here is the sequence of events as they happen:
 *   - receiver_process() creates a second port and closes the initial port.
 *   - receiver process creates three sender processes with different
 *     priorities.
 *   - receiver process calls receive(). Since there are no messages
 *     pending, the receiver becomes RECEIVE_BLOCKED
 *   - execution resumes in third_sender_process(). This process calls
 *     message() on the closed port. The sender becomes MESSAGE_BLOCKED.
 *     Since the sender sends a message to the closed port, the receiver
 *     will not be woken up.
 *   - execution resumes in first_sender_process(). This process does a
 *     send() to the open port. This will unblock the receiver.
 *     Execution resumes with the receiver.
 *   - receiver calls receive() again to receive the next message.
 *   - Execution resumes at second_sender_process()
 *   - This process does a message() to the open port of the receiver.
 *   - Execution resumes at the receiver
 *   - Receiver does a reply() to the first sender
 *   - Receiver calls receive()
 *   - Execution resumes in the first sender()
 */

void test_ipc_5()
{
    test_reset();
    create_process(test_ipc_5_receiver_process, 7, 0, "Receiver");
    resign();
}

