#include <kernel.h>
#include <test.h>



void test_ipc_6_sender_process_1(PROCESS self, PARAM param)
{
    PORT receiver_port = (PORT) param;
    int data = 11;

    check_process("Receiver", STATE_RECEIVE_BLOCKED, FALSE); 
    check_process("Sender 3", STATE_MESSAGE_BLOCKED, FALSE); 
    check_process("Sender 2", STATE_MESSAGE_BLOCKED, FALSE); 
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: sending a message to port 1...", self->name);
    send(receiver_port, &data);
    test_failed(40);
}  

void test_ipc_6_sender_process_2(PROCESS self, PARAM param)
{
    PORT receiver_port = (PORT) param;
    int data = 22;

    check_process("Receiver", STATE_RECEIVE_BLOCKED, FALSE); 
    check_process("Sender 3", STATE_MESSAGE_BLOCKED, FALSE); 
    check_process("Sender 1", STATE_READY, TRUE); 
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: sending a message to port 2...\n", self->name);
    message(receiver_port, &data);
    test_failed(37);
}  

void test_ipc_6_sender_process_3(PROCESS self, PARAM param)
{
    PORT receiver_port = (PORT) param;
    int data = 33;

    check_process("Receiver", STATE_RECEIVE_BLOCKED, FALSE); 
    check_process("Sender 2", STATE_READY, TRUE); 
    check_process("Sender 1", STATE_READY, TRUE); 
    if (test_result != 0) {
	print_all_processes(kernel_window);
	test_failed(test_result);
    }

    kprintf("%s: sending a message to port 3...\n", self->name);
    message(receiver_port, &data);
    test_failed(37);
}  

void test_ipc_6_receiver(PROCESS self, PARAM param)
{
   PORT port1;
   PORT port2;
   PORT port3;
   PROCESS sender_1;
   PROCESS sender_2;
   PROCESS sender_3;
   int second_sender;
   int *data1;
   int *data2;
   int *data3;

   check_sum += 1;

   port1 = self->first_port;
   kprintf("%s: creating port 3 and port 2...\n", self->name);
   port3 = create_port();
   port2 = create_port(); 
   kprintf("%s: closing port 3 and port 2...\n", self->name);
   close_port(port2);
   close_port(port3);

   check_port(port1, self->name, TRUE);
   check_port(port2, self->name, FALSE);
   check_port(port3, self->name, FALSE);
   if (test_result != 0)
      test_failed(test_result);

   create_process(test_ipc_6_sender_process_3, 5, (PARAM) port3, "Sender 3");
   create_process(test_ipc_6_sender_process_2, 5, (PARAM) port2, "Sender 2");
   create_process(test_ipc_6_sender_process_1, 4, (PARAM) port1, "Sender 1");

   /*
    * receive first message
    */
   kprintf("%s: receiving first message...\n", self->name);
   data1 = (int*) receive(&sender_1);
   kprintf("\n%s: received a message from %s, parameter = %d.\n",
           self->name, sender_1->name, * data1);

   if (string_compare(sender_1->name, "Sender 2") == 1)
       test_failed(60);
   if (string_compare(sender_1->name, "Sender 3") == 1)
       test_failed(60);
   if (string_compare(sender_1->name, "Sender 1") != 1)
       test_failed(58);
    
   check_process("Sender 3", STATE_MESSAGE_BLOCKED, FALSE); 
   check_process("Sender 2", STATE_MESSAGE_BLOCKED, FALSE); 
   check_process("Sender 1", STATE_REPLY_BLOCKED, FALSE); 
   if (test_result != 0) {
       print_all_processes(kernel_window);
       test_failed(test_result);
   }
  
   /*
    * receive second message
    */
   kprintf("%s: opening port 2 and port 3...\n", self->name);
   open_port(port2);
   open_port(port3);

   check_sum += 2; // to check that first message is receiverd.

   kprintf("%s: receiving second message...\n", self->name);
   data2 = (int*) receive (&sender_2); 
   kprintf("%s: received a message from %s, parameter = %d.\n",
           self->name, sender_2->name, * data2);

   // second message can be from either sender 2 or sender 3, depending
   // on implementation of create_port(). 
   second_sender = 0;
   if (string_compare(sender_2->name, "Sender 2") == 1) {
       check_process("Sender 2", STATE_READY, TRUE); 
       check_process("Sender 3", STATE_MESSAGE_BLOCKED, FALSE); 
       second_sender = 2;
   } else if (string_compare(sender_2->name, "Sender 3") == 1) {
       check_process("Sender 3", STATE_READY, TRUE); 
       check_process("Sender 2", STATE_MESSAGE_BLOCKED, FALSE); 
       second_sender = 3;
   } else
       test_failed(44);
    
   check_sum += 4; // to check that second message is received.

   /* 
    * receive third message
    */
   kprintf("%s: receiving third message...\n", self->name);
   data3 = (int*) receive(&sender_3); 
   kprintf("%s: received a message from %s, parameter = %d.\n",
           self->name, sender_3->name, * data3);

   if (string_compare(sender_3->name, "Sender 1") == 1)
       test_failed(44);

   if (second_sender == 2) {
       if (string_compare(sender_3->name, "Sender 2") == 1)
	   test_failed(44);
       if (string_compare(sender_3->name, "Sender 3") != 1)
	   test_failed(58);
   } else {
       if (string_compare(sender_3->name, "Sender 3") == 1)
	   test_failed(44);
       if (string_compare(sender_3->name, "Sender 2") != 1)
	   test_failed(58);
   }
 
   check_process("Sender 3", STATE_READY, TRUE); 
   check_process("Sender 2", STATE_READY, TRUE); 
   check_process("Sender 1", STATE_REPLY_BLOCKED, FALSE); 
   if (test_result != 0) {
       print_all_processes(kernel_window);
       test_failed(test_result);
   }

   check_sum += 8; // to check that third message is received.

   return_to_boot();
}    


/* This test creates a receiver process.
 * The execution sequences are:
 *  1. The receiver creates three sender processes with lower priority.
 *     sender_2 and sender_3 has higher priority than sender_1.
 *  2. The receiver creates two new ports: port3 and port2 
 *  3. The receiver closes its port2 and port3.
 *  4. The receiver executes a receive() and become STATE_RECEIVE_BLOCKED
 *  5. Sender_process_3 gets the chance to run. It does a message() to port3 of 
 *     the receiver and become STATE_MESSAGE_BLOCKED. It is added to the
       blocked list of port3.
 *  6. Sender_process_2 gets the chance to run. It does a message() to port2 of 
 *     the receiver and become STATE_MESSAGE_BLOCKED. It is added to the 
       blocked list of port2.
 *  7. Sender_process_1 gets the chance to run. It does a send() to port1 of 
 *     the receiver and become STATE_REPLY_BLOCKED. 
 *  8. The receiver wakes up and receives the message from sender_process_1.  
 *  9. The receiver opens port2 and port3.
 *  10. The receiver does a receive, and receiver the message from sender_
       process_3 (or sender_process_2, depending on the implementation of 
       create_new_port() ). 
 *  11. The receiver executes a receive() again and receives the message from 
 *     sender_process_2 ( or sender_process_3. )
 *  This test case tests scanning the ports. 
 */
void test_ipc_6() 
{
    PORT port1;
  
    test_reset();
    port1 = create_process(test_ipc_6_receiver, 6, 0, "Receiver");

    check_sum = 0;
    resign();
    
    kprintf("Back to boot.\n");
    if (check_sum != 15)
	test_failed(59);
}

