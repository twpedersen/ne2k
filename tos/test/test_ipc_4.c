#include <kernel.h>
#include <test.h>



void test_ipc_4_sender_process_1(PROCESS self, PARAM param)
{
    PORT receiver_port = (PORT) param;
    int data = 11;

    kprintf("%s: sending message, Parameter = %D\n",
	    self->name, data);
    send(receiver_port, &data);

    test_failed(37);
}  

void test_ipc_4_sender_process_2(PROCESS self, PARAM param)
{
    PORT receiver_port = (PORT) param;
    int data = 22;

    kprintf("%s: sending message, Parameter = %D\n",
	    self->name, data);
    send(receiver_port, &data);

    test_failed(37);
}  

void test_ipc_4_receiver(PROCESS self, PARAM param)
{
   PROCESS sender_1;
   PROCESS sender_2;
   int *data1;
   int *data2;

   check_sum = 0;

   data1 = (int*) receive(&sender_1); 
   kprintf("\n%s: received message from %s,  Parameter = %d.\n",
           self->name, sender_1->name, * data1);

   if (string_compare(sender_1->name, "Sender 2") == 1)
       test_failed(57);
   if (string_compare(sender_1->name, "Sender 1") != 1)
       test_failed(58);

   check_sum += 1;

   data2 = (int*) receive(&sender_2); 
   kprintf("\n%s: received message from %s,  Parameter = %d.\n", 
           self->name, sender_2->name, * data2);

   if (string_compare(sender_2->name, "Sender 1") == 1)
       test_failed(44);
   if (string_compare(sender_2->name, "Sender 2") != 1)
       test_failed(58);

   check_sum += 2;
   return_to_boot();
} 


/* This test creates a receiver process and two sender processes.
 * The two sender processes has higher priority than the receiver process.
 * The execute sequence is as following:
 *   1. Sender 1 executes a send(). Since the receiver is not 
 *      RECEIVE_BLOCKED, the sender will be added to the send blocked list.
 *   2. Sender 2 executes a send(). It will be added to the 
 *      send blocked list as well.
 *   3. Since both of the senders are now blocked, the receiver gets a change
 *      to run.
 *   4. The receiver executes a receive() to receive the message from
 *      Sender 1.
 *   5. The receiver executes a receive() to receiver the message from 
 *      Sender 2.
 * This test case tests send block list.
 */
void test_ipc_4()
{
    PORT new_port;
  
    test_reset();
    new_port = create_process(test_ipc_4_receiver, 5, 0, "Receiver");
    create_process(test_ipc_4_sender_process_1, 6, (PARAM) new_port, 
                   "Sender 1");
    create_process(test_ipc_4_sender_process_2, 6, (PARAM) new_port, 
                   "Sender 2"); 

    resign();

    kprintf("Back to boot\n");
    if (check_sum != 3)
	test_failed(59);
}

