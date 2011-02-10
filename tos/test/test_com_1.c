
#include <kernel.h>
#include <test.h>


/*
 * A test case for the serial line device driver subsystem. The
 * test process sends a message to the COM process with a request
 * to send "Hello World!" to COM1. This test case needs to be run
 * with the RS232 loopback simulation. Before launching Bochs,
 * run the script ~/tos/toos/serial/loopback.pyw
 * The loopback simulation should echo the "Hello World!" string
 * and send it back to Bochs.
 */



void test_com_1()
{
    kprintf("======== test_com_1 ========");
    kprintf("You need to run 'loopback.pyw' before you run this test.");

    char buffer[13];
   /* uses 12 char for "Hello World!" and an extra char '\0'
      to indicate the end of the string */

    COM_Message msg;
    int         i;
    
    test_reset();
    init_interrupts();
    init_null_process();
    init_timer();
    init_com();
    
    print_all_processes(kernel_window);
    kprintf("\n");
    
    msg.output_buffer    = "Hello World!";
    msg.input_buffer     = buffer;
    msg.len_input_buffer = 12;
    
    send(com_port, &msg);
    
    for (i = 0; i < 12; i++)
	kprintf("%c", buffer[i]);
    kprintf("\n");

    buffer[12] = '\0';  // to make it a C string
    if (string_compare(buffer, msg.output_buffer) == 0)
        test_failed(85);
}
