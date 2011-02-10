
#include <kernel.h>

#include "disptable.c"


PROCESS active_proc;


/*
 * Ready queues for all eight priorities.
 */
PCB *ready_queue [MAX_READY_QUEUES];

/*
 * The bits in ready_procs tell which ready queue is empty.
 * The MSB of ready_procs corresponds to ready_queue[7].
 */
unsigned ready_procs;



/*
 * add_ready_queue
 *----------------------------------------------------------------------------
 * The process pointed to by p is put the ready queue.
 * The appropiate ready queue is determined by p->priority.
 */

void add_ready_queue (PROCESS proc)
{
    int          prio;
    volatile int flag;
    
    DISABLE_INTR (flag);
    assert (proc->magic == MAGIC_PCB);
    prio = proc->priority;
    if (ready_queue [prio] == NULL) {
	/* The only process on this priority level */
	ready_queue [prio] = proc;
	proc->next         = proc;
	proc->prev 	   = proc;
	ready_procs |= 1 << prio;
    } else {
	/* Some other processes on this priority level */
	proc->next  = ready_queue [prio];
	proc->prev  = ready_queue [prio]->prev;
	ready_queue [prio]->prev->next = proc;
	ready_queue [prio]->prev       = proc;
    }
    proc->state = STATE_READY;
    ENABLE_INTR (flag);
}



/*
 * remove_ready_queue
 *----------------------------------------------------------------------------
 * The process pointed to by p is dequeued from the ready
 * queue.
 */

void remove_ready_queue (PROCESS proc)
{
    int          prio;
    volatile int flag;
    
    DISABLE_INTR (flag);
    assert (proc->magic == MAGIC_PCB);
    prio = proc->priority;
    if (proc->next == proc) {
	/* No further processes on this priority level */
	ready_queue [prio] = NULL;
	ready_procs &= ~(1 << prio);
    } else {
	ready_queue [prio] = proc->next;
	proc->next->prev   = proc->prev;
	proc->prev->next   = proc->next;
    }
    ENABLE_INTR (flag);
}



/*
 * dispatcher
 *----------------------------------------------------------------------------
 * Determines a new process to be dispatched. The process
 * with the highest priority is taken. Within one priority
 * level round robin is used.
 */

PROCESS dispatcher()
{
    PROCESS      new_proc;
    unsigned     i;
    volatile int flag;
    
    DISABLE_INTR (flag);
    
    /* Find queue with highest priority that is not empty */
    i = table[ready_procs];
    assert (i != -1);
    if (i == active_proc->priority)
	/* Round robin within the same priority level */
	new_proc = active_proc->next;
    else
	/* Dispatch a process at a different priority level */
	new_proc = ready_queue [i];
    ENABLE_INTR (flag);
    return new_proc;
}



/*
 * resign
 *----------------------------------------------------------------------------
 * The current process gives up the CPU voluntarily. The
 * next running process is determined via dispatcher().
 * The stack of the calling process is setup such that it
 * looks like an interrupt.
 */
void resign()
{
    /*
     *	PUSHFL
     *	CLI
     *	POPL	%EAX		; EAX = Flags
     *	XCHGL	(%ESP),%EAX     ; Swap return adr with flags
     *	PUSH	%CS 		; Push CS
     *	PUSHL	%EAX		; Push return address
     *  PUSHL	%EAX		; Save process' context
     *  PUSHL   %ECX
     *  PUSHL   %EDX
     *  PUSHL   %EBX
     *  PUSHL   %EBP
     *  PUSHL   %ESI
     *  PUSHL   %EDI
     */
    asm ("pushfl;cli;popl %eax;xchgl (%esp),%eax");
    asm ("push %cs;pushl %eax");
    asm ("pushl %eax;pushl %ecx;pushl %edx");
    asm ("pushl %ebx;pushl %ebp;pushl %esi;pushl %edi");

    /* Save the context pointer SS:ESP to the PCB */
    asm ("movl %%esp,%0" : "=r" (active_proc->esp) : );

    /* Dispatch new process */
    active_proc = dispatcher();

    /* Restore context pointer SS:ESP */
    asm ("movl %0,%%esp" : : "r" (active_proc->esp));
    
    /*
     *  POPL  %EDI      ; Restore previously saved context
     *  POPL  %ESI
     *  POPL  %EBP
     *  POPL  %EBX
     *  POPL  %EDX
     *  POPL  %ECX
     *  POPL  %EAX
     *	IRET		; Return to new process
     */
    asm ("popl %edi;popl %esi;popl %ebp;popl %ebx");
    asm ("popl %edx;popl %ecx;popl %eax");
    asm ("iret");
}



/*
 * init_dispatcher
 *----------------------------------------------------------------------------
 * Initializes the necessary data structures.
 */

void init_dispatcher()
{
    int i;

    for (i = 0; i < MAX_READY_QUEUES; i++)
	ready_queue [i] = NULL;

    ready_procs = 0;
    
    /* Setup first process */
    add_ready_queue (active_proc);
}
