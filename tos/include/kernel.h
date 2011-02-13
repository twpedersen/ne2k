
#ifndef __KERNEL__
#define __KERNEL__

#include <assert.h>
#include <stdarg.h>

#define TRUE	1
#define FALSE	0


#ifndef NULL
#define NULL	((void *) 0)
#endif


typedef int BOOL;

/*=====>>> stdlib.c <<<=====================================================*/

int k_strlen(const char* str);
void* k_memcpy(void* dst, const void* src, int len);
int k_memcmp(const void* b1, const void* b2, int len);


/*=====>>> mem.c <<<========================================================*/

typedef unsigned       MEM_ADDR;
typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned       LONG;

void poke_b(MEM_ADDR addr, BYTE value);
void poke_w(MEM_ADDR addr, WORD value);
void poke_l(MEM_ADDR addr, LONG value);
BYTE peek_b(MEM_ADDR addr);
WORD peek_w(MEM_ADDR addr);
LONG peek_l(MEM_ADDR addr);


/*=====>>> window.c <<<=====================================================*/

typedef struct {
  int  x, y;
  int  width, height;
  int  cursor_x, cursor_y;
  char cursor_char;
} WINDOW;

extern WINDOW* kernel_window;


void move_cursor(WINDOW* wnd, int x, int y);
void remove_cursor(WINDOW* wnd);
void show_cursor(WINDOW* wnd);
void clear_window(WINDOW* wnd);
void output_char(WINDOW* wnd, unsigned char ch);
void output_string(WINDOW* wnd, const char *str);
void wprintf(WINDOW* wnd, const char* fmt, ...);
void kprintf(const char* fmt, ...);


/*=====>>> process.c <<<====================================================*/

/*
 * Max. number of processes
 */
#define MAX_PROCS		20


/*
 * Max. number of ready queues
 */
#define MAX_READY_QUEUES	8


#define STATE_READY 		0
#define STATE_SEND_BLOCKED	1
#define STATE_REPLY_BLOCKED 	2
#define STATE_RECEIVE_BLOCKED	3
#define STATE_MESSAGE_BLOCKED	4
#define STATE_INTR_BLOCKED 	5


#define MAGIC_PCB 0x4321dcba


struct _PCB;
typedef struct _PCB* PROCESS;

struct _PORT_DEF;
typedef struct _PORT_DEF* PORT;

typedef struct _PCB {
    unsigned       magic;
    unsigned       used;
    unsigned short priority;
    unsigned short state;
    MEM_ADDR       esp;
    PROCESS        param_proc;
    void*          param_data;
    PORT           first_port;
    PROCESS        next_blocked;
    PROCESS        next;
    PROCESS        prev;
    char*          name;
} PCB;


extern PCB pcb[];

typedef unsigned PARAM;

/*
* The name of the boot process.
*/
#define boot_name "Boot process"
PORT create_process(void (*new_proc) (PROCESS, PARAM),
		    int prio,
		    PARAM param,
		    char *proc_name);
#ifdef XXX
PROCESS fork();
#endif
void print_process(WINDOW* wnd, PROCESS p);
void print_all_processes(WINDOW* wnd);
void init_process();



/*=====>>> dispatch.c <<<===================================================*/

extern PROCESS active_proc;
extern PCB* ready_queue[];


PROCESS dispatcher();
void add_ready_queue (PROCESS proc);
void remove_ready_queue (PROCESS proc);
void resign();
void init_dispatcher();


/*=====>>> null.c <<<=======================================================*/

void init_null_process();


/*=====>>> ipc.c <<<========================================================*/

#define MAX_PORTS	(MAX_PROCS * 2)

#define MAGIC_PORT  0x1234abcd

typedef struct _PORT_DEF {
    unsigned  magic;
    unsigned  used;              /* Port slot used? */
    unsigned  open;              /* Port open? */
    PROCESS   owner;             /* Owner of this port */
    PROCESS   blocked_list_head; /* First local blocked process */
    PROCESS   blocked_list_tail; /* Last local blocked process */
    struct _PORT_DEF *next;            /* Next port */
} PORT_DEF;


PORT create_port();
PORT create_new_port (PROCESS proc);
void open_port (PORT port);
void close_port (PORT port);
void send (PORT dest_port, void* data);
void message (PORT dest_port, void* data);
void* receive (PROCESS* sender);
void reply (PROCESS sender);
void init_ipc();


/*=====>>> intr.c <<<=======================================================*/

#define DISABLE_INTR(save)	asm ("pushfl");                   \
                                asm ("popl %0" : "=r" (save) : ); \
				asm ("cli");

#define ENABLE_INTR(save) 	asm ("pushl %0" : : "m" (save)); \
				asm ("popfl");



typedef struct 
{
    unsigned short offset_0_15;
    unsigned short selector;
    unsigned short dword_count : 5;
    unsigned short unused      : 3;
    unsigned short type        : 4;
    unsigned short dt          : 1;
    unsigned short dpl         : 2;
    unsigned short p           : 1;
    unsigned short offset_16_31;
} IDT;

#define CODE_SELECTOR 0x8
#define DATA_SELECTOR 0x10
#define MAX_INTERRUPTS 256
#define IDT_ENTRY_SIZE 8


extern BOOL interrupts_initialized;

void init_idt_entry (int intr_no, void (*isr) (void));
void wait_for_interrupt (int intr_no);
void init_interrupts ();


/*=====>>> timer.c <<<===================================================*/

#define TIMER_IRQ   0x60

extern PORT timer_port;

typedef struct _Timer_Message 
{
    int num_of_ticks;
} Timer_Message;

void sleep(int num_of_ticks);
void init_timer();


/*=====>>> inout.c <<<======================================================*/

unsigned char inportb (unsigned short port);
void outportb (unsigned short port, unsigned char value);


/*=====>>> com.c <<<=====================================================*/

#define COM1_IRQ    0x64
#define COM1_PORT   0x3f8

#define COM2_IRQ    0x63
#define COM2_PORT   0x2f8


extern PORT com_port;

typedef struct _COM_Message 
{
    char* output_buffer;
    char* input_buffer;
    int   len_input_buffer;
} COM_Message;

void init_com();


/*=====>>> keyb.c <<<====================================================*/

#define KEYB_IRQ	0x61

extern PORT keyb_port;

typedef struct _Keyb_Message {
    char* key_buffer;
} Keyb_Message;

void init_keyb();

/*=====>>> shell.c <<<===================================================*/

void init_shell();

/*=====>>> train.c <<<===================================================*/

void init_train(WINDOW* wnd);

/*=====>>> pacman.c <<<==================================================*/

void init_pacman(WINDOW* wnd, int num_ghosts);

/*=====>>> ne2k.c <<<=====================================================*/

#define ETH_ALEN	6

struct ne2k_phy {
	unsigned short portaddr;
	union {
		unsigned char byte[ETH_ALEN];
		long long n;
	} macaddr;
};

/* everyone has access to the card? */
struct ne2k_phy ne2k_phy;

void init_ne2k();
#endif
