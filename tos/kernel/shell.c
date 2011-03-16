#include <kernel.h>


static WINDOW shell_wnd = {0, 9, 61, 16, 0, 0, 0xDC};
static WINDOW train_wnd = {0, 0, 80, 8, 0, 0, ' '};
static WINDOW pacman_wnd = {61, 8, 0, 0, 0, 0, ' '};
static WINDOW divider_wnd = {0, 8, 80, 1, 0, 0, ' '};
static WINDOW ne2k_wnd = {60, 0, 20, 20, 0, 2, ' '};



void run_train_app(WINDOW* wnd)
{
    static int already_run = 0;

    if (already_run) {
	wprintf(&shell_wnd, "Train application already running.\n\n");
	return;
    }

    already_run = 1;
    init_train(wnd);
}


void run_pacman_app(WINDOW* wnd)
{
    static int already_run = 0;

    if (already_run) {
	wprintf(&shell_wnd, "PacMan already running.\n\n");
	return;
    }

    already_run = 1;
    init_pacman(wnd, 4);
}



void read_line(char* buffer, int max_len)
{
    Keyb_Message msg;
    char ch;
    int i = 0;

    while (1) {
	msg.key_buffer = &ch;
	send (keyb_port, &msg);
	switch (ch) {
	case '\b':
	    if (i == 0)
		continue;
	    i--;
	    break;
	case 13:
	    buffer[i] = '\0';
	    wprintf(&shell_wnd, "\n");
	    return;
	default:
	    if (i == max_len)
		break;
	    buffer[i++] = ch;
	    break;
	}
	wprintf(&shell_wnd, "%c", ch);
    }
}


int is_command(char* s1, char* s2)
{
    while (*s1 == *s2 && *s2 != '\0') {
	s1++;
	s2++;
    }
    return *s2 == '\0';
}

void process_command(char* command)
{
    if (is_command(command, "ps")) {
	print_all_processes(&shell_wnd);
	return;
    }

    if (is_command(command, "clear")) {
	clear_window(&shell_wnd);
	return;
    }

    if (is_command(command, "pacman")) {
	run_pacman_app(&pacman_wnd);
	return;
    }

    if (is_command(command, "train")) {
	run_train_app(&train_wnd);
	return;
    }

	if (is_command(command, "macaddr")) {
		ne2k_print_mac(&ne2k_wnd);
		return;
	}
	
	if(is_command(command, "ndd_pktdump")){
		ne2k_pktdump(&ne2k_wnd);
		return;
	}
		
	if(is_command(command, "ndd_regdump")){
		ne2k_reg_hexdump(&ne2k_wnd);	
		return;
	}
	
    if(is_command(command,"ndd_pktsend")){
    	ne_sendtestpacket();
    return;    
    }
    
    if (is_command(command, "go")) {
	set_train_speed("4");
	return;
    }

    if (is_command(command, "stop")) {
	set_train_speed("0");
	return;
    }

    if (is_command(command, "rev")) {
	set_train_speed("D");
	return;
    }

    if (is_command(command, "help")) {
	wprintf(&shell_wnd, "Commands:\n");
	wprintf(&shell_wnd, "  - help   show this help\n");
	wprintf(&shell_wnd, "  - clear  clear window\n");
	wprintf(&shell_wnd, "  - ps     show all processes\n");
	wprintf(&shell_wnd, "  - pacman start PacMan\n");
	wprintf(&shell_wnd, "  - go     make the train go\n");
	wprintf(&shell_wnd, "  - stop   make the train stop\n");
	wprintf(&shell_wnd, "  - rev    reverse train direction\n");
	wprintf(&shell_wnd, "  - train  start train application\n");
	wprintf(&shell_wnd, "  - macaddr  get the MAC address of your NE2K\n");
	wprintf(&shell_wnd, "  - ndd_pktdump prints the received network packets\n");
	wprintf(&shell_wnd, "  - ndd_regdump prints the ne2k register pages\n");
	wprintf(&shell_wnd, "  - ndd_pktsend send a broadcast n/w arp packet\n");

	return;
    }

    /* Room for more commands! */
    wprintf(&shell_wnd, "Syntax error! Type 'help' for help.\n");
}


void print_prompt()
{
    wprintf(&shell_wnd, "> ");
}


void shell_process(PROCESS self, PARAM param)
{
    char buffer[80];
    int i;

    clear_window(&shell_wnd);
    clear_window(&train_wnd);
    clear_window(&divider_wnd);
    for (i = 0; i < 79; i++) output_char(&divider_wnd, 196);

    wprintf(&shell_wnd, "TOS Shell\n");
    wprintf(&shell_wnd, "---------\n\n");

    while (1) {
	print_prompt();
	read_line(buffer, 80);
	process_command(buffer);
    }
}


void init_shell()
{
    create_process(shell_process, 5, 0, "Shell Process");
    resign();
}
