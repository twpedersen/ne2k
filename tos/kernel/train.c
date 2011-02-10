
#include <kernel.h>


WINDOW* train_wnd;
char* trainID = "20";
char* zamboniID ="24";
char* preferred_speed = "5";

//**************************
// function prototypes
//**************************

void send_command(char* command, int input_len, char* input_buffer);
//send a command to the train system
//the command should not cause any input

void append_string(char* string1, char* string2);
//append string2 to the end of string1

void set_switch(char* switchID, char* setting);
//set a switch
//setting is either "G" or "R"

void set_train_speed(char* speed);
//speed can be "0"-"5", or "D" for reversing direction

char probe(char* contactID);
//the probe result is returned as a char ('0' or '1')

void contact_status_notifier(char* contactID, char* status);
//return to the caller when a contact is in desired status

void init_train_settings();
//initialize the train 

int  check_layout(); 
//check the borad layout (the train's original position.)
//return 120 for config 1 and 2, return 340 for config 3 and 4

int check_zam_dir();
//check the direction of zamboni
//return 0 if there is no zamboni, 1 if the Zamboni runs clockwise, 
//2 if it runs conterclock. 

void run_config12();
void run_config34();
void run_config1z();
void run_config2z();
void run_config3z();
void run_config4z();
void run_train_app();


void append_string(char* string1, char* string2) {
   int i;
   int string1_len;
   for (string1_len = 0; string1[string1_len] != '\0'; string1_len ++);
   for (i = 0; string2[i] != '\0'; i ++)
      string1 [i + string1_len] = string2[i];
   string1 [i + string1_len] = '\0';
}

//**********************
//send_command
//**********************

void send_command(char* command, int input_len, char* input_buffer) {
   COM_Message msg;
   char full_command[20];

   full_command[0] = '\0';
   append_string(full_command, command);
   append_string(full_command, "\015");
   msg.output_buffer = full_command;
   msg.len_input_buffer = input_len;
   msg.input_buffer = input_buffer;
   //kprintf("%s", full_command);
   sleep(9);
   send(com_port, &msg);
}

void set_switch(char* switchID, char* setting) {
   char command[10];
   char dummy;

   command[0] = 'M';
   command[1] = '\0';
   append_string(command, switchID);
   append_string(command, setting);
   send_command(command, 0, &dummy);
}

void set_train_speed(char* speed) {
   char command[10];
   char dummy;

   command[0] = 'L';
   command[1] = '\0';
   append_string(command, trainID);
   if (speed[0] != 'D') {
      append_string(command, "S");
      append_string(command, speed);
   } else 
      append_string(command, "D");
   send_command(command, 0, &dummy);
}



//**********************
//probe 
//**********************
char probe (char* contactID) {
   char command[10];
   char input[3];
   char dummy;

   send_command("R", 0, &dummy);

   command[0] = 'C';
   command[1] = '\0';
   append_string(command, contactID);
   send_command(command, 3, input); 
   //kprintf("Probing result of contact %s is .... %c\n ", contactID, input[1]);
   return input[1];
}


//***********************************************************
//contact_status_notifier
//**********************************************************

void contact_status_notifier(char* contactID, char* status) {
   char input;

   while (42) {
      input = probe (contactID);
      if (input == status[0])
         return;
   }

}

//*********************************
//initialize the train application
//*********************************
void init_train_settings() {
    wprintf(train_wnd, "Initialize the system.");

    set_switch("9", "R");
    set_switch("8", "G");
    set_switch("7", "G");
    wprintf(train_wnd, ".");
    set_switch("6", "G");
    set_switch("5", "G");
    set_switch("4", "G");
    wprintf(train_wnd, ".");
    set_switch("3", "G");
    set_switch("2", "G");
    set_switch("1", "G");
    
    wprintf(train_wnd, "Done\n");
}

//***********************************************************
//check the borad layout (the train's original position.)
//return 12 for config 1 and 2, return 34 for config 3 and 4
//**********************************************************
int check_layout()
{
    char input;
    wprintf(train_wnd, "Check layout...");
    
    input = probe("8");
    
    wprintf(train_wnd, "Done\n");
    
    if (input == '0') 
	return 340;
    else 
	return 120;
}

//***********************************************************
//check the direction of zamboni
//return 0 if no zamboni, 1 if it runs clockwise, 2 if it 
//runs conterclock. 
//**********************************************************
int check_zam_dir()
{
    BOOL found;
    int i;
    char input;
    int time67;   //the times takes for Zamboni to travel from contact 6 to 4 
    int time74;   //the times takes for Zamboni to travel from contact 7 to 4 
    
    wprintf(train_wnd, "Check if zamboni is there");

    //check if the zamboni is there
    //strategy: keep probing contact 4 until Zambino gets there
    //or until it has probed 60 times.
    found = FALSE;

    for (i = 0; found == FALSE && i < 25; i ++) {              
	output_char(train_wnd, '.');   
	input = probe("6");
	if (input == '1') { 
	    found = TRUE;
	    wprintf(train_wnd, "Found\n");
	}
	sleep(30);
    }
    
    if (found == FALSE) {
	wprintf(train_wnd, "Not found\n");
	return 0;
    }

    //check zamboni's direction
    wprintf(train_wnd, "Check zamboni direction......"); 

    //check how long it takes for zamboni to reach contact 7 
    found = FALSE;
    for (i = 0; found == FALSE && i < 60; i++) {
	output_char(train_wnd, '.');
	input = probe("7");
	if (input == '1') 
	    found = TRUE;      
	sleep(30);
    }

    time67 = i;

    //check how long it then takes zamboni to reach contact 4
    found = FALSE;
    for (i = 0; found == FALSE && i < 60; i++) {
	output_char(train_wnd, '.');
	input = probe("4");
	if (input == '1') 
	    found = TRUE;
	sleep(30);     
    }

    time74 = i;

    wprintf(train_wnd, "Done\n");

    //clockwise: time67 < time74
    //counterclockwise: otherwise
    if (time67 < time74)
	return 1;
    else
	return 2;
}


//***********************************************************
//Runs configuration 1 or 2 without zamboni 
//**********************************************************

void run_config12()
{
    set_switch("2", "G"); //haijie, students won't know this

    set_switch("5", "R");
    set_train_speed(preferred_speed);
    set_switch("4", "R");

    contact_status_notifier("1", "1");
    set_train_speed("0");
    set_train_speed("D");
    set_switch("6", "R");
    set_train_speed(preferred_speed);

    contact_status_notifier("8", "1");
    set_train_speed("0");

    wprintf(train_wnd, "I am back safely!\n\n");
}


//***********************************************************
//Runs configuration 3 or 4 without zamboni 
//**********************************************************

void run_config34()
{
    set_switch("1", "R"); //haijie, students won't know this
    set_switch("2", "R"); //haijie, students won't know this
    
    set_switch("4", "R");
    set_switch("3", "R");
    set_train_speed(preferred_speed);
    set_switch("5", "R");
    set_switch("6", "G");
    set_switch("7", "R");

    contact_status_notifier("12", "1");
    set_train_speed("0");
    set_train_speed("D");
    set_switch("7", "R");
    set_switch("8", "R");
    set_train_speed(preferred_speed);

    contact_status_notifier("13", "1");
    set_train_speed("0");
    set_train_speed("D");
    set_switch("8", "G");
    set_switch("5", "G");
    set_train_speed(preferred_speed);

    contact_status_notifier("5", "1");
    set_train_speed("0");

    wprintf(train_wnd, "I am back safely!\n\n");
}


//***********************************************************
//Runs configuration 1 with zamboni 
//**********************************************************

void run_config1z()
{
    set_switch("6", "R");
    set_switch("4", "R");

    contact_status_notifier("10", "1");
    set_train_speed(preferred_speed);

    contact_status_notifier("1", "1");
    set_switch("1", "R");
    set_train_speed("0");
    set_train_speed("D");
    
    contact_status_notifier("6", "1");
    set_train_speed(preferred_speed);
    
    contact_status_notifier("10", "1");
    set_switch("5", "R");

    contact_status_notifier("8", "1");
    set_train_speed("0");
    set_switch("5", "G"); 

    wprintf(train_wnd, "I am back safely!\n\n");
}

//***********************************************************
//Runs configuration 2 with zamboni 
//**********************************************************

void run_config2z()
{
    set_switch("5", "R");
    set_switch("6", "R");
    set_switch("1", "R");
    
    contact_status_notifier("4", "1");
    set_switch("4", "R");
    set_train_speed(preferred_speed);
    
    
    contact_status_notifier("1", "1");
    set_switch("4", "G");
    set_train_speed("0");
    set_train_speed("D");
   
    contact_status_notifier("4", "1");
    set_train_speed(preferred_speed);
    
    contact_status_notifier("8", "1");
    set_train_speed("0");
    wprintf(train_wnd, "I am back safely!\n\n");
}

//***********************************************************
//Runs configuration 3 with zamboni 
//**********************************************************

void run_config3z()
{
    contact_status_notifier("10", "1");
    set_train_speed(preferred_speed);
    
    contact_status_notifier("7", "1");
    set_switch("4", "G");
    set_switch("5", "R");
    set_switch("6", "G");
    set_switch("7", "G");
    
    contact_status_notifier("9", "1");
    set_switch("5", "G");

    contact_status_notifier("12", "1");
    set_train_speed("0");
    set_train_speed("D");
    set_switch("7", "R");
    
    contact_status_notifier("10", "1");
    set_switch("1", "R");
    set_switch("2", "R");
    set_switch("8", "R");
    set_train_speed(preferred_speed);
   
    //contact_status_notifier("15", "1");
    contact_status_notifier("12", "0");
    set_switch("7", "G");

    contact_status_notifier("13", "1");
    set_switch("8", "G");
    set_train_speed("0");
    set_train_speed("D");
    
    contact_status_notifier("7", "1");
    set_train_speed(preferred_speed);
    
    contact_status_notifier("4", "1");
    set_switch("4", "R");
    set_switch("3", "R");

    contact_status_notifier("5", "1"); 
    set_switch("4", "G");
    set_train_speed("0");
   
    wprintf(train_wnd, "I am back safely!\n\n");
}



//***********************************************************
//Runs configuration 4 with zamboni 
//**********************************************************

void run_config4z()
{
    contact_status_notifier("4", "1");
    set_train_speed(preferred_speed);
    set_switch("5", "R");
    
    contact_status_notifier("12", "1");
    set_train_speed("0");
    set_train_speed("D");
    set_switch("7", "R");
    set_switch("2", "R");
    
    contact_status_notifier("13", "1");
    set_train_speed(preferred_speed);
    
    contact_status_notifier("13", "0");
    contact_status_notifier("13", "1");
    set_train_speed("0");
    set_train_speed("D");
    
    contact_status_notifier("7", "1");
    set_train_speed(preferred_speed);
    set_switch("8", "G");

    contact_status_notifier("4", "1");
    set_switch("8", "R");
    set_switch("4", "R");
    set_switch("3", "R");
    
    contact_status_notifier("5", "1");
    set_train_speed("0");

    wprintf(train_wnd, "I am back safely!\n\n"); 
}

//**************************
//run the train application
//**************************

void train_process(PROCESS self, PARAM param)
{
    int layout;
    int dir;

    train_wnd = (WINDOW*) param;
    
    wprintf(train_wnd, "\n");
    init_train_settings();

    //check the configuration and call the corresponding function
    layout = check_layout();
    dir = check_zam_dir();
    //dir = 0; 
    
    switch (layout + dir) {
    case 120: 
	wprintf(train_wnd, "It is configuration 1 or 2 without Zamboni.\n");
	wprintf(train_wnd, "I will move soon.\n");
	run_config12();
	break;
    case 340:
	wprintf(train_wnd, "It is configuration 3 or 4 without Zamboni.\n");
	wprintf(train_wnd, "I will move soon.\n");
	run_config34();
	break;
    case 121:
	wprintf(train_wnd, "It is configuration 1 with Zamboni.\n");
	wprintf(train_wnd, "I will move soon.\n");
	run_config1z();
	break;
    case 122:
	wprintf(train_wnd, "It is configuration 2 with Zamboni.\n");
	wprintf(train_wnd, "I will move soon.\n"); 
	run_config2z();
	break;
    case 341:
	wprintf(train_wnd, "It is configuration 3 with Zamboni.\n");
	wprintf(train_wnd, "I will move soon.\n");
	run_config3z();
	break;
    case 342:
	wprintf(train_wnd, "It is configuration 4 with Zamboni.\n");
	wprintf(train_wnd, "I will move soon.\n");
	run_config4z();
	break;
    }
    remove_ready_queue(active_proc);
    resign();
}


void init_train(WINDOW* wnd)
{
    create_process(train_process, 5, (PARAM) wnd, "Train process");
}
