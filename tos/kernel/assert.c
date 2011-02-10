
#include <kernel.h>

static WINDOW error_window = {0, 24, 80, 1, 0, 0, ' '};


int failed_assertion(const char* ex, const char* file, int line)
{
    asm ("cli");
    clear_window(&error_window);
    wprintf(&error_window, "Failed assertion '%s' at line %d of %s",
	    ex, line, file);
    while (1) ;
    return 0;
}


void panic_mode(const char* msg, const char* file, int line)
{
    asm ("cli");
    clear_window(&error_window);
    wprintf(&error_window, "PANIC: '%s' at line %d of %s",
	    msg, line, file);
    while (1) ;
}
