
#ifndef __LIB_H__
#define __LIB_H__

/*
 * This header file describes the API of the TOS reference implementation
 * This API is NOT to be used to implement any TOS functions.
 */

void lib_clear_window(WINDOW* wnd);
void lib_output_char(WINDOW* wnd, unsigned char ch);
void lib_output_string(WINDOW* wnd, const char *str);
void lib_wprintf(WINDOW* wnd, const char* fmt, ...);
void lib_kprintf(const char* fmt, ...);
void lib_vsprintf(char *buf, const char *fmt, va_list argp);
char *lib_printnum(char *b, unsigned int u, int base,
		   BOOL negflag, int length, BOOL ladjust,
		   char padc, BOOL upcase);


#endif
