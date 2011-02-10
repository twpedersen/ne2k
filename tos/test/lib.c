#include <kernel.h>
#include <lib.h>


#define poke_b lib_poke_b
#define poke_w lib_poke_w
#define poke_l lib_poke_l
#define peek_b lib_peek_b
#define peek_w lib_peek_w
#define peek_l lib_peek_l

#include "../kernel/mem.c"


#define default_color lib_default_color
#define peek_screen   lib_peek_screen
#define poke_screen   lib_poke_screen
#define scroll_window lib_scroll_window
#define move_cursor   lib_move_cursor
#define remove_cursor lib_remove_cursor
#define show_cursor   lib_show_cursor
#define clear_window  lib_clear_window
#define output_char   lib_output_char
#define output_string lib_output_string
#define printnum      lib_printnum
#define vsprintf      lib_vsprintf
#define wprintf       lib_wprintf
#define kernel_window lib_kernel_window
#define kprintf       lib_kprintf

#include "../kernel/window.c"
