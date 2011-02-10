
#include <kernel.h>
#include <test.h>

void test_window_1()
{
    test_reset();
    output_string(kernel_window, "Hello World!\n");

    char* expected_output[] = {
	"Hello World!",
	NULL
    };
    check_screen_output(expected_output);
    if (test_result != 0 )
       test_failed(test_result);
}

