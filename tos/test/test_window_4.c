
#include <kernel.h>
#include <test.h>


/*
 * Make sure that scrolling and wrap-around works in a window
 */
void test_window_4()
{
    WINDOW test_window = {5, 3, 10, 5, 0, 0, ' '};
    int i, j;

    test_reset();
    for (i = 0; i < 26; i++) {
	for (j = 0; j < i; j++) {
	    output_char(&test_window, 'A' + j);
	}
	output_char(&test_window, '\n');
    }
    output_char(&test_window, '*');

    char* expected_output[] = {
	"",
	"",
	"",
	"     UVWX",
	"     ABCDEFGHIJ",
	"     KLMNOPQRST",
	"     UVWXY",
        "     *",
	NULL
    };

    check_screen_output(expected_output); 
    if (test_result != 0)
	test_failed(test_result);
}
