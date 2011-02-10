
#include <kernel.h>
#include <test.h>
#include <lib.h>

void test_mem_1()
{
    int i;

    // The following for-loop effectively clears the screen
    for (i = 0; i < 80 * 25; i++)
	poke_w(0xb8000 + i * 2, 0);
    
    poke_b(0xb8000, 'A');
    poke_b(0xb8001, 0x0f);
    poke_b(0xb8002, peek_b (0xb8000) + 1);
    poke_b(0xb8003, 0x0f);
    poke_w(0xb8004, peek_w (0xb8000));
    poke_l(0xb8006, peek_l (0xb8000));

    char* expected_output[] = {
	"ABAAB",
	NULL
    }; 
 
    test_result = 0;
    check_screen_output(expected_output);
    if (test_result != 0)
       test_failed(1);
}
