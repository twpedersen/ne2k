
#include <kernel.h>
#include <test.h>

/*
 * Sequence of events:
 * - Parent executes fork, thereby creating a copy of itself.
 * - the parent continues to run, but since for the parent fork()
 *   will return not NULL, it skips the if-statement in function f().
 * - resign() switches to the child.
 * - the child resumes from the fork() call.
 * - fork() will return NULL for the child, thereby going into
 *   the if-statement.
 * - The child calls resign(), switching back to the parent.
 * - the parent resumes and eventually exits.
 */


/*
 * Call fork() in an ordinary C-function. This way we make sure that
 * the stack for the new process is aligned properly.
 */


void f()
{
    PROCESS proc;
    
    proc = fork();
    if (proc == NULL) {
	kprintf("Child process\n");
	print_all_processes(kernel_window);
	kprintf("\n");
	// Give the parent a chance to run
	resign();
	test_failed(90);
    }
    // Give the parent a chance to run
    resign();
    kprintf("Parent process\n");
}

void test_fork_1()
{
    test_reset();
    f();
    print_all_processes(kernel_window);
}

