
include ../MakeVars


# delete builtin rules
%.o: %.c
%.o: %.s


%.o: %.s
	$(CC) $(CC_OPT) -o $@ -c $<

%.o: %.c .depend
	$(CC) $(CC_OPT) -o $@ -c $<

OBJ=run_tests.o common.o \
    test_mem_1.o \
    test_window_1.o \
    test_window_2.o \
    test_window_3.o \
    test_window_4.o \
    test_create_process_1.o \
    test_create_process_2.o \
    test_create_process_3.o test_create_process_4.o test_create_process_5.o \
    test_dispatcher_1.o test_dispatcher_2.o \
    test_dispatcher_3.o test_dispatcher_4.o test_dispatcher_5.o \
    test_dispatcher_6.o test_dispatcher_7.o \
    test_ipc_1.o test_ipc_2.o test_ipc_3.o test_ipc_4.o \
    test_ipc_5.o test_ipc_6.o \
    test_isr_1.o test_isr_2.o test_isr_3.o \
    test_timer_1.o \
    test_com_1.o \
    test_fork_1.o

tests: $(OBJ)
	$(LD) $(LD_OPT) -o ../tos.img ../kernel/lib.o ../lib/test.o $(OBJ)


run_ref: $(OBJ)
	$(LD) $(LD_OPT) -o ../tos.img ../lib/kernel.o ../lib/test.o $(OBJ)

host-tests: stdlib-test
	./stdlib-test

#
# override and use CC_HOST for stdlib-test
#
STDLIB_TEST_CFLAGS = $(CC_HOST_OPT)
stdlib-test: stdlib-test.o stdlib.o 
	$(CC_HOST) -o $@ stdlib-test.o stdlib.o

stdlib.o: ../kernel/stdlib.c
	$(CC_HOST) $(STDLIB_TEST_CFLAGS) -I../include -o $@ -c $<

stdlib-test.o: stdlib-test.c
	$(CC_HOST) $(STDLIB_TEST_CFLAGS) -o $@ -c $<

lib: lib.o
	cp lib.o ../lib/test.o

messages:
	xsltproc messages.xsl messages.xml > messages.html

clean :
	rm -f *~ *.o *.bak *.img stdlib-test

ifeq (.depend, $(wildcard .depend))
include .depend
endif

.depend:
	echo '# Module dependencies' > .depend
	$(MAKEDEPEND) -- $(CC_OPT) $(INCLUDES) -- $(OBJS:.o=.c) >> .depend
