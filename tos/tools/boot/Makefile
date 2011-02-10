NASM = nasm
DD = dd
MAKEDEPEND = makedepend -f-

all: stage1.bin stage2.bin

stage1.bin: boot.s
	$(NASM) boot.s -o stage1.bin

stage2.bin: second-stage.s
	$(NASM) second-stage.s -o stage2.bin

clean:
	rm -f *.o *.bin *.bak .depend

ifeq (.depend, $(wildcard .depend))
include .depend
endif

.depend:
	echo '# Module dependencies' > .depend
	$(MAKEDEPEND) >> .depend
