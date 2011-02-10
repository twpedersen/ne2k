.text

	.align 4
.globl _start
.globl start

_start:
start:
	movw $0x10,%ax
	movw %ax,%ds
	movw %ax,%es
	movw %ax,%fs
	movw %ax,%gs
	movw %ax,%ss
	movl $640 * 1024, %esp
	call kernel_main
L1:
	jmp L1
