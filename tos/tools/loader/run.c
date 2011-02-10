/*
 * run.c
 *---------------------------------------------------------
 * Run ELF    executable.                          AP  3/01
 */

#pragma inline


#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <time.h>

#include "elf.h"
#include "run.h"

#define WL_NOT_SUPPORTED	0xfe
#define ENABLE_A20		0xdf
#define DISABLE_A20		0xdd



void clear_segment_descr (int i)
{
    char	*ptr;
    int		x;

    ptr = (char *) GDT_BASE;
    ptr += i * SIZE_OF_GDT_ENTRY;
    for( x = 0; x < SIZE_OF_GDT_ENTRY; x++ ) *ptr++ = '\0';
}



void define_segment_base (int i, unsigned long base)
{
    char *ptr;

    ptr = (char *) GDT_BASE;
    ptr += i * SIZE_OF_GDT_ENTRY;
    *( ptr + 2 ) = base & BYTE;
    *( ptr + 3 ) = ( base >> 8 ) & BYTE;
    *( ptr + 4 ) = ( base >> 16 ) & BYTE;
    *( ptr + 7 ) = ( base >> 24 ) & BYTE;
}



void define_segment_limit( int i, unsigned long limit )
{
    char *ptr;

    ptr = (char *) GDT_BASE;
    ptr += i * SIZE_OF_GDT_ENTRY;
    *( ptr + 0 ) = limit & BYTE;
    *( ptr + 1 ) = ( limit >> 8 ) & BYTE;
    *( ptr + 6 ) = ( limit >> 16 ) & 0x0f;
}



void define_segment_attr( int i, unsigned int type )
{
    char *ptr;
    unsigned int	t;
    
    t = 0;
    t |= 0x8000;		/* Granularity = 1 */
    t |= 0x4000;		/* D = 1 */
    t |= 0x0080;		/* Present bit = 1 */
    t |= 0x0010;		/* Memory segment */
    t |= type & 0x0f;
    ptr = (char *) GDT_BASE;
    ptr += i * SIZE_OF_GDT_ENTRY;
    *( ptr + 5 ) = t & BYTE;
    *( ptr + 6 ) |= ( t >> 8 ) & BYTE;
}



void special_bcopy( long dest, char *source_ptr, int source_size )
{
    int i;
    char *dest_ptr;
    
    dest_ptr = MK_FP (0, dest);
    for (i = 0; i < source_size; i++)
	*dest_ptr++ = *source_ptr++;
}



void run_executable ()
{
    long	dest;
    char	gdt_ptr[ 6 ];
    int	*gdt_length;
    long	*gdt_pos;
    long	size;
    long	x;
    long	i;

    asm	.386p
    asm	cli
	
    dest = TEXT_SEGMENT_BASE + phdr_text.virtaddr;
    x = phdr_text.filesize;
    i = 0;
    while( x != 0 ) {
	size = ( x > MAX_MALLOC ) ? MAX_MALLOC : x;
	special_bcopy( dest, text_seg[ i++ ], size );
	dest += size;
	x -= size;
    }
    
    dest = TEXT_SEGMENT_BASE + phdr_data.virtaddr;
    x = phdr_data.filesize;
    i = 0;
    while( x != 0 ) {
	size = ( x > MAX_MALLOC ) ? MAX_MALLOC : x;
	special_bcopy( dest, data_seg[ i++ ], size );
	dest += size;
	x -= size;
    }
    
    /* Null Segment Descriptor */
    clear_segment_descr( 0 );
    
    /* OS text descriptor */
    clear_segment_descr( 1 );
    define_segment_base( 1, TEXT_SEGMENT_BASE );
    define_segment_limit( 1, 0x00ffffff );
    define_segment_attr( 1, ATTR_CODE );
    
    /* OS data descriptor */
    clear_segment_descr( 2 );
    define_segment_base( 2, /* text_size + */ TEXT_SEGMENT_BASE );
    define_segment_limit( 2, 0x00ffffff );
    define_segment_attr( 2, ATTR_DATA );
    
    gdt_length = (int *) &gdt_ptr[ 0 ];
    gdt_pos    = (long *) &gdt_ptr[ 2 ];
    
    *gdt_length = 3 * 8 - 1;
    *gdt_pos    = GDT_BASE;
    
    asm	lgdt	pword ptr gdt_ptr

    /* Enable protection */
    asm	smsw	ax
    asm	or	ax,1
    asm	lmsw	ax

    /* Now we're in 16 bit p-mode. Do jump to clear pre-fetch queue */
    asm	jmp	flush
    flush:

    /* Opcode for JMP FAR */
    asm	db	066h,0eah
    /* Offset to TOS-bootstrap */
    asm	dw 0,0
    /* Code selector in GDT */
    asm	dw	8

    for( ; ; ) ;
}
