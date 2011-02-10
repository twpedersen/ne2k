/*
 * main.c
 *---------------------------------------------------------
 * Load and execute a HERMES executable.           AP  2/91
 */

#include <stdarg.h>
#include <stdio.h>
#include <alloc.h>
#include <sys\stat.h>

#include "elf.h"


static char *elf_error_msg[] = {
	/*  0 */ "Improper parameters",
	/*	1 */ "Couldn't open file %s",
	/*	2 */ "Problems while accessing %s",
	/*	3 */ "Executable is too big",
	/*	4 */ "Not enough memory available",
	/*	5 */ "Bad file format",
	/*      6 */ "Bad magic number",
	/*      7 */ "Can only load 32-bit ELF files",
	/*      8 */ "Can only process little endian encoding",
	/*      9 */ "Header version not 1",
	/*     10 */ "File not ELF-executable",
	/*     11 */ "Can only process x86 architecture",
	/*     12 */ "File version not 1",
	/*     13 */ "Can only process two program header entries"
	};


#define CHECK(err, x) if (!(x)) { fprintf (stderr, elf_error_msg [err]); \
				  fprintf (stderr, "\n"); exit (0); }


char *text_seg[ MAX_MALLOC_ENTRIES ];
char *data_seg[ MAX_MALLOC_ENTRIES ];


ELFPrgHeader phdr_text;
ELFPrgHeader phdr_data;


void load_elf_header (FILE* fp)
{
    ELFHeader hdr;
    int n;

    n = fread (&hdr, sizeof (ELFHeader), 1, fp);
    CHECK (5, n==1);
    
    /* Check magic number */
    CHECK (6, hdr.magic[0] == 0x7f);
    CHECK (6, hdr.magic[1] == 'E');
    CHECK (6, hdr.magic[2] == 'L');
    CHECK (6, hdr.magic[3] == 'F');
    
    /* Check address size == 32 bit */
    CHECK (7, hdr.class == 1);
    
    /* Check byteorder == little endian */
    CHECK (8, hdr.byteorder == 1);
    
    /* Check header version == 1 */
    CHECK (9, hdr.hversion == 1);
    
    /* Check filetype == executable */
    CHECK (10, hdr.filetype == 2);
    
    /* Check architecture type == x86 */
    CHECK (11, hdr.archtype == 3);
    
    /* Check file version == 1 */
    CHECK (12, hdr.fversion == 1);
    
    /* Check entries in program header == 2 */
    CHECK (13, hdr.phdrcnt == 2);
    
    
    /* Read the two ELF program headers */
    fseek (fp, hdr.phdrpos, SEEK_SET);
    fread (&phdr_text, sizeof (ELFPrgHeader), 1, fp);
    fread (&phdr_data, sizeof (ELFPrgHeader), 1, fp);
    
#if 0
    printf (".text: offset  =%lx\n", phdr_text.offset);
    printf ("       virtaddr=%lx\n", phdr_text.virtaddr);
    printf ("       filesize=%lx\n", phdr_text.filesize);
    printf ("       memsize =%lx\n", phdr_text.memsize);
    printf (".data: offset  =%lx\n", phdr_data.offset);
    printf ("       virtaddr=%lx\n", phdr_data.virtaddr);
    printf ("       filesize=%lx\n", phdr_data.filesize);
    printf ("       memsize =%lx\n", phdr_data.memsize);
#endif
}

void load_elf_exe (FILE* fp)
{
    long	alloc;
    long	x;
    long	i;
    
    load_elf_header (fp);
    
    CHECK (5, phdr_text.filesize != 0);
    CHECK (3, phdr_text.filesize < MAX_MALLOC * MAX_MALLOC_ENTRIES);
    CHECK (3, phdr_data.filesize < MAX_MALLOC * MAX_MALLOC_ENTRIES);
    
    /* Read .text/.rodata segment */
    fseek (fp, phdr_text.offset, SEEK_SET);
    x = phdr_text.filesize;
    i = 0;
    while (x != 0) {
	alloc = (x > MAX_MALLOC) ? MAX_MALLOC : x;
	text_seg [i] = (char *) malloc (alloc);
	CHECK (4, text_seg [i] != NULL);
	fread (text_seg [i], 1, alloc, fp);
	x -= alloc;
	i++;
    }
    
    /* Read .data segment */
    fseek (fp, phdr_data.offset, SEEK_SET);
    x = phdr_data.filesize;
    i = 0;
    while (x != 0) {
	alloc = (x > MAX_MALLOC) ? MAX_MALLOC : x;
	data_seg [i] = (char *) malloc (alloc);
	CHECK (4, data_seg [i] != NULL);
	fread (data_seg [i], 1, alloc, fp);
	x -= alloc;
	i++;
    }
    
    fclose (fp);
}

