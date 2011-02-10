/*
 * elf.h
 *---------------------------------------------------------
 * Basic definitions for ELF                       AP  3/01
 */

#ifndef __ELF_H__
#define __ELF_H__

#include <stdio.h>

#define MAX_MALLOC		10000l
#define MAX_MALLOC_ENTRIES	20

extern char* text_seg [MAX_MALLOC_ENTRIES];
extern char* data_seg [MAX_MALLOC_ENTRIES];


typedef struct _ELFHeader {
	char	magic [4];
	char	class;
	char	byteorder;
	char	hversion;
	char	pad [9];
	int	filetype;
	int	archtype;
	long	fversion;
	long	entry;
	long	phdrpos;
	long	shdrpos;
	long	flags;
	int	hdrsize;
	int	phdrent;
	int	phdrcnt;
	int	shdrent;
	int	shdrcnt;
	int	strsec;
} ELFHeader;

typedef struct _ELFPrgHeader {
	long	type;
	long	offset;
	long	virtaddr;
	long	physaddr;
	long	filesize;
	long	memsize;
	long	flags;
	long	align;
} ELFPrgHeader;

extern ELFPrgHeader phdr_text;
extern ELFPrgHeader phdr_data;

void load_elf_exe (FILE* fp);

#endif
