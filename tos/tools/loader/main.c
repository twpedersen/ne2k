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
#include "run.h"



static char *error_msg[] = {
	/*  0 */ "Improper parameters",
	/*	1 */ "Couldn't open file %s",
	/*	2 */ "Problems while accessing %s",
	/*	3 */ "Executable is too big",
	/*	4 */ "Not enough memory available",
	/*	5 */ "Bad file format"
	};



void load_error (int x, ...)
{
    va_list args;

    va_start (args, x);
    fprintf (stderr, "TOS boot loader. Written by A. Puder\n");
    fprintf (stderr, "***** ");
    vfprintf (stderr, error_msg[x], args);
    fprintf (stderr, "\n");
    va_end (args);
    exit (-1);
}


void
main (int argc, char *argv[])
{
    FILE	*fp;
    struct stat	statbuf;

    if (argc != 2) load_error (0);
    fp = fopen (argv[1], "rb");
    if (fp == NULL) load_error (1, argv[1]);
    if (stat (argv[1], &statbuf)) load_error (2, argv[1]);
    if (statbuf.st_size < 24) load_error (5);
    load_elf_exe (fp);

    /* THE ROAD OF NO RETURN!!! */
    run_executable();
}

