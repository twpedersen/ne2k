[BITS 16]
%define ROOT_SEG 0x60
%define FAT_SEG 0x800
%define IMG_SEG 0x1000

%define SECOND_STAGE_BASE 0xe000
	
%define first_data_sector 0x7c00+0800h
%define next_seg 0x7c00+0208h

%define MaxRootEntries 0x7c00+11h ;this is part of the fat system 
%define sc_p_clu 0x7c00+0Dh ;byte Sectors per cluster

%define sc_b4_fat 0x7c00+0Eh ;word Sectors (in partition) before FAT
%define fats 0x7c00+10h ;byte Number of FATs
%define dir_ent 0x7c00+11h ;word Number of root directory entries
%define sc_p_fat 0x7c00+16h ;word Sectors per FAT
%define sc_p_trk 0x7c00+18h ;word Sectors per track
%define heads 0x7c00+1Ah ;word Number of heads
%define sc_b4_prt 0x7c00+1Ch ;dword Sectors before partition
%define drive 0x7c00+24h ;byte Drive number set by bios

org SECOND_STAGE_BASE
;
;
; Second Segment
;
; This portion of the boot sector is split and added in during the first disk read. I order to deal with
; a kernel larger the 64k the file is read in to memory and the segment register is adjusted instead of the
; memory offset.
 
    xor ax, ax ; to clear the ax register (the accepted way to clear the register)
    ;mov ds, ax ; to clear ds register (done for speed)
    ;mov ss, ax ; to clear ss register (done for speed)
    ;mov dx, ax ;
    mov sp, SECOND_STAGE_BASE
    ;mov es, ax

 

read_part:
; in order to read the fat file system, portions of the inforation is contained ; in the boot sector
    
    mov dl, [drive] ;Drive number
    ;not really important
    mov al, [fats] ;Number of FATs
    ; al = [fats] (my case 2)
    mul word [sc_p_fat] ; * Sectors per FAT
    ; al = [fats] * [sc_p_fat] (my case 18)
    add ax, [sc_b4_fat] ; + Sectors before FAT
    ; ax = [fats] * [sc_p_fat] + [sc_b4_fat] (my case 19)
    ; in my case this should be 19
    ;AX = logical Sector of Root directory
    push ax ;preserving ax
 
;int compute_max_root(){
;
; int *maxroot, size_of_ent_bytes, size_of_sec_bytes, output;
;
; maxroot=short int(*)"0x7C11";
; size_of_ent_bytes=32;
; size_of_sec_bytes=512;
;
; output=(*maxroot * size_of_ent_bytes)/size_of_sec_bytes;
; 
; return(output);
;}
compute_max_root:
    mov ax, [MaxRootEntries] 
    ;setting ax to mac root entry
    shl ax, 5 ;seting bx to 32 size of each files entry in the root directory
    shr ax, 9 ;deviding by 512
    mov cx, ax
    pop ax
    mov [first_data_sector], ax
    add [first_data_sector], cx
    mov bx, 0x0800
 
.1: 
    push cx

    call read_disk;
    
    pop cx
    inc ax
    add bx, 0x200
    loop .1 ; looped to read cx sectors into [0000:0800]
    push ax
    mov ax, WORD [MaxRootEntries]
    pop ax
    mov cx, WORD [MaxRootEntries]
    ; cx set for loop to compare filenames
    mov di, 0x0800 ; segment where disk was read to, needed for lodsb
    
    
.loop: 
    push cx ; protecting CX from rep statement 
    mov cx, 0x000B ; length of filename
    mov si, filename2 ; si = filename to compare to
    push di ; preserve di from increment cmpsb
    rep cmpsb
    je prepare_to_open_file; jmp out of loop 
    pop di ; get DI prior to increment
    pop cx ; get previous CX for loop
    add di, 0x0020 ; length of entry
    loop .loop ; do CX times
    
    jmp reboot 

prepare_to_open_file:
    call get_disk_read_information
    
    call read_fat_into_memory
    
    
    push ax ; save fat cluster
    xor bx, bx ; clear BX for initial read of kernel

	;; read kernel to 0x10000 (== 64k)
    add bx, 0x1000
    mov es, bx
    xor bx, bx
    call disk_to_mem 
    pop ax
    push es
    mov es,bx
    ; get fat cluster for further information
    pop es

    
call next_cluster_in_chain

 
 
begin_kernel_load:
  
    xor ecx,ecx
    mov ds, cx
    mov es, cx   
    mov ax, cx
    
init_train:
    
    ;initialize serial port
    ;information availble at:
    ;http://www.clipx.net/ng/bios/ng2231e.php
    
    xor ax, ax
    mov al, 10100111b ; 2400, no parity, 2, 8
    xor dx, dx
    int 14h 

    ;send four dummy command
    call send_dummy_command
    call send_dummy_command
    call send_dummy_command    
    call send_dummy_command
    call send_dummy_command
   
    ; set train speed to 0
    mov al,'L'
    call send_char_com1

    mov al,'2'
    call send_char_com1

    mov al,'0'
    call send_char_com1

    mov al,'S'
    call send_char_com1

    mov al,'0'
    call send_char_com1

    mov al, 015q
    call send_char_com1

    call delay

    ;set zamboni speed to 5
    mov al,'L'
    call send_char_com1
    
    mov al,'2'
    call send_char_com1
    
    mov al, '4'
    call send_char_com1

    mov AL, 'S'
    call send_char_com1
    
    mov al, '6'
    call send_char_com1

    mov al, 015q
    call send_char_com1

    call delay
    
    
    xor ax,ax
    int 13h
    
    cli

copy_routine:

;Get size of first segment
    
    mov ax, 0x1000
    mov es, ax
    mov cx, [ES:0x0044]

.1: 
    dec cx   
    mov bx,cx
    add bx, 0x1000
    mov al, byte[ES:BX]
    mov bx, cx
    mov [BX], al
    or cx, cx
    jnz .1

; get location of second segment
    
    mov ax, 0x1000
;grrr

    mov es, ax
;virtual address
    mov ax, [ES:0x0058]
    mov dx, [ES:0x005c]

;set CX to size


    mov cx, [ES:0x0064]

.2:
 
    dec cx   
    mov bx,cx
    add bx, ax
    push ax
    mov al, byte[ES:BX]
    mov bx, cx
    add bx, dx
    mov [BX], al
    pop ax
    or cx, cx
    jnz .2    

    
;el:
;    jmp el
         
    lgdt[gdt_ptr]
    smsw ax
    or ax,1
    lmsw ax
    
    jmp flush
flush:
    db 066h,0eah
    dw 0,0
    dw 8
    
delay:
    mov ah,0
    int 1ah			; Gets current time 
    add dx,4			; Go 4 timer ticks into the future
    push dx
delay1:
    mov ah,0
    int 1ah
    pop ax
    push ax
    cmp  ax,dx
    jnz delay1
    pop ax
    ret


send_dummy_command:
    mov al,'M'
    call send_char_com1

    mov al, 015q
    call send_char_com1

    call delay
    ret

align 8, db 0
 
real_gdt: 
    ;; First 8 bytes are all zero
    ;; null segment descriptor
    db 0x00
    db 0x00
    db 0x00
    db 0x00
    db 0x00
    db 0x00
    db 0x00
    db 0x00
    ;; OS text descriptor
    db 0xff
    db 0xff
    db 0x00
    db 0x00
    db 0x00
    db 0x9a
    db 0xcf
    db 0x00
    ;; OS Data Descriptor
    db 0xff
    db 0xff
    db 0x00
    db 0x00
    db 0x00
    db 0x92
    db 0xcf
    db 0x00
    ;; Global data descriptor
    db 0xff
    db 0xff
    db 0x00
    db 0x00
    db 0x00
    db 0x92
    db 0xcf
    db 0x00
 
gdt_ptr:
    dw 0x001f
    dw real_gdt
    dw 0x0000

send_char_com1:
; sends al to com1
; information at:
; http://www.clipx.net/ng/bios/ng22614.php

    mov ah, 01h
    mov dx, 0
    int 14h
    ret
    
%include "utils.s"
filename2 db "TOS     IMG"
