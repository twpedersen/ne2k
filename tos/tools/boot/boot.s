[BITS 16]
%define ROOT_SEG 0x60
%define FAT_SEG 0x800
%define IMG_SEG 0x1000

%define SECOND_STAGE_BASE 0xe000
	
%define first_data_sector 9000h

%define MaxRootEntries  7c11h ;this is part of the fat system 
%define sc_p_clu 7c0Dh ;byte  Sectors per cluster

%define sc_b4_fat 7c0Eh ;word  Sectors (in partition) before FAT
%define fats 7c10h ;byte  Number of FATs
%define dir_ent 7c11h ;word  Number of root directory entries
%define sc_p_fat 7c16h ;word  Sectors per FAT
%define sc_p_trk 7c18h ;word  Sectors per track
%define heads 7c1Ah ;word  Number of heads
%define sc_b4_prt 7c1Ch ;dword Sectors before partition
%define drive 7c24h ;byte  Drive number set by bios


org 0x7C00
;This is the location of the boot loader in memory after it is read from the boot sector

entry:
  ;
  ; The following code fills the short space between bytes 0-0Eh
  ;
   
 jmp  short begin
  
  ;The jmp is to clear the boot sector
  
 
 nop
  ; Random nop to fill space
  ; followed by
  ; Zeros to fill the FAT information
 
 times 0x3B db 0;


begin:
    xor ax, ax ; to clear the ax register (the accepted way to clear the register)
    mov ds, ax ; to clear ds register
    mov es, ax ; to clear ds register
    mov ss, ax ; to clear ss register
    mov dx, ax ;
    mov sp, 0x7C00
    
 
root_dir_read:
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
;   int *maxroot, size_of_ent_bytes, size_of_sec_bytes, output;
;
;   maxroot=short int(*)"0x7C11";
;   size_of_ent_bytes=32;
;   size_of_sec_bytes=512;
;
;   output=(*maxroot * size_of_ent_bytes)/size_of_sec_bytes;
;   
;   return(output);
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
    call  read_disk;
    pop cx
    inc ax
    add bx, 0x200
    loop .1 
    ; looped to read cx sectors into [0000:0800]
    mov cx, WORD [MaxRootEntries]
    ; cx set for loop to compare filenames
    mov di, 0x0800  
    ; segment where disk was read to, needed for lodsb

.loop:

    push cx ; protecting CX from rep statement 
    mov cx, 0x000B  ; length of filename
    mov si, filename ; si = filename to compare to
    push di ; preserve di from increment cmpsb
    rep  cmpsb
    je  prepare_to_open_file ; jmp out of loop 
    pop di ; get DI prior to increment
    pop cx ; get previous CX for loop
    add di, 0x0020 ; length of entry
    loop .loop ; do CX times
    jmp error

prepare_to_open_file:

    call get_disk_read_information
    
    call read_fat_into_memory
    push es
    push ax ; save fat cluster
    xor bx, bx ; clear BX for initial read of kernel
    add bx, (SECOND_STAGE_BASE >> 4)
    mov es, bx
    xor bx, bx
    call disk_to_mem 
    pop ax ; get fat cluster for further information
 
    call next_cluster_in_chain
    pop es
 
jmp_to_stage2:
    jmp SECOND_STAGE_BASE
    

 
error: mov si, errmsg;
    call  print
    jmp reboot
good: 
    mov si, goodmsg
    call print
    ret

%include "utils.s" 


goodmsg db 10,"Successful Disk Read",13,10,0; 
errmsg db 10,"Disk Read Error",13,10,0;

hexvalueterm db 13,10,0

size equ $ - entry
%if size+11+2 > 512
  %error "Size is too large."
%endif
 times (512 - size - 11 - 2) db 0
  
filename db "STAGE2  BIN"
  db 0x55, 0xAA;
 

