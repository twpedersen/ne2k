;Utilities for Boot loader

;print subroutine
;
;takes assumes si contains message to print.
;Input:
; si = string to print.


print:	
            mov     ax, 0xE0D 
                                ;setting the ax register of bios in order to produce a character on the screen
            mov     bx, 7     
                                ;I think this is a mistake and should be BL instead of BX 
.1:	        
            int     10h       
                                ; bios call to print charactor from al 
            ss      lodsb          
                                ; load a character from si to al
            test    al, al    
                                ; a test to see if al is zero so we can stop the loop
            jnz	    .1
            ret  


;print_reg_value subroutine
;
;prints ax in hex assumes 16bit lenght maximum 
;destroys contents of dx on execution
;much manlier than I had assumed
print_reg_value:
            push        si      ; matched
            push        di      ;matched
            mov	        di,0x0300
            push	    dx		;matched saves initial value of dx
            xor	        dx,dx	; clears dx
            add	        dx,04h	; for(dx=4 ; dx>0; dx--)
            push	    ax	  	; matched I don't want to distroy the original value of ax
.1:	
            dec	        dx	    ; dx--
            rol	        ax,04h	; rotaes four high bits to low bits
            push	    ax	  	; push the roated 
            and         ax, 0Fh	; ax becomes last four bits of ax
            cmp         ax, 0Ah	; see if ax is bellow or above 10
            jae	        .3		; if above 10 jmp to abovc
.2:	
            add	        ax,30h		
            stosb
            pop	ax
            jmp	        .4
.3:	
            add	        ax,37h
            stosb	
            pop	        ax		; Remember assembly is linear and .4 is next
.4:     
            test    	dx,dx
            jnz	.1
            mov 	    ax,13
            stosb
            mov         ax,10
            stosb
            xor         ax,ax
            stosb	
            mov	        si,0x0300
            call     	print
            pop	        ax 
            pop	        dx 
            pop	        di 
            pop	        si 
            
            ret
;My routine assumes disk access is only 16bit. This is good for small 
;disks and floppies, but will fail on modern hard disks.
;
;
;inputs:
; ax = Relative partition sector
; bx = offset
; es = segment
;
;Modified Registers
;
; cx, dx
;
;Memory locations at ES:BX are modified from BX-0x0200
;
 
read_disk:
        push        ax	        ; putting ax on the stack
                                ; When int 13h is executed 
                                ; ah contains the error
           
        push	    bx	        ; bx pushed on the stack
                                ; to make computation less confining
                                ; its retrevied before readin
        push        dx		
        pop	        dx
        xor	        dx, dx  	; when deviding dx:ax is treated as one
                                ; value so this zeros dx
                        
;converting from LBA to CHS
    
        add	        ax, [sc_b4_prt]
                                ; ax does not take into account boot sector
                                ; so this adjusts for it 
                            
        mov     	bx, [sc_p_trk]
                                ; bx = tracks
        div	        bx	        ; sector / tracks = [ax]CHStrack R [dx](CHSsector-1)
        inc	        dx	        ; dx=CHSsector
        mov 	    cl, dl	    ; for a 1.44 floppy dx !> 18, dx !> 0x0012 so dh=00 dl=CHSsector
                                ; cl=CHSsector as needed for int 13
        xor	        dx, dx      ; clearing xor in preperation for devision 
        div	        word [heads] 
                                ; [ax]ABStrack / heads = [ax]CHStrack R [dx]head
        mov	        ch, al	    ; for a floppy al !> 
        mov	        dh, dl      ; dx for floppy 0 or 1 so dh=00 dl=CHShead
                                ; dh=CHShead as needed for int 13
        mov	        ah, 02h     ; set the high bit for reading as needed by int13
        mov	        al, 01h     ; read one sector(SUPER INEFFICENT) for int13
        mov	        dl, [drive] ; set drive from memory loaded from bios
        pop	        bx	        ; retrieve bx for offset
                                ; es unmodified from entry
        int	0x13	            ; call interupt 
                                ; sector is now read and placed in [ES:BX]
        pop	ax	                ; return ax its proper value
        ret

        
get_disk_read_information:
;;; inputs : di (set to fat entry 11char in, after the filename)
;;; modifies : AX, BX, CX, SI
;;; returns: 
    xor ax, ax
    mov si, di ; set si = di
    add si, 0Fh ; skip data not needed, to get to fat sector
    lodsw  ; load 16bit value in AX, ax=fat cluster
    ret

read_fat_into_memory:
;;; inputs : none
;;; modifies bx, cx
;;; returns: fat at 0x0800

    push ax ; save AX from computation
    mov ax, [sc_b4_fat] ; get sector to start reading at
    mov cx, [sc_p_fat] ; set loop to number of sectors in fat
    mov bx, 0x800 ; set memory segment
 .1:
    push cx ; save cx
    call read_disk 
    pop cx ; return cx
    inc ax ; adjust ax, and read again
    add bx, 0x0200 ; adjust bx for sector
    loop .1 ; loop for number of sectors
    pop ax ; get ax
    ret

next_cluster_in_chain:
.1: 
    mov si, 0x800 ; 
    mov  dx, ax
    mov cx, ax ; set cx = ax
    shr cx, 1 ; devide by 2
    add cx, ax ; add ax and cx which is file_sys_cluster * 1.5 = fat bytes
    add si, cx
    lodsw  ;16 bit word into ax  
    test dx, 01h
    jz .2
    shr ax,4
    jmp .3
.2: 
    and ax, 0xFFF
.3: 
    cmp ax, 0xFF8
    jae .4
    push ax
    push dx
    call disk_to_mem
    pop dx
    pop ax
    jmp .1
.4:
    ret
    
disk_to_mem: 
    ;Rewritten to use ES as counter instead of BX
    call cluster_to_sector
    push cx
    xor bx,bx
    call read_disk
    mov bx, es
    add bx, 0x020
    mov es, bx
    xor bx,bx
    pop cx
    ret

cluster_to_sector:
    sub ax, 03h
    add ax, [first_data_sector]
    inc ax
    ret

                             
reboot:	xor	ah, ah    ; clear ah in order to read a charactor
	int	16h       ; wait for input from keyboard
	int	19h       ; REBOOT system