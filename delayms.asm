;;-----------------------------------------------------------------------------
;; File: delayms.asm
;; Contents: functions to kill time
;;-----------------------------------------------------------------------------

.module		DELAY1MS
.globl		_EZUSB_Delay1ms

dps = 0x86

.area EZUSB (CODE)
	
_EZUSB_Delay1ms:
; Delay for 1 millisecond (1000 microseconds).
; 10 cycles * 166.6 ns per cycle is 1.66 microseconds per loop.
; 1000 microseconds / 1.66 = 602.  [assumes 24 MHz clock]
;		
		mov	a, #0			; Clear dps so that we're using dph and dpl!	
		mov	dps, a			; 
		mov	dptr,#(0xffff - 602) 	; long pulse for operating
		mov	r4,#5

_loop:	     	inc     dptr            ; 3 cycles
		mov     a,dpl           ; 2 cycles
                orl     a,dph           ; 2 cycles
                jnz     _loop		; 3 cycles
;
_er_end:		ret
		
