;;-----------------------------------------------------------------------------
;; File: USBJumpTable.asm
;; Contents: 
;;-----------------------------------------------------------------------------
.module      USBJmpTbl

.globl  _ISR_Sudav, _ISR_Sof, _ISR_Sutok, _ISR_Susp, _ISR_Ures, _ISR_Highspeed, _ISR_Ep0ack, _ISR_Stub, _ISR_Ep0in, _ISR_Ep0out, _ISR_Ep1in, _ISR_Ep1out, _ISR_Ep2inout, _ISR_Ep4inout, _ISR_Ep6inout, _ISR_Ep8inout,_ISR_Ibn

.globl  _ISR_Ep0pingnak, _ISR_Ep1pingnak, _ISR_Ep2pingnak, _ISR_Ep4pingnak, _ISR_Ep6pingnak, _ISR_Ep8pingnak, _ISR_Errorlimit, _ISR_Ep2piderror, _ISR_Ep4piderror, _ISR_Ep6piderror, _ISR_Ep8piderror, _ISR_Ep2pflag

.globl  _ISR_Ep4pflag, _ISR_Ep6pflag, _ISR_Ep8pflag, _ISR_Ep2eflag, _ISR_Ep4eflag, _ISR_Ep6eflag, _ISR_Ep8eflag, _ISR_Ep2fflag, _ISR_Ep4fflag, _ISR_Ep6fflag, _ISR_Ep8fflag, _ISR_GpifComplete, _ISR_GpifWaveform

.globl _USB_Jump_Table

;------------------------------------------------------------------------------
; USB Jump Table
;------------------------------------------------------------------------------
.area USB_JUMP_TABLEUSBJT (CODE,ABS)
.org  0x600
	  _USB_Jump_Table:   
      ljmp  _ISR_Sudav            ;(00) Setup Data Available
      .db   0
      ljmp  _ISR_Sof              ;(04) Start of Frame
      .db   0
      ljmp  _ISR_Sutok            ;(08) Setup Data Loading
      .db   0
      ljmp  _ISR_Susp             ;(0C) Global Suspend
      .db    0
      ljmp  _ISR_Ures             ;(10) USB Reset     
      .db   0
      ljmp  _ISR_Highspeed        ;(14) Entered High Speed
      .db   0
      ljmp  _ISR_Ep0ack           ;(18) EP0ACK
      .db   0
      ljmp  _ISR_Stub             ;(1C) Reserved
      .db   0
      ljmp  _ISR_Ep0in            ;(20) EP0 In
      .db   0
      ljmp  _ISR_Ep0out           ;(24) EP0 Out
      .db   0
      ljmp  _ISR_Ep1in            ;(28) EP1 In
      .db   0
      ljmp  _ISR_Ep1out           ;(2C) EP1 Out
      .db   0
      ljmp  _ISR_Ep2inout         ;(30) EP2 In/Out
      .db   0
      ljmp  _ISR_Ep4inout         ;(34) EP4 In/Out
      .db   0
      ljmp  _ISR_Ep6inout         ;(38) EP6 In/Out
      .db   0
      ljmp  _ISR_Ep8inout         ;(3C) EP8 In/Out
      .db   0
      ljmp  _ISR_Ibn              ;(40) IBN
      .db   0
      ljmp  _ISR_Stub             ;(44) Reserved
      .db   0
      ljmp  _ISR_Ep0pingnak       ;(48) EP0 PING NAK
      .db   0
      ljmp  _ISR_Ep1pingnak       ;(4C) EP1 PING NAK
      .db   0
      ljmp  _ISR_Ep2pingnak       ;(50) EP2 PING NAK
      .db   0
      ljmp  _ISR_Ep4pingnak       ;(54) EP4 PING NAK
      .db   0
      ljmp  _ISR_Ep6pingnak       ;(58) EP6 PING NAK
      .db   0
      ljmp  _ISR_Ep8pingnak       ;(5C) EP8 PING NAK
      .db   0
      ljmp  _ISR_Errorlimit       ;(60) Error Limit
      .db   0
      ljmp  _ISR_Stub             ;(64) Reserved
      .db   0
      ljmp  _ISR_Stub             ;(68) Reserved
      .db   0
      ljmp  _ISR_Stub             ;(6C) Reserved
      .db   0
      ljmp  _ISR_Ep2piderror      ;(70) EP2 ISO Pid Sequence Error
      .db   0
      ljmp  _ISR_Ep4piderror      ;(74) EP4 ISO Pid Sequence Error
      .db   0
      ljmp  _ISR_Ep6piderror      ;(78) EP6 ISO Pid Sequence Error
      .db   0
      ljmp  _ISR_Ep8piderror      ;(7C) EP8 ISO Pid Sequence Error
      .db   0

;INT4_Jump_Table
      ljmp  _ISR_Ep2pflag         ;(80) EP2 Programmable Flag
      .db   0
      ljmp  _ISR_Ep4pflag         ;(84) EP4 Programmable Flag
      .db   0
      ljmp  _ISR_Ep6pflag         ;(88) EP6 Programmable Flag
      .db   0
      ljmp  _ISR_Ep8pflag         ;(8C) EP8 Programmable Flag
      .db   0
      ljmp  _ISR_Ep2eflag         ;(90) EP2 Empty Flag
      .db   0
      ljmp  _ISR_Ep4eflag         ;(94) EP4 Empty Flag
      .db   0
      ljmp  _ISR_Ep6eflag         ;(98) EP6 Empty Flag
      .db   0
      ljmp  _ISR_Ep8eflag         ;(9C) EP8 Empty Flag
      .db   0
      ljmp  _ISR_Ep2fflag         ;(A0) EP2 Full Flag
      .db   0
      ljmp  _ISR_Ep4fflag         ;(A4) EP4 Full Flag
      .db   0
      ljmp  _ISR_Ep6fflag         ;(A8) EP6 Full Flag
      .db   0
      ljmp  _ISR_Ep8fflag         ;(AC) EP8 Full Flag
      .db   0
      ljmp  _ISR_GpifComplete     ;(B0) GPIF Operation Complete
      .db   0
      ljmp  _ISR_GpifWaveform     ;(B4) GPIF Waveform
      .db   0
      