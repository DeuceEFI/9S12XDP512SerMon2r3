;.pagewidth  120t
;*********************************************************************
;* Title:  S12SerMonxrx.asm        Copyright (c) Motorola 2003
;*********************************************************************
;* Author: Jim Sibigtroth - Motorola TSPG - 8/16 Bit Division
;* Author: Jim Williams - Motorola TSPG - 8/16 Bit Division
;*
;* Description: Bootloader/Monitor program for S12X
;* Bootloader will reside in 2K of block protected memory at the
;* end of the memory map of an S12X MCU ($F7FF-$FFFF).
;*
;* Since this code is located in the vector space, all interrupt
;* vectors will be mirrored to the pseudo vector table in user
;* erasable and reprogrammable flash memory just before the start
;* of the protected bootloader code.
;*
;* If a non-FFFF user reset vector is programmed into the
;* pseudo-reset vector, the bootloader will jump to that routine
;* so the user can control all options including write-once bits.
;*
;* This monitor program implements 23 primitive monitor commands that
;* are very similar to BDM commands. Third-party tool vendors can
;* adapt their existing BDM-based tools to work through a serial I/O
;* cable rather than a BDM pod, simply by providing a set of alternate
;* interface routines. Although this monitor approach has some
;* limitations compared to the BDM approach, it provides a free or
;* very low cost alternative for the most cost-sensitive users.
;*
;* This monitor uses SCI0 as the primary interface to the target MCU
;* system and SCI0 Rx interrupts are used to break out of a running
;* user program. This implies that some monitor functions will not be
;* available if the I bit in the CCR is not clear during execution of
;* the user's program. During debug of user initialization programs
;* and interrupt service routines when the I bit is not clear, trace
;* and breakpoint functions still work as expected because these
;* functions use on-chip breakpoint logic. 
;*
;*
;*
;* Revision History: not yet released
;* Rev #     Date      Who     Comments
;* -----  -----------  ------  ---------------------------------------
;*  2.00   04-SEP-03   JPW     First Release.
;*  2.01   03-DEC-03   JPW     MC9S12NE64 support added, fixed user jump table,
;*                             fixed PLL/Timer Ch.7 corruption 
;*                             Added Flash/EEPROM support > 12.8MHz OSC
;*  2.02   23-JUL-04   JPW     Fixed PPAGE coruption in erase and programming
;*                             routines - Allow $8000 page to be programmed
;*                             if PPAGE = $00. Supports GNU compiler.
;*  2.03   MAY 09      BDA     Update for S12X
;*  2.03a  JAN 14      BDA     Correction for Flash Bulk Erase
;
;
softwareID1:	equ	$0106     ;Software revision (date)
softwareID2:	equ	$2014     ;Software revision (year)
softwareID3:	equ	$0203     ;Software revision (ver)

;*
;*
;*********************************************************************
;*********************************************************************
;* Motorola reserves the right to make changes without further notice
;* to any product herein to improve reliability, function, or design.
;* Motorola does not assume any liability arising out of the
;* application or use of any product, circuit, or software described
;* herein; neither does it convey any license under its patent rights
;* nor the rights of others.  Motorola products are not designed,
;* intended, or authorized for use as components in systems intended
;* for surgical implant into the body, or other applications intended
;* to support life, or for any other application in which the failure
;* of the Motorola product could create a situation where personal
;* injury or death may occur.  Should Buyer purchase or use Motorola
;* products for any such intended or unauthorized application, Buyer
;* shall indemnify and hold Motorola and its officers, employees,
;* subsidiaries, affiliates, and distributors harmless against all
;* claims, costs, damages, and expenses, and reasonable attorney fees
;* arising out of, directly or indirectly, any claim of personal
;* injury or death associated with such unintended or unauthorized
;* use, even if such claim alleges that Motorola was negligent
;* regarding the design or manufacture of the part.
;*
;* Motorola is a registered trademark of Motorola, Inc.
;*********************************************************************

	XDEF	Startup	;make symbol visible to the MW linker

;*********************************************************************
;* Include standard definitions that are common to all derivatives
;*********************************************************************
;             base    10           ;ensure default number base to decimal

	nolist	  ;turn off listing
	include	"S12SerMon2r3.inc"
	include	"S12SerMon2r3.def"
	list	;turn listing back on

;*********************************************************************
;* General equates for bootloader/monitor program valid for all
;* derivatives
;*********************************************************************
BootStart:	equ	$F800	;start of protected boot block
MONITOR_HI:	equ	$7F	;Global space that Monitor is in
RamLast:	equ	$3fff	;last RAM location (all devices)
Window: 	equ	$8000	;PPAGE Window start
RomStart:	equ	$4000	;start of flash
VecTabSize:	equ	$80+$70	;size of vector table
VectorTable:	equ	$10000-VecTabSize	;start of vector table
PVecTable:	equ	BootStart-VecTabSize	;start of pseudo vector table
FProtStart:	equ	$FF00	;start of FLASH protection/security
FProtBlksz:	equ	$C7	;protect code for boot block ($C7 2K)
;FProtBlksz:	equ	$FF	;protect code for boot block (none)
FSecure:	equ	$BE	;Disable Security and backdoor access
;FSecure:	equ	$00	;Enable Security and backdoor access

BusFreq:	equ	((OscFreq/(initREFDV+1))*(initSYNR+1))
baud:	equ	((BusFreq/16)*10)/baudrate	;sets baud rate

;longBreak:	equ	1500	;delay time for >30-bit break
; 30*1000*1000/115200=260.4 uSec
; 1/24e6 * 5 * 1500 * 1e6 = 312.5 uSec
; n = (30*1000*1000/115200)/(1/24e6 * 5 * 1e6)
; = 36*24e4/(5*1152)
; make TxD low at least 300us (30 bits @ 115200 baud)
; 5~ * 42ns/~ * 1500 = 315us (not exact, just >30 bit times)
longBreak:	equ	(360*cpuclk)/(5*baudrate)
asciiCR:	equ	$0D	;ascii carriage return

flagReg:	equ	SCI0CR1	;SCI control reg of SCI0
RunFlag:	equ	WAKE	;SCI Wake bit used as run/mon flag
ArmFlag:	equ	RSRC	;SCI RSRC bit used for ARM storage
TraceFlag:	equ	ILT	;SCI Idle bit used as trace flag
; 1=SWI caused by return from Trace1; 0=SWI from breakpoint or DBG

;
;CPU S12X CCR immediately after reset is:
initUCcr:	equ	%11010000     ;initial value for user's CCR
;                    SX-I----     ;I interrupts masked
								  ;(SXHINZVC=11x1xxxx).
initUCch:	equ	0	;initial value for user's CCH
; IPL is zero by default - allow all interrupts

ErrNone:	equ	$E0	;code for no errors
ErrCmnd:	equ	$E1	;command not recognized
ErrRun:	equ	$E2	;command not allowed in run mode
ErrSP:	equ	$E3	;SP was out of range
ErrWriteSP:	equ	$E4	;attempted to write bad SP value
ErrByteNVM:	equ	$E5	;write_byte attempt NVM
ErrFlash:	equ	$E6	;FACCERR or FPVIOL error
ErrFlErase:	equ	$E7	;Error code not implemented
ErrGoVec:	equ	$E8	;Error code not implemented
ErrEeErase:	equ 	$E9	;EACCERR or EPVIOL error

StatHalt:	equ	$02	;stopped by Halt command
StatTrace:	equ	$04	;returned from a Trace1 command
StatBreak:	equ	$06	;Breakpoint or DBG (SWI) request
StatCold:	equ	$08	;just did a cold reset
StatWarm:	equ	$0C	;warm start because int with bad SP

;*********************************************************************
;* User CPU registers stack frame...
;*   +0  UCch   <- Monitor's SP
;*   +1  UCcr
;*   +2  UDreg   (B:A)
;*   +4  UXreg
;*   +6  UYreg
;*   +8  UPc
;*   +10  ---     <- User's SP
; Offsets from actual SP to user CPU regs while in monitor
;*********************************************************************

UCch:	equ	0	;user's CCH register
UCcr:	equ	1	;user's CCR register
UDreg:	equ	2	;user's D register (B:A)
UXreg:	equ	4	;user's X register
UYreg:	equ	6	;user's Y register
UPc:	equ	8	;user's PC
SPOffset:	equ	10	;offset of stack pointer while in monitor

MaxMonStack	equ	35	;maximum number of bytes used by Monitor
LowSPLimit	equ	RamStart+MaxMonStack-SPOffset
HighSPLimit	equ	RamLast-SPOffset+1

; named locations on stack if SWI with bad SP value
;*********************************************************************
;* Start of code and/or constant data
;*********************************************************************
	org	BootStart	;beginning of protected flash
;*********************************************************************
;* Main startup program - real reset vector points to here
;*********************************************************************
ColdStart:  
Startup:
main:
	bset	SwPullup,mSwPullup	;enable pullup on force monitor sw
	clra	 ;A=0=cold start; see tsta @ ChkCold

;**********************************************************************
; Decide whether to go to user reset or bootloader/monitor
;**********************************************************************
;  (a) default to monitor if high byte user pseudo-vector is erased ($FF)
;**********************************************************************
	ldab	vector00-($10000-BootStart)	;check for user reset Vector
	comb	;if erased COMB result will be 0
	beq	Monitor	;default to monitor if no vector

;**********************************************************************
; Test the state of some  pins to force entering the monitor
;   Depending on the hardware configuration enable/disable/modify the
;   sections below
;**********************************************************************
;	bset   SwPullup,mSwPullup	;enable pullup on force monitor sw
	bset	PPSS,PPSS0	;enable pullup on RxD0 pin
	bset	PERS,PERS0	;on RxD0 pin

	clrb
	dbne	b,*	;delay to allow sw pin to pull up
;**********************************************************************
;  (b) force monitor if SwPort bit SWITCH = 0 
;  Note: this port is configured after reset as input with pull-up
;  With no connection to this pin the monitor jumps to run mode
;**********************************************************************
	ldab	SwPort	;get port value
	bitb	#Switch	;test the sw bit
	beq	Monitor

;**********************************************************************
;  (c) force monitor if RxD low (from host)  PORTS bit 0
;      This is true if the host holds RxD on break level
;  Note: this port is configured after reset as input with pull-up
;**********************************************************************
;	brclr	PTS,PTS0,Monitor	;to monitor if RxD low
	bclr	PPSS,PPSS0	;restore reset state on RxD0 pin
	bclr	PERS,PERS0	;restore reset state on RxD1 pin

;**********************************************************************
;  finally jump to the user application (by pseudo vector)
;**********************************************************************
; Assembler's way to do - jmp [xx,pcr]
	jmp	[vector00-($10000-BootStart),pcr]	;go here
;user reset vector holds address of user code to run on boot
;*********************************************************************
;  Formal start of Monitor code
;*********************************************************************
Monitor:
; Keep the Direct Page Register at the default of $00
	clr	$11	;Clear Direct Page Register

; Initialize Interrupt Priorities for Serial Monitor    
	movb	#$ff,$0121	;set Interrupt Vector Base Reg
	movb	#$d0,$0127	;Interrupt Req Config Addr Reg
	movb	#$03,$012b	;SCI0 Int=Priority 3
	movb	#$f0,$0127	;Interrupt Req Config Addr Reg
	movb	#$04,$012b	;SWI=Brkpt, Priority 4
	bclr	SwPullup,mSwPullup	;restore reset state

; Initialize clock generator and PLL 
	bclr	CLKSEL,PLLSEL	;disengage PLL to system
	bset	PLLCTL,PLLON	;turn on PLL
	movb	#initSYNR,SYNR	;set PLL multiplier 
	movb	#initREFDV,REFDV	;set PLL divider
	nop
	nop
	brclr	CRGFLG,LOCK,*+0	;while (!(crg.crgflg.bit.lock==1))
	bset	CLKSEL,PLLSEL	;engage PLL to system
;
; set flash/EEPROM clock to 200 kHz
;
;  IF OscFreq >= 12800
	movb	#(((OscFreq/200)/8)-1)+$40,FCLKDIV	;Flash CLK = 200 kHz
	movb	#(((OscFreq/200)/8)-1)+$40,ECLKDIV	;Eeprom CLK = 200 kHz
;  ELSE	
;	movb	#((OscFreq/200)-1),FCLKDIV	;Flash CLK = 200 kHz
;	movb	#((OscFreq/200)-1),ECLKDIV	;Eeprom CLK = 200 kHz
;  ENDIF
;
; Set stack pointer to last (highest) RAM location
;
stackInit:	lds	#RamLast+1	;point one past RAM

; Setup initial user CPU register values (user register stack frame)
;   A holds the initial state value
initUregs:	ldx	BootStart-2	;load user reset vector
	pshx	    ;$00 to user UPc      $3FFe
	ldx	#$0000
	pshx	;$00 to user UYreg    $3ffc
	pshx	;$00 to user UXreg    $3ffa
	pshx	;$00 to user UDreg    $3ff8
	ldab	#initUCcr	;initial value for user CCR
	pshb	;to UCcr              $3ff7
	ldab  #initUCch	;initial value for user CCH
	pshb	;to UCch              $3ff6

;
; set baud rate (to 115.2 kbaud) and turn on Rx and Tx
;
	movb	#(baud/256),SCI0BDH
	movb	#(baud&$ff),SCI0BDL  ;Low byte of baud rate
	clr	SCI0CR1
	movb	#initSCI0CR2,SCI0CR2	;Rx and Tx on

;
; if warm start, skip break. A is a flag to indicate cold vs warm start.
; Avoid using A above here *****
;
ChkCold:	tsta	;0=cold start, non-zero=warm start
	beq	coldBrk	;if cold send break
;
; Send a warm start prompt and wait for new commands
;
	ldaa	#ErrSP	;error code for bad SP
	jsr	PutChar	;send error code (1st prompt char)
	ldaa	#StatWarm	;status code for warm start
	bra	EndPrompt	;finish warm start prompt
;
; Cold start so Generate long break to host
;
coldBrk:     
	brclr	SCI0SR1,TDRE,*	;wait for Tx (preamble) empty
	bset	SCI0CR2,SBK	;start sending break after preamble

	ldx	#longBreak	;at least 30 bit times for Windows
BrkLoop:	cpx	#0	;[2]done?
	dbne	x,BrkLoop	;[3]
	bclr	SCI0CR2,SBK	;stop sending breaks

waitforCR:	jsr	GetChar	;should be asciiCR or $00 with FE=1
	cmpa	#asciiCR	;.eq. if 115.2K baud OK
	bne	waitforCR

;*********************************************************************
;* end of reset initialization, begin body of program
;*********************************************************************
;
; Send a cold start prompt and wait for new commands
;
	ldaa	#ErrNone	;code for no errors ($E0)
	jsr	PutChar	;send error code (1st prompt char)
	ldaa	#StatCold	;status code for cold start ($08)
	bra	EndPrompt	;finish warm start prompt
;
; normal entry point after a good command
; Prompt is an alt entry point if an error occurred during a command
; endPrompt is an alternate entry for Trace1, Break (SWI), Halt,
; or warm/cold resets so an alternate status value can be sent
; with the prompt
;

CommandOK:	ldaa	#ErrNone	;code for no errors ($E0)
Prompt:	jsr	PutChar	;send error code
	ldaa	flagReg	;0 means monitor active mode
	anda	#RunFlag	;mask for run/monitor flag (SCI WAKE)
	lsra	;shift flag to LSB
	lsra	; for output as status
	lsra	;$00=monitor active, $01=run
EndPrompt:	jsr	PutChar	;send status code
	ldaa	#'>'
	jsr	PutChar	;send 3rd character of prompt seq
             
;test flagReg for run / DBG arm status.
	brclr	flagReg,RunFlag,Prompt1	;no exit if run flag clr
	brclr	flagReg,ArmFlag,PromptRun	;If DBG was not armed just run
	bset	DBGC1,ARM	;re-arm DBG module
PromptRun:	jmp	GoCmd	;run mode so return to user program

Prompt1:	jsr	GetChar	;get command code character
	ldx	#commandTbl	;point at first command entry
CmdLoop:	cmpa	,x	;does command match table entry?
	beq	DoCmd	;branch if command found
	leax	3,x
	cpx	#tableEnd	;see if past end of table
	bne	CmdLoop	;if not, try next entry
	ldaa	#ErrCmnd	;code for unrecognized command
	bra	Prompt	;back to prompt; command error
             
DoCmd:	ldx	1,x	;get pointer to command routine
	jmp	,x	;go process command
;
; all commands except GO, Trace_1, and Reset to user code - jump to
; Prompt after done. Trace_1 returns indirectly via a SWI.
;
;*********************************************************************
;* Command table for bootloader/monitor commands
;*  each entry consists of an 8-bit command code + the address of the
;*  routine to be executed for that command.
;*********************************************************************
commandTbl:	fcb	$A1
	fdb	RdByteCmd	;read byte
	fcb	$A2
	fdb	WtByteCmd	;write byte
	fcb	$A3
	fdb	RdWordCmd	;read word of data 
	fcb	$A4
	fdb	WtWordCmd	;write word of data 
	fcb	$A5
	fdb	RdNextCmd	;read next word
	fcb	$A6
	fdb	WtNextCmd	;write next word
	fcb	$A7
	fdb	ReadCmd	;read n bytes of data
	fcb	$A8
	fdb	WriteCmd	;write n bytes of data
	fcb	$A9
	fdb	RdRegsCmd	;read CPU registers
	fcb	$AA
	fdb	WriteSpCmd	;write SP
	fcb	$AB
	fdb	WritePcCmd	;write PC
	fcb	$AC
	fdb	WriteIYCmd	;write IY
	fcb	$AD
	fdb	WriteIXCmd	;write IX
	fcb	$AE
	fdb	WriteDCmd	;write D
	fcb	$AF
	fdb	WriteCcrCmd	;write CCR
	fcb	$B1
	fdb	GoCmd	;go
	fcb	$B2
	fdb	Trace1Cmd	;trace 1
	fcb	$B3
	fdb	HaltCmd	;halt
	fcb	$B4
	fdb	ResetCmd	;reset - to user vector or monitor
;            $B5 - Command not implemented
	fcb	$B6	;code - erase flash command
	fdb	EraseAllCmd	;erase all flash and eeprom command routine
	fcb	$B7	;return device ID
	fdb	DeviceCmd
	fcb	$B8	;erase current flash bank selected in PPAGE
	fdb	ErsPage
	fcb	$B9	;Bulk erase eeprom
	fdb	EraseEECmd	;
tableEnd:	equ	*	;end of command table marker

;*********************************************************************
;* Device ID Command -  Ouputs hex word from device ID register
;*********************************************************************
DeviceCmd:	ldaa	#$DC	;get part S12X descriptor
	jsr	PutChar	;out to term
	ldaa	PARTIDH	;get part ID high byte
	jsr	PutChar	;out to term
	ldaa	PARTIDL	;get part ID low byte
	jsr	PutChar	;out to term
	ldaa	#ErrNone	;error code for no errors
	jmp	Prompt	;ready for next command

;*********************************************************************
;* Halt Command - halts user application and enters Monitor
;*   This command is normally sent by the debugger while the user
;*   application is running. It changes the state variable in order
;*   to stay in the monitor
;*********************************************************************
HaltCmd:	bclr	flagReg,RunFlag ;run/mon flag = 0; monitor active
	ldaa	#ErrNone	;error code for no errors
	jsr	PutChar	;send error code
	ldaa	#StatHalt	;status code for Halt command
	jmp	EndPrompt	;send status and >
;*********************************************************************
;* Halt or continue user code by Rx interrupt of SCI. User code will 
;* continue if Run load switch is in run position and a resonable
;* Sci user vector is found.
;*********************************************************************
SciIsr:	brclr	DBGC1,ARM,SciIsr1 ;Arm not set so continue
	;above must be brclr as COF will be
	;Stored in trace buffer
	bset	flagReg,ArmFlag	;Save ARM flag
	bclr	DBGC1,ARM	;Arm bit in Dbgc1 cleared to stop DBG
SciIsr1:     
	bset	SwPullup,mSwPullup	;enable pullup on monitor sw
	bset	flagReg,RunFlag	;set run/mon flag (run mode)
	ldab	#AllowSci0	; defined in the .def file
	cmpb	#$01	; is it set?
	bne	SciIsrExit	; if AllowSci0 is set
	; Test run switch to allow user
	; Sci0 function to run
;**********************************************************************
;*  Force monitor if SwPort bit SWITCH = 0 
;*  Note: this port is configured after reset as input with pull-up
;*   if this pin in not connect sci0 will be directed to user sci0
;**********************************************************************
	ldab	SwPort	;get port value
	bitb	#Switch	;test the sw bit
	beq	SciIsrExit
;*********************************************************************
;* This routine checks for an unprogrammed SCI user interrupt
;* vector and returns to monitor if execution of an unprogrammed
;* user SCI vector is attempted
;*********************************************************************
	ldy	$F000+(vector20-BootStart)	; Get user SCI vector for SCI0

;	ldy	$F000+(vector21-BootStart)	; Get user SCI vector for SCI1

	cpy	#$FFFF	;is it programmed?	
	beq	SciIsrExit	; if not exit
	jmp	0,Y	;if programmed the go there.

SciIsrExit:  
	bclr	SwPullup,mSwPullup	;restore reset state
	jmp	Prompt1
;* unlike most ISRs, this one does not end in an RTI. If/when we
;* return to running the user program, we will re-enable Rx interrupts

;*********************************************************************
;* Reset Command - forces a reset - if user pseudo-vector is not blank
;*  (or some other conditions are met - see ColdStart:) processing will
;*  start at the user-specified reset pseudo-vector location and the
;*  user has full control of all write-once registers. Otherwise reset
;*  causes the bootloader/monitor program to cold start.
;*********************************************************************
ResetCmd:

	ldaa	#RSBCK|!CR2|!CR1|CR0	;Cop disabled in BDM

	 staa	COPCTL	; turn on cop monitor
	 cmpa	COPCTL	; load to see if user touched it
	 beq	CopLock	; wait for COP reset
	 jmp	ColdStart	; can't use COP just start over
CopLock:	orcc	#$10	; disable interrupts
	 bra	 *

;*********************************************************************
;* SWI service routine - trace1 or breakpoint from user code
;*  SWI saves user CPU registers on stack and returns to monitor
;*  control at a new command prompt.
;*  User CPU registers stack frame...
;*
;*   +0  UCch   <- SP after SWI stacking and on entry to this ISR
;*   +1  UCcr
;*   +2  UDreg   (B:A)
;*   +4  UXreg
;*   +6  UYreg
;*   +8  UPc
;*   +10  ---     <- User's SP
;*********************************************************************
Breakpoint:	clr	DBGC2	;Bkpct0 cleared to disabled
	clr	DBGC1	;Dbgc1 cleared to disarm DBG
	bclr	flagReg,RunFlag	;run/mon flag = monitor active
	ldab	#StatTrace	;set status to Trace (SWI) -> B
	;and enter monitor

;*********************************************************************
;* This is the entry point to the monitor from the user application
;*   A contains the status value that reflects run status
;*
;* If SP isn't within valid RAM, it can't support the monitor so the
;* monitor is forced to initialize the SP and user registers.
;*********************************************************************
ReenterMon:  
	cps	#LowSPLimit+1	;check against lower limit
	blo	badSP	; note: +1 => A is not pushed yet
	cps	#HighSPLimit+1	;check against upper limit
	bhi	badSP
	ldaa	#ErrNone	;error code for no errors
	jsr	PutChar	;send error code
	tba	;status code from B to A
	brclr	flagReg,TraceFlag,SWIdone	;0 indicates not Trace
	bclr	flagReg,TraceFlag	;acknowledge trace flag
	ldaa	#StatTrace	;status code for Trace1 return
SWIdone:	jmp	EndPrompt	;send status and >
badSP:	ldaa	#ErrSP	;set error code to bad stack pointer
	jmp	stackInit
	bclr	flagReg,ArmFlag	;Save ARM flag

;*********************************************************************
;* Erase EE Command -  mass
;*  erase all EEPROM locations
;*
;* Eeprom erasure assumes no protection. (Mass command will fail)
;*********************************************************************
EraseEECmd:
	bsr	EEsub	; Erase EEPROM
*
	ldaa	FSTAT
	anda	#$30	;mask all but PVIOL or ACCERR
	bne	ErsPageErr1	;back to prompt-flash error
	ldaa	#ErrNone	;code for no errors ($E0)
	jmp	Prompt	;ready for next command

ErsPageErr1:	ldaa	#ErrEeErase	;Erase error code ($E9)
	jmp	Prompt	;ready for next command

* This subroutine actually does the EEPROM mass erase
EEsub:	jsr	abClr	;abort commands and clear errors
	ldy	#EEpromStart	; get device eeprom start
	std	0,y	; write to eeprom (latch address)
	; data is don't care (but needed)
	movb	#MassErase,ECMD	;mass erase command
	movb	#CBEIF,ESTAT	;register the command
	nop	; wait a few cycles for
	nop	; command to sync.
	nop
ChkDoneE1:	ldaa	ESTAT	;wait for CBEIF=CCIF=1 (cmnd done)
	bpl	ChkDoneE1	;loop if command buffer full (busy)
	asla	;moves CCIF to MSB (set/clear N bit)
	bpl	ChkDoneE1	;loop if CCIF=0 (not done)
	rts

;*********************************************************************
;* Erase Command - Use repeated page erase commands to erase all flash
;*  except bootloader in protected block at the end of flash, and mass
;*  erase all EEPROM locations
;*
;* Eeprom erasure assumes no protection. (Mass command will fail)
;*********************************************************************
EraseAllCmd:	bsr	EEsub	; Erase EEPROM
;
; erase flash pages from RomStart to start of protected bootloader
; no need to check for errors because we cleared them before EE erase
;
ErsBlk0:	ldab  PPAGE	; Save current PPAGE to Stack
	pshb
	; sector erase all full blocks
	ldab	#PagesBlk	; Get number of banks/blocks
	decb	; erase all but last
	stab	1,-sp	; save counter
	ldaa	#$FF	; highest bank
	sba	; Compute lowest page-1
	staa	PPAGE	; PPAGE for first 16K page of block 0
	; (passed in the A accumulator).
	clr	FCNFG	; set block select bits to 0.
ErsBlk0Lp:	ldx	#SectorSize	; select sector size
	ldd	#$4000	; Window size
	idiv	; compute total number of sectors
	tfr	x,d	; get number of sectors in B
	ldx	#Window	; point to the start of the PPAGE window.
	bsr	ErsSectors	; go erase the PPAGE window a sector at a time.
	inc	PPAGE	; go to the next PPAGE.
	dec	0,sp	; done with all full PPAGE blocks?
	bne	ErsBlk0Lp	;   no? then erase more blocks.

	ldx	#SectorSize	; select sector size
	ldd	#((BootStart-$c000))	; get size - protected amount
	idiv	; compute total number of sectors
	; minus the bootblock.
	tfr	x,d	; get number of sectors in B
	ldx	#Window	; point to the start of the PPAGE window.
	bsr	ErsSectors	; go erase the PPAGE window a sector at a time.
	pulb	; remove the page count from the stack.

; erase all sectors outside the bootblock.
;
;********************************************************************
;bulk erase all the rest
;********************************************************************
	ldab	#LowestPage	; select lowest bank
BlockLoop:	stab	PPAGE	; must match array selection
	bsr	BulkErase	; erase it
	ldab	PPAGE	;get ppage back
	addb	#PagesBlk	;
	cmpb	#($FF-PagesBlk)	; see if last block
	bmi	BlockLoop

EraseDone:	pulb
	stab	PPAGE	; Move PPAGE from Stack
OkCommand:	jmp	CommandOK	;back to no error and prompt
;
BulkErase:	pshx	;save address
	ldaa	#$FF
	staa	FPROT	;setting needed bits to 1
BE1:	ldaa	FSTAT	;is CBEIF set?
	bpl	BE1	;No, so loop until it is
	anda	#$30	;ACCERR/PVIOL set?
	beq	BE2	;No, so good to go
	ldaa	#$30
	staa	FSTAT	;Clear those bits
BE2:	ldx	#Window	;must point into bank
	std	,x	;latch address to erase
	movb	#MassErase,FCMD	; Select mass erase
	movb	#CBEIF,FSTAT	;register the command
	nop	;wait a few cycles for
	nop	;command to sync.
	nop
	nop
ChkDoneF:	ldaa   FSTAT	;wait for CBEIF=CCIF=1 (cmnd done)
	bpl	ChkDoneF	;loop if command buffer full (busy)
	asla	;moves CCIF to MSB (set/clear N bit)
	bpl	ChkDoneF	;loop if CCIF=0 (not done)
	pulx	;get address back
	rts
;Erase 'b' (accumulator) sectors beginning at address 'x' (index register)
;
ErsSectors:	exg	b,y	;put the sector count in y.
ErsSectLp:	std	,x
	movb	#SecErase,FCMD	;perform a sector erase.
	jsr	DoOnStack	;finish command from stack-based sub
	tsta	;check for 0=OK
	bne	ErsSectErr	;back to prompt-flash erase error
	leax	SectorSize,x	;point to the next sector.
	dbne	y,ErsSectLp	;continue to erase remaining sectors.
	rts

ErsSectErr:	puld	; clear stack
	bra	ErsPageErr

ErsPage:	jsr	abClr	; abort commands and clear errors
	ldab	PPAGE	; get current ppage

	lsrb	; calculate the value of the block select bits based
	lsrb	; on bits 3:2 of the PPAGE register value. (<256k)
	ldy	#SectorSize	; get high byte of size
	cpy	#$0200	; if larger than $200 shift again
	beq	ErsPage1	; otherwise skip ahead
	lsrb	; on bits 4:3 of the PPAGE register value. (512k)
ErsPage1:	comb
	andb	#$03	; mask off all but the lower 2 bits.
	stab	FCNFG	; select the block to erase.
	ldab	PPAGE	; get current ppage
	cmpb	#$FF	; is it the page with the monitor
	bne	ErsFullPage	; no then erase all of page
	ldx	#SectorSize	; select sector size
	ldd	#((BootStart-$c000))	; get size - protected amount
	idiv	; compute total number of sectors
	; minus the bootblock.
	tfr	x,d	; get number of sectors in B
	ldx	#Window	; point to the start of the PPAGE window.
	bsr	ErsSectors	; go erase the PPAGE window a sector at a time.
	bra	EraPageStat	; back to no error and prompt

ErsFullPage:	ldx	#SectorSize	; select sector size
	ldd	#$4000	; Window size
	idiv	; compute total number of sectors
	tfr	x,d	; get number of sectors in B
	ldx	#Window	; point to the start of the PPAGE window.
	bsr	ErsSectors	; go erase the PPAGE window a sector at a time.
	bra	EraPageStat	;back to no error and prompt

EraPageStat:	ldaa	FSTAT
	anda	#$30	;mask all but PVIOL or ACCERR
	bne	ErsPageErr	;back to prompt-flash error
	ldaa	#ErrNone	;code for no errors ($E0)
	jmp	Prompt	;ready for next command

ErsPageErr:	ldaa	#ErrFlash	;code for Flash error ($E6)
	jmp	Prompt	;ready for next command

;*********************************************************************
;* Read Byte Command - read specified address and return the data
;*  8-bit command code from host to SCI0 RxD
;*  16-bit address (high byte first) from host to SCI0 RxD
;*  8-bit data sent back to host through SCI0 TxD
;*********************************************************************
RdByteCmd:	jsr	getX	;get address to read from
	ldaa	,x	;read the requested location
	jsr	PutChar	;send it out SCI0
	jmp	CommandOK	;ready for next command

;*********************************************************************
;* Read Word Command - read specified block of data
;*  8-bit command code from host to SCI0 RxD
;*  16-bit address (high byte first) from host to SCI0 RxD
;*  16-bit number sent back to host through SCI0 TxD
;* Special case of block read.
;*********************************************************************
RdWordCmd:	jsr	getX	;get address to read from
sendExit:	ldd	,x	;read the requested location
	jsr	PutChar	;send it out SCI0
	tba
	jsr	PutChar	;send it out SCI0
	jmp	CommandOK	;ready for next command

;*********************************************************************
;* Read Command - read specified block of data
;*  8-bit command code from host to SCI0 RxD
;*  16-bit address (high byte first) from host to SCI0 RxD
;*  8-bit number of bytes-1 to sent back to host through SCI0 TxD
;*********************************************************************
ReadCmd:	jsr	getX	;get address to read from
	jsr	GetChar	;get number of bytes to read
	tab
	incb	;correct counter (0 is actually 1)
ReadNext:	ldaa	,x	;read the requested location
	jsr	PutChar	;send it out SCI0
	inx
	decb
	bne	ReadNext
	ldaa	#ErrNone	;code for no errors ($E0)
xPrompt:	jmp	Prompt	;ready for next command

;*********************************************************************
;* Write Command - write specified block of data
;*  8-bit command code from host to SCI0 RxD
;*  16-bit address (high byte first) from host to SCI0 RxD
;*  8-bit number of bytes-1 to write from host to SCI0 TxD
;*  8-bit values to write
;* this function used Word writes whenever possible. This is:
;* a) when more than one byte is still to write
;* b) and the address is even
;*********************************************************************
WriteCmd:	jsr	getX	;get address to write to
	jsr	GetChar	;get number of bytes to read
	tab
	incb	;correct counter (0 is actually 1)
WriteNext:	cmpb	#1	;if only one byte left
	pshb	;preserve byte counter
	beq	WriteByte	;write it 
	tfr	x,a	;is address odd
	bita	#1
	bne	WriteByte	;write a byte first

WriteWord:	jsr	GetChar	;get high byte
	tab	;save in B
	dec	,sp	;decrement byte counter (on stack)
	jsr	GetChar	;get low byte
	exg	a,b	;flip high and low byte
	jsr	WriteD2IX	;write or program data to address
	pulb	;restore byte counter	   
	bne	WriteError	;error detected
	inx	;increment target address
	bra	Write1         

WriteByte:	jsr	GetChar	;get data to write
	jsr	WriteA2IX	;write or program data to address
	pulb	;restore byte counter
	bne	WriteError	;error detected
Write1:	inx	;increment target address
	decb	;decrement byte counter 
	bne	WriteNext
	ldaa	#ErrNone	;code for no errors ($E0)             
	bra	xPrompt	;then back to prompt
                 
SkipBytes:	jsr	GetChar	;read remaining bytes                               
WriteError:	decb	;
	bne	SkipBytes
	ldaa	#ErrFlash	;code for Flash error ($E6)
WriteDone:	bra	xPrompt	;then back to prompt

;*********************************************************************
;* Read Next Command - IX=IX+2; read m(IX,IX=1) and return the data
;*  8-bit command code from host to SCI0 RxD
;*  16-bit data sent back to host through SCI0 TxD
;*  uses current value of IX from user CPU regs stack frame
;*********************************************************************
RdNextCmd:	brclr	flagReg,RunFlag,notRun	;do command if not run
	clra	;data = $00 (can't read real data)
	jsr	PutChar	;send $00 instead of read_next data
	jsr	PutChar	;send $00 instead of read_next data
	ldaa	#ErrRun	;code for run mode error
xCmnd:	jmp	Prompt	;back to prompt; run error
notRun:
	bsr	preInc	;get, pre-inc, & update user IX
	jmp	sendExit	;get data, send it, & back to prompt

;*********************************************************************
;* Write Byte Command - write specified address with specified data
;*  8-bit command code from host to SCI0 RxD
;*  16-bit address (high byte first) from host to SCI0 RxD
;*  8-bit data from host to SCI0 RxD
;*********************************************************************
WtByteCmd:	jsr	getX	;get address to write to
WriteNext2:	jsr	GetChar	;get data to write
	jsr	CheckModule
	beq	isRAMbyte
	bra	WriteByteNVM	;deny access (byte NVM access)

isRAMbyte:	staa	0,x	;write to RAM or register
	clra	;force Z=1 to indicate OK

WriteExit:	ldaa	#ErrNone	;code for no errors ($E0)
	jmp	Prompt	;ready for next command

WriteByteNVM:	ldaa	#ErrByteNVM	;code for byte NVM error ($E5)
	jmp	Prompt	;ready for next command

;*********************************************************************
;* Write Word Command - write word of data
;*  8-bit command code from host to SCI0 RxD
;*  16-bit address (high byte first) from host to SCI0 RxD
;*  16-bit value to write
;*********************************************************************
WtWordCmd:	jsr	getX	;get address to write to
	ldab  #02	;one word +1
	pshb	;save it on stack
	bra	WriteWord	;get & write data, & back to prompt

;*********************************************************************
;* Write Next Command - IX=IX+1; write specified data to m(IX)
;*  8-bit command code from host to SCI0 RxD
;*  16-bit data from host to SCI0 RxD
;*
;*  uses current value of IX from user CPU regs stack frame
;*********************************************************************
WtNextCmd:	brclr	flagReg,RunFlag,notRunW	;do command if not run
	jsr	getX	;clear data
	ldaa	#ErrRun	;code for run mode error
xCmndW:	jmp	Prompt	;back to prompt; run error

notRunW:
	bsr	preInc	;get, pre-inc, & update user IX
	ldab	#02	;one word +1
	pshb	;save it on stack
	bra	WriteWord	;get & write data, & back to prompt

;*********************************************************************
;* utility to get IX from stack frame and pre increment it by 2
;* assumes interrupts are blocked while in monitor
;*********************************************************************
preInc:	leas	2,sp
	ldx	UXreg,sp	;get user X
	inx	;pre-increment
	inx	;pre-increment
	stx	UXreg,sp	;put adjusted user X back on stack
	leas	-2,sp
	rts	;pre-incremented IX still in IX

;*********************************************************************
;* Read Registers Command - read user's CPU register values
;*
;*  16-bit SP value (high byte first) sent to host through SCI0 TxD
;*  16-bit PC value (high byte first) sent to host through SCI0 TxD
;*  16-bit IY value (high byte first) sent to host through SCI0 TxD
;*  16-bit IX value (high byte first) sent to host through SCI0 TxD
;*  16-bit D  value (high byte first) sent to host through SCI0 TxD
;*   8-bit CCR value sent to host through SCI0 TxD
;*
;* User CPU registers stack frame...
;*
;*   +0  UCch   <- Monitor's SP
;*   +1  UCcr
;*   +2  UDreg   (B:A)
;*   +4  UXreg
;*   +6  UYreg
;*   +8  UPc
;*   +10  ---     <- User's SP
;*********************************************************************
RdRegsCmd:	tsx	;IX = Monitor SP +2
	leax	SPOffset,x	;correct SP value
	jsr	put16	;send user SP out SCI0
	ldx	UPc,sp	;user PC to IX
	jsr	put16	;send user PC out SCI0
	ldx	UYreg,sp	;user IY to IX
	jsr	put16	;send user IY out SCI0
	ldx	UXreg,sp	;user IX to IX
	jsr	put16	;send user IX out SCI0
	ldx	UDreg,sp	;user D to IX
	exg	d,x
	exg	a,b	;flip as D is stacked B:A
	exg	d,x
	jsr	put16	;send user D out SCI0
	ldaa	UCcr,sp	;user CCR to A
	jsr	PutChar	;send user CCR out SCI0
	jmp	CommandOK	;back to prompt

;*********************************************************************
;* Write CCR Command - write user's CCR register value
;*  8-bit command code from host to SCI0 RxD
;*  8-bit data for CCR from host to SCI0 RxD
;*********************************************************************
WriteCcrCmd:	jsr	GetChar	;read new CCR value
	staa	UCcr,sp	;replace user CCR value
	jmp	CommandOK	;back to no error and prompt

;*********************************************************************
;* Write D Command - write user's D register value
;*  8-bit command code from host to SCI0 RxD
;*  16-bit data (high byte first) for D from host to SCI0 RxD
;*********************************************************************
WriteDCmd:	jsr	getX	;read new D value
	exg	d,x
	exg	a,b	;flip as D is stacked B:A
	exg	d,x
	stx	UDreg,sp	;replace user D value
	jmp	CommandOK	;back to no error and prompt

;*********************************************************************
;* Write IX Command - write user's IX register value
;*  8-bit command code from host to SCI0 RxD
;*  16-bit data (high byte first) for IX from host to SCI0 RxD
;*********************************************************************
WriteIXCmd:	jsr	getX	;read new IX value
	stx	UXreg,sp	;replace user IX value
	jmp	CommandOK	;back to no error and prompt

;*********************************************************************
;* Write IY Command - write user's IY register value
;*  8-bit command code from host to SCI0 RxD
;*  16-bit data (high byte first) for IY from host to SCI0 RxD
;*********************************************************************
WriteIYCmd:	jsr	getX	;read new IY value
	stx	UYreg,sp	;replace user IY value
	jmp	CommandOK	;back to no error and prompt

;*********************************************************************
;* Write PC Command - write user's PC register value
;*  8-bit command code from host to SCI0 RxD
;*  16-bit data (high byte first) for PC from host to SCI0 RxD
;*********************************************************************
WritePcCmd:	jsr	getX          ;read new PC thru SCI0 to IX
	stx	UPc,sp	;replace user PC value
	jmp	CommandOK	;back to no error and prompt

;*********************************************************************
;* Write SP Command - write user's SP register value
;*  8-bit command code from host to SCI0 RxD
;*  16-bit data (high byte first) for SP from host to SCI0 RxD
;*
;*  Since other user CPU register values are stored on the stack, the
;*  host will need to re-write the other user registers after SP is
;*  changed. This routine just changes SP itself.
;*
;*  SP value is user's SP & it is adjusted (-10) to accommodate the
;*  user CPU register stack frame.
;*
;*  If the host attempts to set the user SP value <RamStart or >RamLast
;*  then the change is ignored, because such values would not support
;*  proper execution of the monitor firmware.
;*********************************************************************
WriteSpCmd:
	bsr	getX	;new SP value now in IX
	leax	-SPOffset,x	;correct SP value
	cpx	#LowSPLimit	;check against lower limit
	blo	spBad
	cpx	#HighSPLimit	;check against upper limit
	bhi	spBad
	txs	;IX -> SP
	jmp	CommandOK	;back to no error and prompt
spBad:	ldaa	#ErrWriteSP	;error code for stack errors
;	bsr	PutChar	;send error code
	jmp	Prompt	;send status and >

;*********************************************************************
;* Trace 1 Command - trace one user instruction starting at current PC
;*  8-bit command code from host to SCI0 RxD
;*
;*  If an interrupt was already pending, the user PC will point at the
;*  ISR after the trace and the opcode at the original address will
;*  not have been executed. (Because the interrupt response is
;*  considered to be an instruction to the CPU.)
;*********************************************************************

Trace1Cmd:
	bset	flagReg,TraceFlag	;so at SWI we know it was Trace
	clr	DBGC1	;make sure DBG module disarmed, and at Comparator A
;
; NOTE: In the following code segment the mov instructions are being
; avoided since it is possible to save a byte with direct addressing
;*Set up comparators
;Comparator A: Tagged address compare to the start of monitor code
;Tag the instruction, enable comparator A
	ldd	#((TAG|COMPE)*256)+MONITOR_HI
	staa	DBGXCTL
	stab	DBGXAH
	movw	#BootStart,DBGXAM
;*Set up state sequencer
;DBGSCR1 selected for Comparator A
	clr	DBGSCRX	;Any match in state 1 -> state 2 

;Comparator B: Tagged address compare to the end of monitor code
	movb	#$01,DBGC1	;Select Comparator B for configuration
;Tag the instruction, enable comparator B
	staa	DBGXCTL
	stab	DBGXAH
	ldd	#(BootStart+$7FF)
	std	DBGXAM
;*Set up state sequencer
;DBGSCR2 selected for Comparator B
	ldaa	#$02
	staa	DBGSCRX	;Any match in state 2 -> Final State

;*Set up comparators A and B outside range check on Match 0
; A already has #2
	staa	DBGC2

;Comparator C: Disabled
; A already has #2
	staa	DBGC1	;Select Comparator C for configuration
	clr	DBGXCTL	;Comparator disabled

;Comparator D: Disabled
;Select Comparator D for configuration
	inca	;A = 3 now
	staa	DBGC1	;Select Comparator D for configuration
	clr	DBGXCTL	;Comparator disabled

;*Set up software interrupt
	ldaa	#ARM|DBGBRK
	staa	DBGC1	;ARM, S12X breakpt to be generated
	rti	; restore regs and go to user code

;*********************************************************************
;* Go Command - go to user's program at current PC address
;*  8-bit command code from host to SCI RxD
;* - no prompt is issued 
;*  typically, an SWI will cause control to pass back to the monitor
;*********************************************************************
GoCmd:	bset SCI0CR2,RIE	;need to enable SCI0 Rx interrupts to
	; enter monitor on any char received
	bclr	flagReg,TraceFlag	; run flag clr
	rti	;restore regs and exit
;*********************************************************************
;* Utility to send a 16-bit value out X through SCI
;*********************************************************************
put16:	exg	d,x	;move IX to A
	bsr	PutChar	;send high byte
	tba	;move B to A
	bsr	PutChar	;send low byte
	rts

;*********************************************************************
;* Utility to get a 16-bit value through SCI into X
;*********************************************************************
getX:
	bsr	GetChar	;get high byte
	tab	;save in B
	bsr	GetChar	;get low byte
	exg	a,b	;flip high and low byte
	exg	d,x	;16-bit value now in IX
	rts
;*********************************************************************
;* GetChar - wait indefinitely for a character to be received
;*  through SCI (until RDRF becomes set) then read char into A
;*  and return. Reading character clears RDRF. No error checking.
;*
;* Calling convention:
;*            bsr    GetChar
;*
;* Returns: received character in A
;*********************************************************************
GetChar:	brset	SCI0SR1,RDRF,RxReady	;exit loop when RDRF=1
	bra	GetChar	;loop till RDRF set
RxReady:	ldaa	SCI0DRL	;read character into A
	rts	;return

;*********************************************************************
;* PutChar - sends the character in A out SCI
;*
;* Calling convention:
;*            ldaa    data          ;character to be sent
;*            bsr    PutChar
;*
;* Returns: nothing (A unchanged)
;*********************************************************************
PutChar:	brclr	SCI0SR1,TDRE,PutChar	;wait for Tx ready
	staa	SCI0DRL	;send character from A
	rts

;*********************************************************************
;* CheckModule - check in what memory type the address in IX points to
;*  The location may be RAM, FLASH, EEPROM, or a register
;*  if the vector table is addresses, IX is changed to point to the
;*  same vector in the pseudo vector table
;*  returns in B: 1 FLASH or EEPROM
;*                0 RAM or register (all the rest of the address space)
;*               -1 access denied (monitor or pseudo vector)
;*  all registers are preserved except B
;*********************************************************************
CheckModule:	pshd	;preserve original data
	cpx	#RomStart
	blo	check4EE	;skip if not flash
	cpx	#VectorTable
	bhs	isVector	;is it in the real vector table
	cpx	#PVecTable
	blo	isToProgram	;pseudo vector table or monitor area
	ldab	#$FF	;access denied (N=1, Z=0)
	puld	;restore original data (D)
	rts

isVector:	leax	BootStart,x	;access pseudo vector table
	bra	isToProgram

check4EE:
	cpx	#EEpromStart
	blo	isRAM	;treat as RAM or registers
	cpx	#EEpromEnd	;Greater than allocated EE space?
	bhi	isRAM	;must be registers or RAM
isToProgram:	ldab	#1	;set flgs - signal FLASH (N=0, Z=0)
	puld	;restore original data (D)
	rts

isRAM:	clrb	;signal RAM  (N=0, Z=1)
	puld	;restore original data (D)
	rts

;*********************************************************************
;* WriteD2IX - Write the data in D (word) to the address in IX
;*  The location may be RAM, FLASH, EEPROM, or a register
;*  if FLASH or EEPROM, the operation is completed before return
;*  IX and A preserved, returns Z=1 (.EQ.) if OK
;*
;*********************************************************************
WriteD2IX:	pshx	;preserve original address
	pshd	;preserve original data
	bsr	CheckModule
	bmi	ExitWrite	;deny access (monitor or pseudo vector)
	beq	isRAMword
	cpd	0,x	;FLASH or EEPROM needs programming
	beq	ExitWrite	;exit (OK) if already the right data
	pshd	;temp save data to program
	tfr	x,b	;low byte of target address -> B
	bitb	#1	;is B0 = 1?
	bne	oddAdrErr	;then it's odd addr -> exit
	ldd	0,x	;$FFFF if it was erased
	cpd	#$FFFF	;Z=1 if location was erased first
oddAdrErr:	puld	;recover data, don't change CCR
	bne	ExitWrite	;exit w/ Z=0 to indicate error
	bra	DoProgram

isRAMword:	std	0,x	;write to RAM or register
	clra	;force Z=1 to indicate OK
	bra	ExitWrite

;*********************************************************************
;* WriteA2IX - Write the data in A (byte) to the address in IX
;*  The location may be RAM, FLASH, EEPROM, or a register
;*  if FLASH or EEPROM, the operation is completed before return
;*  IX and A preserved, returns Z=1 (.EQ.) if OK
;*
;* Note: Byte writing to the FLASH and EEPROM arrays is a violation
;*       of the HC9S12 specification. Doing so, will reduce long term
;*       data retention and available prog / erase cycles
;*
;*********************************************************************

WriteA2IX:	pshx	;preserve original address
	pshd	;preserve original data
	bsr	CheckModule
	bmi	ExitWrite	;deny access (monitor or pseudo vector)
	beq	isWRAMbyte      
	cmpa	0,x	;FLASH or EEPROM needs programming 
	beq	ExitWrite	;exit (OK) if already the right data
	ldab	0,x	;$FF if it was erased
	incb	;Z=1 if location was erased first
	bne	ExitWrite	;exit w/ Z=0 to indicate error
	tfr	x,b	;test least significant bit
	bitb	#1	;is B0 = 1?
	bne	isOddAdr	;then it's odd addr.             
isEvenAdr:	ldab	1,x	;low byte of D (A:B) from memory
	bra	DoProgram                     
isOddAdr:	tab	;move to low byte of D (A:B)
	dex	;point to even byte
	ldaa	,x	;high byte of D (A:B) from memory  
	bra	DoProgram

isWRAMbyte:	staa	0,x	;write to RAM or register
	clra	;force Z=1 to indicate OK
	bra	ExitWrite 

; Programs D to IX in either FLASH or EEPROM
DoProgram:
	bsr	abClr	;abort commands and clear errors
	cpx	#RomStart	;simple test only
	blo	itsEE	; details already verified
	bsr	ProgFlash	;program the requested location
	bra	ExitWrite	;exit (Z indicates good or bad)
itsEE:
	bsr	ProgEE	;program the requested location
; exit Write?2IX functions (Z indicates good or bad)
ExitWrite:	puld	;restore original data (D)
	pulx	;restore original address (IX)
	rts

;*********************************************************************
;* Progee - program a single word in the S12X EEPROM
;*  the location is assumed to be previously erased. This routine
;*  waits for the command to complete.
;*
;* On entry... IX - points at the EEPROM address to be programmed
;*  A - holds the data value to be programmed
;*
;* Calling convention:
;*           bsr    Prog1ee
;*
;* Returns: IX unchanged and A = ESTAT shifted left by 2 bits
;*  Z=1 if OK, Z=0 if protect violation or access error
;*********************************************************************
ProgEE:	std	,x	;latch address & data to program
	ldaa	#ProgWord	;Select program word command
	staa	ECMD	;issue word program command
	ldaa	#CBEIF
	staa	ESTAT	;[pwpp] register command
	nop	;[p]
	nop
	nop
ChkDoneEE:	ldaa	ESTAT	;[prpp] (min 4~ before 1st read)
	anda	#$C0	; mask all but 2 msb
	lsla	;CCIF now in MSB
	bpl	ChkDoneEE	;wait for queued commands to finish
	asla	;A=00 & Z=1 unless PVIOL or ACCERR
xProgEE:	rts
;
; utility sub to abort previous commands in flash and EEPROM
; and clear any pending errors
;
abClr:	psha
	ldaa	#PVIOL+ACCERR	;mask
	staa	ESTAT	;abort any command and clear errors
	staa	FSTAT	;abort any command and clear errors
	pula
	rts

;*********************************************************************
;* Progflash - programs one word of S12X FLASH
;*  This routine waits for the command to complete before returning.
;*  assumes location was blank. This routine can be run from FLASH
;*
;* On entry... IX - points at the FLASH byte to be programmed
;*             D holds the data for the location to be programmed
;*
;* Calling convention:
;*           bsr    Prog1flash
;*
;* Uses: DoOnStack which uses SpSub
;* Returns: IX unchanged and A = FSTAT bits PVIOL and ACCERR only
;*  Z=1 if OK, Z=0 if protect violation or access error
;*********************************************************************
ProgFlash:	ldy	PPAGE	; Move PPAGE to Stack for storage
	pshy
	pshd
	cpx	#$8000	; if <$8000 then bank $FD
	blo	its3E	;set ppage to $FD
	cpx	#$C000	; if > $BFFF then bank $FF
	blo	ProgFlash1	;else use ppage value
	movb	#$FF,PPAGE	;set ppage $FF
	bra	ProgFlash1
its3E:	movb	#$FD,PPAGE	;set ppage $FD

ProgFlash1:	ldab	PPAGE
	beq	ppagezero
ProgFlash2:	lsrb	; calculate the value of the block select bits based
	lsrb	; on bits 3:2 of the PPAGE register value. (<256k)
	ldy	#SectorSize	; get high byte of size
	cpy	#$0200	; if larger than $200 shift again
	beq	nBlockLoopb
	lsrb	; on bits 4:3 of the PPAGE register value. (512k)

nBlockLoopb:	comb
	andb	#$03	; mask off all but the lower 2 bits.
	stab	FCNFG	; select the block to program.
	cmpb	#$00	; if block zero use DoOnStack method
	puld
	beq	ProgFlashSP

ProgFlshRom:	std	,x	;latch address & data to program
	ldaa	#ProgWord	;Select program word command
	staa	FCMD	;issue byte program command
	ldaa	#CBEIF
	bsr	SpSub	;register command & wait to finish
	ldaa	FSTAT
	anda	#$30	;mask all but PVIOL or ACCERR
	tfr	ccr,b	;get copy of ccr
	puly
	sty	PPAGE	; Move PPAGE from Stack
	tfr	b,ccr	;restore ccr and int condition
	rts	; Z=1 if OK, Z=0 if error

ppagezero:	ldab	#LowestPage	;PPAGE was zero (reset state)
	stab	PPAGE	; Then set to lowest page.
	bra	ProgFlash2

ProgFlashSP:	std	,x	;latch address and data
	ldaa	#ProgWord	;Select program word command
	staa	FCMD	;issue byte program command
	jsr	DoOnStack;
	tfr	ccr,b	;get copy of ccr
	puly
	sty	PPAGE	;	 Move PPAGE from Stack
	tfr	b,ccr	;restore ccr and int condition
	rts	; Z=1 if OK, Z=0 if error

; DoOnStack will register the command then wait for it to finish
;  in this unusual case where DoOnStack is the next thing in program
;  memory, we don't need to call it. The rts at the end of DoOnStack
;  will return to the code that called Prog1flash.
;
;*********************************************************************
;* DoOnStack - copy SpSub onto stack and call it (see also SpSub)
;*  De-allocates the stack space used by SpSub after returning from it.
;*  Allows final steps in a flash prog/erase command to execute out
;*  of RAM (on stack) while flash is out of the memory map
;*  This routine can be used for flash word-program or erase commands
;*
;* Calling Convention:
;*           bsr    DoOnStack
;*
;* Uses 22 bytes on stack + 2 bytes if BSR/bsr used to call it
;* returns IX unchanged
;********************************************************************
DoOnStack:
	pshx	;save IX
	ldx	#SpSubEnd-2	;point at last word to move to stack

SpmoveLoop:
	ldd	2,x-	;read from flash
	pshd	;move onto stack
	cpx	#SpSub-2	;past end?
	bne	SpmoveLoop	;loop till whole sub on stack
	tfr	sp,x	;point to sub on stack
	ldaa	#CBEIF	;preload mask to register command
	jsr	,x	;execute the sub on the stack
	leas	SpSubEnd-SpSub,sp	;de-allocate space used by sub
	ldaa	FSTAT	;get result of operation
	anda	#$30	;and mask all but PVIOL or ACCERR
	pulx	;restore IX
	rts	;to flash where DoOnStack was called

;*********************************************************************
;* SpSub - register flash command and wait for Flash CCIF
;*  this subroutine is copied onto the stack before executing
;*  because you can't execute out of flash while a flash command is
;*  in progress (see DoOnStack to see how this is used)
;*
;* Uses 18 bytes on stack + 2 bytes if a BSR/bsr calls it
;*********************************************************************
EVEN:	;Make code start word aligned
SpSub:       
	tfr	ccr,b	;get copy of ccr
	orcc	#$10	;disable interrupts
	staa	FSTAT	;[PwO] register command
	nop	;[O] wait min 4~ from w cycle to r
	nop	;[O]
	nop	;[O]
	brclr	FSTAT,CCIF,*	;[rfPPP] wait for queued commands to finish
	tfr	b,ccr	;restore ccr and int condition
	rts	;back into DoOnStack in flash
SpSubEnd:

;*********************************************************************
;* SPURISRHandler this routine checks for unprogrammed Spurious 
;*  interrupt vector and returns an $E3 error code if execution of an
;*  unprogrammed vector is attempted
;*********************************************************************

SPURISRHandler:
	pulx	;pull bsr return address off stack
	ldy	$F710	;Location of User's Spurious ISR vector
	cpy	#$FFFF
	beq	SPURBadVector
	jmp	,Y

;*********************************************************************
;* Invalid (erased) vector fetched
;*   low byte of vector address is Status (passed in B to ReenterMon)
;* Leave stack frame from ISR on stack to refresh monitor registers.
;*********************************************************************

SPURBadVector:	;$F710 = Location of User's Spurious ISR vector
	ldab	#$10	;low byte of vector address in B
	jmp	ReenterMon	;and enter monitor

;*********************************************************************
;* User Pseudo-vector Equates (just before protected block)
;*  real vectors point here, each pseudo-vector is a bra instruction
;*  to the user's ISR.
;*********************************************************************
	org	$FDE6	;Keep exact addressing of previous version
XGATEISRTable:
uvector119:	bsr	SPURISRHandler	; vector 119 - Spurious Interrupt
uvector118:	;bsr	XGATEISRHandler	; vector 118 - System Call
uvector117:	;bsr	XGATEISRHandler	; vector 117 - MPU Access Error
uvector116:	;bsr	XGATEISRHandler	; vector 116 - XGATE software error
uvector115:	;bsr    XGATEISRHandler   ; /* vector 115*/
uvector114:	;bsr    XGATEISRHandler    ; /* vector 114*/
uvector113:	;bsr    XGATEISRHandler    ; /* vector 113*/
uvector112:	;bsr    XGATEISRHandler    ; /* vector 112*/
uvector111:	;bsr    XGATEISRHandler    ; /* vector 111*/
uvector110:	;bsr    XGATEISRHandler   ; /* vector 110*/
uvector109:	;bsr    XGATEISRHandler    ; /* vector 109*/
uvector108:	;bsr    XGATEISRHandler    ; /* vector 108*/
uvector107:	;bsr    XGATEISRHandler    ; /* vector 107*/
uvector106:	;bsr    XGATEISRHandler    ; /* vector 106*/
uvector105:	;bsr    XGATEISRHandler    ; /* vector 105*/
uvector104:	;bsr    XGATEISRHandler    ; /* vector 104*/
uvector103:	;bsr    XGATEISRHandler    ; /* vector 103*/
uvector102:	;bsr    XGATEISRHandler    ; /* vector 102*/
uvector101:	;bsr    XGATEISRHandler    ; /* vector 101*/
uvector100:	;bsr    XGATEISRHandler   ; /* vector 100*/
uvector99:	;bsr    XGATEISRHandler    ; /* vector 99 */
uvector98:	;bsr    XGATEISRHandler   ; /* vector 98 */
uvector97:	;bsr	XGATEISRHandler	; vector 97 - ATD1 Compare
uvector96:	;bsr	XGATEISRHandler	; vector 96 - ATD0 Compare
uvector95:	;bsr	XGATEISRHandler	; vector 95 - TIM Pulse Acc Input Edge
uvector94:	;bsr	XGATEISRHandler	; vector 94 - TIM Pulse Acc A Overflow
uvector93:	;bsr	XGATEISRHandler	; vector 93 - TIM overflow
uvector92:	;bsr	XGATEISRHandler	; vector 92 - TIM7
uvector91:	;bsr	XGATEISRHandler	; vector 91 - TIM6
uvector90:	;bsr	XGATEISRHandler	; vector 90 - TIM5
uvector89:	;bsr	XGATEISRHandler	; vector 89 - TIM4
uvector88:	;bsr	XGATEISRHandler	; vector 88 - TIM3
uvector87:	;bsr	XGATEISRHandler	; vector 87 - TIM2
uvector86:	;bsr	XGATEISRHandler	; vector 86 - TIM1
uvector85:	;bsr	XGATEISRHandler	; vector 85 - TIM0
uvector84:	;bsr	XGATEISRHandler	; vector 84 - SCI7
uvector83:	;bsr	XGATEISRHandler	; vector 83 - PIT 7
uvector82:	;bsr	XGATEISRHandler	; vector 82 - PIT 6
uvector81:	;bsr	XGATEISRHandler	; vector 81 - PIT 5
uvector80:	;bsr	XGATEISRHandler	; vector 80 - PIT 4
uvector79:	bsr	XGATEISRHandler	; vector 79 - RAM access violation
uvector78:	bsr	XGATEISRHandler	; vector 78 - XGATE SW err interrupt
uvector77:	bsr	XGATEISRHandler	; vector 77 - XGATE SW trigger 7
uvector76:	bsr	XGATEISRHandler	; vector 76 - XGATE SW trigger 6
uvector75:	bsr	XGATEISRHandler	; vector 75 - XGATE SW trigger 5
uvector74:	bsr	XGATEISRHandler	; vector 74 - XGATE SW trigger 4
uvector73:	bsr	XGATEISRHandler	; vector 73 - XGATE SW trigger 3
uvector72:	bsr	XGATEISRHandler	; vector 72 - XGATE SW trigger 2
uvector71:	bsr	XGATEISRHandler	; vector 71 - XGATE SW trigger 1
uvector70:	bsr	XGATEISRHandler	; vector 70 - XGATE SW trigger 0
uvector69:	bsr	XGATEISRHandler	; vector 69 - PIT 3 (Periodic Interrupt Timer)
uvector68:	bsr	XGATEISRHandler	; vector 68 - PIT 2
uvector67:	bsr	XGATEISRHandler	; vector 67 - PIT 1
uvector66:	bsr	XGATEISRHandler	; vector 66 - PIT 0
uvector65:	bsr	XGATEISRHandler	; vector 65 - Reserved
uvector64:	bsr	XGATEISRHandler	; vector 64 - Autonomous Periodical Interrupt
 
;*********************************************************************
;* XGATEISRHandler this routine checks for unprogrammed interrupt
;*  vectors and returns an $E3 error code if execution of an
;*  unprogrammed vector is attempted
;*********************************************************************

XGATEISRHandler:
	pulx	;pull bsr return address off stack
	ldy	((PVecTable+$50-XGATEISRTable)-2),X
	cpy	#$FFFF
	beq	XGATEBadVector
	jmp	,Y

;*********************************************************************
;* Invalid (erased) vector fetched
;*   low byte of vector address is Status (passed in B to ReenterMon)
;* Leave stack frame from ISR on stack to refresh monitor registers.
;*********************************************************************

XGATEBadVector:	leax	((PVecTable+$50-XGATEISRTable)-2),X
	xgdx	;low byte of vector address in B
	jmp	ReenterMon	;and enter monitor


ISRTable:
uvector63:	bsr	ISRHandler	; vector 63 - Low voltage interrupt
uvector62:	bsr	ISRHandler	; vector 62 - IIC1 bus
uvector61:	bsr	ISRHandler	; vector 61 - SCI5
uvector60:	bsr	ISRHandler	; vector 60 - SCI4
uvector59:	bsr	ISRHandler	; vector 59 - SCI3
uvector58:	bsr	ISRHandler	; vector 58 - SCI2
uvector57:	bsr	ISRHandler	; vector 57 - PWM emer shutdown
uvector56:	bsr	ISRHandler	; vector 56 - Port P
uvector55:	bsr	ISRHandler	; vector 55 - CAN4 transmit
uvector54:	bsr	ISRHandler	; vector 54 - CAN4 receive
uvector53:	bsr	ISRHandler	; vector 53 - CAN4 errors
uvector52:	bsr	ISRHandler	; vector 52 - CAN4 wake-up
uvector51:	bsr	ISRHandler	; vector 51 - CAN3 transmit
uvector50:	bsr	ISRHandler	; vector 50 - CAN3 receive
uvector49:	bsr	ISRHandler	; vector 49 - CAN3 errors
uvector48:	bsr	ISRHandler	; vector 48 - CAN3 wake-up
uvector47:	bsr	ISRHandler	; vector 47 - CAN2 transmit
uvector46:	bsr	ISRHandler	; vector 46 - CAN2 receive
uvector45:	bsr	ISRHandler	; vector 45 - CAN2 errors
uvector44:	bsr	ISRHandler	; vector 44 - CAN2 wake-up
uvector43:	bsr	ISRHandler	; vector 43 - CAN1 transmit
uvector42:	bsr	ISRHandler	; vector 42 - CAN1 receive
uvector41:	bsr	ISRHandler	; vector 41 - CAN1 errors
uvector40:	bsr	ISRHandler	; vector 40 - CAN1 wake-up
uvector39:	bsr	ISRHandler	; vector 39 - CAN0 transmit
uvector38:	bsr	ISRHandler	; vector 38 - CAN0 receive
uvector37:	bsr	ISRHandler	; vector 37 - CAN0 errors
uvector36:	bsr	ISRHandler	; vector 36 - CAN0 wake-up
uvector35:	bsr	ISRHandler	; vector 35 - FLASH
uvector34:	bsr	ISRHandler	; vector 34 - EEPROM
uvector33:	bsr	ISRHandler	; vector 33 - SPI2
uvector32:	bsr	ISRHandler	; vector 32 - SPI1
uvector31:	bsr	ISRHandler	; vector 31 - IIC0 bus
uvector30:	bsr	ISRHandler	; vector 30 - Reserved
uvector29:	bsr	ISRHandler	; vector 29 - CRG self clock mode
uvector28:	bsr	ISRHandler	; vector 28 - CRG PLL lock
uvector27:	bsr	ISRHandler	; vector 27 - Pulse Acc B overflow
uvector26:	bsr	ISRHandler	; vector 26 - Mod down counter underflow
uvector25:	bsr	ISRHandler	; vector 25 - Port H
uvector24:	bsr	ISRHandler	; vector 24 - Port J
uvector23:	bsr	ISRHandler	; vector 23 - ATD1
uvector22:	bsr	ISRHandler	; vector 22 - ATD0
uvector21:	bsr	ISRHandler	; vector 21 - SCI1
uvector20:	bsr	ISRHandler	; SCI0, used for monitor (place holder only)
uvector19:	bsr	ISRHandler	; vector 19 - SPI0
uvector18:	bsr	ISRHandler	; vector 18 - Pulse Acc input edge
uvector17:	bsr	ISRHandler	; vector 17 - Pulse Acc A overflow
uvector16:	bsr	ISRHandler	; vector 16 - ECT overflow 
uvector15:	bsr	ISRHandler	; vector 15 - ECT 7
uvector14:	bsr	ISRHandler	; vector 14 - ECT 6
uvector13:	bsr	ISRHandler	; vector 13 - ECT 5
uvector12:	bsr	ISRHandler	; vector 12 - ECT 4
uvector11:	bsr	ISRHandler	; vector 11 - ECT 3
uvector10:	bsr	ISRHandler	; vector 10 - ECT 2
uvector09:	bsr	ISRHandler	; vector 09 - ECT 1
uvector08:	bsr	ISRHandler	; vector 08 - ECT 0
uvector07:	bsr	ISRHandler	; vector 07 - Real Time Interrupt
uvector06:	bsr	ISRHandler	; vector 06 - IRQ
uvector05:	bsr	ISRHandler	; vector 05 - XIRQ
	bsr	ISRHandler	; SWI, used for breakpoints (place holder only)
;* The last three are fixed at the top of Flash
uvector03:	bsr	ISRHandler	; vector 03 - Unimplemented instruction trap
uvector02:	bsr	ISRHandler	; vector 02 - COP watchdog reset
uvector01:	bsr	ISRHandler	; vector 01 - Clock monitor reset
	bsr	ISRHandler	; /* Reset vector */

;*********************************************************************
;* ISRHandler this routine checks for unprogrammed interrupt
;*  vectors and returns an $E3 error code if execution of an
;*  unprogrammed vector is attempted
;*********************************************************************

ISRHandler:
	pulx	;pull bsr return address off stack
	ldy	((PVecTable+$70-ISRTable)-2),X
	cpy	#$FFFF
	beq	BadVector
	jmp	,Y

;*********************************************************************
;* Invalid (erased) vector fetched
;*   low byte of vector address is Status (passed in B to ReenterMon)
;* Leave stack frame from ISR on stack to refresh monitor registers.
;*********************************************************************

BadVector:	leax	((PVecTable+$70-ISRTable)-2),X
	xgdx	;low byte of vector address in B
	jmp	ReenterMon	;and enter monitor

EndMonitor:

  IF EndMonitor >= FProtStart
    FAIL "Monitor code overflows into protection/security area"
  ENDIF

;*********************************************************************
;* Jump table for external use of routines.
;*********************************************************************
	org	FProtStart-26	;immediately before the vector table
	jmp	PutChar
	jmp	GetChar
	jmp	EraseAllCmd
	jmp	DoOnStack
	jmp	WriteD2IX

	org	FProtStart-8
	fdb	softwareID4	;Software device type (deviceID)
	fdb	softwareID1	;Software revision (date)
	fdb	softwareID2	;Software revision (year)
	fdb	softwareID3	;Software revision (ver)
;
;*********************************************************************
;* FLASH configuration: protection, security
;*********************************************************************
	org	FProtStart	; enable protection
	fdb	$FFFF	; Skip Backdoor Key
	fdb	$FFFF	; Skip Backdoor Key
	fdb	$FFFF	; Skip Backdoor Key
	fdb	$FFFF	; Skip Backdoor Key

	fdb	$FFFF	; Skip Reserved

	fcb	$FF	; protection block 3
	fcb	$FF	; protection block 2
	fcb	$FF	; protection block 1
	fcb	FProtBlksz	; protection block 0
	fcb	$FF	; Default value for COPCTL
	fcb	FSecure	; set security and backdoor access
;*********************************************************************
;* Define all vectors even if program doesn't use them all
;*********************************************************************

	org	VectorTable
vector119:	fdb	uvector119	; vector 119 - Spurious Interrupt
vector118:	fdb	uvector118	; vector 118 - System Call
vector117:	fdb	uvector117	; vector 117 - MPU Access Error
vector116:	fdb	uvector116	; vector 116 - XGATE software error
vector115:	fdb	$ffff	;    uvector115 ; vector 115
vector114:	fdb	$ffff	;    uvector114 ; vector 114
vector113:	fdb	$ffff	;    uvector113 ; vector 113
vector112:	fdb	$ffff	;    uvector112 ; vector 112
vector111:	fdb	$ffff	;    uvector111 ; vector 111
vector110:	fdb	$ffff	;    uvector110 ; vector 110

vector109:	fdb	$ffff	;    uvector109 ; vector 109
vector108:	fdb	$ffff	;    uvector108 ; vector 108
vector107:	fdb	$ffff	;    uvector107 ; vector 107
vector106:	fdb	$ffff	;    uvector106 ; vector 106
vector105:	fdb	$ffff	;    uvector105 ; vector 105
vector104:	fdb	$ffff	;    uvector104 ; vector 104
vector103:	fdb	$ffff	;    uvector103 ; vector 103
vector102:	fdb	$ffff	;    uvector102 ; vector 102
vector101:	fdb	$ffff	;    uvector101 ; vector 101
vector100:	fdb	$ffff	;    uvector100 ; vector 100

vector99:	fdb	$ffff	;    uvector99  ; vector 99
vector98:	fdb	$ffff	;    uvector98  ; vector 98
vector97:	fdb	uvector97	; vector 97 - ATD1 Compare
vector96:	fdb	uvector96	; vector 96 - ATD0 Compare
vector95:	fdb	uvector95	; vector 95 - TIM Pulse Acc Input Edge
vector94:	fdb	uvector94	; vector 94 - TIM Pulse Acc A Overflow
vector93:	fdb	uvector93	; vector 93 - TIM overflow
vector92:	fdb	uvector92	; vector 92 - TIM7
vector91:	fdb	uvector91	; vector 91 - TIM6
vector90:	fdb	uvector90	; vector 90 - TIM5
vector89:	fdb	uvector89	; vector 89 - TIM4
vector88:	fdb	uvector88	; vector 88 - TIM3
vector87:	fdb	uvector87	; vector 87 - TIM2
vector86:	fdb	uvector86	; vector 86 - TIM1
vector85:	fdb	uvector85	; vector 85 - TIM0
vector84:	fdb	uvector84	; vector 84 - SCI7
vector83:	fdb	uvector83	; vector 83 - PIT 7
vector82:	fdb	uvector82	; vector 82 - PIT 6
vector81:	fdb	uvector81	; vector 81 - PIT 5
vector80:	fdb	uvector80	; vector 80 - PIT 4
vector79:	fdb	uvector79	; vector 79 - RAM access violation
vector78:	fdb	uvector78	; vector 78 - XGATE SW err interrupt
vector77:	fdb	uvector77	; vector 77 - XGATE SW trigger 7
vector76:	fdb	uvector76	; vector 76 - XGATE SW trigger 6
vector75:	fdb	uvector75	; vector 75 - XGATE SW trigger 5
vector74:	fdb	uvector74	; vector 74 - XGATE SW trigger 4
vector73:	fdb	uvector73	; vector 73 - XGATE SW trigger 3
vector72:	fdb	uvector72	; vector 72 - XGATE SW trigger 2
vector71:	fdb	uvector71	; vector 71 - XGATE SW trigger 1
vector70:	fdb	uvector70	; vector 70 - XGATE SW trigger 0
vector69:	fdb	uvector69	; vector 69 - PIT 3 (Periodic Interrupt Timer)
vector68:	fdb	uvector68	; vector 68 - PIT 2
vector67:	fdb	uvector67	; vector 67 - PIT 1
vector66:	fdb	uvector66	; vector 66 - PIT 0
vector65:	fdb	uvector65	; vector 65 - Reserved - Hi Temp
vector64:	fdb	uvector64	; vector 64 - Autonomous Periodical Interrupt

vector63:	fdb	uvector63	; vector 63 - Low voltage interrupt
vector62:	fdb	uvector62	; vector 62 - IIC1 bus
vector61:	fdb	uvector61	; vector 61 - SCI5
vector60:	fdb	uvector60	; vector 60 - SCI4
vector59:	fdb	uvector59	; vector 59 - SCI3
vector58:	fdb	uvector58	; vector 58 - SCI2
vector57:	fdb	uvector57	; vector 57 - PWM emer shutdown
vector56:	fdb	uvector56	; vector 56 - Port P 
vector55:	fdb	uvector55	; vector 55 - CAN4 transmit
vector54:	fdb	uvector54	; vector 54 - CAN4 receive
vector53:	fdb	uvector53	; vector 53 - CAN4 errors
vector52:	fdb	uvector52	; vector 52 - CAN4 wake-up
vector51:	fdb	uvector51	; vector 51 - CAN3 transmit
vector50:	fdb	uvector50	; vector 50 - CAN3 receive
vector49:	fdb	uvector49	; vector 49 - CAN3 errors
vector48:	fdb	uvector48	; vector 48 - CAN3 wake-up
vector47:	fdb	uvector47 	; vector 47 - CAN2 transmit
vector46:	fdb	uvector46	; vector 46 - CAN2 receive
vector45:	fdb	uvector45	; vector 45 - CAN2 errors
vector44:	fdb	uvector44	; vector 44 - CAN2 wake-up
vector43:	fdb	uvector43	; vector 43 - CAN1 transmit
vector42:	fdb	uvector42	; vector 42 - CAN1 receive
vector41:	fdb	uvector41	; vector 41 - CAN1 errors
vector40:	fdb	uvector40	; vector 40 - CAN1 wake-up
vector39:	fdb	uvector39	; vector 39 - CAN0 transmit
vector38:	fdb	uvector38	; vector 38 - CAN0 receive
vector37:	fdb	uvector37	; vector 37 - CAN0 errors
vector36:	fdb	uvector36	; vector 36 - CAN0 wake-up
vector35:	fdb	uvector35	; vector 35 - FLASH
vector34:	fdb	uvector34	; vector 34 - EEPROM/Flash Fault
vector33:	fdb	uvector33	; vector 33 - SPI2
vector32:	fdb	uvector32	; vector 32 - SPI1
vector31:	fdb	uvector31	; vector 31 - IIC0 bus
vector30:	fdb	uvector30	; vector 30 - Reserved - SCI6
vector29:	fdb	uvector29	; vector 29 - CRG self clock mode
vector28:	fdb	uvector28	; vector 28 - CRG PLL lock
vector27:	fdb	uvector27	; vector 27 - Pulse Acc B overflow
vector26:	fdb	uvector26	; vector 26 - Mod down counter underflow
vector25:	fdb	uvector25	; vector 25 - Port H
vector24:	fdb	uvector24	; vector 24 - Port J
vector23:	fdb	uvector23	; vector 23 - ATD1
vector22:	fdb	uvector22	; vector 22 - ATD0

;vector21:	fdb	SciIsr	; vector 21 - SCI1 comment out for SCI0
vector21:	fdb	uvector21	; vector 21 - SCI1 comment out for SCI1

vector20:	fdb	SciIsr	; vector 20 - SCI0 comment for SCI1
;vector20:	fdb	uvector20	; vector 20 - SCI0 comment out for SCI0

vector19:	fdb	uvector19	; vector 19 - SPI0
vector18:	fdb	uvector18	; vector 18 - Pulse Acc input edge
vector17:	fdb	uvector17	; vector 17 - Pulse Acc A overflow
vector16:	fdb	uvector16	; vector 16 - ECT overflow 
vector15:	fdb	uvector15	; vector 15 - ECT 7
vector14:	fdb	uvector14	; vector 14 - ECT 6
vector13:	fdb	uvector13	; vector 13 - ECT 5
vector12:	fdb	uvector12	; vector 12 - ECT 4
vector11:	fdb	uvector11	; vector 11 - ECT 3
vector10:	fdb	uvector10	; vector 10 - ECT 2
vector09:	fdb	uvector09	; vector 09 - ECT 1
vector08:	fdb	uvector08	; vector 08 - ECT 0
vector07:	fdb	uvector07	; vector 07 - Real Time Interrupt
vector06:	fdb	uvector06	; vector 06 - IRQ
vector05:	fdb	uvector05	; vector 05 - XIRQ
vector04:	fdb	Breakpoint	; vector 04 - SWI = Breakpoint
vector03:	fdb	uvector03	; vector 03 - Unimplemented instruction trap
;* The last three are fixed at the top of Flash
vector02:	fdb	ColdStart	; vector 02 - COP watchdog reset
vector01:	fdb	uvector01	; vector 01 - Clock monitor reset
vector00:	fdb	ColdStart	; Reset vector


.nolist	;skip the symbol table

;*****************************************************************

