; FUJI INSTAX BACK MCU ASSEMBLY CODE
; INTEL 8052 INSTRUCTIONS
; DESIGNED FOR STC8F2K08 MCU CHIP
; ASSEMBLE USING SDAS8051 ASSEMBLER
; ZENING LU
; MAY 4, 2019
; CHIP: STC8F2K08S2
; FREQ: 4MHz
.MODULE INSTAX_BACK
; STC51 REGISTERS
AUXR	.EQU	0x8E
T2L		.EQU	0xD7
T2H		.EQU	0xD6
P4	.EQU	0xE8
P4_0	.EQU	0xE8
P5	.EQU	0xC8
P5.4	.EQU	0xCC
P1M0	.EQU	0x92
P1M1	.EQU	0x91
P2M0	.EQU	0x96
P2M1	.EQU	0x95
P3M0	.EQU	0xB2
P3M1	.EQU	0xB1
P5M0	.EQU	0xCA
P5M1	.EQU	0xC9
IAP_DATA    .EQU    0xC2
IAP_ADDRH   .EQU    0xC3
IAP_ADDRL   .EQU    0xC4
IAP_CMD     .EQU    0xC5
IAP_TRIG    .EQU    0xC6
IAP_CONTR   .EQU    0xC7
WT_4M       .EQU    0x84
VOCTRL		.EQU	0xBB
RSTCFG		.EQU	0xFF
ENLVR		.EQU	0x40
LVDF		.EQU	0x20	; PCON.5
ELVD		.EQU	IE+6
; GPIO PLANS
KEY .EQU P3.2
ROLLER .EQU P5.4 ; HIGH effective
AP .EQU P1.5
AN .EQU P1.4
BP .EQU P1.7
BN .EQU P1.6
SEG_A .EQU P1.2
SEG_B .EQU P1.1
SEG_C .EQU P1.0
SEG_D .EQU P3.7
SEG_E .EQU P3.4
SEG_F .EQU P3.6 ; Common Anode
SEG_G .EQU P3.5

LVD3V0	.EQU	0x03
; GLOBAL CONSTANTS
LEN_OP .EQU 0d05
ROLLER_DURATION .EQU 0d120 ; x 100ms
STEPS_PER_ROUND .EQU 0d20
STEPPER_GEAR_RATIO .EQU 0d500;
STEPPER_STEPS_INIT .EQU STEPS_PER_ROUND * STEPPER_GEAR_RATIO
STEPPER_CALI_STEPLEN .EQU STEPS_PER_ROUND * STEPPER_GEAR_RATIO / 100

.area SAVEDVAR (ABS, DATA)
FILM_CNT .EQU 0x7F ; film counter 0-9
STEP_CNT .EQU 0x7D ; stepper steps countdown 2 bytes BIG ENDIAN
STEP_DIR .EQU 0x7C ; direction -1:backward 1: forward
STEP_STATE .EQU 0x7B ; 0: AB,1:A-B ,2:A-B-,3:AB- 4-cycle driving
STEPPER_ROLLBACK .EQU 0x7A ; 0: forward 1: backward
ROLLER_CNT .EQU 0x79	; Roller remain count 2 bytes BIG ENDIAN
STEPPER_STEPS	.EQU	0x77	; 2 Bytes 0x77-0x78
SAVE_VALID .EQU 0x70 ; 0: invalid saved data, 1: valid saved data
BEGIN_SAVED_VAR	.EQU	0x70 ; the initial address of SAVED VAR 

.area GLOBALVAR (ABS, DATA)
IDLE_PERIOD .EQU 0x6F ; record how many ticks the key has not been pressed
KEY_PERIOD .EQU 0x6E ; execute and reset key history with KEY_PERIOD overflows (>2s)
KEY_STATE .EQU 0x6D ; key state machine: state number 0: init 1: hold 2: transit 3: process
KEY_HLD_CNT .EQU 0x6C ; key hold time counter
KEY_RLS_CNT .EQU 0x6B ; key release time counter
KEY_N_RECORD .EQU 0x6A ; number of historical key records
KEY_HISTORY .EQU  0x65 ; 0x65-0x69 BYTE[5]
LEN_BUF_UART	.EQU	8	; UART Buffer Length
BUF_UART1	.EQU 0x5D ; UART1 RECV BUFFER 8 bytes
WPTR_UART1	.EQU 0X5C ; Write Pointer
RPTR_UART1	.EQU 0x5B	; Read Pointer
STEPPER_STEPS_CALIB .EQU 0x59 ; 2 Bytes 0x59-0x5A
IS_CALIB .EQU 0x01		; calibrating flag BIT
BUSY_UART1	.EQU 0x00	; UART1 BUSY flag BIT
SEG_PATTERN .EQU 0x2F

.area MAINCODE (ABS, CODE)
.ORG 0x00 ;START POINT
		LJMP SETUP
.ORG 0x03 ;INT0 INTERRUPT
		LJMP INTERRUPT_KEY
.ORG 0x0B ;TIMER0 INTERRUPT
		LJMP INTERRUPT_T0
.ORG 0x1B ;TIMER1 INTERRUPT
		LJMP INTERRUPT_T1
.ORG 0x23 ; UART1 INTERRUPT
		LJMP INTERRUPT_UART1
.ORG 0x33 ; LOW VOLTAGE INTERRUPT
		LJMP INTERRUPT_LVD
.ORG 0xD3 ;SETUP AND MAIN
SETUP:
		MOV SP, #0x2F ; Redirect Stack Pointer 
		CLR EA
		; Init IO Modes
		CLR P5.4 ; Pull down the motor driver 
		MOV P5M0, #0x10
		MOV P5M1, #0x00 ; P5 Strong Pull-up
		MOV P1M0, #0x00
		MOV P1M1, #0x00 ; P1 Weak Pull-up
		MOV P3M0, #0x00
		MOV P3M1, #0x00 ; P3 Weak Pull-up
		MOV P1, #0xff
		MOV P3, #0xff
		; INIT Register settings
		; Init Global Variables
		LCALL READ_DATA
		MOV A, SAVE_VALID
		CJNE A, #0xAA, VAR_INIT ; init variable if SAVE_VALID != 0xAA
		SJMP NOT_VAR_INIT
	VAR_INIT:
		MOV FILM_CNT, #0
		MOV STEP_CNT, #0
		MOV STEP_CNT+1, #0
		MOV STEP_DIR, #1
		MOV STEP_STATE, #0
		MOV STEPPER_ROLLBACK, #0
		MOV ROLLER_CNT, #0
		;MOV STEPPER_STEPS+1, #<STEPPER_STEPS_INIT
		;MOV STEPPER_STEPS, #>STEPPER_STEPS_INIT
		MOV SAVE_VALID, #0xAA
NOT_VAR_INIT:
		LCALL INIT_VARS
		; Init Timer 0 to be the main scheduler ticking at 100ms
		LCALL INIT_T0
		SETB TR0		
		SETB ET0
		LCALL INIT_T1
		CLR TR1
		CLR ET1
		CLR EX0 ; stop responding to the key 
		SETB IT0 ; INT0 falling edge interrupt
		; CLEAR LVD FLAG
		ANL PCON, #~LVDF 
		MOV RSTCFG, #0x03 ; trigger LVD at 3.0V
		MOV A, FILM_CNT
		LCALL UPDATE_SEG_DISPLAY
		SETB ELVD
		SETB EA

		
MAIN:
		SJMP .; MAIN LOOP

INIT_VARS:
		PUSH PSW
		PUSH 0x00
		PUSH 0x01
		MOV PSW, 0x00	; USING REGISTER GROUP 0
		MOV IDLE_PERIOD, #0
		MOV KEY_PERIOD, #0
		MOV KEY_STATE, #0
		MOV KEY_HLD_CNT, #0
		MOV KEY_RLS_CNT, #0
		MOV KEY_N_RECORD, #0
		; Init KEY_HISTORY array
		MOV R0, #KEY_HISTORY
		MOV R1, #5
	1$: ;For i = 1 to 5
		MOV @R0, #0
		INC R0
		DJNZ R1, 1$
		;Next i
		MOV WPTR_UART1, #0
		MOV RPTR_UART1, #0
		CLR IS_CALIB
		POP 0x01
		POP 0x00
		POP PSW
		RET


INIT_T0:	; 100ms@6MHz 
		ANL AUXR,#0x7F		
		ANL TMOD,#0xF0		
		MOV TL0,#0xB0		
		MOV TH0,#0x3C
		CLR TF0
		RET

INIT_T1: ; 300us@6MHz 
		ANL AUXR,#0xBF		
		ANL TMOD,#0x0F		
		MOV TL1,#0x6A	
		MOV TH1,#0xFF		
		CLR TF1			
		RET

INIT_UART1:	; UART1@9600bps@6MHz generating T2 
		MOV WPTR_UART1, #0
		MOV RPTR_UART1, #0
		MOV SCON,#0x50		
		ORL AUXR,#0x01
		ANL AUXR,#0xFB
		MOV T2L,#0xF3
		MOV T2H,#0xFF		
		ORL AUXR,#0x10		
		RET
		

INTERRUPT_T0:
		PUSH ACC
		PUSH PSW
		PUSH DPL
		PUSH DPH
		MOV PSW, #0x08; Using register group 1
		MOV DPTR, #STR_TICK0
		LCALL SENDSTR_UART1
; ROLLER PROCESS
		MOV A, ROLLER_CNT
		JZ IDLE_PROCESS
		DEC ROLLER_CNT
		LJMP INT_T0_DONE
IDLE_PROCESS:
		CLR ROLLER ; stop roller
		JNB IS_CALIB, 1$ ; Exit if is calibrating
		LJMP INT_T0_DONE
	1$:
		MOV A, IDLE_PERIOD
		CJNE A, #100, KEY_PROCESS
		MOV IDLE_PERIOD, #0
		LCALL POWER_DOWN
KEY_PROCESS:
		; process key press
		MOV A, KEY_STATE
		JZ KEY_STATE0
		DEC A
		JZ KEY_STATE1
		DEC A
		JZ KEY_STATE2
		DEC A
		JZ KEY_STATE3
KEY_STATE0: ; INIT STATE
		JNB KEY, 1$; BRANCH IF THE KEY IS PRESSED
		; IF KEY_PERIOD > 10 execute 
		MOV A, KEY_PERIOD
		CJNE A, #10, 0$
		MOV KEY_PERIOD, #0
		; EXECUTE KEY HISTORY
		MOV KEY_PERIOD, #0 ; Clear KEY_PERIOD COUNTER
		MOV A, KEY_N_RECORD
		JZ 0$; BRANCH IF KEY HISTORY IS VACANT
		; CHECK KEY HISTORY
		MOV DPTR, #KEYOP_TABLE ; LOAD ADDRESS OF KEY OPERATION CODE TABLE
		MOV R2, #0 ; KEY OP TABLE bias
		MOV R0, #0 ; LOOP VAR i = 0 current key op code row
	2$: ; FOR LOOP, iterate through the key op codes
		MOV A, R0
		MOV B, #LEN_KEYOP
		MUL AB ; BA = A * LEN_KEYOP
		MOV R2, A ; key op table bias = LEN_OP * i
		MOV R1, #KEY_HISTORY ; LOAD Addr OF KEY HISTORY
		MOV R3, KEY_N_RECORD ; LOOP VAR j = 5 entires per op code
	3$: ;	FOR LOOP, iterate through 5 entries of the key op code
		MOV A, R2
		MOVC A, @A+DPTR ; load key op table entry
		SUBB A, @R1 ; COMPARE TABLE ENTRY AND HISTORY ENTRY
		JNZ 20$ ; CONTINUE to next row of table IF mismatch
		INC R1 ; next history table entry
		INC R2 ; next op table entry
		DJNZ R3, 3$ ; NEXT j
		; FULL MATCH, load addr of Key OP handler Entry point
		MOV DPTR, #OP_ENT_PNTS
		MOV A, R0
		RL A
		MOV R0, A
		MOVC A, @A+DPTR ; READ Higher BYTE
		XCH A,R0
		INC A
		MOVC A, @A+DPTR
		MOV DPH, R0 ; READ Lower BYTE
		MOV DPL, A
		CLR A ; A = 0
		JMP @A+DPTR ; Jump to the op handler entry point
	20$: ; NEXT i
		INC R0
		CJNE R0, #N_KEYOPS, 2$
	0$: ; KEY EXECUTION PERIOD HAS NOT ARRIVED
		INC KEY_PERIOD ; 
		INC IDLE_PERIOD ;
		SJMP KEY_DONE
	1$: ; KEY IS PRESSED
		MOV KEY_PERIOD, #0
		MOV IDLE_PERIOD, #0
		INC KEY_STATE ; PROCEED IF THE KEY IS PRESSED
		MOV KEY_HLD_CNT, #0
		SJMP KEY_DONE
KEY_STATE1: ; HOLD STATE
		JB KEY, 1$ ; BRANCH IF THE KEY IS RELEASED
		INC KEY_HLD_CNT ; INCREMENT THE COUNTER IF THE KEY IS HOLD 
		SJMP KEY_DONE
	1$:
		INC KEY_STATE ; PROCEED TO STATE2 IF THE KEY IS RELEASED
		MOV KEY_RLS_CNT, #0
		SJMP KEY_DONE
KEY_STATE2: ; TRANSITION STATE
		INC KEY_RLS_CNT
		JNB KEY, 1$ ; BRANCHE IF THE KEY IS PRESSED
		; BRANCH IF KEY_RLS_CNT DOES NOT OVERFLOW
		MOV A, KEY_RLS_CNT
		CJNE A, #0x10, KEY_DONE ; IF KEY_RLS_CNT < 10, stay
		; IF the Counter has overflowed, proceed to process state
		INC KEY_STATE
		SJMP KEY_DONE
	1$:
		; KEY PRESSED
		DEC KEY_STATE ; Back to the previous state
		SJMP KEY_DONE
KEY_STATE3: ; PROCESS STATE
		MOV R0, KEY_HLD_CNT
		MOV KEY_HLD_CNT, #0 ; CLEAR KEY HLD CNT
		MOV A, KEY_N_RECORD
		CJNE A, #LEN_OP, 1$ ; Branch if KEY HISTORY is not full
		SJMP KEY_PROCESS_DONE ; quit if KEY HISTORY is full
	1$:
		MOV A, R0
		CLR C
		SUBB A, #10 ; KEY_HLD_CNT >= 1s
		JNC  LONG_KEY; BRANCH IF KEY_HLD_CNT >= 10
		MOV A, R0
		CLR C
		SUBB A, #1 ; KEY_HLD_CNT > 100ms
		JNC SHORT_KEY; BRANCH IF KEY_HLD_CNT == 1
		; OTHREWISE if KEY_HLD_CNT = 0
		SJMP KEY_PROCESS_DONE
LONG_KEY:
		; PROCESS KEY
		MOV A, #KEY_HISTORY
		ADD A, KEY_N_RECORD
		MOV R0, A
		MOV @R0, #2 ; RECORD LONG KEY
		INC KEY_N_RECORD
		SJMP KEY_PROCESS_DONE
SHORT_KEY:
		MOV A, #KEY_HISTORY
		ADD A, KEY_N_RECORD
		MOV R0, A
		MOV @R0, #1 ; RECORD LONG KEY
		INC KEY_N_RECORD
KEY_PROCESS_DONE: ; KEY PROCESS FINISHED
		MOV KEY_STATE, #0 ; Back to the initial state
KEY_DONE:
		SJMP INT_T0_DONE
INT_T0_DONE:
		POP DPH
		POP DPL
		POP PSW
		POP ACC
		RETI

KEYOP0: ; . DISPLAY
		MOV A, FILM_CNT
		LCALL UPDATE_SEG_DISPLAY
		LJMP INT_T0_DONE

KEYOP1: ; - PRINT
		; START ROLLER
		MOV A, ROLLER_CNT
		JNZ 0$
		MOV A, STEP_CNT
		ORL A, STEP_CNT+1
		JNZ 0$
		; Start Roller
		MOV ROLLER_CNT, #ROLLER_DURATION
		SETB ROLLER
		; Start stepper
		MOV STEP_CNT, #>STEPPER_STEPS_INIT; Higher byte
		MOV STEP_CNT+1, #<STEPPER_STEPS_INIT; Lower byte 
		MOV STEP_DIR, #1
		MOV STEPPER_ROLLBACK, #0
		LCALL ENABLE_STEPPER
		INC FILM_CNT
		MOV A, FILM_CNT
		LCALL UPDATE_SEG_DISPLAY
	0$:
		LJMP INT_T0_DONE
KEYOP2: ; CLEAR
		MOV A, #0
		MOV FILM_CNT, A
		LCALL UPDATE_SEG_DISPLAY
		LJMP INT_T0_DONE


INTERRUPT_T1: ; stepper motor pacer
		PUSH ACC
		PUSH PSW
		MOV A, STEP_CNT
		ORL A, STEP_CNT+1
		JNZ PROCEED_STEP; IF STEP_CNT != 0, PROCEED TO NEXT STEP
		; ELSE
		MOV A, STEPPER_ROLLBACK
		JNZ STEPPER_DONE ; if STEPPER_ROLLBACK == 1, quit
		; else rollback the stepper
		MOV STEP_CNT, #>STEPPER_STEPS_INIT
		MOV STEP_CNT+1, #<STEPPER_STEPS_INIT
		MOV STEPPER_ROLLBACK, #1
		XRL STEP_DIR, #0xff ; bitwise NOT
		INC STEP_DIR ;STEP_DIR = -STEP_DIR
PROCEED_STEP:
	; proceed to the next step
		MOV A, STEP_STATE
		ADD A, STEP_DIR
		; if step_state < 0, step_step = 3
		JNB A.7, 5$
		MOV A, #3
	5$: ; if step_state == 4, step_step = 0
		CJNE A, #4, 6$
		MOV A, #0
	6$:
		CLR AP
		CLR BP
		CLR AN
		CLR BN
		CJNE A, #0, 1$ ;AB
		SETB AP
		SETB BP
	1$: CJNE A, #1, 2$ ;A-B
		SETB AN
		SETB BP
	2$: CJNE A, #2, 3$ ;A-B-
		SETB AN
		SETB BN
	3$: CJNE A, #3, 4$ ;AB-
		SETB AP
		SETB BN
	4$:
		MOV STEP_STATE, A
		;DEC 2Bytes STEP_CNT
		CLR C
		MOV A, STEP_CNT+1
		SUBB A, #1
		MOV STEP_CNT+1, A
		JNC 7$ 
		DEC STEP_CNT
	7$:
		SJMP INT_T1_DONE
STEPPER_DONE:
		; quit and stop Timer 1
		CLR TR1
		LCALL DISABLE_STEPPER
INT_T1_DONE:
		POP PSW
		POP ACC
		RETI

INTERRUPT_UART1:
		PUSH ACC
		PUSH PSW
		MOV PSW, 0x10 ; Using register group 2
		JNB TI, RECV_UART1
		CLR TI
		CLR BUSY_UART1
RECV_UART1:
		JNB RI, UART1_DONE
		CLR RI
		MOV A, SBUF
		CJNE A, #'\r', 0$	; Each command ends with \r, if so jump to command process routine
		; Command ends, process routine
		MOV A, WPTR_UART1
		ADD A, #BUF_UART1
		MOV R0, A
		MOV @R0, #0	; append a zero ending to the string
		MOV R0, #BUF_UART1
		LCALL SENDMEMSTR_UART1
		MOV A, BUF_UART1
		LCALL PROC_COMM
		MOV WPTR_UART1, #0
	0$:
		MOV A, WPTR_UART1
		ADD A, #BUF_UART1
		MOV R0, A
		MOV @R0, SBUF
		INC WPTR_UART1
		ANL WPTR_UART1, #0x07 ; wrap to 0 when WPTR_UART1 >= 8
UART1_DONE:
		POP PSW
		POP ACC
		RETI

PROC_COMM:	; INPUT: A, Process commands from UART 
		PUSH DPH
		PUSH DPL
		PUSH PSW
		PUSH ACC
		; comm = 'F'	Hock forward
	1$:	CJNE A, #'F', 2$
		MOV STEP_DIR, #1
		MOV STEP_CNT, #0
		MOV STEP_CNT+1, #STEPPER_CALI_STEPLEN
		LCALL ENABLE_STEPPER
		; increament calib steps
		MOV A, STEPPER_STEPS_CALIB+1
		CLR C
		ADDC A, #STEPPER_CALI_STEPLEN
		MOV STEPPER_STEPS_CALIB+1, A
		JNC 11$
		INC STEPPER_STEPS_CALIB
		11$: ; NOT CARRY TO HIGHER BYTE
		SJMP PROC_UART1_DONE
		; comm = 'B'	Hock backward
	2$: CJNE A, #'B', 3$
		MOV STEP_DIR, #-1
		MOV STEP_CNT, #0
		MOV STEP_CNT+1, #STEPPER_CALI_STEPLEN
		LCALL ENABLE_STEPPER
		; Decrement calib steps
		MOV A, STEPPER_STEPS_CALIB+1
		CLR C
		SUBB A, #STEPPER_CALI_STEPLEN
		MOV STEPPER_STEPS_CALIB+1, A
		JNC 21$
		DEC STEPPER_STEPS_CALIB
		21$:
		SJMP PROC_UART1_DONE
		; comm = 'P'	Roller roll
	3$: CJNE A, #'P', 4$
		MOV ROLLER_CNT, #ROLLER_DURATION
		SETB ROLLER
		SJMP PROC_UART1_DONE
		; comm = 'L'	Calibrate
	4$:	CJNE A, #'L', 5$
		MOV STEPPER_STEPS_CALIB+1, #0
		MOV STEPPER_STEPS_CALIB, #0
		SETB IS_CALIB
		SJMP PROC_UART1_DONE
		; comm = 'S'	Save calibrate
	5$: CJNE A, #'S', 6$
		MOV A, STEPPER_STEPS_CALIB
		MOV STEP_CNT, A
		MOV A, STEPPER_STEPS_CALIB+1
		MOV STEP_CNT+1, A
		LCALL SAVE_DATA
		CLR IS_CALIB
		SJMP PROC_UART1_DONE
		; comm = '0' - '9' set film counter
	6$:	; if '0' <= A <= '9'
		CLR C
		SUBB A, #'0'
		JC 10$ ; if A < '0', bad command
		MOV R1, A
		SUBB A, #9
		JNC 10$ ; if A > '9', bad command
		MOV FILM_CNT, R1
		SJMP PROC_UART1_DONE
	10$:; Bad command
		MOV DPTR, #STR_BADCOMM
		LCALL SENDSTR_UART1
	PROC_UART1_DONE:
		MOV DPTR, #STR_OK
		LCALL SENDSTR_UART1
		POP ACC
		POP PSW
		POP DPL
		POP DPH
		RET

SEND_UART1:	; INPUT: ACC, OUTPUT: NONE
		JB BUSY_UART1, .
		SETB BUSY_UART1
		MOV SBUF, A
		RET

SENDSTR_UART1: ; INPUT: DPTR Addr, ends with \0
		PUSH ACC
		PUSH DPH
		PUSH DPL
	1$:
		CLR A
		MOVC A, @A+DPTR
		JZ 0$
		LCALL SEND_UART1
		INC DPTR
		SJMP 1$
	0$:
		POP DPL
		POP DPH
		POP ACC
		RET

SENDMEMSTR_UART1:	; INPUT: R0 addr, string ends with \0
		PUSH ACC
		MOV A, @R0
		JZ 0$
		LCALL SEND_UART1
		INC R0
	0$:
		POP ACC
		RET

ENABLE_STEPPER:
		SETB TR1
		SETB ET1
		RET

DISABLE_STEPPER:
		CLR TR1
		CLR ET1
		SETB AP
		SETB BP
		SETB AN
		SETB BN
		RET




UPDATE_SEG_DISPLAY: ; ARGS: A = content 
		PUSH ACC
		PUSH DPH
		PUSH DPL
		MOV DPTR, #SEG_TABLE
		MOVC A, @A+DPTR ; load seg table
		MOV SEG_PATTERN, A	; BIT ADDRESSABLE SEGMENT PATTERN
		; A
		SETB SEG_A
		JB (SEG_PATTERN-0x20)*8 + 6, 1$
		CLR SEG_A
	1$: ;B
		SETB SEG_B
		JB (SEG_PATTERN - 0x20)*8 + 5, 2$
		CLR SEG_B
	2$:	;C
		SETB SEG_C
		JB (SEG_PATTERN - 0x20)*8 + 4, 3$
		CLR SEG_C
	3$:	;D
		SETB SEG_D
		JB (SEG_PATTERN - 0x20)*8 + 3, 4$
		CLR SEG_D
	4$:	;E
		SETB SEG_E
		JB (SEG_PATTERN - 0x20)*8 + 2, 5$
		CLR SEG_E
	5$:	;F
		SETB SEG_F
		JB (SEG_PATTERN - 0x20)*8 + 1, 6$
		CLR SEG_F
	6$:	;G
		SETB SEG_G
		JB (SEG_PATTERN - 0x20)*8, 0$
		CLR SEG_G
	0$:
		POP DPL
		POP DPH
		POP ACC
		RET


DISABLE_SEG_DISPLAY:
		SETB SEG_A
		SETB SEG_B
		SETB SEG_C
		SETB SEG_D
		SETB SEG_E
		SETB SEG_F
		SETB SEG_G
		RET

POWER_DOWN:
		LCALL DISABLE_SEG_DISPLAY
		LCALL DISABLE_STEPPER
		LCALL SAVE_DATA
		MOV P0, #0xff
		MOV P1, #0xff
		MOV P2, #0xff
		MOV P3, #0xff
		MOV VOCTRL, #0x80 ; using EXT0 to wake up
		SETB EA
		SETB EX0
		NOP
		NOP
		ORL PCON, #0x02 ; Set power down flag 
		NOP
		NOP
		CLR EX0
		LCALL INIT_VARS
		RET

INTERRUPT_KEY:
		RETI

INTERRUPT_LVD:
		LCALL STOP_TASKS
		MOV A, #LOWVOLT_SYM
		LCALL UPDATE_SEG_DISPLAY	; Display Low voltage symbol
		ANL	PCON, #~LVDF
		RETI

STOP_TASKS:
		; stop roller
		MOV ROLLER_CNT, #0
		MOV ROLLER_CNT+1, #0
		CLR ROLLER
		; stop stepper
		LCALL DISABLE_STEPPER
		RET

; EEPROM IAP OPERATIONS
EEPROM_START_ADDR .EQU 0x1000
IAP_IDLE:
            MOV     IAP_CONTR,#0                
            MOV     IAP_CMD,#0                  
            MOV     IAP_TRIG,#0                 
            MOV     IAP_ADDRH,#0x80              
            MOV     IAP_ADDRL,#0
            RET

IAP_READ:	
            MOV     IAP_CONTR,#WT_4M           
            MOV     IAP_CMD,#1                  
            MOV     IAP_ADDRL,DPL               
            MOV     IAP_ADDRH,DPH               
            MOV     IAP_TRIG,#0x5A               
            MOV     IAP_TRIG,#0xA5              
            NOP     
            MOV     A,IAP_DATA                  
            LCALL   IAP_IDLE                    
            RET

IAP_PROGRAM: 
            MOV     IAP_CONTR,#WT_4M           
            MOV     IAP_CMD,#2                  
            MOV     IAP_ADDRL,DPL               
            MOV     IAP_ADDRH,DPH               
            MOV     IAP_DATA,A                  
            MOV     IAP_TRIG,#0x5A               
            MOV     IAP_TRIG,#0xA5              
            NOP     
            LCALL   IAP_IDLE                    
            RET

IAP_ERASE:
            MOV     IAP_CONTR,#WT_4M           
            MOV     IAP_CMD,#3                  
            MOV     IAP_ADDRL,DPL               
            MOV     IAP_ADDRH,DPH               
            MOV     IAP_TRIG,#0x5A               
            MOV     IAP_TRIG,#0xA5              
            NOP
            LCALL   IAP_IDLE                    
            RET

SAVE_DATA: 
		CLR EA
		PUSH ACC
		PUSH DPH
		PUSH DPL
		PUSH 0x00
		MOV DPTR, #EEPROM_START_ADDR
		LCALL IAP_ERASE
		MOV R0, #0x60
	1$:
		MOV A, @R0
		LCALL IAP_PROGRAM
		INC R0
		INC DPTR
		CJNE R0, #0x80, 1$
		POP 0x00
		POP DPL
		POP DPH
		POP ACC
		SETB EA
		RET

READ_DATA:	; Load saved variables from EEPROM
		PUSH ACC
		PUSH PSW
		PUSH 0x00	; R0
		PUSH DPH
		PUSH DPL
		MOV PSW, #0x00	; Using register set 1
		MOV DPTR, #EEPROM_START_ADDR
		MOV R0, #BEGIN_SAVED_VAR
	1$:
		LCALL IAP_READ
		MOV @R0, A
		INC DPTR
		INC R0
		CJNE R0, #0x80, 1$
		POP DPL
		POP DPH
		POP 0x00
		POP PSW
		POP ACC
		RET

STR_OK:	.strz	"OK!\r"
STR_TICK0: .strz "T0:\r"
STR_TICK1: .strz "T1:\r"
STR_BADCOMM:	.strz	"BAD COMMAND!\r"
SEG_TABLE:	; Common anode 7-segment table 
		.db 0x01, 0x4f, 0x12, 0x06, 0x4c, 0x24, 0x20, 0x0f, 0x00, 0x04 ;0-9
		.db 0x08, 0x60, 0x72, 0x42, 0x30, 0x38, 0x21, 0x48, 0x79, 0x47 ;a-j
		.db 0x58, 0x71													;K,L
LOWVOLT_SYM .EQU 21
LEN_KEYOP .EQU 5
N_KEYOPS .EQU 3
KEYOP_TABLE:
		.db 1, 0, 0, 0, 0 ; . Show counter
		.db 2, 0, 0, 0, 0 ; - Print
		.db 2, 1, 2, 1, 0 ; -.-. C Clear counter
OP_ENT_PNTS:
		.dw KEYOP0, KEYOP1, KEYOP2 ; function table for key operation handlers
