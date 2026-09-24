; FINAL PROJECT: EEBOT ROBOT GUIDANCE CHALLENGE
; COE538 Microprocessor Systems | Toronto Metropolitan University
; Developed as a group project by:
; Hamidullah Mohmmad Ali    501102715    Section 17
; Daniel Chylek            501052912    Section 17
; Shayaan Rahsedin         501249815    Section 17
; HCS12 / MC9S12C32, CodeWarrior absolute assembly, nominal 24 MHz bus.
; Calibration and timing MUST be measured on the target robot.

              XDEF  Entry, _Startup
              ABSENTRY Entry
              INCLUDE "derivative.inc"

;***************************************************************************************************
; equates section
;***************************************************************************************************
LCD_CNTR      EQU   PTJ
LCD_DAT       EQU   PORTB
LCD_E         EQU   $80
LCD_RS        EQU   $40
NULL          EQU   0
SPACE         EQU   ' '
BOW_BUMP      EQU   $04                  ; AN2, active low
REAR_BUMP     EQU   $08                  ; AN3, active low
ST_START      EQU   0
ST_FWD        EQU   1
ST_STOP       EQU   2
ST_TURN_L     EQU   3
ST_TURN_R     EQU   4
ST_REV_RECOV  EQU   5
ST_ALIGN_L    EQU   6
ST_ALIGN_R    EQU   7
ST_PROBE      EQU   8
STATE_COUNT   EQU   9
MODE_LEARN    EQU   0
MODE_RETRY    EQU   1
MODE_RETURN   EQU   2
PATH_MAX      EQU   32
NO_HEADING    EQU   $FF
EXIT_LEFT     EQU   $01
EXIT_FWD      EQU   $02
EXIT_RIGHT    EQU   $04
; Timer overflow is ~43.69 ms with a 24 MHz bus and prescale /16.
; All elapsed intervals below are <128 ticks; unsigned subtraction wraps safely.
PROBE_TICKS   EQU   3                    ; Advance from side detection to pivot centre
TURN_MIN      EQU   5                    ; Ignore original line during first 219 ms
UTURN_MIN     EQU   12                   ; Ignore original line during first 524 ms
TURN_LIMIT    EQU   70                   ; Stop if a turn fails to acquire the line
REV_TICKS     EQU   7
ALIGN_TICKS   EQU   2
JUNC_TICKS    EQU   5
LOST_TICKS    EQU   35
HOME_TICKS    EQU   12                   ; Blank line end after final return junction
HUD_TICKS     EQU   6
SETTLE_50US   EQU   100                  ; 5 ms per multiplexer channel

; Sensor calibration: adjust these values for the target robot and surface.
; Pattern detectors: dark tape is at/below CAL - THRESH (unsigned).
CAL_LINE      EQU   $9D
CAL_BOW       EQU   $CA
CAL_PORT      EQU   $CC
CAL_MID       EQU   $CA
CAL_STBD      EQU   $CC
LINE_THRESH   EQU   $18
BOW_THRESH    EQU   $30
PORT_THRESH   EQU   $20
MID_THRESH    EQU   $20
STBD_THRESH   EQU   $15

;***************************************************************************************************
; variable/data section - one contiguous RAM block, explicitly cleared at reset
;***************************************************************************************************
              ORG   $3800
RAM_BEGIN     EQU   *
ROBOT_STATE   RMB   1
TOF_COUNTER   RMB   1
STATE_TICK    RMB   1
TURN_WAIT     RMB   1
TURN_CLEAR    RMB   1
TURN_HITS     RMB   1
NAV_MODE      RMB   1
HEADING       RMB   1                    ; N=0, E=1, S=2, W=3; relative origin is arbitrary
TARGET_DIR    RMB   1
PATH_COUNT    RMB   1
START_ARMED   RMB   1
REAR_ARMED    RMB   1
JUNC_LOCK     RMB   1
JUNC_CLEAR    RMB   1
JUNC_TICK     RMB   1
EXIT_MASK     RMB   1
LOST_ACTIVE   RMB   1
LOST_TICK     RMB   1
FAULT_CODE    RMB   1                    ; 0=normal, 1=state, 2=route, 3=turn, 4=line, 5=ADC
HUD_TICK      RMB   1
HUD_PAGE      RMB   1
IR_LINE       RMB   1                    ; Physical sensor multiplexer order:
IR_BOW        RMB   1                    ; line, bow, PORT, mid, starboard
IR_PORT       RMB   1
IR_MID        RMB   1
IR_STBD       RMB   1
SENSOR_NUM    RMB   1
TAPE_BOW      RMB   1
TAPE_PORT     RMB   1
TAPE_MID      RMB   1
TAPE_STBD     RMB   1
TEMP          RMB   1
TEN_THOUS     RMB   1
THOUSANDS     RMB   1
HUNDREDS      RMB   1
TENS          RMB   1
UNITS         RMB   1
NO_BLANK      RMB   1
BCD_SPARE     RMB   2
TOP_LINE      RMB   21                   ; 20 characters plus NULL
BOT_LINE      RMB   21
PATH_IN       RMB   PATH_MAX            ; Arrival heading at each visited junction
PATH_OUT      RMB   PATH_MAX            ; Chosen/corrected departure heading
PATH_ALT      RMB   PATH_MAX            ; Other departure heading, or $FF
RAM_END       EQU   *

;***************************************************************************************************
; code section
;***************************************************************************************************
              ORG   $4000
Entry:
_Startup:
              SEI
              LDS   #$4000
              JSR   PORTS_INIT
              JSR   MOTOR_STOP
              LDX   #RAM_BEGIN
RESET_RAM     CLR   1,X+
              CPX   #RAM_END
              LBNE   RESET_RAM
              MOVB  #1,HEADING          ; Start facing nominal East
              JSR   ADC_INIT_SINGLE
              JSR   LCD_INIT
              JSR   LCD_BUF_INIT
              JSR   TOF_INIT
              CLI
MAIN          JSR   IR_LEDS_ON
              JSR   IR_READ_ALL
              JSR   IR_LEDS_OFF
              JSR   IR_CLASSIFY
              JSR   STATE_DISPATCH
              JSR   HUD_REFRESH
              LBRA   MAIN

;***************************************************************************************************
; dispatcher - one handler per call, invalid states stop the motors
;***************************************************************************************************
STATE_DISPATCH LDAA ROBOT_STATE
              CMPA  #STATE_COUNT
              LBLO   DISPATCH_VALID
              MOVB  #1,FAULT_CODE
              JMP   HALT_ROBOT
DISPATCH_VALID TAB
              LSLB
              LDX   #handlerTable
              ABX
              LDX   0,X
              JMP   0,X                 ; Handler RTS returns to MAIN's JSR
handlerTable  DC.W  DO_START,DO_FWD,DO_STOP,DO_TURN_L,DO_TURN_R
              DC.W  DO_REV_RECOV,DO_ALIGN_L,DO_ALIGN_R,DO_PROBE

;***************************************************************************************************
; movement and navigation
;***************************************************************************************************
DO_START      JSR   MOTOR_STOP
              LDAA  PORTAD0
              ANDA  #$0C
              CMPA  #$0C
              LBEQ   START_RELEASED
              MOVB  #1,START_ARMED
              RTS
START_RELEASED TST  START_ARMED          ; Press either bumper, then release to start
              LBEQ   START_EXIT
              MOVB  #1,REAR_ARMED
              MOVB  #ST_FWD,ROBOT_STATE
              JSR   MOTOR_FWD
START_EXIT    RTS

DO_STOP       JSR   MOTOR_STOP           ; Latched stop; reset/reload starts a new run
              RTS
HALT_ROBOT    MOVB  #ST_STOP,ROBOT_STATE
              JMP   MOTOR_STOP
ROUTE_FAULT   MOVB  #2,FAULT_CODE
              JMP   HALT_ROBOT

DO_FWD        BRSET PORTAD0,REAR_BUMP,FWD_REAR_RELEASED
              TST   REAR_ARMED
              LBEQ   FWD_CHECK_BOW
              CLR   REAR_ARMED
              LDAA  NAV_MODE
              CMPA  #MODE_RETURN
              LBEQ  HALT_ROBOT
              MOVB  #MODE_RETURN,NAV_MODE ; Destination tap: reverse learned route
              JMP   BEGIN_UTURN
FWD_REAR_RELEASED MOVB #1,REAR_ARMED
FWD_CHECK_BOW BRSET PORTAD0,BOW_BUMP,FWD_SENSORS
              LDAA  NAV_MODE
              CMPA  #MODE_RETURN
              LBEQ  HALT_ROBOT           ; Obstacle during return: stop
              CMPA  #MODE_RETRY
              LBEQ  ROUTE_FAULT
              TST   PATH_COUNT
              LBEQ  ROUTE_FAULT          ; No recorded junction to recover to
              MOVB  #MODE_RETRY,NAV_MODE
              MOVB  #ST_REV_RECOV,ROBOT_STATE
              MOVB  TOF_COUNTER,STATE_TICK
              JSR   MOTOR_REV
              RTS
FWD_SENSORS   JSR   JUNCTION_UNLOCK
              TST   JUNC_LOCK
              LBNE   FWD_TRACK
              LDAA  TAPE_PORT
              ORAA  TAPE_STBD
              LBEQ   FWD_TRACK
              CLR   EXIT_MASK
              TST   TAPE_PORT
              LBEQ   FWD_TEST_STBD
              BSET  EXIT_MASK,EXIT_LEFT
FWD_TEST_STBD TST   TAPE_STBD
              LBEQ   FWD_PROBE
              BSET  EXIT_MASK,EXIT_RIGHT
FWD_PROBE     MOVB  #ST_PROBE,ROBOT_STATE
              MOVB  TOF_COUNTER,STATE_TICK
              JSR   MOTOR_FWD
              RTS
; Differential line sensor is centred within CAL_LINE +/- LINE_THRESH.
; Verify steering polarity against the motor wiring during calibration.
FWD_TRACK     LDAA  TAPE_BOW
              ORAA  TAPE_MID
              LBNE   FWD_HAVE_LINE
              LDAA  TAPE_PORT
              ORAA  TAPE_STBD
              LBNE   FWD_HAVE_LINE
              TST   LOST_ACTIVE
              LBNE   FWD_LOST_WAIT
              MOVB  #1,LOST_ACTIVE
              MOVB  TOF_COUNTER,LOST_TICK
FWD_LOST_WAIT LDAB  #LOST_TICKS
              LDAA  NAV_MODE
              CMPA  #MODE_RETURN
              LBNE   FWD_LOST_CHECK
              TST   PATH_COUNT
              LBNE   FWD_LOST_CHECK
              LDAB  #HOME_TICKS
FWD_LOST_CHECK LDAA TOF_COUNTER
              SUBA  LOST_TICK
              CBA
              LBLO   FWD_KEEP_GOING
              LDAA  NAV_MODE
              CMPA  #MODE_RETURN
              LBNE   FWD_LINE_FAULT
              TST   PATH_COUNT
              LBEQ  HALT_ROBOT           ; At blank start area after last junction
FWD_LINE_FAULT MOVB #4,FAULT_CODE
              JMP   HALT_ROBOT
FWD_HAVE_LINE CLR   LOST_ACTIVE
              LDAA  IR_LINE
              CMPA  #CAL_LINE-LINE_THRESH
              LBLO   FWD_CORRECT_LEFT
              CMPA  #CAL_LINE+LINE_THRESH
              LBHI   FWD_CORRECT_RIGHT
FWD_KEEP_GOING JMP  MOTOR_FWD
FWD_CORRECT_LEFT JMP MOTOR_LEFT
FWD_CORRECT_RIGHT JMP MOTOR_RIGHT

; Require time AND three clean scans beyond a junction before recognizing another.
JUNCTION_UNLOCK TST JUNC_LOCK
              LBEQ   UNLOCK_EXIT
              LDAA  TAPE_PORT
              ORAA  TAPE_STBD
              LBNE   UNLOCK_RESET
              LDAA  TOF_COUNTER
              SUBA  JUNC_TICK
              CMPA  #JUNC_TICKS
              LBLO   UNLOCK_EXIT
              INC   JUNC_CLEAR
              LDAA  JUNC_CLEAR
              CMPA  #3
              LBLO   UNLOCK_EXIT
              CLR   JUNC_LOCK
              LBRA   UNLOCK_EXIT
UNLOCK_RESET  CLR   JUNC_CLEAR
UNLOCK_EXIT   RTS

DO_PROBE      BRCLR PORTAD0,BOW_BUMP,PROBE_BLOCKED
              JSR   MOTOR_FWD
              LDAA  TOF_COUNTER
              SUBA  STATE_TICK
              CMPA  #PROBE_TICKS
              LBLO   PROBE_EXIT
              TST   TAPE_BOW            ; Bow now samples the outgoing straight leg
              LBEQ   PROBE_DECIDE
              BSET  EXIT_MASK,EXIT_FWD
PROBE_DECIDE  JSR   MOTOR_STOP
              JSR   ROUTE_DECIDE
              LDAA  ROBOT_STATE
              CMPA  #ST_STOP
              LBEQ   PROBE_EXIT
              JMP   BEGIN_DIRECTION
PROBE_BLOCKED JMP   ROUTE_FAULT
PROBE_EXIT    RTS

; Route records use absolute headings; no maze-specific decisions are encoded.
; Designed for branching tape mazes with no more than two outgoing exits.
; Forced corners are also recorded, which makes reverse traversal symmetric.
ROUTE_DECIDE  LDAA  NAV_MODE
              CMPA  #MODE_RETURN
              LBEQ  ROUTE_RETURN
              CMPA  #MODE_RETRY
              LBEQ  ROUTE_RETRY
              LDAB  PATH_COUNT
              CMPB  #PATH_MAX
              LBHS  ROUTE_FAULT
              LDX   #PATH_IN
              ABX
              MOVB  HEADING,0,X
              LDX   #PATH_ALT
              ABX
              MOVB  #NO_HEADING,0,X
              LDAA  #NO_HEADING
              STAA  TARGET_DIR
              BRCLR EXIT_MASK,EXIT_LEFT,CHOOSE_FWD
              LDAA  HEADING
              DECA
              ANDA  #3
              JSR   ROUTE_ADD_OPTION
CHOOSE_FWD    BRCLR EXIT_MASK,EXIT_FWD,CHOOSE_RIGHT
              LDAA  HEADING
              JSR   ROUTE_ADD_OPTION
CHOOSE_RIGHT  BRCLR EXIT_MASK,EXIT_RIGHT,CHOOSE_DONE
              LDAA  HEADING
              INCA
              ANDA  #3
              JSR   ROUTE_ADD_OPTION
CHOOSE_DONE   LDAA  ROBOT_STATE
              CMPA  #ST_STOP
              LBEQ   ROUTE_EXIT
              LDAA  TARGET_DIR
              CMPA  #NO_HEADING
              LBEQ  ROUTE_FAULT
              LDAB  PATH_COUNT
              LDX   #PATH_OUT
              ABX
              STAA  0,X
              INC   PATH_COUNT
ROUTE_EXIT    RTS
; A = candidate absolute heading. Save first choice, then the sole alternative.
ROUTE_ADD_OPTION LDAB TARGET_DIR
              CMPB  #NO_HEADING
              LBNE   ADD_ALTERNATIVE
              STAA  TARGET_DIR
              RTS
ADD_ALTERNATIVE LDAB PATH_COUNT
              LDX   #PATH_ALT
              ABX
              LDAB  0,X
              CMPB  #NO_HEADING
              LBNE  ROUTE_FAULT          ; More than two exits is outside challenge
              STAA  0,X
              RTS
ROUTE_RETRY   TST   PATH_COUNT
              LBEQ  ROUTE_FAULT
              LDAB  PATH_COUNT
              DECB
              LDX   #PATH_ALT
              ABX
              LDAA  0,X
              CMPA  #NO_HEADING
              LBEQ   RETRY_BACKTRACK
              STAA  TARGET_DIR
              MOVB  #NO_HEADING,0,X
              LDX   #PATH_OUT
              ABX
              STAA  0,X                 ; Correct the failed decision in place
              MOVB  #MODE_LEARN,NAV_MODE
              JMP   VERIFY_TARGET
; A forced corner (or exhausted decision) has no alternative. Remove it and
; retrace its incoming leg until an earlier junction has an untried branch.
RETRY_BACKTRACK LDX #PATH_IN
              ABX
              LDAA  0,X
              ADDA  #2
              ANDA  #3
              STAA  TARGET_DIR
              DEC   PATH_COUNT
              JMP   VERIFY_TARGET
ROUTE_RETURN  TST   PATH_COUNT
              LBEQ  ROUTE_FAULT
              DEC   PATH_COUNT
              LDAB  PATH_COUNT
              LDX   #PATH_IN
              ABX
              LDAA  0,X
              ADDA  #2                  ; Reverse the original arrival direction
              ANDA  #3
              STAA  TARGET_DIR
              JMP   VERIFY_TARGET
VERIFY_TARGET LDAA  TARGET_DIR
              SUBA  HEADING
              ANDA  #3
              LBEQ   VERIFY_FWD
              CMPA  #1
              LBEQ   VERIFY_RIGHT
              CMPA  #3
              LBNE  ROUTE_FAULT
              BRSET EXIT_MASK,EXIT_LEFT,VERIFY_OK
              JMP   ROUTE_FAULT
VERIFY_RIGHT  BRSET EXIT_MASK,EXIT_RIGHT,VERIFY_OK
              JMP   ROUTE_FAULT
VERIFY_FWD    BRSET EXIT_MASK,EXIT_FWD,VERIFY_OK
              JMP   ROUTE_FAULT
VERIFY_OK     RTS

BEGIN_DIRECTION MOVB #1,JUNC_LOCK
              CLR   TURN_CLEAR
              CLR   TURN_HITS
              CLR   JUNC_CLEAR
              CLR   LOST_ACTIVE
              MOVB  TOF_COUNTER,JUNC_TICK
              MOVB  TOF_COUNTER,STATE_TICK
              LDAA  TARGET_DIR
              SUBA  HEADING
              ANDA  #3
              LBEQ   DIRECTION_STRAIGHT
              MOVB  #TURN_MIN,TURN_WAIT
              CMPA  #3
              LBEQ   DIRECTION_LEFT
              MOVB  #ST_TURN_R,ROBOT_STATE
              JMP   MOTOR_RIGHT
DIRECTION_LEFT MOVB #ST_TURN_L,ROBOT_STATE
              JMP   MOTOR_LEFT
DIRECTION_STRAIGHT MOVB #ST_FWD,ROBOT_STATE
              JMP   MOTOR_FWD

DO_REV_RECOV  JSR   MOTOR_REV
              LDAA  TOF_COUNTER
              SUBA  STATE_TICK
              CMPA  #REV_TICKS
              LBLO   REV_EXIT
              JMP   BEGIN_UTURN
REV_EXIT      RTS
BEGIN_UTURN   LDAA  HEADING
              ADDA  #2
              ANDA  #3
              STAA  TARGET_DIR
              CLR   TURN_CLEAR
              CLR   TURN_HITS
              MOVB  #UTURN_MIN,TURN_WAIT
              MOVB  TOF_COUNTER,STATE_TICK
              MOVB  #ST_TURN_R,ROBOT_STATE
              MOVB  #1,JUNC_LOCK
              CLR   JUNC_CLEAR
              CLR   LOST_ACTIVE
              JMP   MOTOR_RIGHT
DO_TURN_L     JSR   MOTOR_LEFT
              JSR   TURN_CHECK
              LBCC   TURN_EXIT
              MOVB  #ST_ALIGN_L,ROBOT_STATE
              JMP   TURN_ACQUIRED
DO_TURN_R     JSR   MOTOR_RIGHT
              JSR   TURN_CHECK
              LBCC   TURN_EXIT
              MOVB  #ST_ALIGN_R,ROBOT_STATE
TURN_ACQUIRED MOVB  TARGET_DIR,HEADING
              MOVB  TOF_COUNTER,STATE_TICK
              MOVB  TOF_COUNTER,JUNC_TICK
              JSR   MOTOR_FWD
TURN_EXIT     RTS
; Carry set only on acquisition. Timeout stops; handlers cannot overwrite stop.
TURN_CHECK    LDAA  TOF_COUNTER
              SUBA  STATE_TICK
              CMPA  #TURN_LIMIT
              LBLO   TURN_NOT_EXPIRED
              MOVB  #3,FAULT_CODE
              JSR   HALT_ROBOT
              CLC
              RTS
TURN_NOT_EXPIRED CMPA TURN_WAIT
              LBLO   TURN_PENDING
              TST   TURN_CLEAR
              LBEQ   TURN_PENDING
              TST   TAPE_BOW
              LBEQ   TURN_PENDING
              INC   TURN_HITS
              LDAA  TURN_HITS
              CMPA  #2
              LBLO   TURN_STILL_WAIT
              SEC
              RTS
TURN_PENDING  TST   TAPE_BOW
              LBNE   TURN_STILL_WAIT
              MOVB  #1,TURN_CLEAR        ; Must leave original line before reacquiring
              CLR   TURN_HITS
TURN_STILL_WAIT CLC
              RTS
DO_ALIGN_L:
DO_ALIGN_R    JSR   MOTOR_FWD
              LDAA  TOF_COUNTER
              SUBA  STATE_TICK
              CMPA  #ALIGN_TICKS
              LBLO   ALIGN_EXIT
              MOVB  #ST_FWD,ROBOT_STATE
ALIGN_EXIT    RTS

;***************************************************************************************************
; motor helpers - direction changes preserve sensor mux/LED bits
;***************************************************************************************************
MOTOR_FWD     BCLR  PORTA,$03
              BSET  PTT,$30
              RTS
MOTOR_REV     BSET  PORTA,$03
              BSET  PTT,$30
              RTS
MOTOR_LEFT    BSET  PORTA,$01
              BCLR  PORTA,$02
              BSET  PTT,$30
              RTS
MOTOR_RIGHT   BSET  PORTA,$02
              BCLR  PORTA,$01
              BSET  PTT,$30
              RTS
MOTOR_STOP    BCLR  PTT,$30
              RTS

;***************************************************************************************************
; ports, ADC and guider sensors
;***************************************************************************************************
PORTS_INIT    BCLR  PTT,$30              ; Set output latch before enabling motor pins
              BSET  DDRT,$30
              BCLR  PORTA,$3F
              BSET  DDRA,$3F
              BCLR  DDRAD,$FF
              BSET  ATDDIEN,$0C
              BSET  DDRB,$F0             ; LCD data uses upper nibble of PORTB
              BCLR  PTJ,$C0
              BSET  DDRJ,$C0
              RTS
ADC_INIT_SINGLE MOVB #$C0,ATDCTL2        ; Power up; fast flag clear
              LDY   #2
              JSR   delay_50us
              MOVB  #$08,ATDCTL3         ; One result per software-triggered conversion
              MOVB  #$97,ATDCTL4         ; 8-bit, sample 4 clocks, ATD clock = bus/48
              RTS
ADC_INIT_SCAN JMP   ADC_INIT_SINGLE      ; Five mux inputs scanned in software
IR_LEDS_ON    BSET  PORTA,$20
              RTS
IR_LEDS_OFF   BCLR  PORTA,$20
              RTS
IR_READ_ALL   CLR   SENSOR_NUM
              LDX   #IR_LINE
IR_READ_LOOP  LDAA  SENSOR_NUM
              JSR   IR_SELECT_ONE
              LDY   #SETTLE_50US
              JSR   delay_50us
              LDAA  #$81                ; Right-justified, unsigned, single AN1
              JSR   ADC_READ
              STAA  1,X+
              INC   SENSOR_NUM
              LDAA  SENSOR_NUM
              CMPA  #5
              LBLO   IR_READ_LOOP
              RTS
IR_SELECT_ONE PSHA
              LDAA  PORTA
              ANDA  #$E3
              STAA  TEMP
              PULA
              LSLA
              LSLA
              ANDA  #$1C
              ORAA  TEMP
              STAA  PORTA
              RTS
; A = ATDCTL5 command; A returns the 8-bit sample. X preserved.
ADC_READ      STAA  ATDCTL5
              LDY   #$FFFF
ADC_WAIT      BRSET ATDSTAT0,$80,ADC_DONE
              DBNE  Y,ADC_WAIT
              MOVB  #5,FAULT_CODE
              JSR   HALT_ROBOT
              CLRA
              RTS
ADC_DONE      LDAA  ATDDR0L
              RTS
IR_CLASSIFY   CLR   TAPE_BOW
              CLR   TAPE_PORT
              CLR   TAPE_MID
              CLR   TAPE_STBD
              LDAA  IR_BOW
              CMPA  #CAL_BOW-BOW_THRESH
              LBHI   CLASS_PORT
              INC   TAPE_BOW
CLASS_PORT    LDAA  IR_PORT
              CMPA  #CAL_PORT-PORT_THRESH
              LBHI   CLASS_MID
              INC   TAPE_PORT
CLASS_MID     LDAA  IR_MID
              CMPA  #CAL_MID-MID_THRESH
              LBHI   CLASS_STBD
              INC   TAPE_MID
CLASS_STBD    LDAA  IR_STBD
              CMPA  #CAL_STBD-STBD_THRESH
              LBHI   CLASS_EXIT
              INC   TAPE_STBD
CLASS_EXIT    RTS

;***************************************************************************************************
; timer overflow interrupt
;***************************************************************************************************
TOF_INIT      MOVB  #$80,TSCR1
              MOVB  #$80,TFLG2
              MOVB  #$84,TSCR2
              RTS
TOF_ISR_HANDLER INC TOF_COUNTER
              MOVB  #$80,TFLG2           ; Write one to clear TOF (never read/modify/write)
              RTI

;***************************************************************************************************
; LCD HUD - 20x2, refreshed only every six ticks; no clear/flicker in main loop
;***************************************************************************************************
stateTable    FCC   'start  '
              FCB   0
              FCC   'forward'
              FCB   0
              FCC   'stop   '
              FCB   0
              FCC   'left   '
              FCB   0
              FCC   'right  '
              FCB   0
              FCC   'reverse'
              FCB   0
              FCC   'align L'
              FCB   0
              FCC   'align R'
              FCB   0
              FCC   'probe  '
              FCB   0
HEX_TABLE     FCC   '0123456789ABCDEF'
HUD_REFRESH   LDAA  TOF_COUNTER
              SUBA  HUD_TICK
              CMPA  #HUD_TICKS
              LBLO  HUD_EXIT
              MOVB  TOF_COUNTER,HUD_TICK
              JSR   LCD_BUF_INIT
              LDAA  #$80                ; Battery on AN0; do not alter mux
              JSR   ADC_READ
              LDAB  #39
              MUL
              ADDD  #600                ; Battery ADC offset (millivolts)
              JSR   BIN_TO_BCD
              JSR   BCD_TO_ASCII
              MOVB  #'B',TOP_LINE
              MOVB  #':',TOP_LINE+1
              MOVB  TEN_THOUS,TOP_LINE+2
              LDAA  THOUSANDS
              CMPA  #' '
              LBNE   HUD_THOUS
              LDAA  #'0'
HUD_THOUS     STAA  TOP_LINE+3
              MOVB  #'.',TOP_LINE+4
              LDAA  HUNDREDS
              CMPA  #' '
              LBNE   HUD_HUND
              LDAA  #'0'
HUD_HUND      STAA  TOP_LINE+5
              MOVB  #'V',TOP_LINE+6
              LDAA  NAV_MODE
              ADDA  #'0'
              STAA  TOP_LINE+8
              LDAB  ROBOT_STATE
              CMPB  #STATE_COUNT
              LBLO   HUD_STATE_OK
              LDAB  #ST_STOP
HUD_STATE_OK  LSLB
              LSLB
              LSLB
              LDX   #stateTable
              ABX
              LDY   #TOP_LINE+10
              LDAB  #7
HUD_COPY_STATE LDAA 1,X+
              STAA  1,Y+
              DBNE  B,HUD_COPY_STATE
              LDAA  TOF_COUNTER
              BITA  #$08
              LBEQ   HUD_ALIVE_OFF
              MOVB  #'*',TOP_LINE+19
HUD_ALIVE_OFF INC   HUD_PAGE
              LDAA  HUD_PAGE
              BITA  #$04
              LBNE   HUD_ROUTE_PAGE
              JSR   HUD_SHOW_IR
              LBRA   HUD_WRITE
HUD_ROUTE_PAGE MOVB #'N',BOT_LINE
              MOVB  #':',BOT_LINE+1
              LDAA  PATH_COUNT
              JSR   BYTE_TO_HEXASCII
              STD   BOT_LINE+2
              MOVB  #'H',BOT_LINE+5
              MOVB  #':',BOT_LINE+6
              LDAA  HEADING
              ADDA  #'0'
              STAA  BOT_LINE+7
              MOVB  #'E',BOT_LINE+9
              MOVB  #':',BOT_LINE+10
              LDAA  FAULT_CODE
              ADDA  #'0'
              STAA  BOT_LINE+11
              MOVB  #'B',BOT_LINE+13
              MOVB  #':',BOT_LINE+14
              LDAA  PORTAD0
              COMA
              ANDA  #$0C
              LSRA
              LSRA
              ADDA  #'0'
              STAA  BOT_LINE+15
HUD_WRITE     LDAA  #$80
              JSR   LCD_CMD
              LDX   #TOP_LINE
              JSR   LCD_PUTS
              LDAA  #$C0
              JSR   LCD_CMD
              LDX   #BOT_LINE
              JSR   LCD_PUTS
HUD_EXIT      RTS
HUD_SHOW_IR   MOVB  #'L',BOT_LINE
              LDAA  IR_LINE
              JSR   BYTE_TO_HEXASCII
              STD   BOT_LINE+1
              MOVB  #'B',BOT_LINE+4
              LDAA  IR_BOW
              JSR   BYTE_TO_HEXASCII
              STD   BOT_LINE+5
              MOVB  #'P',BOT_LINE+8
              LDAA  IR_PORT
              JSR   BYTE_TO_HEXASCII
              STD   BOT_LINE+9
              MOVB  #'M',BOT_LINE+12
              LDAA  IR_MID
              JSR   BYTE_TO_HEXASCII
              STD   BOT_LINE+13
              MOVB  #'S',BOT_LINE+16
              LDAA  IR_STBD
              JSR   BYTE_TO_HEXASCII
              STD   BOT_LINE+17
              RTS
LCD_BUF_INIT  LDX   #TOP_LINE
              LDAB  #20
              LDAA  #SPACE
BUF_TOP       STAA  1,X+
              DBNE  B,BUF_TOP
              CLR   0,X
              LDX   #BOT_LINE
              LDAB  #20
BUF_BOTTOM    STAA  1,X+
              DBNE  B,BUF_BOTTOM
              CLR   0,X
              RTS

; HD44780 4-bit startup: raw 3,3,3,2 nibbles precede full-byte commands.
LCD_INIT      LDY   #2000
              JSR   delay_50us
              LDAA  #$30
              JSR   LCD_NIBBLE
              LDY   #100
              JSR   delay_50us
              LDAA  #$30
              JSR   LCD_NIBBLE
              LDY   #4
              JSR   delay_50us
              LDAA  #$30
              JSR   LCD_NIBBLE
              LDY   #4
              JSR   delay_50us
              LDAA  #$20
              JSR   LCD_NIBBLE
              LDY   #4
              JSR   delay_50us
              LDAA  #$28
              JSR   LCD_CMD
              LDAA  #$0C
              JSR   LCD_CMD
              LDAA  #$06
              JSR   LCD_CMD
LCD_CLEAR     LDAA  #$01
              JMP   LCD_CMD
LCD_CMD       BCLR  LCD_CNTR,LCD_RS
              PSHA
              JSR   LCD_STROBE
              PULA
              CMPA  #$03                ; Clear/home need longer execution time
              LBHI   LCD_CMD_EXIT
              LDY   #40
              JSR   delay_50us
LCD_CMD_EXIT  RTS
LCD_PUTC      BSET  LCD_CNTR,LCD_RS
              JMP   LCD_STROBE
LCD_PUTS      LDAA  1,X+
              LBEQ   LCD_PUTS_EXIT
              JSR   LCD_PUTC
              LBRA   LCD_PUTS
LCD_PUTS_EXIT RTS
LCD_STROBE    PSHA
              JSR   LCD_NIBBLE
              PULA
              LSLA
              LSLA
              LSLA
              LSLA
              JSR   LCD_NIBBLE
              LDY   #1
              JMP   delay_50us
LCD_NIBBLE    STAA  LCD_DAT              ; Data stable before enable rises
              NOP
              BSET  LCD_CNTR,LCD_E
              LDY   #10
LCD_E_WAIT    DBNE  Y,LCD_E_WAIT
              BCLR  LCD_CNTR,LCD_E
              NOP
              RTS
; Y = count of approximately 50 us delays, X preserved, Y consumed.
delay_50us    CPY   #0                   ; Zero must not underflow to 65536 iterations
              LBEQ   DELAY_EXIT
              PSHX
DELAY_OUTER   LDX   #300
DELAY_INNER   NOP
              DBNE  X,DELAY_INNER
              DBNE  Y,DELAY_OUTER
              PULX
DELAY_EXIT    RTS

;***************************************************************************************************
; number conversion utilities
;***************************************************************************************************
BIN_TO_BCD           XGDX
                  LDAA #0
                  STAA TEN_THOUS
                  STAA THOUSANDS
                  STAA HUNDREDS
                  STAA TENS
                  STAA UNITS
                  STAA BCD_SPARE
                  STAA BCD_SPARE+1
                  CPX #0
                  LBEQ CON_EXIT
                  XGDX
                  LDX #10
                  IDIV
                  STAB UNITS
                  CPX #0
                  LBEQ CON_EXIT
                  XGDX
                  LDX #10
                  IDIV
                  STAB TENS
                  CPX #0
                  LBEQ CON_EXIT
                  XGDX
                  LDX #10
                  IDIV
                  STAB HUNDREDS
                  CPX #0
                  LBEQ CON_EXIT
                  XGDX
                  LDX #10
                  IDIV
                  STAB THOUSANDS
                  CPX #0
                  LBEQ CON_EXIT
                  XGDX
                  LDX #10
                  IDIV
                  STAB TEN_THOUS

CON_EXIT          RTS

; A is a DDRAM address; add the command bit and set the LCD cursor.
LCD_POS_CRSR      ORAA #%10000000
                  JSR LCD_CMD
                  RTS

; A byte -> ASCII high nibble in A, low nibble in B; X clobbered.
BYTE_TO_HEXASCII               PSHA
                      TAB
                      ANDB #%00001111
                      CLRA
                      ADDD #HEX_TABLE
                      XGDX
                      LDAA 0,X
                      PULB
                      PSHA
                      RORB
                      RORB
                      RORB
                      RORB
                      ANDB #%00001111
                      CLRA
                      ADDD #HEX_TABLE
                      XGDX
                      LDAA 0,X
                      PULB
                      RTS

; Five unpacked digits -> ASCII, with leading zero suppression.
BCD_TO_ASCII           LDAA    #0
                  STAA    NO_BLANK

C_TTHOU           LDAA    TEN_THOUS
                  ORAA    NO_BLANK
                  LBNE     NOT_BLANK1

ISBLANK1          LDAA    #' '
                  STAA    TEN_THOUS
                  LBRA     C_THOU

NOT_BLANK1        LDAA    TEN_THOUS
                  ORAA    #$30
                  STAA    TEN_THOUS
                  LDAA    #$1
                  STAA    NO_BLANK

C_THOU            LDAA    THOUSANDS
                  ORAA    NO_BLANK
                  LBNE     NOT_BLANK2

ISBLANK2          LDAA    #' '
                  STAA    THOUSANDS
                  LBRA     C_HUNS

NOT_BLANK2        LDAA    THOUSANDS
                  ORAA    #$30
                  STAA    THOUSANDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_HUNS            LDAA    HUNDREDS
                  ORAA    NO_BLANK
                  LBNE     NOT_BLANK3

ISBLANK3          LDAA    #' '
                  STAA    HUNDREDS
                  LBRA     C_TENS

NOT_BLANK3        LDAA    HUNDREDS
                  ORAA    #$30
                  STAA    HUNDREDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_TENS            LDAA    TENS
                  ORAA    NO_BLANK
                  LBNE     NOT_BLANK4

ISBLANK4          LDAA    #' '
                  STAA    TENS
                  LBRA     C_UNITS

NOT_BLANK4        LDAA    TENS
                  ORAA    #$30
                  STAA    TENS

C_UNITS           LDAA    UNITS
                  ORAA    #$30
                  STAA    UNITS
                  RTS



;***************************************************************************************************
; interrupt vectors
;***************************************************************************************************
              ORG   $FFDE
              DC.W  TOF_ISR_HANDLER
              ORG   $FFFE
              DC.W  Entry
