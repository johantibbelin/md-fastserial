; Fast-serial user firmware module
; (C) 2026 by Johan Tibbelin
; License: GPL v3
;
; Reached via rom_function: jmp USERFW ($FA0800) when the RP writes CMD_START
; to CMD_MAGIC_SENTINEL_ADDR.  At this point the system is in supervisor mode
; (CA_INIT bit 27 = after GEMDOS init, before disk boot), so we can patch the
; trap-13 vector and call GEMDOS Malloc directly.
;
; What this file does
; -------------------
; userfw (entry point):
;   1. Malloc 8 bytes of ST RAM: [old_trap13_vec:4][rx_read_ptr:4]
;   2. Send the buffer address to the RP via CMD_SET_SHARED_VAR so the RP
;      stores it in SHARED_VARIABLES[2] ($FA2018).  The trap handler reads
;      that slot to locate the buffer.
;   3. Save the old trap-#13 vector, install serial_bios_hook.
;   4. rts — returns all the way back to the TOS bootloader (the original
;      return address was on the supervisor stack from when TOS called the
;      CA_INIT routine).  TOS continues booting GEM normally.
;
; serial_bios_hook (at $FA08xx, permanent cartridge ROM):
;   Intercepts BIOS calls for device 1 (AUX:):
;     Bconstat(1) → read SHARED_VARIABLES[3] ($FA201C, RP write ptr);
;                   return -1 if write_ptr != local read_ptr, else 0.
;     Bconin(1)   → spin until data; read byte from ring buffer at
;                   APP_FREE[$FA2300]; advance local read_ptr; send
;                   APP_SERIAL_RX_ACK to RP; return byte in D0.
;     Bconout(1,c)→ send APP_SERIAL_TX to RP with char in D3.w.
;     Bcostat(1)  → return -1 (always ready to send).
;   All other BIOS calls are forwarded to the original handler.
;
; Shared memory used (must match rp/src/include/chandler.h)
;   $FA2018 = SHARED_VARIABLES[2] = ST RAM buffer pointer (RP writes after SET)
;   $FA201C = SHARED_VARIABLES[3] = RP→ST ring buffer write pointer (RP writes)
;   $FA2300 = APP_FREE             = RP→ST ring buffer (2048 bytes)

    ; send_sync_command_to_sidecart is defined in main.o (sidecart_functions.s)
    xref send_sync_command_to_sidecart

; Local send_sync macro using JSR (32-bit absolute) instead of BSR (16-bit
; PC-relative).  Cross-object AOUT rawbin1 symbols are resolved as absolute
; addresses by vlink, which overflows a 16-bit BSR relocation field; JSR
; with an absolute long address accepts the full 32-bit value.
send_sync   macro
                    move.w  #CMD_RETRIES_COUNT, d7
.\@retry:
                    movem.l d1-d7, -(sp)
                    moveq.l #\2, d1
                    move.w  #\1, d0
                    jsr     send_sync_command_to_sidecart
                    movem.l (sp)+, d1-d7
                    tst.w   d0
                    beq.s   .\@ok
                    dbf     d7, .\@retry
.\@ok:
                    endm

    section text

; ---------------------------------------------------------------------------
; Constants (re-declared here because equ values don't cross object files)
; ---------------------------------------------------------------------------
ROM4_ADDR               equ $FA0000
ROMCMD_START_ADDR       equ $FB0000
CMD_MAGIC_NUMBER        equ $ABCD
CMD_RETRIES_COUNT       equ 3
RANDOM_TOKEN_ADDR       equ (ROM4_ADDR + $2004)
RANDOM_TOKEN_SEED_ADDR  equ (ROM4_ADDR + $2008)
COMMAND_TIMEOUT         equ $0000FFFF
COMMAND_WRITE_TIMEOUT   equ COMMAND_TIMEOUT

; CMD_SET_SHARED_VAR command code (matches sidecart_functions.s)
CMD_SET_SHARED_VAR      equ 1

; New serial command codes (must match rp/src/include/chandler.h)
APP_SERIAL_TX           equ 2       ; ST→RP: char byte (payload 2)
APP_SERIAL_RX_ACK       equ 3       ; ST→RP: new read pointer (payload 4)

; Shared variable slot addresses
SERIAL_ST_BUFPTR_ADDR   equ (ROM4_ADDR + $2018)   ; SHARED_VARIABLES[2]
SERIAL_RX_WR_PTR_ADDR   equ (ROM4_ADDR + $201C)   ; SHARED_VARIABLES[3]
SERIAL_RX_RING_ADDR     equ (ROM4_ADDR + $2300)   ; APP_FREE ring buffer
SERIAL_RING_SIZE        equ 2048
SERIAL_RING_MASK        equ (SERIAL_RING_SIZE - 1)

; SHARED_VARIABLE index for the ST RAM buffer pointer
SERIAL_ST_BUFPTR_IDX    equ 2

; Trap-#13 (BIOS) exception vector address
TRAP_BIOS_VECTOR        equ $B4     ; vector 45 = 45*4 = 0xB4

; BIOS function codes
BIOS_BCONSTAT           equ 1
BIOS_BCONIN             equ 2
BIOS_BCONOUT            equ 3
BIOS_BCOSTAT            equ 8

; AUX: device number
SERIAL_DEVICE           equ 1

; ---------------------------------------------------------------------------
; userfw — entry point, installs the BIOS hook then returns to TOS
; ---------------------------------------------------------------------------
; On entry the supervisor stack holds TOS's return address from the
; CA_INIT jsr.  All scratch registers are ours to clobber.
; ---------------------------------------------------------------------------
userfw:
    ; --- Allocate 8 bytes of ST RAM ---
    ; Buffer layout: [old_trap13_vec:4][rx_read_ptr:4]
    move.l  #8, -(sp)               ; size argument
    move.w  #$48, -(sp)             ; GEMDOS Malloc function code
    trap    #1
    addq.l  #6, sp
    ; d0 = address (positive) or negative on failure
    tst.l   d0
    bmi     .userfw_done            ; malloc failed → leave BIOS alone

    move.l  d0, a5                  ; a5 = our data buffer in ST RAM

    ; --- Initialise buffer ---
    clr.l   4(a5)                   ; rx_read_ptr = 0

    ; --- Tell the RP where our buffer lives ---
    ; RP stores the pointer in SHARED_VARIABLES[2] ($FA2018) so the trap
    ; handler (running from cartridge ROM) can find it on every call.
    move.l  #SERIAL_ST_BUFPTR_IDX, d3   ; variable index = 2
    move.l  a5, d4                       ; variable value = buffer address
    send_sync CMD_SET_SHARED_VAR, 8     ; payload: D3.l (idx) + D4.l (value)

    ; --- Install the BIOS hook ---
    move.l  TRAP_BIOS_VECTOR.w, (a5)    ; save old vector at buffer[0]
    move.l  #serial_bios_hook, TRAP_BIOS_VECTOR.w

.userfw_done:
    rts                             ; return to TOS → GEM boots normally

; ---------------------------------------------------------------------------
; serial_bios_hook — BIOS trap-#13 intercept (runs from cartridge ROM)
; ---------------------------------------------------------------------------
; Called in supervisor mode; user stack (USP) has the original BIOS args.
; Stack layout at USP when trap #13 fires:
;   USP+0  function_code.w
;   USP+2  device.w          (for Bconstat/Bconin/Bcostat)
;   USP+4  char.w            (for Bconout only; device is at USP+2)
; Returns value in D0; RTE restores SR/PC to the calling code.
; ---------------------------------------------------------------------------
serial_bios_hook:
    move.l  usp, a0                 ; a0 → user stack
    move.w  (a0), d0                ; function code

    cmp.w   #BIOS_BCONSTAT, d0
    beq.s   .bconstat
    cmp.w   #BIOS_BCONIN, d0
    beq.s   .bconin
    cmp.w   #BIOS_BCONOUT, d0
    beq     .bconout
    cmp.w   #BIOS_BCOSTAT, d0
    beq     .bcostat

.passthrough:
    ; Not a function we handle — forward to the original BIOS handler.
    ; Read old vector: buffer ptr is in SHARED_VARIABLES[2], vector at buf[0].
    move.l  SERIAL_ST_BUFPTR_ADDR, a1   ; a1 = ST RAM buffer address
    move.l  (a1), a1                    ; a1 = saved old trap-#13 vector
    jmp     (a1)

; --- Bconstat(1): input status -----------------------------------------------
.bconstat:
    move.w  2(a0), d1               ; device
    cmp.w   #SERIAL_DEVICE, d1
    bne.s   .passthrough
    move.l  SERIAL_RX_WR_PTR_ADDR, d0  ; RP's write_ptr
    move.l  SERIAL_ST_BUFPTR_ADDR, a1
    move.l  4(a1), d1               ; local rx_read_ptr
    cmp.l   d0, d1                  ; write == read?
    beq.s   .bconstat_empty
    moveq   #-1, d0                 ; data available
    rte
.bconstat_empty:
    moveq   #0, d0
    rte

; --- Bconin(1): read one byte ------------------------------------------------
; Blocks until data is present (caller is expected to check Bconstat first).
.bconin:
    move.w  2(a0), d1
    cmp.w   #SERIAL_DEVICE, d1
    bne.s   .passthrough

.bconin_spin:
    move.l  SERIAL_RX_WR_PTR_ADDR, d0
    move.l  SERIAL_ST_BUFPTR_ADDR, a1
    move.l  4(a1), d1               ; rx_read_ptr
    cmp.l   d0, d1
    beq.s   .bconin_spin            ; wait

    ; Read byte from ring buffer
    move.l  d1, d2
    and.l   #SERIAL_RING_MASK, d2   ; d2 = read_ptr % RING_SIZE
    lea     SERIAL_RX_RING_ADDR, a0
    moveq   #0, d0
    move.b  (a0, d2.l), d0         ; d0 = byte from ring

    ; Advance local rx_read_ptr
    addq.l  #1, d1
    move.l  SERIAL_ST_BUFPTR_ADDR, a1
    move.l  d1, 4(a1)

    ; Send ACK to RP: APP_SERIAL_RX_ACK with new read_ptr in D3.l
    ; send_sync saves/restores D1-D7; save D0 (return byte) ourselves.
    move.l  d0, -(sp)
    move.l  d1, d3                  ; new read_ptr as payload
    send_sync APP_SERIAL_RX_ACK, 4 ; 4-byte payload → D3.l
    move.l  (sp)+, d0
    and.l   #$FF, d0                ; return byte in D0 (low byte)
    rte

; --- Bconout(1, char): write one byte ----------------------------------------
.bconout:
    move.w  2(a0), d1
    cmp.w   #SERIAL_DEVICE, d1
    bne     .passthrough
    ; char is at USP+4 (a word; the byte is in the low half)
    move.w  4(a0), d3               ; d3.w = char (low byte is the character)
    send_sync APP_SERIAL_TX, 2      ; 2-byte payload → D3.w
    rte

; --- Bcostat(1): output status -----------------------------------------------
.bcostat:
    move.w  2(a0), d1
    cmp.w   #SERIAL_DEVICE, d1
    bne     .passthrough
    moveq   #-1, d0                 ; always ready to accept output
    rte
