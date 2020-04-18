#include <msp430.h>
.extern os_kernel_mode
.extern os_runningTask
.extern os_irq_sysc
.extern os_SP
.extern os_timeoutQueueDirty
.extern Panic
.extern update_timer
.extern kernel_body
.extern os_schedule
.extern updateHead

#define OS_MAGIC_STACK_WORD 0xA55A ///< the magic word for the task stack prepara
#define OS_IRQS_SUPPORTED 0x00FF ///< the maximum number of supported IRQs

.global os_dispatcher
  .type os_dispatcher, @function
os_dispatcher:
    ; FIXME:
    call #updateHead

        MOV.W os_runningTask, R15
        MOV.W @R15, R1
        MOV.W 2(R15), R4
        MOV.W 4(R15), R5
        MOV.W 6(R15), R6
        MOV.W 8(R15), R7
        MOV.W 10(R15), R8
        MOV.W 12(R15), R9
        MOV.W 14(R15), R10
        MOV.W 16(R15), R11
        MOV.W 18(R15), R12
        MOV.W 20(R15), R13
        MOV.W 22(R15), R14
        MOV.W 24(R15), R15
        MOV #00h, os_kernel_mode
        RETI

.global __kernel_entry
  .type __kernel_entry, @function
__kernel_entry:
    MOV.W #1, os_kernel_mode
        MOV SP, os_SP
        MOV.W os_runningTask, R1
        MOV.W os_SP, 0(R1)
    MOV.W R4, 2(R1)
    MOV.W R5, 4(R1)
    MOV.W R6, 6(R1)
    MOV.W R7, 8(R1)
    MOV.W R8, 10(R1)
    MOV.W R9, 12(R1)
    MOV.W R10, 14(R1)
    MOV.W R11, 16(R1)
    MOV.W R12, 18(R1)
    MOV.W R13, 20(R1)
    MOV.W R14, 22(R1)
    MOV.W R15, 24(R1)

    ; FIXME dont use R12-R15 because the arguments for syscalls are passed there

    MOV.W #__os_stack_begin, SP
    MOV.W os_runningTask, R4
        MOV.W 36(R4), R4
        CMP.W #OS_MAGIC_STACK_WORD, @R4
    JZ __kernel_body
    CALL #panic

.global __kernel_body
  .type __kernel_body, @function
__kernel_body:

        MOV.W &os_irq_sysc, R4
        MOV.W #OS_IRQS_SUPPORTED, R5
        CMP.W R4, R5
        JLO systemcall
        JMP Interruptcall

Interruptcall:
        MOV.W #__os_irq_table, R4
        ADD.W os_irq_sysc, R4
        ADD.W os_irq_sysc, R4
        CMP.W #0, @R4
        CALL @R4
        JMP Timeout

systemcall:
        MOV.W &os_irq_sysc, R4
        CALL R4

    ; FIXME: save the return values from a syscall into the tcb
    ; save return values
    MOV.W SP,os_SP
    MOV.W os_runningTask,SP   ; set the Stack Pointer to reg_context[0] = SP
    ADD.W #26,SP              ; set the Stack Pointer to reg_context[13]
    PUSHM.W #4,R15           ; pushes R15 - R4 to reg_context[12 - 1]
    MOV.W os_SP, SP

        JMP Timeout

Timeout:
        MOV.W os_timeoutQueueDirty, R4
        CMP.W #1, @R4
        JEQ Timer
        JMP os_dispatcher

Timer:
        CALL #update_timer

    ; FIXME call dispatcher here
    JMP os_dispatcher

.macro __hwirq irqno
.global __hwirq_\irqno
  .type __hwirq_\irqno, @function
__hwirq_\irqno:
  MOV.W #\irqno, os_irq_sysc
  JMP __kernel_entry
.endm

__hwirq 1
__hwirq 2
__hwirq 52

.macro __syscall name
.global syscall_\name
    .type syscall_\name, @function
syscall_\name:
    CMP os_kernel_mode, #0
    JNE label_syscall_\name
    push sr
    dint
    nop
    MOV.W #__syscall_\name, os_irq_sysc
    JMP __kernel_entry

label_syscall_\name:
    CALL #__syscall_\name
    ; FIXME return here
    RET
.endm

__syscall mystery
__syscall CAS
__syscall waitEventUntil
__syscall setEvent
__syscall getResource
__syscall releaseResource
