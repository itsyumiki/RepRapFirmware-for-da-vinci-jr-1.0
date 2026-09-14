.syntax unified
.cpu cortex-m0
.thumb

.section .isr_vector,"a",%progbits
.global lpc_vectors
.type lpc_vectors, %object
lpc_vectors:
	.word _estack
	.word Reset_Handler
	.word Default_Handler
	.word Default_Handler
	.word 0
	.word 0
	.word 0
	.word __vector_checksum
	.word 0
	.word 0
	.word 0
	.word Default_Handler
	.word 0
	.word 0
	.word Default_Handler
	.word SysTick_Handler
	.rept 32
	.word Default_Handler
	.endr
.size lpc_vectors, . - lpc_vectors

// Keep NXP Code Read Protection disabled. The LPC11xx boot ROM interprets the
// word at flash offset 0x2FC; reserving it prevents ordinary code from ever
// coinciding with a CRP magic value and locking out UART ISP/SWD updates.
.section .crp,"a",%progbits
.word 0xFFFFFFFF

.section .text.Reset_Handler,"ax",%progbits
.thumb_func
.global Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
	ldr r0, =_sidata
	ldr r1, =_sdata
	ldr r2, =_edata
1:
	cmp r1, r2
	bcs 2f
	ldr r3, [r0]
	str r3, [r1]
	adds r0, #4
	adds r1, #4
	b 1b
2:
	ldr r1, =_sbss
	ldr r2, =_ebss
	movs r3, #0
3:
	cmp r1, r2
	bcs 4f
	str r3, [r1]
	adds r1, #4
	b 3b
4:
	bl main
5:
	b 5b
.size Reset_Handler, . - Reset_Handler

.extern SysTick_Handler
.section .text.Default_Handler,"ax",%progbits
.thumb_func
.global Default_Handler
.type Default_Handler, %function
Default_Handler:
	b Default_Handler
.size Default_Handler, . - Default_Handler
