.syntax unified
.cpu cortex-m3
.thumb

.include "include/STM32F103xx.inc"

.text

.type Reset_Handler, %function
.global Reset_Handler
Reset_Handler:
	ldr r0, =_DataStart
	ldr r1, =_DataEnd
	ldr r2, =_DataLoad

	b 2f
1:	ldr r3, [r2], #4
	str r3, [r0], #4
2:	cmp r0, r1
	blo 1b
	
	ldr r0, =_BssStart
	ldr r1, =_BssEnd
	ldr r2, =0

	b 2f
1:	str r2, [r0], #4
2:	cmp r0, r1
	blo 1b
    
    bl configSysClock
	bl main
1:	bkpt
	b 1b
	.ltorg

.type configSysClock, %function
configSysClock:
    @ Turn on HSE
	ldr r0, =RCC
	ldr r1, =((1 << RCC_CR_HSION)| (1 << RCC_CR_HSEON))
	str r1, [r0, #RCC_CR]
	
	@ Pre-configure PLL (see startup-Clock-setup-info.txt) 
	ldr r2, =((0<<RCC_CFGR_PLLMUL)|(1<<RCC_CFGR_PLLXTPRE)|(1 << RCC_CFGR_PLLSRC)| (8<<RCC_CFGR_HPRE))
	str r2, [r0, #RCC_CFGR]

	@ Pre-Calculate value for RCC_CR 
	orr r1, #(1 << RCC_CR_PLLON)
	@ Wait for HSE ready
1:	ldr r3, [r0, #RCC_CR]
	ands r3, #(1 << RCC_CR_HSERDY)
	beq 1b
	@ Turn on PLL
	str r1, [r0, #RCC_CR]
	
	@ Pre-Calculate value for RCC_CFGR
	orr r2, #(2 << RCC_CFGR_SW)
	@ Wait for PLL ready
1:	ldr r3, [r0, #RCC_CR]
	ands r3, #(1 << RCC_CR_PLLRDY)
	beq 1b
	
	@ in my case, Sys Clock only 4 MHz, not exceeding 24 MHz,
	@ so it is not necessary to add wait state to the flash
        
	@ Switch system clock to PLL
	str r2, [r0, #RCC_CFGR]

	@ Pre-Calculate value for RCC_CR 
	@ (while waiting PLL to become selected by RCC as SysClock source)
	bic r1, #(1 << RCC_CR_HSION)
	@ Wait for switch to PLL
1:	ldr r3, [r0, #RCC_CFGR]
	and r3, #(3 << RCC_CFGR_SWS)
	cmp r3, #(2 << RCC_CFGR_SWS)
	bne 1b
	
	@ Turn off HSI to save power
	str r1, [r0, #RCC_CR]
	
	bx lr
	.ltorg
