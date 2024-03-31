<img src="https://github.com/mytechnotalent/STM32F4_LoRa_UART_Driver/blob/main/STM32F4_LoRa_UART_Driver.png?raw=true">

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# STM32F4 LoRa UART Driver
## PRE-RELEASE
An STM32F4, RYLR998 LoRa UART driver written entirely in Assembler.

<br>

# Code
```assembler
/**
 * FILE: main.s
 *
 * DESCRIPTION:
 * This file contains the assembly code for a STM32F401 LoRa UART driver utilizing the STM32F401CC6 
 * microcontroller.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: March 15, 2024
 * UPDATE DATE: March 30, 2024
 *
 * ASSEMBLE AND LINK w/ SYMBOLS:
 * 1. arm-none-eabi-as -g main.s -o main.o
 * 2. arm-none-eabi-ld main.o -o main.elf -T STM32F401CCUX_FLASH.ld
 * 3. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program main.elf verify reset exit"
 * ASSEMBLE AND LINK w/o SYMBOLS:
 * 1. arm-none-eabi-as -g main.s -o main.o
 * 2. arm-none-eabi-ld main.o -o main.elf -T STM32F401CCUX_FLASH.ld
 * 3. arm-none-eabi-objcopy -O binary --strip-all main.elf main.bin
 * 3. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program main.bin 0x08000000 verify reset exit"
 * DEBUG w/ SYMBOLS:
 * 1. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg
 * 2. arm-none-eabi-gdb main.elf
 * 3. target remote :3333
 * 4. monitor reset halt
 * 5. l
 * DEBUG w/o SYMBOLS:
 * 1. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg
 * 2. arm-none-eabi-gdb main.bin
 * 3. target remote :3333
 * 4. monitor reset halt
 * 5. x/8i $pc
 */


.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb


/**
 * The start address for the .data section defined in linker script.
 */
.word _sdata

/**
 * The end address for the .data section defined in linker script.
 */
.word _edata

/**
 * The start address for the initialization values of the .data section defined in linker script.
 */
.word _sidata

/**
 * The start address for the .bss section defined in linker script.
 */
.word _sbss

/**
 * The end address for the .bss section defined in linker script.
 */
.word _ebss


/**
 * Provide weak aliases for each Exception handler to the Default_Handler. As they are weak aliases, any function
 * with the same name will override this definition.
 */
.macro weak name
  .global \name
  .weak \name
  .thumb_set \name, Default_Handler
  .word \name
.endm


/**
 * Initialize the .isr_vector section. The .isr_vector section contains vector table.
 */
.section .isr_vector, "a"

/**
 * The STM32F401CCUx vector table. Note that the proper constructs must be placed on this to ensure that it ends up
 * at physical address 0x00000000.
 */
.global isr_vector
.type isr_vector, %object
isr_vector:
  .word _estack
  .word Reset_Handler
   weak NMI_Handler
   weak HardFault_Handler
   weak MemManage_Handler
   weak BusFault_Handler
   weak UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
   weak SVC_Handler
   weak DebugMon_Handler
  .word 0
   weak PendSV_Handler
   weak SysTick_Handler
  .word 0
   weak EXTI16_PVD_IRQHandler                              // EXTI Line 16 interrupt PVD through EXTI line detection 
   weak TAMP_STAMP_IRQHandler                              // Tamper and TimeStamp interrupts through the EXTI line
   weak EXTI22_RTC_WKUP_IRQHandler                         // EXTI Line 22 interrupt RTC Wakeup interrupt, EXTI line
   weak FLASH_IRQHandler                                   // FLASH global interrupt
   weak RCC_IRQHandler                                     // RCC global interrupt
   weak EXTI0_IRQHandler                                   // EXTI Line0 interrupt
   weak EXTI1_IRQHandler                                   // EXTI Line1 interrupt
   weak EXTI2_IRQHandler                                   // EXTI Line2 interrupt
   weak EXTI3_IRQHandler                                   // EXTI Line3 interrupt
   weak EXTI4_IRQHandler                                   // EXTI Line4 interrupt
   weak DMA1_Stream0_IRQHandler                            // DMA1 Stream0 global interrupt
   weak DMA1_Stream1_IRQHandler                            // DMA1 Stream1 global interrupt
   weak DMA1_Stream2_IRQHandler                            // DMA1 Stream2 global interrupt
   weak DMA1_Stream3_IRQHandler                            // DMA1 Stream3 global interrupt
   weak DMA1_Stream4_IRQHandler                            // DMA1 Stream4 global interrupt
   weak DMA1_Stream5_IRQHandler                            // DMA1 Stream5 global interrupt
   weak DMA1_Stream6_IRQHandler                            // DMA1 Stream6 global interrupt
   weak ADC_IRQHandler                                     // ADC1 global interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak EXTI9_5_IRQHandler                                 // EXTI Line[9:5] interrupts
   weak TIM1_BRK_TIM9_IRQHandle                            // TIM1 Break interrupt and TIM9 global interrupt
   weak TIM1_UP_TIM10_IRQHandler                           // TIM1 Update interrupt and TIM10 global interrupt
   weak TIM1_TRG_COM_TIM11_IRQHandler                      // TIM1 T/C interrupts, TIM11 global interrupt
   weak TIM1_CC_IRQHandler                                 // TIM1 Capture Compare interrupt
   weak TIM2_IRQHandler                                    // TIM2 global interrupt
   weak TIM3_IRQHandler                                    // TIM3 global interrupt
   weak TIM4_IRQHandler                                    // TIM4 global interrupt
   weak I2C1_EV_IRQHandler                                 // I2C1 event interrupt
   weak I2C1_ER_IRQHandler                                 // I2C1 error interrupt
   weak I2C2_EV_IRQHandler                                 // I2C2 event interrupt
   weak I2C2_ER_IRQHandler                                 // I2C2 error interrupt
   weak SPI1_IRQHandler                                    // SPI1 global interrupt
   weak SPI2_IRQHandler                                    // SPI2 global interrupt
   weak USART1_IRQHandler                                  // USART1 global interrupt
   weak USART2_IRQHandler                                  // USART2 global interrupt
  .word 0                                                  // reserved
   weak EXTI15_10_IRQHandler                               // EXTI Line[15:10] interrupts
   weak EXTI17_RTC_Alarm_IRQHandler                        // EXTI Line 17 interrupt / RTC Alarms (A and B) EXTI
   weak EXTI18_OTG_FS_WKUP_IRQHandler                      // EXTI Line 18 interrupt / USBUSB OTG FS Wakeup EXTI
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak DMA1_Stream7_IRQHandler                            // DMA1 Stream7 global interrupt
  .word 0                                                  // reserved
   weak SDIO_IRQHandler                                    // SDIO global interrupt
   weak TIM5_IRQHandler                                    // TIM5 global interrupt
   weak SPI3_IRQHandler                                    // SPI3 global interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak DMA2_Stream0_IRQHandler                            // DMA2 Stream0 global interrupt
   weak DMA2_Stream1_IRQHandler                            // DMA2 Stream1 global interrupt
   weak DMA2_Stream2_IRQHandler                            // DMA2 Stream2 global interrupt
   weak DMA2_Stream3_IRQHandler                            // DMA2 Stream3 global interrupt
   weak DMA2_Stream4_IRQHandler                            // DMA2 Stream4 global interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak OTG_FS_IRQHandler                                  // USB On The Go FS global interrupt
   weak DMA2_Stream5_IRQHandler                            // DMA2 Stream5 global interrupt
   weak DMA2_Stream6_IRQHandler                            // DMA2 Stream6 global interrupt
   weak DMA2_Stream7_IRQHandler                            // DMA2 Stream7 global interrupt
   weak USART6_IRQHandler                                  // USART6 global interrupt
   weak I2C3_EV_IRQHandler                                 // I2C3 event interrupt
   weak I2C3_ER_IRQHandler                                 // I2C3 error interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak SPI4_IRQHandler                                    // SPI4 global interrupt

/**
 * @brief  This code is called when processor starts execution.
 *
 *         This is the code that gets called when the processor first starts execution following a reset event. We 
 *         first define and init the bss section and then define and init the data section, after which the 
 *         application supplied main routine is called.
 *
 * @param  None
 * @retval None
 */
.type Reset_Handler, %function
.global Reset_Handler
Reset_Handler:
  LDR   R4, =_estack                                       // load address at end of the stack into R0
  MOV   SP, R4                                             // move address at end of stack into SP
  LDR   R4, =_sdata                                        // copy the data segment initializers from flash to SRAM
  LDR   R5, =_edata                                        // copy the data segment initializers from flash to SRAM
  LDR   R6, =_sidata                                       // copy the data segment initializers from flash to SRAM
  MOVS  R7, #0                                             // copy the data segment initializers from flash to SRAM
  B     .Reset_Handler_Loop_Copy_Data_Init                 // branch
.Reset_Handler_Copy_Data_Init:
  LDR   R8, [R6, R7]                                       // copy the data segment initializers into registers
  STR   R8, [R4, R7]                                       // copy the data segment initializers into registers
  ADDS  R7, R7, #4                                         // copy the data segment initializers into registers
.Reset_Handler_Loop_Copy_Data_Init:
  ADDS  R8, R4, R7                                         // initialize the data segment
  CMP   R8, R5                                             // initialize the data segment
  BCC   .Reset_Handler_Copy_Data_Init                      // branch if carry is clear
  LDR   R6, =_sbss                                         // copy the bss segment initializers from flash to SRAM
  LDR   R8, =_ebss                                         // copy the bss segment initializers from flash to SRAM
  MOVS  R7, #0                                             // copy the bss segment initializers from flash to SRAM
  B     .Reset_Handler_Loop_Fill_Zero_BSS                  // branch
.Reset_Handler_Fill_Zero_BSS:
  STR   R7, [R6]                                           // zero fill the bss segment
  ADDS  R6, R6, #4                                         // zero fill the bss segment
.Reset_Handler_Loop_Fill_Zero_BSS:
  CMP   R6, R8                                             // zero fill the bss segment
  BCC   .Reset_Handler_Fill_Zero_BSS                       // branch if carry is clear
  BL    main                                               // call function

/**
 * @brief  This code is called when the processor receives and unexpected interrupt.
 *
 *         This is the code that gets called when the processor receives an unexpected interrupt. This simply enters 
 *         an infinite loop, preserving the system state for examination by a debugger.
 *
 * @param  None
 * @retval None
 */
.type Default_Handler, %function
.global Default_Handler
Default_Handler:
  BKPT                                                     // set processor into debug state
  B.N   Default_Handler                                    // call function, force thumb state

/**
 * Initialize the .text section. The .text section contains executable code.
 */
.section .text

/**
 * @brief  Entry point for initialization and setup of specific functions and main routine.
 *
 *         This function is the entry point for initializing and setting up specific functions. It calls other 
 *         functions to enable certain features and then enters a loop for further execution.
 *
 * @param  None
 * @retval None
 */
.type main, %function
.global main
main:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  BL    GPIOA_Enable                                       // call function
  BL    GPIOB_Enable                                       // call function
  BL    GPIOC_Enable                                       // call function
  BL    I2C1_Enable                                        // call function
  BL    GPIOA_PA2_Alt_Function_Mode_Enable                 // call function
  BL    GPIOA_PA3_Alt_Function_Mode_Enable                 // call function
  BL    GPIOB_PB8_Alt_Function_Mode_Enable                 // call function
  BL    GPIOB_PB8_Open_Drain_Enable                        // call function
  BL    GPIOB_PB9_Alt_Function_Mode_Enable                 // call function
  BL    GPIOB_PB9_Open_Drain_Enable                        // call function
  BL    GPIOB_PB0_Input_Enable                             // call function
  BL    GPIOB_PB1_Input_Enable                             // call function
  BL    GPIOB_PB2_Input_Enable                             // call function
  BL    GPIOB_PB3_Input_Enable                             // call function
  BL    GPIOB_PB4_Input_Enable                             // call function
  BL    GPIOB_PB5_Input_Enable                             // call function
  BL    I2C1_Init                                          // call function
  BL    SSD1306_Init                                       // call function
  BL    SSD1306_Turn_On_Display                            // call function
  BL    USART2_Enable                                      // call function
  BL    USART2_Transmit_Enable                             // call function
  BL    USART2_Receive_Enable                              // call function
  //BL    LoRa_Setup                                       // call function
  BL    Loop                                               // call function
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables the GPIOA peripheral by setting the corresponding RCC_AHB1ENR bit.
 *
 * @details This function enables the GPIOA peripheral by setting the corresponding RCC_AHB1ENR bit. It loads the 
 *          address of the RCC_AHB1ENR register, retrieves the current value of the register, sets the GPIOAEN bit, 
 *          and stores the updated value back into the register.
 *
 * @param   None
 * @retval  None
 */
GPIOA_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40023830                                    // load address of RCC_AHB1ENR register
  LDR   R5, [R4]                                           // load value inside RCC_AHB1ENR register
  ORR   R5, #(1<<0)                                        // set the GPIOAEN bit
  STR   R5, [R4]                                           // store value into RCC_AHB1ENR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables the GPIOB peripheral by setting the corresponding RCC_AHB1ENR bit.
 *
 * @details This function enables the GPIOB peripheral by setting the corresponding RCC_AHB1ENR bit. It loads the 
 *          address of the RCC_AHB1ENR register, retrieves the current value of the register, sets the GPIOBEN bit, 
 *          and stores the updated value back into the register.
 *
 * @param   None
 * @retval  None
 */
GPIOB_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40023830                                    // load address of RCC_AHB1ENR register
  LDR   R5, [R4]                                           // load value inside RCC_AHB1ENR register
  ORR   R5, #(1<<1)                                        // set the GPIOBEN bit
  STR   R5, [R4]                                           // store value into RCC_AHB1ENR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables the GPIOC peripheral by setting the corresponding RCC_AHB1ENR bit.
 *
 * @details This function enables the GPIOC peripheral by setting the corresponding RCC_AHB1ENR bit.  It loads the 
 *          address of the RCC_AHB1ENR register, retrieves the current value of the register, sets the GPIOCEN bit, 
 *          and stores the updated value back into the register.
 *
 * @param   None
 * @retval  None
 */
GPIOC_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40023830                                    // load address of RCC_AHB1ENR register
  LDR   R5, [R4]                                           // load value inside RCC_AHB1ENR register
  ORR   R5, #(1<<2)                                        // set the GPIOCEN bit
  STR   R5, [R4]                                           // store value into RCC_AHB1ENR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOA pin 2 to operate in alternate function mode.
 *
 * @details This function configures GPIOA pin 2 to operate in alternate function mode. It modifies the GPIOA_MODER 
 *          and GPIOA_AFRL registers to set the necessary bits for alternate function mode on pin 2. The MODER2 bit 
 *          is set to select alternate function mode, and the AFRL2 bits are set to specify the desired alternate 
 *          function.
 *
 * @param   None
 * @retval  None
 */
GPIOA_PA2_Alt_Function_Mode_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020000                                    // load address of GPIOA_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOA_MODER register
  ORR   R5, #(1<<5)                                        // set the MODER2 bit
  BIC   R5, #(1<<4)                                        // clear the MODER2 bit
  STR   R5, [R4]                                           // store value into GPIOA_MODER register
  LDR   R4, =0x40020020                                    // load address of GPIOA_AFRL register
  LDR   R5, [R4]                                           // load value inside GPIOA_AFRL register
  BIC   R5, #(1<<11)                                       // clear the AFRL2 bit
  ORR   R5, #(1<<10)                                       // set the AFRL2 bit
  ORR   R5, #(1<<9)                                        // set the AFRL2 bit
  ORR   R5, #(1<<8)                                        // set the AFRL2 bit
  STR   R5, [R4]                                           // store value into GPIOA_AFRL register
  POP   {R4-R12, LR}                                       // pop registers R4-R12 , LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOA pin 3 to operate in alternate function mode.
 *
 * @details This function configures GPIOA pin 3 to operate in alternate function mode. It modifies the GPIOA_MODER 
 *          and GPIOA_AFRL registers to set the necessary bits for alternate function mode on pin 3. The MODER3 bit 
 *          is set to select alternate function mode, and the AFRL3 bits are set to specify the desired alternate
 *          function.
 *
 * @param   None
 * @retval  None
 */
GPIOA_PA3_Alt_Function_Mode_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020000                                    // load address of GPIOA_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOA_MODER register
  ORR   R5, #(1<<7)                                        // set the MODER3 bit
  BIC   R5, #(1<<6)                                        // clear the MODER3 bit
  STR   R5, [R4]                                           // store value into GPIOA_MODER register
  LDR   R4, =0x40020020                                    // load address of GPIOA_AFRL register
  LDR   R5, [R4]                                           // load value inside GPIOA_AFRL register
  BIC   R5, #(1<<15)                                       // clear the AFRL3 bit
  ORR   R5, #(1<<14)                                       // set the AFRL3 bit
  ORR   R5, #(1<<13)                                       // set the AFRL3 bit
  ORR   R5, #(1<<12)                                       // set the AFRL3 bit
  STR   R5, [R4]                                           // store value into GPIOA_AFRL register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables Alternative Function Mode on GPIOB Pin 8.
 *
 * @details This assembly function enables the Alternative Function Mode on GPIOB Pin 8 by configuring the 
 *          corresponding bits in the GPIOB_MODER and GPIOB_AFRH registers. It sets the pin to Alternative Function 
 *          Mode and configures the specific alternative function for Pin 8.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB8_Alt_Function_Mode_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020400                                    // load address of GPIOB_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOB_MODER register
  ORR   R5, #(1<<17)                                       // set the MODER8 bit
  BIC   R5, #(1<<16)                                       // clear the MODER8 bit
  STR   R5, [R4]                                           // store value into GPIOB_MODER register
  LDR   R4, =0x40020424                                    // load address of GPIOB_AFRH register
  LDR   R5, [R4]                                           // load value inside GPIOB_AFRH register
  BIC   R5, #(1<<3)                                        // clear the AFRH8 bit
  ORR   R5, #(1<<2)                                        // set the AFRH8 bit
  BIC   R5, #(1<<1)                                        // clear the AFRH8 bit
  BIC   R5, #(1<<0)                                        // clear the AFRH8 bit
  STR   R5, [R4]                                           // store value into GPIOB_AFRH register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables Open Drain Mode on GPIOB Pin 8.
 *
 * @details This assembly function enables Open Drain Mode on GPIOB Pin 8 by setting the corresponding bit in the 
 *          GPIOB_OTYPER register.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB8_Open_Drain_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020404                                    // load address of GPIOB_OTYPER register
  LDR   R5, [R4]                                           // load value inside GPIOB_OTYPER register
  ORR   R5, #(1<<8)                                        // set the OT8 bit
  STR   R5, [R4]                                           // store value into GPIOB_OTYPER register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables Alternative Function Mode on GPIOB Pin 9.
 *
 * @details This assembly function enables the Alternative Function Mode on GPIOB Pin 9 by configuring the 
 *          corresponding bits in the GPIOB_MODER and GPIOB_AFRH registers. It sets the pin to Alternative Function 
 *          Mode and configures the specific alternative function for Pin 9.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB9_Alt_Function_Mode_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020400                                    // load address of GPIOB_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOB_MODER register
  ORR   R5, #(1<<19)                                       // set the MODER9 bit
  BIC   R5, #(1<<18)                                       // clear the MODER9 bit
  STR   R5, [R4]                                           // store value into GPIOB_MODER register
  LDR   R4, =0x40020424                                    // load address of GPIOB_AFRH register
  LDR   R5, [R4]                                           // load value inside GPIOB_AFRH register
  BIC   R5, #(1<<7)                                        // clear the AFRH9 bit
  ORR   R5, #(1<<6)                                        // set the AFRH9 bit
  BIC   R5, #(1<<5)                                        // clear the AFRH9 bit
  BIC   R5, #(1<<4)                                        // clear the AFRH9 bit
  STR   R5, [R4]                                           // store value into GPIOB_AFRH register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables Open Drain Mode on GPIOB Pin 9.
 *
 * @details This assembly function enables Open Drain Mode on GPIOB Pin 9 by setting the corresponding bit in the 
 *          GPIOB_OTYPER register.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB9_Open_Drain_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020404                                    // load address of GPIOB_OTYPER register
  LDR   R5, [R4]                                           // load value inside GPIOB_OTYPER register
  ORR   R5, #(1<<9)                                        // set the OT9 bit
  STR   R5, [R4]                                           // store value into GPIOB_OTYPER register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOB pin 0 to operate in input mode.
 *
 * @details This function configures GPIOB pin 0 to operate in input mode. It modifies the GPIOB_MODER register to 
 *          set the necessary bits for input mode on pin 0. The MODER0 bit is set to input mode.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB0_Input_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020400                                    // load address of GPIOB_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOB_MODER register
  BIC   R5, #(1<<1)                                        // clear the MODER0 bit
  BIC   R5, #(1<<0)                                        // clear the MODER0 bit
  STR   R5, [R4]                                           // store value into GPIOB_MODER register
  LDR   R4, =0x4002040C                                    // load address of GPIOB_PUPDR register
  LDR   R5, [R4]                                           // load value inside GPIOB_PUPDR register
  BIC   R5, #(1<<1)                                        // clear the PUPDR0 bit
  ORR   R5, #(1<<0)                                        // set the PUPDR0 bit
  STR   R5, [R4]                                           // store value into GPIOB_PUPDR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOB pin 1 to operate in input mode.
 *
 * @details This function configures GPIOB pin 1 to operate in input mode. It modifies the GPIOB_MODER register to 
 *          set the necessary bits for input mode on pin 1. The MODER1 bit is set to input mode.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB1_Input_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020400                                    // load address of GPIOB_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOB_MODER register
  BIC   R5, #(1<<3)                                        // clear the MODER1 bit
  BIC   R5, #(1<<2)                                        // clear the MODER1 bit
  STR   R5, [R4]                                           // store value into GPIOB_MODER register
  LDR   R4, =0x4002040C                                    // load address of GPIOB_PUPDR register
  LDR   R5, [R4]                                           // load value inside GPIOB_PUPDR register
  BIC   R5, #(1<<3)                                        // clear the PUPDR1 bit
  ORR   R5, #(1<<2)                                        // set the PUPDR1 bit
  STR   R5, [R4]                                           // store value into GPIOB_PUPDR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOB pin 2 to operate in input mode.
 *
 * @details This function configures GPIOB pin 2 to operate in input mode. It modifies the GPIOB_MODER register to 
 *          set the necessary bits for input mode on pin 2. The MODER2 bit is set to input mode.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB2_Input_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020400                                    // load address of GPIOB_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOB_MODER register
  BIC   R5, #(1<<5)                                        // clear the MODER2 bit
  BIC   R5, #(1<<4)                                        // clear the MODER2 bit
  STR   R5, [R4]                                           // store value into GPIOB_MODER register
  LDR   R4, =0x4002040C                                    // load address of GPIOB_PUPDR register
  LDR   R5, [R4]                                           // load value inside GPIOB_PUPDR register
  BIC   R5, #(1<<5)                                        // clear the PUPDR2 bit
  ORR   R5, #(1<<4)                                        // set the PUPDR2 bit
  STR   R5, [R4]                                           // store value into GPIOB_PUPDR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOB pin 3 to operate in input mode.
 *
 * @details This function configures GPIOB pin 3 to operate in input mode. It modifies the GPIOB_MODER register to 
 *          set the necessary bits for input mode on pin 3. The MODER3 bit is set to input mode.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB3_Input_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020400                                    // load address of GPIOB_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOB_MODER register
  BIC   R5, #(1<<7)                                        // clear the MODER3 bit
  BIC   R5, #(1<<6)                                        // clear the MODER3 bit
  STR   R5, [R4]                                           // store value into GPIOB_MODER register
  LDR   R4, =0x4002040C                                    // load address of GPIOB_PUPDR register
  LDR   R5, [R4]                                           // load value inside GPIOB_PUPDR register
  BIC   R5, #(1<<7)                                        // clear the PUPDR3 bit
  ORR   R5, #(1<<6)                                        // set the PUPDR3 bit
  STR   R5, [R4]                                           // store value into GPIOB_PUPDR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOB pin 4 to operate in input mode.
 *
 * @details This function configures GPIOB pin 4 to operate in input mode. It modifies the GPIOB_MODER register to 
 *          set the necessary bits for input mode on pin 4. The MODER4 bit is set to input mode.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB4_Input_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020400                                    // load address of GPIOB_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOB_MODER register
  BIC   R5, #(1<<9)                                        // clear the MODER4 bit
  BIC   R5, #(1<<8)                                        // clear the MODER4 bit
  STR   R5, [R4]                                           // store value into GPIOB_MODER register
  LDR   R4, =0x4002040C                                    // load address of GPIOB_PUPDR register
  LDR   R5, [R4]                                           // load value inside GPIOB_PUPDR register
  BIC   R5, #(1<<9)                                        // clear the PUPDR4 bit
  ORR   R5, #(1<<8)                                        // set the PUPDR4 bit
  STR   R5, [R4]                                           // store value into GPIOB_PUPDR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOB pin 5 to operate in input mode.
 *
 * @details This function configures GPIOB pin 5 to operate in input mode. It modifies the GPIOB_MODER register to 
 *          set the necessary bits for input mode on pin 5. The MODER5 bit is set to input mode.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB5_Input_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020400                                    // load address of GPIOB_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOB_MODER register
  BIC   R5, #(1<<11)                                       // clear the MODER5 bit
  BIC   R5, #(1<<10)                                       // clear the MODER5 bit
  STR   R5, [R4]                                           // store value into GPIOB_MODER register
  LDR   R4, =0x4002040C                                    // load address of GPIOB_PUPDR register
  LDR   R5, [R4]                                           // load value inside GPIOB_PUPDR register
  BIC   R5, #(1<<11)                                       // clear the PUPDR5 bit
  ORR   R5, #(1<<10)                                       // set the PUPDR5 bit
  STR   R5, [R4]                                           // store value into GPIOB_PUPDR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables USART2 peripheral and configures its settings for communication using an interrupt.
 *
 * @details This function enables the USART2 peripheral by setting the corresponding RCC_APB1ENR bit.  It also 
 *          configures the USART2 settings, including the baud rate and control register settings.  The USART2_BRR 
 *          register is set to achieve a baud rate of 9600, and the USART2_CR1 register is modified to enable USART2
 *          (UE bit) and receive interrupt (RXNEIE bit) and enable transmission (TE bit) and enable receive (RE bit).
 *
 * @param   None
 * @retval  None
 */
USART2_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40023840                                    // load address of RCC_APB1ENR register
  LDR   R5, [R4]                                           // load value inside RCC_APB1ENR register
  ORR   R5, #(1<<17)                                       // set the USART2EN bit
  STR   R5, [R4]                                           // store value into RCC_AHB1ENR register
  LDR   R4, =0x40004408                                    // load address of USART2_BRR register 
  LDR   R5, [R4]                                           // load value inside USART2_BRR register
  MOV   R5, #0x683                                         // set register to 9600 baud
  STR   R5, [R4]                                           // store value into USART2_BRR register
  LDR   R4, =0x4000440C                                    // load address of USART2_CR1 register
  LDR   R5, [R4]                                           // load value inside USART2_CR1 register
  ORR   R5, #(1<<13)                                       // set the UE bit
  STR   R5, [R4]                                           // store value into USART2_CR1 register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Sends a single character over USART2.
 *
 * @details This function sends a single character over USART2 by writing it to the USART2_DR register.  It first 
 *          checks if the transmit buffer is empty (TXE bit) in the USART2_SR register.  If the buffer is not empty, 
 *          it waits until it becomes empty before writing the character to USART_DR.
 *
 * @param   R0: The character to be sent over USART2.
 * @retval  None
 */
USART2_Transmit_Character:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  MOV   R4, R0                                             // copy first arg into R4
  LDR   R5, =0x40004400                                    // load address of USART2_SR register
.USART2_Transmit_Character_Loop:
  LDR   R6, [R5]                                           // load value inside USART2_SR register
  AND   R6, #(1<<7)                                        // read TXE bit
  CMP   R6, #0x00                                          // test TX FIFO is not full
  BEQ   .USART2_Transmit_Character_Loop                    // branch if equal
  LDR   R5, =0x40004404                                    // load value inside USART2_DR register
  STR   R4, [R5]                                           // store value into USART2_DR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Receives a character over USART2.
 *
 * @details This function receives a character over USART2 by reading the USART2_DR register. The received character
 *          is then returned.
 *
 * @param   None
 * @retval  R0: The received character over USART2.
 */
USART2_Receive_Character:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40004400                                    // load address of USART2_SR register
.USART2_Receive_Character_Loop:
  BL    GPIOB_PB5_Read_Value                               // call function
  CMP   R0, #0x20                                          // check if button is pressed
  BNE   .USART2_Receive_Character_Exit                     // branch if not equal
  LDR   R5, [R4]                                           // load value inside USART_SR register
  AND   R5, #(1<<5)                                        // read the RXNE bit, if RXNE is 0, data is not recv
  CMP   R5, #0                                             // cmp RXNE bit to 0 
  BEQ   .USART2_Receive_Character_Loop                     // branch if equal
  LDR   R4, =0x40004404                                    // load value inside USART2_DR register
  LDRB  R0, [R4]                                           // load byte inside USART2_DR register
.USART2_Receive_Character_Exit:
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Clear USART2 buffer.
 *
 *         This function clears the receive buffer of USART2 by reading the data register (USART2_DR)
 *         and discarding its content.
 *
 * @param  None
 * @retval None
 */
USART2_Clear_Buffer:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40004400                                    // load address of USART2_SR register
  LDR   R5, [R4]                                           // load value inside USART_SR register
  LDR   R4, =0x40004404                                    // load value inside USART2_DR register
  LDR   R5, [R4]                                           // load byte inside USART2_DR register          
  MOV   R5, #0                                             // move zero into R5
  STR   R5, [R4]                                           // store value into USART2_DR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables I2C1 Peripheral.
 *
 * @details This assembly function enables the I2C1 peripheral by setting the corresponding bit in the RCC_APB1ENR
 *          register. It loads the address of the RCC_APB1ENR register, retrieves the current value of the register,
 *          sets the I2C1EN bit, and stores the updated value back into the register.
 *
 * @param   None
 * @retval  None
 */
I2C1_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40023840                                    // load address of RCC_APB1ENR register
  LDR   R5, [R4]                                           // load value inside RCC_APB1ENR register
  ORR   R5, #(1<<21)                                       // set the I2C1EN bit
  STR   R5, [R4]                                           // store value into RCC_APB1ENR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Initializes and enables the I2C1 peripheral.
 *
 * @details This assembly function initializes and enables the I2C1 peripheral by configuring the relevant bits in
 *          the I2C1_CR1, I2C1_CR2, I2C1_CCR, and I2C1_TRISE registers. It performs a software reset, sets the 
 *          frequency and duty cycle, configures the clock control register, sets the rise time, and finally, enables
 *          the I2C1 peripheral.
 *
 * @param   None
 * @retval  None
 */
I2C1_Init:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40005400                                    // load address of I2C1_CR1 register
  LDR   R5, [R4]                                           // load value inside I2C1_CR1 register
  ORR   R5, #(1<<15)                                       // set the SWRST bit
  STR   R5, [R4]                                           // store value into I2C1_CR1 register
  BIC   R5, #(1<<15)                                       // clear the SWRST bit
  STR   R5, [R4]                                           // store value into I2C1_CR1 register
  LDR   R4, =0x40005404                                    // load address of I2C1_CR2 register
  LDR   R5, [R4]                                           // load value inside I2C1_CR2 register
  ORR   R5, #(1<<5)                                        // set the FREQ bit
  ORR   R5, #(1<<4)                                        // set the FREQ bit
  BIC   R5, #(1<<3)                                        // clear the FREQ bit
  BIC   R5, #(1<<2)                                        // clear the FREQ bit
  ORR   R5, #(1<<1)                                        // set the FREQ bit
  BIC   R5, #(1<<0)                                        // clear the FREQ bit
  STR   R5, [R4]                                           // store value into I2C1_CR2 register
  LDR   R4, =0x4000541C                                    // load address of I2C1_CCR register
  LDR   R5, [R4]                                           // load value inside I2C1_CCR register
  ORR   R5, #(1<<15)                                       // set the F/S bit
  ORR   R5, #(1<<14)                                       // set the DUTY bit
  ORR   R5, #(1<<1)                                        // set the CCR bit
  STR   R5, [R4]                                           // store value into I2C1_CCR register
  LDR   R4, =0x40005420                                    // load address of I2C1_TRISE register
  LDR   R5, [R4]                                           // load value inside I2C1_TRISE register
  BIC   R5, #(1<<5)                                        // clear the TRISE bit
  ORR   R5, #(1<<4)                                        // set the TRISE bit
  BIC   R5, #(1<<3)                                        // clear the TRISE bit
  ORR   R5, #(1<<2)                                        // set the TRISE bit
  BIC   R5, #(1<<1)                                        // clear the TRISE bit
  BIC   R5, #(1<<0)                                        // clear the TRISE bit
  STR   R5, [R4]                                           // store value into I2C1_TRISE register
  LDR   R4, =0x40005400                                    // load address of I2C1_CR1 register
  LDR   R5, [R4]                                           // load value inside I2C1_CR1 register
  ORR   R5, #(1<<0)                                        // set the PE bit
  STR   R5, [R4]                                           // store value into I2C1_CR1 register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Initializes the SSD1306 OLED Display.
 *
 * @details This assembly function initializes the SSD1306 OLED display by configuring the I2C communication 
 *          parameters and sending the necessary commands to set up the display parameters.
 *
 * @param   None
 * @retval  None
 */
SSD1306_Init:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  BL    Thirty_Microsecond_Delay                           // call function
  BL    Thirty_Microsecond_Delay                           // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x20                                          // set memory addressing mode, page addressing mode
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xB0                                          // set page start address for page addressing mode (0-7 pages)
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xA1                                          // set segment re-map, col addr 127 mapped to SEG0
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xC8                                          // set COM output scan direction, remapped
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x00                                          // set lower col start addr for page addr mode
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x10                                          // set higher col start addr for page addr mode
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xD5                                          // set display clock
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xF0                                          // divide ratio/oscillator freq
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x12                                          // set COM pins hardware config
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode 
  MOV   R2, #0xDB                                          // set VCOMH deselect level
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x20                                          // 0.77 VCC
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x8D                                          // charge pump setting
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0x14                                          // enable charge pump
  BL    I2C_Write_Byte                                     // call function
  BL    SSD1306_Clear_Screen                               // call function
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Turns on the SSD1306 OLED Display.
 *
 * @details This assembly function turns on the SSD1306 OLED display by sending the necessary command through I2C 
 *          communication to set the display panel to an active state.
 *
 * @param   None
 * @retval  None
 */
SSD1306_Turn_On_Display:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xAF                                          // set display on
  BL    I2C_Write_Byte                                     // call function
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Turns off the SSD1306 OLED Display.
 *
 * @details This assembly function turns off the SSD1306 OLED display by sending the necessary command through I2C
 *          communication to set the display panel to an inactive state.
 *
 * @param   None
 * @retval  None
 */
SSD1306_Turn_Off_Display:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, #0xAE                                          // set display off
  BL    I2C_Write_Byte                                     // call function
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Sets the cursor position on the SSD1306 OLED Display.
 *
 * @details This assembly function sets the cursor position on the SSD1306 OLED display by sending the necessary 
 *          commands through I2C communication. It specifies the lower and higher column start addresses along with
 *          the page start address to define the cursor position.
 *
 * @param   R0: Lower column start address.
 * @param   R1: Higher column start address.
 * @param   R2: Page start address.
 *
 * @retval  None
 */
SSD1306_Set_Cursor:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  MOV   R4, R0                                             // copy first arg into R4
  MOV   R5, R1                                             // copy second arg into R5
  MOV   R6, R2                                             // copy third arg into R6 
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, R4                                             // lower col start addr
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // command mode
  MOV   R2, R5                                             // higher col start addr
  BL    I2C_Write_Byte                                     // call function
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x00                                          // memory addr
  MOV   R2, R6                                             // page start addr
  BL    I2C_Write_Byte                                     // call function
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Clears the screen of the SSD1306 OLED Display.
 *
 * @details This assembly function clears the entire screen of the SSD1306 OLED display by sending the necessary 
 *          commands and data through I2C communication. It utilizes the SSD1306_Set_Cursor function to position the 
 *          cursor at the beginning of the display and then writes data to fill the screen with zeros.
 *
 * @param   None
 * @retval  None
 */
SSD1306_Clear_Screen:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x10                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  BL    SSD1306_Set_Cursor                                 // call function
  MOV   R3, #0x00                                          // set counter
.SSD1306_Clear_Screen_Loop:
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x40                                          // data mode
  MOV   R2, #0                                             // data
  BL    I2C_Write_Byte                                     // call function
  ADD   R3, #0x1                                           // increment counter
  CMP   R3, #0x480                                         // cmp if 0x480
  BNE   .SSD1306_Clear_Screen_Loop                         // branch not equal
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Displays a letter (character array) on the SSD1306 OLED Display.
 *
 * @details This assembly function displays a letter (character array) on the SSD1306 OLED display by utilizing the 
 *          SSD1306_Set_Cursor function to position the cursor and then writing the letter data to the display memory
 *          through I2C communication. The function also calls the SSD1306_Turn_On_Display function to ensure the 
 *          display is active.
 *
 * @param   R0: Lower column start address.
 * @param   R1: Higher column start address.
 * @param   R2: Page start address.
 * @param   R3: Address of the character array to be displayed.
 *
 * @retval  None
 */
SSD1306_Display_Letter:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  MOV   R4, R0                                             // copy first arg into R4
  MOV   R5, R1                                             // copy second arg into R5
  MOV   R6, R2                                             // copy third arg into R6
  MOV   R7, R3                                             // copy fourth arg into R7
  BL    SSD1306_Set_Cursor                                 // call function
  MOV   R8, #0                                             // set counter
.SSD1306_Display_Letter_Loop:
  MOV   R0, #0x3C                                          // SSD1306 I2C addr
  MOV   R1, #0x40                                          // data mode
  LDRB  R2, [R7, R8]                                       // load byte at addr in R7 and inc by counter
  BL    I2C_Write_Byte                                     // call function
  ADDS  R8, #1                                             // increment counter
  CMP   R8, #6                                             // compare if end of array
  BNE   .SSD1306_Display_Letter_Loop                       // branch if not equal
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Writes a byte to the I2C device.
 *
 * @details This assembly function writes a byte to the I2C device. It waits for the device to be ready and sends the
 *          data byte using I2C communication.
 *
 * @param   R0: I2C device address.
 * @param   R1: I2C data mode (0x00 for command, 0x40 for data).
 * @param   R2: Byte of data to be sent.
 *
 * @retval  None
 */
I2C_Write_Byte:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  MOV   R4, R0                                             // copy first arg into R4
  MOV   R5, R1                                             // copy second arg into R5
  MOV   R6, R2                                             // copy third arg into R6
.I2C_Wait_Not_Busy:
  LDR   R7, =0x40005418                                    // load address of I2C1_SR2 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR2 register
  TST   R8, #(1<<1)                                        // read the BUSY bit, if 0, then BNE
  BNE   .I2C_Wait_Not_Busy                                 // branch if not equal
  LDR   R7, =0x40005400                                    // load address of I2C1_CR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_CR1 register
  ORR   R8, #(1<<8)                                        // set the START bit
  ORR   R8, #(1<<0)                                        // set the PE bit
  STR   R8, [R7]                                           // store value into I2C1_CR1 register
.I2C_Wait_Start:
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  TST   R8, #(1<<0)                                        // read the SB bit, if 1, then BEQ
  BEQ   .I2C_Wait_Start                                    // branch if equal
  LDR   R7, =0x40005410                                    // load address of I2C1_DR register
  LSL   R4, #1                                             // left shift to make room for the rw bit
  STR   R4, [R7]                                           // store value into I2C1_DR register
.I2C_Wait_Addr_Flag:
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  TST   R8, #(1<<1)                                        // read the ADDR bit, if 1, then BEQ
  BEQ   .I2C_Wait_Addr_Flag                                // branch if equal
  LDR   R7, =0x40005418                                    // load address of I2C1_SR2 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR2 register
  STR   R8, [R7]                                           // store value into I2C1_SR2 register
.I2C_Wait_Data_Empty_Send_Mem_Addr:
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  TST   R8, #(1<<7)                                        // read the TxE bit, if 0, then BEQ
  BEQ   .I2C_Wait_Data_Empty_Send_Mem_Addr                 // branch if equal
  LDR   R7, =0x40005410                                    // load address of I2C1_DR register
  STR   R5, [R7]                                           // store value into I2C1_DR register
.I2C_Wait_Data_Empty_Send_Data:
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  TST   R8, #(1<<7)                                        // read the TxE bit, if 1, then BNE
  BEQ   .I2C_Wait_Data_Empty_Send_Data                     // branch if equal
.I2C_Send_Data:
  LDR   R7, =0x40005410                                    // load address of I2C1_DR register
  STR   R6, [R7]                                           // store value into I2C1_DR register
.I2C_Wait_Data_Transfer_Finished:
  LDR   R7, =0x40005414                                    // load address of I2C1_SR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_SR1 register
  TST   R8, #(1<<2)                                        // read the BTF bit, if 0, then BEQ
  BEQ   .I2C_Wait_Data_Transfer_Finished                   // branch if equal
  LDR   R7, =0x40005400                                    // load address of I2C1_CR1 register
  LDR   R8, [R7]                                           // load value inside I2C1_CR1 register
  ORR   R8, #(1<<9)                                        // set the STOP bit
  ORR   R8, #(1<<0)                                        // set the PE bit
  STR   R8, [R7]                                           // store value into I2C1_CR1 register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Read the value of pin PB0.
 *
 *         This function reads the value of pin PB0 (GPIO pin 0 of GPIO port B).
 *
 * @param  None
 * @retval uint32_t Value of pin PB0 (0 or 1)
 */
GPIOB_PB0_Read_Value:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020410                                    // load address of GPIOB_IDR register
  LDR   R0, [R4]                                           // load value inside GPIOB_IDR register
  AND   R0, #(1<<0)                                        // read the IDR0 bit
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Read the value of pin PB1.
 *
 *         This function reads the value of pin PB1 (GPIO pin 1 of GPIO port B).
 *
 * @param  None
 * @retval uint32_t Value of pin PB1 (0 or 1)
 */
GPIOB_PB1_Read_Value:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020410                                    // load address of GPIOB_IDR register
  LDR   R0, [R4]                                           // load value inside GPIOB_IDR register
  AND   R0, #(1<<1)                                        // read the IDR1 bit
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Read the value of pin PB2.
 *
 *         This function reads the value of pin PB2 (GPIO pin 2 of GPIO port B).
 *
 * @param  None
 * @retval uint32_t Value of pin PB2 (0 or 1)
 */
GPIOB_PB2_Read_Value:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020410                                    // load address of GPIOB_IDR register
  LDR   R0, [R4]                                           // load value inside GPIOB_IDR register
  AND   R0, #(1<<2)                                        // read the IDR2 bit
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Read the value of pin PB3.
 *
 *         This function reads the value of pin PB3 (GPIO pin 3 of GPIO port B).
 *
 * @param  None
 * @retval uint32_t Value of pin PB3 (0 or 1)
 */
GPIOB_PB3_Read_Value:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020410                                    // load address of GPIOB_IDR register
  LDR   R0, [R4]                                           // load value inside GPIOB_IDR register
  AND   R0, #(1<<3)                                        // read the IDR3 bit
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Read the value of pin PB4.
 *
 *         This function reads the value of pin PB4 (GPIO pin 4 of GPIO port B).
 *
 * @param  None
 * @retval uint32_t Value of pin PB4 (0 or 1)
 */
GPIOB_PB4_Read_Value:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020410                                    // load address of GPIOB_IDR register
  LDR   R0, [R4]                                           // load value inside GPIOB_IDR register
  AND   R0, #(1<<4)                                        // read the IDR4 bit
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Read the value of pin PB5.
 *
 *         This function reads the value of pin PB5 (GPIO pin 5 of GPIO port B).
 *
 * @param  None
 * @retval uint32_t Value of pin PB5 (0 or 1)
 */
GPIOB_PB5_Read_Value:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020410                                    // load address of GPIOB_IDR register
  LDR   R0, [R4]                                           // load value inside GPIOB_IDR register
  AND   R0, #(1<<5)                                        // read the IDR5 bit
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Enable USART2 transmission.
 *
 *         This function enables transmission on USART2 by setting the TE bit in the USART2_CR1 register
 *         and clearing the RE bit to disable reception.
 *
 * @param  None
 * @retval None
 */
USART2_Transmit_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x4000440C                                    // load address of USART2_CR1 register
  LDR   R5, [R4]                                           // load value inside USART2_CR1 register
  ORR   R5, #(1<<3)                                        // set the TE bit
  STR   R5, [R4]                                           // store value into USART2_CR1 register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Enable USART2 reception.
 *
 *         This function enables reception on USART2 by setting the RE bit in the USART2_CR1 register
 *         and clearing the TE bit to disable transmission.
 *
 * @param  None
 * @retval None
 */
USART2_Receive_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x4000440C                                    // load address of USART2_CR1 register
  LDR   R5, [R4]                                           // load value inside USART2_CR1 register
  ORR   R5, #(1<<2)                                        // set the RE bit
  STR   R5, [R4]                                           // store value into USART2_CR1 register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Delay for approximately 30 microseconds.
 *
 *         This function creates a delay of approximately 30 microseconds.
 *
 * @param  None
 * @retval None
 */
Thirty_Microsecond_Delay:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack 
  MOV   R4, #7                                             // number of loops
.Thirty_Microsecond_Delay_Outer_Loop:
  MOV   R5, #0xFFFF                                        // set initial delay count
.Thirty_Microsecond_Delay_Inner_Loop:
  SUB   R5, #1                                             // decrement delay count
  CMP   R5, #0                                             // check if delay count reached zero
  BNE   .Thirty_Microsecond_Delay_Inner_Loop               // continue loop if delay count not reached zero
  SUB   R4, #1                                             // decrement loop counter
  CMP   R4, #0                                             // check if delay count reached zero
  BNE   .Thirty_Microsecond_Delay_Outer_Loop               // continue outer loop if more loops to go
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Menu selection loop.
 *
 *         This function continuously checks for button inputs to perform menu actions.
 *         If GPIOB_PB0 is pressed, it generates a message.
 *         If GPIOB_PB1 is pressed, it receives, parses, and displays a message.
 *         After each action, the function clears the screen and returns to the menu loop.
 *
 * @param  None
 * @retval None
 */
Menu:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x11                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  LDR   R3, =LETTER_M                                      // load the address of array M
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x12                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  LDR   R3, =LETTER_E                                      // load the address of array E
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x13                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  LDR   R3, =LETTER_N                                      // load the address of array N
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x14                                          // higher col start addr
  MOV   R2, #0xB0                                          // page start addr
  LDR   R3, =LETTER_U                                      // load the address of array U
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x11                                          // higher col start addr
  MOV   R2, #0xB3                                          // page start addr
  LDR   R3, =LETTER_T                                      // load the address of array T
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x12                                          // higher col start addr
  MOV   R2, #0xB3                                          // page start addr
  LDR   R3, =LETTER_X                                      // load the address of array X
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x11                                          // higher col start addr
  MOV   R2, #0xB5                                          // page start addr
  LDR   R3, =LETTER_R                                      // load the address of array R
  BL    SSD1306_Display_Letter                             // call function
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, #0x12                                          // higher col start addr
  MOV   R2, #0xB5                                          // page start addr
  LDR   R3, =LETTER_X                                      // load the address of array X
  BL    SSD1306_Display_Letter                             // call function
.Menu_Select:
  BL    Thirty_Microsecond_Delay                           // call function
  BL    GPIOB_PB0_Read_Value                               // call function
  CMP   R0, #0x1                                           // check if button is pressed
  BNE   .Menu_Select_GPIOB_PB0_Button_Pressed              // branch if not equal
  BL    GPIOB_PB1_Read_Value                               // call function
  CMP   R0, #0x2                                           // check if button is pressed
  BNE   .Menu_Select_GPIOB_PB1_Button_Pressed              // branch if not equal
  B     .Menu_Select                                       // call function
.Menu_Select_GPIOB_PB0_Button_Pressed:
  BL    SSD1306_Clear_Screen                               // call function
  BL    Generate_Message                                   // call function
  B     .Menu_Exit                                         // branch
.Menu_Select_GPIOB_PB1_Button_Pressed:
  BL    SSD1306_Clear_Screen                               // call function
  BL    Receive_Message                                    // call function
  BL    Parse_Message                                      // call function
  BL    Display_Message                                    // call function
  B     .Menu_Exit                                         // branch
.Menu_Exit:
  BL    SSD1306_Clear_Screen                               // call function
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Generate a message based on button inputs.
 *
 *         This function generates a message based on button inputs and sends it through USART2.
 *
 * @param  None
 * @retval None
 */
Generate_Message:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  BL    USART2_Clear_Buffer                                // call function
  BL    Thirty_Microsecond_Delay                           // call function
  BL    Thirty_Microsecond_Delay                           // call function
  MOV   R4, #0                                             // set array index to zero
  MOV   R5, #0x11                                          // higher col start addr
  LDR   R6, =LETTERS_ARRAY                                 // load the address of array
  MOV   R7, #0                                             // set array pointer
  LDR   R8, =message_transmit_array                        // load addr of message array
  MOV   R9, #0                                             // set current char
  MOV   R10, #0x11                                         // set left of screen
  MOV   R11, #0x15                                         // set right of screen
  MOV   R12, #0xB0                                         // page start addr
.Generate_Message_Loop:
  BL    Thirty_Microsecond_Delay                           // call function
  BL    GPIOB_PB0_Read_Value                               // call function
  CMP   R0, #0x1                                           // check if button is pressed
  BNE   .Generate_Message_GPIOB_PB0_Button_Pressed         // branch if not equal
  BL    GPIOB_PB1_Read_Value                               // call function
  CMP   R0, #0x2                                           // check if button is pressed
  BNE   .Generate_Message_GPIOB_PB1_Button_Pressed         // branch if not equal
  BL    GPIOB_PB2_Read_Value                               // call function
  CMP   R0, #0x4                                           // check if button is pressed
  BNE   .Generate_Message_GPIOB_PB2_Button_Pressed         // branch if not equal
  BL    GPIOB_PB3_Read_Value                               // call function
  CMP   R0, #0x8                                           // check if button is pressed
  BNE   .Generate_Message_GPIOB_PB3_Button_Pressed         // branch if not equal
  BL    GPIOB_PB4_Read_Value                               // call function
  CMP   R0, #0x10                                          // check if button is pressed
  BNE   .Generate_Message_GPIOB_PB4_Button_Pressed         // branch if not equal
  BL    GPIOB_PB5_Read_Value                               // call function
  CMP   R0, #0x20                                          // check if button is pressed
  BNE   .Generate_Message_GPIOB_PB5_Button_Pressed         // branch if not equal
  B     .Generate_Message_Loop                             // branch
.Generate_Message_GPIOB_PB0_Button_Pressed:
  BL    Thirty_Microsecond_Delay                           // call function
  CMP   R5, R11                                            // check if we are at right of screen
  BEQ   .Generate_Message_Exit                             // branch if equal
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R12                                            // page start addr
  LDR   R3, [R6, R4, LSL #2]                               // load the char based on index in R6
  LDRB  R9, [R3, #6]                                       // move current char byte at addr into R9
  BL    SSD1306_Display_Letter                             // call function
  ADD   R4, #1                                             // increment index
  CMP   R4, #26                                            // check if end of array is reached
  BNE   .Generate_Message_Loop                             // if not, continue loop
  MOV   R4, #0                                             // reset index to 0 when end of array is reached
  B     .Generate_Message_Loop                             // continue loop
.Generate_Message_GPIOB_PB1_Button_Pressed:
  BL    Thirty_Microsecond_Delay                           // call function
  CMP   R5, R11                                            // check if we are at right of screen
  BEQ   .Generate_Message_Exit                             // branch if equal
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R12                                            // page start addr
  LDR   R3, [R6, R4, LSL #2]                               // load the char based on index in R6
  LDRB  R9, [R3, #6]                                       // move current char byte at addr into R9 
  BL    SSD1306_Display_Letter                             // call function
  SUB   R4, #1                                             // increment index
  CMP   R4, #-1                                            // check if end of array is reached
  BNE   .Generate_Message_Loop                             // if not, continue loop
  MOV   R4, #25                                            // reset index to array end
  B     .Generate_Message_Loop                             // continue loop
.Generate_Message_GPIOB_PB2_Button_Pressed:
  BL    Thirty_Microsecond_Delay                           // call function
  CMP   R5, R11                                            // check if we are at right of screen
  BEQ   .Generate_Message_Loop                             // branch if equal
  STR   R9, [R8, R7]                                       // store char in the array at index in R4
  ADD   R5, #1                                             // increment page addr
  ADD   R7, #1                                             // increment array pointer
  B     .Generate_Message_Loop                             // continue loop
.Generate_Message_GPIOB_PB3_Button_Pressed:
  BL    Thirty_Microsecond_Delay                           // call function
  CMP   R5, R10                                            // check if we are at left of screen
  BEQ   .Generate_Message_Loop                             // branch if equal
  STR   R9, [R8, R7]                                       // store char in the array at index in R4
  SUB   R5, #1                                             // increment page addr
  ADD   R7, #1                                             // increment array pointer
  B     .Generate_Message_Loop                             // continue loop
.Generate_Message_GPIOB_PB4_Button_Pressed:
  BL    Thirty_Microsecond_Delay                           // call function
  MOV   R5, #0                                             // init index for the array
  MOV   R6, #4                                             // array length
.Generate_Message_Send_Char_Loop:
  CMP   R5, R6                                             // compare current index with array length
  BEQ   .Generate_Message_Exit                             // branch if equal
  LDRB  R7, [R8, R5]                                       // load addr from the array at index R5 
  MOV   R0, #0x41                                          // 'A'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x54                                          // 'T'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x2B                                          // '+'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x53                                          // 'S'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x45                                          // 'E'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x4E                                          // 'N'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x44                                          // 'D'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x3D                                          // '='
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x30                                          // '0'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x2C                                          // ','
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x31                                          // '1'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x2C                                          // ','
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, R7                                             // current char at array position in R7
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0xD                                           // '\r'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0xA                                           // '\n'
  BL    USART2_Transmit_Character                          // call function
  ADD   R5, #1                                             // increment index
  BL    Thirty_Microsecond_Delay                           // call function
  BL    Thirty_Microsecond_Delay                           // call function
  BL    Thirty_Microsecond_Delay                           // call function
  BL    Thirty_Microsecond_Delay                           // call function
  B     .Generate_Message_Send_Char_Loop                   // branch 
.Generate_Message_GPIOB_PB5_Button_Pressed:
  BL    Thirty_Microsecond_Delay                           // call function
  B     .Generate_Message_Exit                             // branch
.Generate_Message_Exit:
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Receive a message.
 *
 *         This function receives a message character by character through USART2 and stores it in the 
 *         message full receive array.
 *
 * @param  None
 * @retval None
 */
Receive_Message:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack 
  LDR   R4, =message_full_receive_array                    // load addr of message full receive array
  MOV   R5, #-1                                            // set counter                                                                             
.Receive_Message_Loop:
  CMP   R5, #75                                            // compare if at end of our recv buffer
  BEQ   .Receive_Message_Exit                              // branch if equal
  ADD   R5, #1                                             // increment counter     
  BL    USART2_Receive_Character                           // call function
  STRB  R0, [R4, R5]                                       // store char into message_full_receive_array, offset R5
  B     .Receive_Message_Loop                              // branch
.Receive_Message_Exit:  
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Parse a received message.
 *
 *         This function parses a received message stored in the message_full_receive_array
 *         and extracts specific characters to store in the message parsed receive array.
 *
 * @param  None
 * @retval None
 */
Parse_Message:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =message_full_receive_array                    // load addr of message full receive array
  LDR   R5, =message_parsed_receive_array                  // load addr of message parsed receive array
  MOV   R7, #2                                             // set counter
.Parse_Message_Check_Third_Byte:
  LDRB  R6, [R4, R7]                                       // load first byte into R6
  CMP   R6, #0x43                                          // check to make sure it is 'C'
  BEQ   .Parse_Message_Loop                                // branch if equal
  ADD   R7, #1                                             // increment counter
  B     .Parse_Message_Check_Third_Byte                    // branch
.Parse_Message_Loop:
  ADD   R8, R7, #7                                         // increment counter to get first correct character
  LDRB  R6, [R4, R8]                                       // load offset of message full receive array
  STRB  R6, [R5, #0]                                       // store byte message_parsed_receive_array
  ADD   R8, R7, #26                                        // increment counter to get second correct character
  LDRB  R6, [R4, R8]                                       // load offset 28 of message full receive array
  STRB  R6, [R5, #1]                                       // store byte message_parsed_receive_array
  ADD   R8, R7, #45                                        // increment counter to get second correct character
  LDRB  R6, [R4, R8]                                       // load offset of message full receive array
  STRB  R6, [R5, #2]                                       // store byte message_parsed_receive_array
  ADD   R8, R7, #64                                        // increment counter to get second correct character
  LDRB  R6, [R4, R8]                                       // load offset of message full receive array
  STRB  R6, [R5, #3]                                       // store byte message_parsed_receive_array
.Parse_Message_Exit:
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller 

/**
 * @brief  Display a received message.
 *
 *         This function displays a received message on an SSD1306 OLED display. It iterates through the characters 
 *         of the message and calls a function to display each character on the OLED screen.
 *
 * @param  None
 * @retval None
 */
Display_Message:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =message_parsed_receive_array                  // load addr of message parsed receive array
  MOV   R5, #0x14                                          // set counter
  MOV   R6, #-1                                            // set counter
  MOV   R7, #0xB1                                          // page start addr
  MOV   R8, #0x19                                          // set right of screen
.Display_Message_Loop:
  CMP   R6, #3                                             // compare if at end of our message parsed recv buffer
  BEQ   .Display_Message_Until_Exit                        // branch if equal
  ADD   R6, #1                                             // increment counter
  LDRB  R0, [R4, R6]                                       // load byte in message parsed receive array offset R6
  CMP   R0, 0x41                                           // compare 'A'
  BEQ   .Display_Message_Letter_A                          // branch if equal
  CMP   R0, 0x42                                           // compare 'B'
  BEQ   .Display_Message_Letter_B                          // branch if equal
  CMP   R0, 0x43                                           // compare 'C'
  BEQ   .Display_Message_Letter_C                          // branch if equal
  CMP   R0, 0x44                                           // compare 'D'
  BEQ   .Display_Message_Letter_D                          // branch if equal
  CMP   R0, 0x45                                           // compare 'E'
  BEQ   .Display_Message_Letter_E                          // branch if equal
  CMP   R0, 0x46                                           // compare 'F'
  BEQ   .Display_Message_Letter_F                          // branch if equal
  CMP   R0, 0x47                                           // compare 'G'
  BEQ   .Display_Message_Letter_G                          // branch if equal
  CMP   R0, 0x48                                           // compare 'H'
  BEQ   .Display_Message_Letter_H                          // branch if equal
  CMP   R0, 0x49                                           // compare 'I'
  BEQ   .Display_Message_Letter_I                          // branch if equal
  CMP   R0, 0x4A                                           // compare 'J'
  BEQ   .Display_Message_Letter_J                          // branch if equal
  CMP   R0, 0x4B                                           // compare 'K'
  BEQ   .Display_Message_Letter_K                          // branch if equal
  CMP   R0, 0x4C                                           // compare 'L'
  BEQ   .Display_Message_Letter_L                          // branch if equal
  CMP   R0, 0x4D                                           // compare 'M'
  BEQ   .Display_Message_Letter_M                          // branch if equal
  CMP   R0, 0x4E                                           // compare 'N'
  BEQ   .Display_Message_Letter_N                          // branch if equal
  CMP   R0, 0x4F                                           // compare 'O'
  BEQ   .Display_Message_Letter_O                          // branch if equal
  CMP   R0, 0x50                                           // compare 'P'
  BEQ   .Display_Message_Letter_P                          // branch if equal
  CMP   R0, 0x51                                           // compare 'Q'
  BEQ   .Display_Message_Letter_Q                          // branch if equal
  CMP   R0, 0x52                                           // compare 'R'
  BEQ   .Display_Message_Letter_R                          // branch if equal
  CMP   R0, 0x53                                           // compare 'S'
  BEQ   .Display_Message_Letter_S                          // branch if equal
  CMP   R0, 0x54                                           // compare 'T'
  BEQ   .Display_Message_Letter_T                          // branch if equal
  CMP   R0, 0x55                                           // compare 'U'
  BEQ   .Display_Message_Letter_U                          // branch if equal
  CMP   R0, 0x56                                           // compare 'V'
  BEQ   .Display_Message_Letter_V                          // branch if equal
  CMP   R0, 0x57                                           // compare 'W'
  BEQ   .Display_Message_Letter_W                          // branch if equal
  CMP   R0, 0x58                                           // compare 'X'
  BEQ   .Display_Message_Letter_X                          // branch if equal
  CMP   R0, 0x59                                           // compare 'Y'
  BEQ   .Display_Message_Letter_Y                          // branch if equal
  CMP   R0, 0x5A                                           // compare 'Z'
  BEQ   .Display_Message_Letter_Z                          // branch if equal
  CMP   R0, 0x00                                           // compare null
  BEQ   .Display_Message_LETTER_NULL                       // branch if equal
.Display_Message_Letter_A:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_A                                      // load the address of array A
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_B:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_B                                      // load the address of array B
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_C:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_C                                      // load the address of array C
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_D:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_D                                      // load the address of array D
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_E:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_E                                      // load the address of array E
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_F:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_F                                      // load the address of array F
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_G:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_G                                      // load the address of array G
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_H:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_H                                      // load the address of array H
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_I:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_I                                      // load the address of array I
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_J:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_J                                      // load the address of array J
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_K:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_K                                      // load the address of array K
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_L:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_L                                      // load the address of array L
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_M:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_M                                      // load the address of array M
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_N:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_N                                      // load the address of array N
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_O:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_O                                      // load the address of array O
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_P:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_P                                      // load the address of array P
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_Q:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_Q                                      // load the address of array Q
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_R:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_R                                      // load the address of array R
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_S:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_S                                      // load the address of array S
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_T:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_T                                      // load the address of array T
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_U:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_U                                      // load the address of array U
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_V:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_V                                      // load the address of array V
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_W:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_W                                      // load the address of array W
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_X:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_X                                      // load the address of array X
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_Y:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_Y                                      // load the address of array Y
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Letter_Z:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_Z                                      // load the address of array Z
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_LETTER_NULL:
  MOV   R0, #0x00                                          // lower col start addr
  MOV   R1, R5                                             // higher col start addr
  MOV   R2, R7                                             // page start addr
  LDR   R3, =LETTER_NULL                                   // load the address of array Z
  BL    SSD1306_Display_Letter                             // call function
  CMP   R5, R8                                             // check if we are at right of screen
  BNE   .Display_Message_Add_Higher_Col_Value              // branch if not equal
  B     .Display_Message_Loop                              // branch
.Display_Message_Add_Higher_Col_Value:
  ADD   R5, #1                                             // increment higher col addr
  B     .Display_Message_Loop                              // branch
.Display_Message_Until_Exit:  
  BL    GPIOB_PB5_Read_Value                               // call function
  CMP   R0, #0x20                                          // check if button is pressed
  BNE   .Display_Message_Exit                              // branch if not equal
  B     .Display_Message_Until_Exit                        // branch
.Display_Message_Exit:
  MOV   R5, #0x11                                          // higher col start addr 
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Initialize LoRa module setup.
 *
 *         This function sets up the LoRa module by transmitting specific commands via USART2 for configuration.
 *
 * @param  None
 * @retval None
 */
LoRa_Setup:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  MOV   R0, #0x41                                          // 'A'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x54                                          // 'T'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x2B                                          // '+'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x49                                          // 'I'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x50                                          // 'P'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x52                                          // 'R'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x3D                                          // '='
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x39                                          // '9'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x36                                          // '6'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x30                                          // '0'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x30                                          // '0'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0xD                                           // '\r'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0xA                                           // '\n'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x41                                          // 'A'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x54                                          // 'T'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x2B                                          // '+'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x41                                          // 'A'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x44                                          // 'D'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x44                                          // 'D'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x52                                          // 'R'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x45                                          // 'E'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x53                                          // 'S'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x53                                          // 'S'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x3D                                          // '='
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0x30                                          // '0'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0xA                                           // '\r'
  BL    USART2_Transmit_Character                          // call function
  MOV   R0, #0xD                                           // '\n'
  BL    USART2_Transmit_Character                          // call function
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief  Infinite loop function.
 *
 *         This function implements an infinite loop using an unconditional branch, B, statement. It is designed to 
 *         keep the program running indefinitely by branching back to itself.
 *
 * @param  None
 * @retval None
 */
Loop:
  BL    Menu                                               // call function
  B     Loop                                               // infinite loop


/**
 * Initialize the .rodata section. The .rodata section is used for constants and static strings.
 */
.section .rodata

LETTERS_ARRAY:
  .word LETTER_A
  .word LETTER_B
  .word LETTER_C 
  .word LETTER_D 
  .word LETTER_E 
  .word LETTER_F
  .word LETTER_G
  .word LETTER_H
  .word LETTER_I
  .word LETTER_J
  .word LETTER_K
  .word LETTER_L
  .word LETTER_M 
  .word LETTER_N 
  .word LETTER_O
  .word LETTER_P 
  .word LETTER_Q 
  .word LETTER_R 
  .word LETTER_S 
  .word LETTER_T 
  .word LETTER_U 
  .word LETTER_V
  .word LETTER_W 
  .word LETTER_X
  .word LETTER_Y
  .word LETTER_Z

LETTER_A:
  .byte 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C, 0x41
LETTER_B:
  .byte 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x42
LETTER_C:
  .byte 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, 0x43
LETTER_D:
  .byte 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x44
LETTER_E:
  .byte 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x45
LETTER_F:
  .byte 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, 0x46
LETTER_G:
  .byte 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A, 0x47
LETTER_H:
  .byte 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x48
LETTER_I:
  .byte 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00, 0x49
LETTER_J:
  .byte 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x4A
LETTER_K:
  .byte 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x4B
LETTER_L:
  .byte 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x4C
LETTER_M:
  .byte 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x4D
LETTER_N:
  .byte 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x4E
LETTER_O:
  .byte 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x4F
LETTER_P:
  .byte 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x50
LETTER_Q:
  .byte 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x51
LETTER_R:
  .byte 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x52
LETTER_S:
  .byte 0x00, 0x46, 0x49, 0x49, 0x49, 0x31, 0x53
LETTER_T:
  .byte 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x54
LETTER_U:
  .byte 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x55
LETTER_V:
  .byte 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x56
LETTER_W:
  .byte 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x57
LETTER_X:
  .byte 0x00, 0x63, 0x14, 0x08, 0x14, 0x63, 0x58
LETTER_Y:
  .byte 0x00, 0x07, 0x08, 0x70, 0x08, 0x07, 0x59
LETTER_Z:
  .byte 0x00, 0x61, 0x51, 0x49, 0x45, 0x43, 0x5A
LETTER_NULL:
  .byte 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00


/**
 * Initialize the .data section. The .data section is used for initialized global or static variables.
 */
.section .data

message_transmit_array:
  .space 4

message_full_receive_array:
  .space 76

message_parsed_receive_array:
  .space 4


/**
 * Initialize the .bss section. The .bss section is typically used for uninitialized global or static variables.
 */
.section .bss
```

<br>

## License
[MIT](https://raw.githubusercontent.com/mytechnotalent/STM32F4_LoRa_UART_Driver/main/LICENSE)
