
/*
 * Auto generated Run-Time-Environment Component Configuration File
 *      *** Do not modify ! ***
 *
 * Project: 'M4-WSE'
 * Target:  'Debug'
 */

#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H

/*******************************************************************************
 * RTOS Implements
 */
#define RTE_CMSIS_RTOS                  /* CMSIS-RTOS */
#define RTE_CMSIS_RTOS_RTX              /* CMSIS-RTOS Keil RTX */

/*******************************************************************************
 * Re-Target Implements
 */
#define RTE_Compiler_IO_STDERR          /* Compiler I/O: STDERR */
#define RTE_Compiler_IO_STDIN           /* Compiler I/O: STDIN */
#define RTE_Compiler_IO_STDOUT          /* Compiler I/O: STDOUT */
#define RTE_Compiler_IO_TTY             /* Compiler I/O: TTY */
/*
 * Re-Target to ITM
 */
#define RTE_Compiler_IO_STDERR_ITM      /* Compiler I/O: STDERR ITM */
#define RTE_Compiler_IO_STDIN_ITM       /* Compiler I/O: STDIN ITM */
#define RTE_Compiler_IO_STDOUT_ITM      /* Compiler I/O: STDOUT ITM */
#define RTE_Compiler_IO_TTY_ITM         /* Compiler I/O: TTY ITM */
/*
 * Re-Target to user define I/O
 */
//#define RTE_Compiler_IO_STDERR_User     /* Compiler I/O: STDERR User */
//#define RTE_Compiler_IO_STDIN_User      /* Compiler I/O: STDIN User */
//#define RTE_Compiler_IO_STDOUT_User     /* Compiler I/O: STDOUT User */
//#define RTE_Compiler_IO_TTY_User        /* Compiler I/O: TTY User */
//#define RTE_Compiler_IO_File_FS         /*

/*******************************************************************************
 * STM32F4xx HAL Driver Implements
 */
#define RTE_DEVICE_FRAMEWORK_CLASSIC
#define RTE_DEVICE_HAL_COMMON
#define RTE_DEVICE_HAL_ADC
#define RTE_DEVICE_HAL_CAN
//#define RTE_DEVICE_HAL_CRC
//#define RTE_DEVICE_HAL_CEC
//#define RTE_DEVICE_HAL_CRYP
#define RTE_DEVICE_HAL_DAC
//#define RTE_DEVICE_HAL_DCMI
#define RTE_DEVICE_HAL_DMA
#define RTE_DEVICE_HAL_DMA2D
//#define RTE_DEVICE_HAL_ETH
#define RTE_DEVICE_HAL_FLASH
//#define RTE_DEVICE_HAL_NAND
//#define RTE_DEVICE_HAL_NOR
//#define RTE_DEVICE_HAL_PCCARD
//#define RTE_DEVICE_HAL_SRAM
//#define RTE_DEVICE_HAL_SDRAM
//#define RTE_DEVICE_HAL_HASH
#define RTE_DEVICE_HAL_GPIO
#define RTE_DEVICE_HAL_I2C
//#define RTE_DEVICE_HAL_I2S
#define RTE_DEVICE_HAL_IWDG
//#define RTE_DEVICE_HAL_LTDC
//#define RTE_DEVICE_HAL_DSI
#define RTE_DEVICE_HAL_PWR
//#define RTE_DEVICE_HAL_QSPI
#define RTE_DEVICE_HAL_RCC
#define RTE_DEVICE_HAL_RNG
#define RTE_DEVICE_HAL_RTC
//#define RTE_DEVICE_HAL_SAI
//#define RTE_DEVICE_HAL_SD
#define RTE_DEVICE_HAL_SPI
#define RTE_DEVICE_HAL_TIM
#define RTE_DEVICE_HAL_UART
#define RTE_DEVICE_HAL_USART
//#define RTE_DEVICE_HAL_IRDA
//#define RTE_DEVICE_HAL_SMARTCARD
#define RTE_DEVICE_HAL_WWDG
#define RTE_DEVICE_HAL_CORTEX
//#define RTE_DEVICE_HAL_PCD
//#define RTE_DEVICE_HAL_HCD
//#define RTE_DEVICE_HAL_FMPI2C
//#define RTE_DEVICE_HAL_SPDIFRX
//#define RTE_DEVICE_HAL_LPTIM
#define RTE_DEVICE_STARTUP_STM32F4XX    /* Device Startup for STM32F4 */

/*******************************************************************************
 * CMSIS Driver Implements
 */
#define RTE_Drivers_I2C1                /* Driver I2C1 */
#define RTE_Drivers_I2C2                /* Driver I2C2 */
#define RTE_Drivers_I2C3                /* Driver I2C3 */
#define RTE_Drivers_SPI1                /* Driver SPI1 */
#define RTE_Drivers_SPI2                /* Driver SPI2 */
#define RTE_Drivers_SPI3                /* Driver SPI3 */
#define RTE_Drivers_SPI4                /* Driver SPI4 */
#define RTE_Drivers_SPI5                /* Driver SPI5 */
#define RTE_Drivers_SPI6                /* Driver SPI6 */
#define RTE_Drivers_USART1              /* Driver USART1 */
#define RTE_Drivers_USART2              /* Driver USART2 */
#define RTE_Drivers_USART3              /* Driver USART3 */
#define RTE_Drivers_USART4              /* Driver USART4 */
#define RTE_Drivers_USART5              /* Driver USART5 */
#define RTE_Drivers_USART6              /* Driver USART6 */
#define RTE_Drivers_USART7              /* Driver USART7 */
#define RTE_Drivers_USART8              /* Driver USART8 */

#endif /* RTE_COMPONENTS_H */
