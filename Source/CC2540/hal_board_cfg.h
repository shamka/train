/******************************************************************************

 @file  hal_board_cfg.h

 @brief Abstract board-specific registers, addresses, & initialization for H/W
        based on the Texas Instruments CC254x (8051 core).

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2006-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:09
 *****************************************************************************/
#ifndef HAL_BOARD_CFG_H
#define HAL_BOARD_CFG_H

#ifdef __cplusplus
extern "C"
{
#endif


/*******************************************************************************
 * INCLUDES
 */

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"

/*******************************************************************************
 * CONSTANTS
 */

/* Board Identifier */

#if !defined (HAL_BOARD_CC2530EB_REV17) && !defined (HAL_BOARD_CC2530EB_REV13)
  #define HAL_BOARD_CC2530EB_REV17
#endif

/* Clock Speed */

#define HAL_CPU_CLOCK_MHZ             32

/* Sleep Clock */

#define EXTERNAL_CRYSTAL_OSC          0x00  // external 32kHz XOSC
#define INTERNAL_RC_OSC               0x80  // internal 32kHz RCOSC

// For non-USB, assume external, unless an internal crystal is explicitly indicated.
#if !defined (XOSC32K_INSTALLED) || (defined (XOSC32K_INSTALLED) && (XOSC32K_INSTALLED == TRUE))
#define OSC_32KHZ                     EXTERNAL_CRYSTAL_OSC
#else
#define OSC_32KHZ                     INTERNAL_RC_OSC
#endif

// Minimum Time for Stable External 32kHz Clock (in ms)
#define MIN_TIME_TO_STABLE_32KHZ_XOSC 400

/* LCD Max Chars and Buffer */
#define HAL_LCD_MAX_CHARS   16
#define HAL_LCD_MAX_BUFF    25

/* LED Configuration */

#if defined (HAL_BOARD_CC2530EB_REV17) && !defined (HAL_PA_LNA) && !defined (HAL_PA_LNA_CC2590)
  #define HAL_NUM_LEDS                 3
#elif defined (HAL_BOARD_CC2530EB_REV13) || defined (HAL_PA_LNA) || defined (HAL_PA_LNA_CC2590)
  #define HAL_NUM_LEDS                 1
#else
  #error Unknown Board Indentifier
#endif
/* Push Button Configuration */

#define ACTIVE_LOW                     !
#define ACTIVE_HIGH                    !!    /* double negation forces result to be '1' */

/* OSAL NV implemented by internal flash pages. */

// Flash is partitioned into 8 banks of 32 KB or 16 pages.
#define HAL_FLASH_PAGE_PER_BANK        16

// Flash is constructed of 128 pages of 2 KB.
#define HAL_FLASH_PAGE_PHYS            2048

// SNV can use a larger logical page size to accomodate more or bigger items or extend lifetime.
#define HAL_FLASH_PAGE_SIZE            HAL_FLASH_PAGE_PHYS
#define HAL_FLASH_WORD_SIZE            4

// CODE banks get mapped into the XDATA range 8000-FFFF.
#define HAL_FLASH_PAGE_MAP             0x8000

// The last 16 bytes of the last available page are reserved for flash lock bits.
#define HAL_FLASH_LOCK_BITS            16

// NV page definitions must coincide with segment declaration in project *.xcl file.
#if defined NON_BANKED
#define HAL_NV_PAGE_END                30
#elif defined HAL_BOARD_CC2541S
#define HAL_NV_PAGE_END                125
#elif defined HAL_BOARD_F128
#define HAL_NV_PAGE_END                62
#else
#define HAL_NV_PAGE_END                126
#endif

// Re-defining Z_EXTADDR_LEN here so as not to include a Z-Stack .h file.
#define HAL_FLASH_IEEE_SIZE            8
#define HAL_FLASH_IEEE_PAGE            (HAL_NV_PAGE_END+1)
#define HAL_FLASH_IEEE_OSET            (HAL_FLASH_PAGE_SIZE - HAL_FLASH_LOCK_BITS - HAL_FLASH_IEEE_SIZE)
#define HAL_INFOP_IEEE_OSET            0xC

#define HAL_NV_PAGE_CNT                2
#define HAL_NV_PAGE_BEG                (HAL_NV_PAGE_END-HAL_NV_PAGE_CNT+1)

// Used by DMA macros to shift 1 to create a mask for DMA registers.
#define HAL_NV_DMA_CH                  0
#define HAL_DMA_CH_RX                  3
#define HAL_DMA_CH_TX                  4

#define HAL_NV_DMA_GET_DESC()  HAL_DMA_GET_DESC0()
#define HAL_NV_DMA_SET_ADDR(a) HAL_DMA_SET_ADDR_DESC0((a))

/* Critical Vdd Monitoring to prevent flash damage or radio lockup. */

// Vdd/3 / Internal Reference X ENOB --> (Vdd / 3) / 1.15 X 127
#define VDD_2_0  74   // 2.0 V required to safely read/write internal flash.
#define VDD_2_7  100  // 2.7 V required for the Numonyx device.

#define VDD_MIN_RUN   VDD_2_0
#define VDD_MIN_NV   (VDD_2_0+4)  // 5% margin over minimum to survive a page erase and compaction.
#define VDD_MIN_XNV  (VDD_2_7+5)  // 5% margin over minimum to survive a page erase and compaction.

/*******************************************************************************
 * MACROS
 */

/* Cache Prefetch Control */

#define PREFETCH_ENABLE()  st( FCTL = 0x08; )
#define PREFETCH_DISABLE() st( FCTL = 0x04; )

/* Setting Clocks */

// switch to the 16MHz HSOSC and wait until it is stable
#define SET_OSC_TO_HSOSC()                                                     \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & 0x80) | CLKCONCMD_16MHZ;                            \
  while ( (CLKCONSTA & ~0x80) != CLKCONCMD_16MHZ );                            \
}

// switch to the 32MHz XOSC and wait until it is stable
#define SET_OSC_TO_XOSC()                                                      \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & 0x80) | CLKCONCMD_32MHZ;                            \
  while ( (CLKCONSTA & ~0x80) != CLKCONCMD_32MHZ );                            \
}

// set 32kHz OSC and wait until it is stable
#define SET_32KHZ_OSC()                                                        \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & ~0x80) | OSC_32KHZ;                                 \
  while ( (CLKCONSTA & 0x80) != OSC_32KHZ );                                   \
}

/* The OSC_PD register is not documented in the user guide. 
It is meant to be used by TI only */
#define START_HSOSC_XOSC()                                                     \
{                                                                              \
  SLEEPCMD &= ~OSC_PD;            /* start 16MHz RCOSC & 32MHz XOSC */         \
  while (!(SLEEPSTA & XOSC_STB)); /* wait for stable 32MHz XOSC */             \
}

#define STOP_HSOSC()                                                           \
{                                                                              \
  SLEEPCMD |= OSC_PD;             /* stop 16MHz RCOSC */                       \
}

/* Board Initialization */
#define HAL_BOARD_INIT()                                                       \
{                                                                              \
  /* Set to 16Mhz to set 32kHz OSC, then back to 32MHz */                      \
  START_HSOSC_XOSC();                                                          \
  SET_OSC_TO_HSOSC();                                                          \
  SET_32KHZ_OSC();                                                             \
  SET_OSC_TO_XOSC();                                                           \
  STOP_HSOSC();                                                                \
                                                                               \
  /* Enable cache prefetch mode. */                                            \
  PREFETCH_ENABLE();                                                           \
}

/* Driver Configuration */

/* Set to TRUE enable H/W TIMER usage, FALSE disable it */
#ifndef HAL_TIMER
#define HAL_TIMER TRUE
#endif

/* Set to TRUE enable ADC usage, FALSE disable it */
#ifndef HAL_ADC
#define HAL_ADC TRUE
#endif

/* Set to TRUE enable DMA usage, FALSE disable it */
#ifndef HAL_DMA
#define HAL_DMA TRUE
#endif

/* Set to TRUE enable Flash access, FALSE disable it */
#ifndef HAL_FLASH
#define HAL_FLASH TRUE
#endif

/* Set to TRUE enable AES usage, FALSE disable it */
#ifndef HAL_AES
#define HAL_AES TRUE
#endif

#ifndef HAL_AES_DMA
#define HAL_AES_DMA TRUE
#endif

#ifdef __cplusplus
}
#endif

#endif /* HAL_BOARD_CFG_H */

/*******************************************************************************
*/
