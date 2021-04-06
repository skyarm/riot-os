#ifndef SDIO_DEFINES_H
#define SDIO_DEFINES_H

#include "cpu_conf.h"

/*
 *    Specifies the clock transition on which the bit capture is made.
 *  This parameter can be a value of:
 *  @1  0x00000000U            SDMMC_CLOCK_EDGE_RISING
 *  @2  SDMMC_CLKCR_NEGEDGE 
 */
#ifndef SDIO_CLOCK_EDGE
#define SDIO_CLOCK_EDGE (0x00000000U)
#endif

/*
 *    pecifies the clock transition on which the bit capture is made.
 * This parameter can be a value of:
 *  @1  0x00000000U             Disable by pass
 *  @2  SDMMC_CLKCR_BYPASS 
 */
#ifndef SDIO_CLOCK_BYPASS_ENABLE
#define SDIO_CLOCK_BYPASS_ENABLE (0x00000000U)
#endif

/*
 *   Specifies whether SDMMC Clock output is enabled or disabled when the bus is idle.
 * This parameter can be a value of
 *  @1  0x00000000U             Disable power save
 *  @2  SDMMC_CLKCR_PWRSAV      Enable power save
 */
#ifndef SDIO_CLOCK_POWER_SAVE_ENABLE
#define SDIO_CLOCK_POWER_SAVE_ENABLE    (0x00000000U)
#endif

/*
 *    Specifies the SDMMC bus width.
 * This parameter can be a value of:
 *  @1  0x00000000U             SDMMC_BUS_WIDE_1B
 *  @1  SDMMC_CLKCR_WIDBUS_0    SDMMC_BUS_WIDE_4B     
 *  @1  SDMMC_CLKCR_WIDBUS_1    SDMMC_BUS_WIDE_8B
 */
#ifndef SDIO_BUS_WIDE
#define SDIO_BUS_WIDE   (0x0)   
#endif

/**
 * Specifies whether the SDMMC hardware flow control is enabled or disabled.
 * @1   0x00000000U             Disable hardware flow control
 * @2   SDMMC_CLKCR_HWFC_EN     Enable hardware flow control
 */
#ifndef SDIO_HARDWARE_FLOW_CONTROL_ENABLE
#define SDIO_HARDWARE_FLOW_CONTROL_ENABLE   (0x0)
#endif

/*
 *  Specifies the clock frequency of the SDMMC controller.
 *  This parameter can be a value between Min_Data = 0 and Max_Data = 1023
 */
#ifndef SDIO_CLOCK_DIV
#define SDIO_CLOCK_DIV (2)
#endif

/**
 *     Specifies whether external Transceiver is enabled or disabled.
 * This parameter can be a value of:
*/
#if defined(STM32L4P5xx) || defined(STM32L4Q5xx) || defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
#ifndef SDIO_TRANSCERVER        
#define SDIO_TRANSCERVER /*!< Specifies whether external Transceiver is enabled or disabled.
                                      This parameter can be a value of @ref SDMMC_LL_Transceiver */
#endif
#endif /* STM32L4P5xx || STM32L4Q5xx || STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */



#endif /* SDIO_DEFINES_H */
