/*
 * Copyright (C) 2018 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_stm32l0538-disco
 * @{
 *
 * @file
 * @brief       Board specific definitions for the STM32L0538-DISCO evaluation board.
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Xtimer configuration
 * @{
 */
#if (defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32L0)) && \
    !defined(CPU_MODEL_STM32F042K6) && !defined(CPU_MODEL_STM32F031K6)
#define XTIMER_WIDTH                (16)
#endif

#if defined(CPU_FAM_STM32G0)
#define XTIMER_WIDTH                (16)
#endif

#if defined(CPU_FAM_STM32F1)
#define XTIMER_WIDTH                (16)
#define XTIMER_BACKOFF              (19)
#endif

#if defined(CPU_FAM_STM32L1)
#define XTIMER_BACKOFF              (11)
#endif

#if defined(CPU_FAM_STM32F4) || defined(CPU_MODEL_STM32F303ZE)
#define XTIMER_BACKOFF              (8)
#endif
/** @} */


/**
 * @name User button
 * @{
 */
#define BTN0_PIN            GPIO_PIN(PORT_B, 2)
#define BTN1_PIN            GPIO_PIN(PORT_B, 3)
/** @} */

/**
 * @name Macros for controlling the on-board LED (LD3).
 * @{
 */
#define LED0_PIN            GPIO_PIN(PORT_C, 13)
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
