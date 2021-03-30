/*
 * Copyright (C) 2017  Inria
 *               2017  OTA keys
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_bearpi-l431rc
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the bearpi-l431rc board
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author      Vincent Dupont <vincent@otakeys.com>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

/* Add specific clock configuration (HSE, LSE) for this board here */
#ifndef CONFIG_BOARD_HAS_LSE
#define CONFIG_BOARD_HAS_LSE 1
#endif

#include "cfg_i2c1_pb6_pb7.h"
#include "cfg_rtt_default.h"
#include "cfg_timer_tim2.h"
#include "clk_conf.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    DMA streams configuration
 * @{
 */
static const dma_conf_t dma_config[] = {
    {.stream = 4},
};

#define DMA_0_ISR isr_dma1_channel5

#define DMA_NUMOF ARRAY_SIZE(dma_config)
/** @} */

/**
 * @name UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev = USART1,
        .rcc_mask = RCC_APB2ENR_USART1EN,
        .rx_pin = GPIO_PIN(PORT_A, 10),
        .tx_pin = GPIO_PIN(PORT_A, 9),
        .rx_af = GPIO_AF7,
        .tx_af = GPIO_AF7,
#ifdef MODULE_PERIPH_UART_HW_FC
        .cts_pin = GPIO_PIN(PORT_A, 11),
        .rts_pin = GPIO_PIN(PORT_A, 12),
        .cts_af = GPIO_AF7,
        .rts_af = GPIO_AF7,
#endif
        .bus = APB2,
        .irqn = USART1_IRQn,
        .type = STM32_USART,
        .clk_src = 0, /* Use APB clock */
#ifdef MODULE_PERIPH_DMA
        .dma = DMA_STREAM_UNDEF, /**< Logical DMA stream used for TX */
        .dma_chan = 0,           /**< DMA channel used for TX */
#endif
    },
    {
        .dev = USART2,
        .rcc_mask = RCC_APB1ENR1_USART2EN,
        .rx_pin = GPIO_PIN(PORT_A, 15),
        .tx_pin = GPIO_PIN(PORT_A, 2),
        .rx_af = GPIO_AF3,
        .tx_af = GPIO_AF7,
#ifdef MODULE_PERIPH_UART_HW_FC
        .cts_pin = GPIO_PIN(PORT_A, 0),
        .rts_pin = GPIO_PIN(PORT_A, 1),
        .cts_af = GPIO_AF7,
        .rts_af = GPIO_AF7,
#endif
        .bus = APB1,
        .irqn = USART2_IRQn,
        .type = STM32_USART,
        .clk_src = 0, /* Use APB clock */
#ifdef MODULE_PERIPH_DMA
        .dma = DMA_STREAM_UNDEF, /**< Logical DMA stream used for TX */
        .dma_chan = 0,           /**< DMA channel used for TX */
#endif
    },
};

#define UART_0_ISR (isr_usart1)
#define UART_1_ISR (isr_usart2)

#define UART_NUMOF ARRAY_SIZE(uart_config)
/** @} */

/**
 * @name   PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {.dev = TIM1,
     .rcc_mask = RCC_APB2ENR_TIM1EN,
     .chan = {{.pin = GPIO_PIN(PORT_A, 8) /* D9 */, .cc_chan = 0},
              {.pin = GPIO_UNDEF, .cc_chan = 0},
              {.pin = GPIO_UNDEF, .cc_chan = 0},
              {.pin = GPIO_UNDEF, .cc_chan = 0}},
     .af = GPIO_AF1,
     .bus = APB2}};

#define PWM_NUMOF ARRAY_SIZE(pwm_config)
/** @} */

/**
 * @name   SPI configuration
 * @{
 */
static const spi_conf_t spi_config[] = {
    {
        .dev = SPI1,
        .mosi_pin = GPIO_PIN(PORT_B, 5),
        .miso_pin = GPIO_PIN(PORT_B, 4),
        .sclk_pin = GPIO_PIN(PORT_B, 3),
        .cs_pin = GPIO_UNDEF,
        .mosi_af = GPIO_AF5,
        .miso_af = GPIO_AF5,
        .sclk_af = GPIO_AF5,
        .cs_af = GPIO_AF5,
        .rccmask = RCC_APB2ENR_SPI1EN,
        .apbbus = APB2,
#ifdef MODULE_PERIPH_DMA
        .tx_dma = DMA_STREAM_UNDEF, /**< Logical DMA stream used for TX */
        .tx_dma_chan = 0,           /**< DMA channel used for TX */
        .rx_dma = DMA_STREAM_UNDEF, /**< Logical DMA stream used for RX */
        .rx_dma_chan = 0,           /**< DMA channel used for RX */
#endif
    },
    {
        .dev = SPI2,
        .mosi_pin = GPIO_PIN(PORT_C, 3),
        .miso_pin = GPIO_PIN(PORT_C, 2),
        .sclk_pin = GPIO_PIN(PORT_B, 13),
        .cs_pin = GPIO_UNDEF,
        .mosi_af = GPIO_AF5,
        .miso_af = GPIO_AF5,
        .sclk_af = GPIO_AF5,
        .cs_af = GPIO_AF5,
        .rccmask = RCC_APB1ENR1_SPI2EN,
        .apbbus = APB1,
#ifdef MODULE_PERIPH_DMA
        .tx_dma = DMA_STREAM_UNDEF, /**< Logical DMA stream used for TX */
        .tx_dma_chan = 1,           /**< DMA channel used for TX */
        .rx_dma = DMA_STREAM_UNDEF, /**< Logical DMA stream used for RX */
        .rx_dma_chan = 1,           /**< DMA channel used for RX */
#endif
    },
};

#define SPI_NUMOF ARRAY_SIZE(spi_config)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
