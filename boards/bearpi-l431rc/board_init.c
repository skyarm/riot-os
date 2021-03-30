/*
 * Copyright (C) 2014-2017 Freie Universität Berlin
 *               2015 Lari Lehtomäki
 *               2015 TriaGnoSys GmbH
 *               2016-2017 Inria
 *               2016-2017 OTA keys
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_common_stm32
 * @{
 *
 * @file
 * @brief       Board initialization code for all Nucleo boards
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Lari Lehtomäki <lari@lehtomaki.fi>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author      Víctor Ariño <victor.arino@triagnosys.com>
 * @author      José Alamos <jialamos@uc.cl>
 * @author      Vincent Dupont <vincent@otakeys.com>
 *
 * @}
 */

#include "board.h"
#include "periph/gpio.h"
#include "qspi.h"
#include "st7789.h"

extern void isr_button1_pressed(void *);
extern void isr_button2_pressed(void *);

/* initialization of on-board LEDs */
static inline void _init_led_pins(void) { gpio_init(LED0_PIN, GPIO_OUT); }

/* initialization of on-board buttons */
static inline void _init_button_pins(void) {
  gpio_init_int(BTN0_PIN, GPIO_IN_PU, GPIO_BOTH, isr_button1_pressed,
                (void *)0);
  gpio_init_int(BTN1_PIN, GPIO_IN_PU, GPIO_BOTH, isr_button2_pressed,
                (void *)0);
}

const qspi_conf_t qspi_config[] = {{
    .dev = QUADSPI,
    .cs_pin = GPIO_PIN(PORT_B, 11),
    .cs_af = GPIO_AF10,
    .sclk_pin = GPIO_PIN(PORT_B, 10),
    .sclk_af = GPIO_AF10,
    .io_pin_0 = GPIO_PIN(PORT_B, 1),
    .io_af_0 = GPIO_AF10,
    .io_pin_1 = GPIO_PIN(PORT_B, 0),
    .io_af_1 = GPIO_AF10,
    .io_pin_2 = GPIO_UNDEF,
    .io_af_2 = GPIO_AF10,
    .io_pin_3 = GPIO_UNDEF,
    .io_af_3 = GPIO_AF10,
    .rccmask = RCC_AHB3ENR_QSPIEN,
    .apbbus = AHB3,
#ifdef MODULE_PERIPH_DMA
    .dma = 0,      /**< Logical DMA stream used for TX and RX */
    .dma_chan = 5, /**< DMA channel used for TX and TX */
#endif
}};

/**
 * @name   ST7789 configuration
 * @{
 */
const st7789_config_t st7789_config[] = {{
    .spi = SPI_DEV(1),
    .spi_clk = SPI_CLK_10MHZ,
    .spi_mode = SPI_MODE_2,
    .pwr_pin = GPIO_PIN(PORT_B, 15),
    .cs_pin = GPIO_UNDEF,
    .dcx_pin = GPIO_PIN(PORT_C, 6),
    .rst_pin = GPIO_PIN(PORT_C, 7),
    .inverted = 1,
}};
#define ST7789_NUMOF ARRAY_SIZE(st7789_config)
/** @} */

/* initialize the stst7789 GPIOs */
static inline void _init_st7789_pins(void) {
  for (unsigned i = 0; i < ST7789_NUMOF; i++) {
    /* Init Power pin */
    gpio_init(st7789_config[i].pwr_pin, GPIO_OUT);
    /* Init data pin */
    gpio_init(st7789_config[i].dcx_pin, GPIO_OUT);
    /* Init reset pin */
    gpio_init(st7789_config[i].rst_pin, GPIO_OUT);
  }
}

void __attribute__((weak)) board_init(void) {
  /* initialize the CPU */
  cpu_init();

  /* initialize the LED */
  _init_led_pins();

  /* initialize the user buttons */
  _init_button_pins();

  /* initialize the stst7789 GPIOs */
  _init_st7789_pins();
}