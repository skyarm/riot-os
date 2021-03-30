/*
 * Copyright (C) 2018 Koen Zandberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_st7789
 * @{
 *
 * @file
 * @brief       Device driver implementation for the st7789 display controller
 *
 * @author      Koen Zandberg <koen@bergzand.net>
 *
 * @}
 */

#include "st7789.h"
#include "byteorder.h"
#include "kernel_defines.h"
#include "xtimer.h"
#include <assert.h>
#include <string.h>

#define ENABLE_DEBUG 0
#include "debug.h"

static inline void _st7789_spi_acquire(st7789_t dev) {
  spi_acquire(st7789_config[dev].spi, st7789_config[dev].cs_pin,
              st7789_config[dev].spi_mode, st7789_config[dev].spi_clk);
}

static inline void _st7789_cmd_start(st7789_t dev, uint8_t cmd, bool cont) {
  gpio_clear(st7789_config[dev].dcx_pin);
  spi_transfer_bytes(st7789_config[dev].spi, st7789_config[dev].cs_pin, cont,
                     &cmd, NULL, 1);
  gpio_set(st7789_config[dev].dcx_pin);
}

static inline void _write_cmd(st7789_t dev, uint8_t cmd, const uint8_t *data,
                              size_t len) {
  _st7789_cmd_start(dev, cmd, len ? true : false);
  if (len) {
    spi_transfer_bytes(st7789_config[dev].spi, st7789_config[dev].cs_pin,
                       false, data, NULL, len);
  }
}

static inline void _st7789_set_area(st7789_t dev, uint16_t x1, uint16_t x2,
                             uint16_t y1, uint16_t y2) {
  uint16_t data[2];

  data[0] = htons(x1);
  data[1] = htons(x2);
  _write_cmd(dev, 0x2a, (uint8_t *)data, sizeof(data));
  data[0] = htons(y1);
  data[1] = htons(y2);
  _write_cmd(dev, 0x2b, (uint8_t *)data, sizeof(data));
}

int st7789_init(st7789_t dev) {
  uint8_t data[5] = {0};
  
  /* Power off LCD */
  gpio_clear(st7789_config[dev].pwr_pin);

  /* reset device */
  gpio_clear(st7789_config[dev].rst_pin);
  xtimer_msleep(120);
  gpio_set(st7789_config[dev].rst_pin);

  xtimer_msleep(120);

  /* Acquire once at release at the end */
  _st7789_spi_acquire(dev);

  /* Sleep Out */
  _write_cmd(dev, 0x11, NULL, 0);
  xtimer_msleep(120);

  /* Memory Data Access Control */
  data[0] = 0x0;
  _write_cmd(dev, 0x36, data, 1);

  /* RGB 5-6-5-bit  */
  data[0] = 0x65;
  _write_cmd(dev, 0x3A, data, 1);

  /* Porch Setting */
  data[0] = 0x0C;
  data[1] = 0x0C;
  data[2] = 0x00;
  data[3] = 0x33;
  data[4] = 0x33;
  _write_cmd(dev, 0xB2, data, 5);

  /*  Gate Control */
  data[0] = 0x72;
  _write_cmd(dev, 0xB7, data, 1);

  /* VCOM Setting */
  data[0] = 0x3D;
  _write_cmd(dev, 0xBB, data, 1); // Vcom=1.625V

  /* LCM Control */
  data[0] = 0x2C;
  _write_cmd(dev, 0xC0, data, 1);

  /* VDV and VRH Command Enable */
  data[0] = 0x01;
  _write_cmd(dev, 0xC2, data, 1);

  /* VRH Set */
  data[0] = 0x19;
  _write_cmd(dev, 0xC3, data, 1);

  /* VDV Set */
  data[0] = 0x20;
  _write_cmd(dev, 0xC4, data, 2);

  /* Frame Rate Control in Normal Mode */
  data[0] = 0x0F; // 60MHZ
  _write_cmd(dev, 0xC6, data, 1);

  /* Power Control 1 */
  data[0] = 0xA4;
  data[1] = 0xA1;
  _write_cmd(dev, 0xD0, data, 2);

  /* Gamma correction */
  static const uint8_t gamma_pos[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F,
                                      0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
  _write_cmd(dev, 0xE0, gamma_pos, sizeof(gamma_pos));
  static const uint8_t gamma_neg[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F,
                                      0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
  _write_cmd(dev, 0xE1, gamma_neg, sizeof(gamma_neg));

  /* Display Inversion On */
  if (st7789_config[dev].inverted) {
    _write_cmd(dev, 0x21, NULL, 0);
  }

  /* Display on */
  _write_cmd(dev, 0x29, NULL, 0);

  /* Power on */
  gpio_set(st7789_config[dev].pwr_pin);

  spi_release(st7789_config[dev].spi);

  return 0;
}

void st7789_write_cmd(st7789_t dev, uint8_t cmd, const uint8_t *data,
                      size_t len) {
  _st7789_spi_acquire(dev);
  _write_cmd(dev, cmd, data, len);
  spi_release(st7789_config[dev].spi);
}

void st7789_read_cmd(st7789_t dev, uint8_t cmd, uint8_t *data, size_t len) {
  assert(len);
  _st7789_spi_acquire(dev);
  _st7789_cmd_start(dev, cmd, true);
  /* Dummy transfer */
  spi_transfer_byte(st7789_config[dev].spi, st7789_config[dev].cs_pin, true,
                    0x00);
  spi_transfer_bytes(st7789_config[dev].spi, st7789_config[dev].cs_pin, false,
                     NULL, data, len);
  spi_release(st7789_config[dev].spi);
}

void st7789_fill(st7789_t dev, uint16_t x1, uint16_t x2, uint16_t y1,
                 uint16_t y2, uint16_t color) {
  /* Send fill area to the display */

  /* Calculate number of pixels */
  int32_t num_pix = (x2 - x1 + 1) * (y2 - y1 + 1);

  DEBUG("[st7789]: Write x1: %" PRIu16 ", x2: %" PRIu16 ", "
        "y1: %" PRIu16 ", y2: %" PRIu16 ". Num pixels: %lu\n",
        x1, x2, y1, y2, (unsigned long)num_pix);

  _st7789_spi_acquire(dev);
  /* Send fill area to the display */
  _st7789_set_area(dev, x1, x2, y1, y2);
  /* Memory access command */
  _st7789_cmd_start(dev, 0x2c, true);

  color = htons(color);

  uint8_t cache[32]; // FIXME: default cache size;
  uint32_t remain = 0, count = 0;
  uint32_t total = (x2 - x1 + 1) * (y2 - y1 + 1) * 2;
  if (total > sizeof(cache)) {
    remain = total - sizeof(cache);
    count = sizeof(cache);
  } else {
    remain = 0;
    count = total;
  }

  for (;;) {
    for (uint32_t i = 0; i < count / 2; i++) {
      cache[2 * i] = color >> 8;
      cache[2 * i + 1] = color;
    }
    if (remain) {
      spi_transfer_bytes(st7789_config[dev].spi, st7789_config[dev].cs_pin,
                         true, cache, NULL, count);
    } else {
      spi_transfer_bytes(st7789_config[dev].spi, st7789_config[dev].cs_pin,
                         false, cache, NULL, count);
      break;
    }
    if (remain > sizeof(cache)) {
      remain -= sizeof(cache);
      count = sizeof(cache);
    } else {
      count = remain;
      remain = 0;
    }
  }

  spi_release(st7789_config[dev].spi);
}

void st7789_pixmap(st7789_t dev, uint16_t x1, uint16_t x2, uint16_t y1,
                   uint16_t y2, const uint16_t *color) {
  size_t num_pix = (x2 - x1 + 1) * (y2 - y1 + 1);

  DEBUG("[st7789]: Write x1: %" PRIu16 ", x2: %" PRIu16 ", "
        "y1: %" PRIu16 ", y2: %" PRIu16 ". Num pixels: %lu\n",
        x1, x2, y1, y2, (unsigned long)num_pix);

  _st7789_spi_acquire(dev);

  /* Send fill area to the display */
  _st7789_set_area(dev, x1, x2, y1, y2);

  /* Memory access command */
  _st7789_cmd_start(dev, 0x2c, true);

  for (size_t i = 0; i < num_pix - 1; i++) {
    uint16_t ncolor = htons(*(color + i));
    spi_transfer_bytes(st7789_config[dev].spi, st7789_config[dev].cs_pin,
                       true, &ncolor, NULL, sizeof(uint16_t));
  }
  uint16_t ncolor = htons(*(color + num_pix - 1));
  spi_transfer_bytes(st7789_config[dev].spi, st7789_config[dev].cs_pin, false,
                     &ncolor, NULL, sizeof(uint16_t));

  spi_release(st7789_config[dev].spi);
}

void st7789_invert_on(st7789_t dev) {
  uint8_t command = (st7789_config[dev].inverted) ? 0x20 : 0x21;
  st7789_write_cmd(dev, command, NULL, 0);
}

void st7789_invert_off(st7789_t dev) {
  uint8_t command = (st7789_config[dev].inverted) ? 0x21 : 0x20;
  st7789_write_cmd(dev, command, NULL, 0);
}
