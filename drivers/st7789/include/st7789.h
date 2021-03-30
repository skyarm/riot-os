/*
 * Copyright (C) 2018 Koen Zandberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_st7789 ST7789 display driver
 * @ingroup     drivers_display
 *
 * @brief       Driver for the ST7789 display
 *
 * @{
 *
 * @file
 * @brief       Driver for ST7789 display
 *
 * @author      Thomas Zhang <zhangtao@skyarm.com>
 *
 * The ST7789 is a generic display driver for small RGB displays. The driver
 * implemented here operates over SPI to communicate with the device.
 *
 * The device requires colors to be send in big endian RGB-565 format. The
 * @ref CONFIG_ST7789_LE_MODE compile time option can switch this, but only use
 * this when strictly necessary. This option will slow down the driver as it
 * certainly can't use DMA anymore, every short has to be converted before
 * transfer.
 */

#ifndef ST7789_H
#define ST7789_H

#include "periph/gpio.h"
#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ST7789_LCD_HEIGHT 240
#define ST7789_LCD_WIDTH 240
#define ST7789_DEV(x) (x)

/**
 * @brief   Device config for a st7789
 */
typedef struct {
  spi_t spi;           /**< SPI device that the display is connected to */
  spi_clk_t spi_clk;   /**< SPI clock speed to use */
  spi_mode_t spi_mode; /**< SPI mode */
  gpio_t pwr_pin;      /**< pin connected to the CHIP POWER line */
  gpio_t cs_pin;       /**< pin connected to the CHIP SELECT line */
  gpio_t dcx_pin;      /**< pin connected to the DC line */
  gpio_t rst_pin;      /**< pin connected to the reset line */
  bool inverted;       /**< Display works in inverted color mode */
} st7789_config_t;
extern const st7789_config_t st7789_config[];

#ifndef HAVE_ST7789_T
typedef unsigned int st7789_t;
#endif

/**
 * @brief   Setup an st7789 display device
 *
 * @param[out]  dev     device descriptor
 * @param[in]   params  parameters for device initialization
 */
int st7789_init(st7789_t dev);

/**
 * @brief   Fill a rectangular area with a single pixel color
 *
 * the rectangular area is defined as x1 being the first column of pixels and
 * x2 being the last column of pixels to fill. similar to that, y1 is the first
 * row to fill and y2 is the last row to fill.
 *
 * @param[in]   dev     device descriptor
 * @param[in]   x1      x coordinate of the first corner
 * @param[in]   x2      x coordinate of the opposite corner
 * @param[in]   y1      y coordinate of the first corner
 * @param[in]   y2      y coordinate of the opposite corner
 * @param[in]   color   single color to fill the area with
 */
void st7789_fill(st7789_t dev, uint16_t x1, uint16_t x2, uint16_t y1,
                 uint16_t y2, uint16_t color);

/**
 * @brief   Fill a rectangular area with an array of pixels
 *
 * the rectangular area is defined as x1 being the first column of pixels and
 * x2 being the last column of pixels to fill. similar to that, y1 is the first
 * row to fill and y2 is the last row to fill.
 *
 * @note @p color must have a length equal to `(x2 - x1 + 1) * (y2 - y1 + 1)`
 *
 * @param[in]   dev     device descriptor
 * @param[in]   x1      x coordinate of the first corner
 * @param[in]   x2      x coordinate of the opposite corner
 * @param[in]   y1      y coordinate of the first corner
 * @param[in]   y2      y coordinate of the opposite corner
 * @param[in]   color   array of colors to fill the area with
 */
void st7789_pixmap(st7789_t dev, uint16_t x1, uint16_t x2, uint16_t y1,
                   uint16_t y2, const uint16_t *color);

/**
 * @brief   Raw write command
 *
 * @param[in]   dev     device descriptor
 * @param[in]   cmd     command code
 * @param[in]   data    command data to the device
 * @param[in]   len     length of the command data
 */
void st7789_write_cmd(st7789_t dev, uint8_t cmd, const uint8_t *data,
                      size_t len);

/**
 * @brief   Raw read command
 *
 * @pre         len > 0
 *
 * @param[in]   dev     device descriptor
 * @param[in]   cmd     command
 * @param[out]  data    data from the device
 * @param[in]   len     length of the returned data
 */
void st7789_read_cmd(st7789_t dev, uint8_t cmd, uint8_t *data, size_t len);

/**
 * @brief   Invert the display colors
 *
 * @param[in]   dev     device descriptor
 */
void st7789_invert_on(st7789_t dev);

/**
 * @brief   Disable color inversion
 *
 * @param[in]   dev     device descriptor
 */
void st7789_invert_off(st7789_t dev);

#ifdef __cplusplus
}
#endif
#endif /* ST7789_H */
/** @} */
