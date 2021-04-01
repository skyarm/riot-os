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

#ifndef SDIO_H
#define SDIO_H

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "periph/gpio.h"
#include "periph_conf.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CPU_FAM_STM32L4
#define SDIO_STA_TXACT SDMMC_STA_TXACT
#define SDIO_STA_RXACT SDMMC_STA_RXACT
#define SDIO_STA_CCRCFAIL SDMMC_STA_CCRCFAIL
#define SDIO_STA_DCRCFAIL SDMMC_STA_DCRCFAIL
#define SDIO_STA_TXUNDERR SDMMC_STA_TXUNDERR
#define SDIO_STA_RXOVERR SDMMC_STA_RXOVERR
#endif

#ifndef CONFIG_SDIO_DMATRANS_MINLEN
#define CONFIG_SDIO_DMATRANS_MINLEN (4)
#endif
/**
 * @brief   Default SDIO device access macro
 */
#ifndef SDIO_DEV
#define SDIO_DEV(x) (x)
#endif

/**
 * @brief   Structure for SDIO configuration data
 */

#ifdef CPU_FAM_STM32L4
typedef SDMMC_TypeDef SDIO_Typedef;
#endif

typedef struct {
  SDIO_Typedef *dev;   /**< SDIO device base register address */
  gpio_t data_pin_0;   /**< IO0 pin */
  gpio_t data_pin_1;   /**< IO1 pin */
  gpio_t data_pin_2;   /**< IO2 pin */
  gpio_t data_pin_3;   /**< IO3 pin */
  gpio_t sclk_pin;     /**< SCLK pin */
  gpio_t cmd_pin;      /**< HWCS pin, set to GPIO_UNDEF if not mapped */
  gpio_af_t data_af_0; /**< IO0 pin alternate function */
  gpio_af_t data_af_1; /**< IO1 pin alternate function */
  gpio_af_t data_af_2; /**< IO0 pin alternate function */
  gpio_af_t data_af_3; /**< IO1 pin alternate function */
  gpio_af_t sclk_af;   /**< SCLK pin alternate function */
  gpio_af_t cmd_af;    /**< HWCS pin alternate function */
  uint32_t rccmask;    /**< bit in the RCC peripheral enable register */
  uint8_t apbbus;      /**< APBx bus the device is connected to */
#ifdef MODULE_PERIPH_DMA
  dma_t dma;        /**< Logical DMA stream used for TX and RX */
  uint8_t dma_chan; /**< DMA channel used for TX and RX */
#endif
} sdio_config_t;

extern const sdio_config_t sdio_config[];

#ifndef SDIO_NUMOF
#define SDIO_NUMOF 1
#endif

/**
 * @brief   Default type for SDIO devices
 */
#ifndef HAVE_SDIO_T
typedef unsigned int sdio_t;
#endif

/**
 * @brief   Basic initialization of the given QSPI bus
 *
 * This function does the basic initialization including pin configuration for
 * MISO, MOSI, and CLK pins. After initialization, the given device should be
 * in power down state.
 *
 * This function is intended to be called by the board initialization code
 * during system startup to prepare the (shared) SPI device for further usage.
 * It uses the board specific initialization parameters as defined in the
 * board's `periph_conf.h`.
 *
 * Errors (e.g. invalid @p bus parameter) are not signaled through a return
 * value, but should be signaled using the assert() function internally.
 *
 * @note    This function MUST not be called more than once per bus!
 *
 * @param[in] bus       SPI device to initialize
 */
void sdio_init(sdio_t bus);

/**
 * @brief   Initialize the used QSPI bus pins, i.e. MISO, MOSI, and CLK
 *
 * After calling spi_init, the pins must be initialized (i.e. spi_init is
 * calling this function internally). In normal cases, this function will not be
 * used. But there are some devices (e.g. CC110x), that use SPI bus lines also
 * for other purposes and need the option to dynamically re-configure one or
 * more of the used pins. So they can take control over certain pins and return
 * control back to the SPI driver using this function.
 *
 * This function must be called after @ref spi_deinit_pins to return the pins to
 * SPI operation.
 *
 * The pins used are configured in the board's periph_conf.h.
 *
 * @param[in] bus       SPI device the pins are configure for
 */
void sdio_init_pins(sdio_t bus);

/**
 * @brief   Initialize the used QSPI bus pins, i.e. MISO, MOSI, and CLK
 *
 * After calling spi_init, the pins must be initialized (i.e. spi_init is
 * calling this function internally). In normal cases, this function will not be
 * used. But there are some devices (e.g. CC110x), that use SPI bus lines also
 * for other purposes and need the option to dynamically re-configure one or
 * more of the used pins. So they can take control over certain pins and return
 * control back to the SPI driver using this function.
 *
 * This function must be called after @ref spi_deinit_pins to return the pins to
 * SPI operation.
 *
 * The pins used are configured in the board's periph_conf.h.
 *
 * @param[in] bus       SPI device the pins are configure for
 */
void sdio_deinit_pins(sdio_t bus);

/**
 * @brief   Initialize the used QSPI bus pins, i.e. MISO, MOSI, and CLK
 *
 * After calling spi_init, the pins must be initialized (i.e. spi_init is
 * calling this function internally). In normal cases, this function will not be
 * used. But there are some devices (e.g. CC110x), that use SPI bus lines also
 * for other purposes and need the option to dynamically re-configure one or
 * more of the used pins. So they can take control over certain pins and return
 * control back to the SPI driver using this function.
 *
 * This function must be called after @ref spi_deinit_pins to return the pins to
 * SPI operation.
 *
 * The pins used are configured in the board's periph_conf.h.
 *
 * @param[in] bus       SPI device the pins are configure for
 */
int sdio_acquire(sdio_t bus);

/**
 * @brief   Finish an ongoing SPI transaction by releasing the given SPI bus
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
void sdio_release(sdio_t bus);

/**
 * @brief   Initial mode to default value
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
static inline uint32_t sdio_mode_init(void) { return 0; }

/**
 * @brief   Initial mode to default value
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */

void sdio_command(sdio_t bus, uint32_t mode, uint8_t cmd);

/**
 * @brief Send bytes to QSPI device
 *
 * @param[in] bus       SPI device to use
 * @param[in] cs        chip select pin/line to use, set to SPI_CS_UNDEF if chip
 *                      select should not be handled by the SPI driver
 * @param[in] cont      if true, keep device selected after transfer
 * @param[in] out       byte to send out
 *
 * @return              the received byte
 */
void sdio_send_bytes(sdio_t bus, const void *data, size_t count);

/**
 * @brief Receive bytes from QSPI device
 *
 * @param[in] bus       QSPI device to use
 * @param[in] data      data to send out
 */
void qspi_recv_bytes(sdio_t bus, void *data, size_t count);

#ifdef __cplusplus
}
#endif

#endif /* SDIO_H */
/** @} */
