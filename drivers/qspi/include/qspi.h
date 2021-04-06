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

#ifndef QSPI_H
#define QSPI_H

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "periph/gpio.h"
#include "periph_conf.h"
#include "periph_cpu.h"
#include "qspi_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_QSPI_DMATRANS_MINLEN
#define CONFIG_QSPI_DMATRANS_MINLEN (4)
#endif
/**
 * @brief   Default SPI device access macro
 */
#ifndef QSPI_DEV
#define QSPI_DEV(x) (x)
#endif

/**
 * @brief   Structure for QSPI configuration data
 */
typedef struct {
  QUADSPI_TypeDef *dev; /**< QSPI device base register address */
  gpio_t io_pin_0;      /**< IO0 pin */
  gpio_t io_pin_1;      /**< IO1 pin */
  gpio_t io_pin_2;      /**< IO2 pin */
  gpio_t io_pin_3;      /**< IO3 pin */
  gpio_t sclk_pin;      /**< SCLK pin */
  gpio_t cs_pin;        /**< HWCS pin, set to GPIO_UNDEF if not mapped */
  gpio_af_t io_af_0;    /**< IO0 pin alternate function */
  gpio_af_t io_af_1;    /**< IO1 pin alternate function */
  gpio_af_t io_af_2;    /**< IO0 pin alternate function */
  gpio_af_t io_af_3;    /**< IO1 pin alternate function */
  gpio_af_t sclk_af;    /**< SCLK pin alternate function */
  gpio_af_t cs_af;      /**< HWCS pin alternate function */
  uint32_t rccmask;     /**< bit in the RCC peripheral enable register */
  uint8_t apbbus;       /**< APBx bus the device is connected to */
#ifdef MODULE_PERIPH_DMA
  dma_t dma;        /**< Logical DMA stream used for TX and RX */
  uint8_t dma_chan; /**< DMA channel used for TX and RX */
#endif
/*
  uint8_t fifo_threshold;
  uint8_t clock_prescaler;
  bool clock_cycle_shift;
  uint32_t flash_size;
  uint8_t chip_select_hign_time;
  bool clock_mode_high;
*/
} qspi_config_t;

extern const qspi_config_t qspi_config[];

#ifndef QSPI_NUMOF
#define QSPI_NUMOF 1
#endif

/**
 * @brief   Default type for SPI devices
 */
#ifndef HAVE_QSPI_T
typedef unsigned int qspi_t;
#endif

typedef enum {
  qspi_flash_id_1,
  qspi_flash_id_2,
  qspi_flash_id_dual,
} qspi_flash_id_t;

typedef enum {
  qspi_io_0lines,
  qspi_io_1lines,
  qspi_io_2lines,
  qspi_io_4lines
} qspi_io_lines_t;

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
void qspi_init(qspi_t bus);

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
void qspi_init_pins(qspi_t bus);

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
void qspi_deinit_pins(qspi_t bus);

/**
 * @brief   Get the IO0 pin of the given QSPI bus.
 *
 * @param[in] bus       The device to query
 *
 * @note Until this is implemented on all platforms, this requires the
 *       periph_spi_reconfigure feature to be used.
 *
 * @return              The GPIO used for the QSPI IO0 line.
 */
gpio_t qspi_pin_io_0(qspi_t bus);

/**
 * @brief   Get the IO0 pin of the given QSPI bus.
 *
 * @param[in] bus       The device to query
 *
 * @note Until this is implemented on all platforms, this requires the
 *       periph_spi_reconfigure feature to be used.
 *
 * @return              The GPIO used for the QSPI IO0 line.
 */
gpio_t qspi_pin_io_1(qspi_t bus);

/**
 * @brief   Get the IO0 pin of the given QSPI bus.
 *
 * @param[in] bus       The device to query
 *
 * @note Until this is implemented on all platforms, this requires the
 *       periph_spi_reconfigure feature to be used.
 *
 * @return              The GPIO used for the QSPI IO0 line.
 */
gpio_t qspi_pin_clk(qspi_t bus);

/**
 * @brief   Get the IO0 pin of the given QSPI bus.
 *
 * @param[in] bus       The device to query
 *
 * @note Until this is implemented on all platforms, this requires the
 *       periph_spi_reconfigure feature to be used.
 *
 * @return              The GPIO used for the QSPI IO0 line.
 */
gpio_t qspi_pin_cs(qspi_t bus);

/**
 * @brief   Get the IO0 pin of the given QSPI bus.
 *
 * @param[in] bus       The device to query
 *
 * @note Until this is implemented on all platforms, this requires the
 *       periph_spi_reconfigure feature to be used.
 *
 * @return              The GPIO used for the QSPI IO0 line.
 */
gpio_t qspi_pin_io_2(qspi_t bus);
/**
 * @brief   Get the IO0 pin of the given QSPI bus.
 *
 * @param[in] bus       The device to query
 *
 * @note Until this is implemented on all platforms, this requires the
 *       periph_spi_reconfigure feature to be used.
 *
 * @return              The GPIO used for the QSPI IO0 line.
 */
gpio_t qspi_pin_io_3(qspi_t bus);

/**
 * @brief   Start a new QSPI transaction
 *
 * Starting a new SPI transaction will get exclusive access to the SPI bus
 * and configure it according to the given values. If another SPI transaction
 * is active when this function is called, this function will block until the
 * other transaction is complete (qspi_relase was called).
 *
 * @note    This function expects the @p bus and the @p cs parameters to be
 *          valid (they are checked in spi_init and spi_init_cs before)
 *
 * @param[in] bus       SPI device to access
 * @param[in] cs        chip select pin/line to use, set to SPI_CS_UNDEF if chip
 *                      select should not be handled by the SPI driver
 * @param[in] mode      mode to use for the new transaction
 * @param[in] clk       bus clock speed to use for the transaction
 *
 * @return              SPI_OK on success
 * @return              SPI_NOMODE if given mode is not supported
 * @return              SPI_NOCLK if given clock speed is not supported
 */
int qspi_acquire(qspi_t bus, qspi_flash_id_t id);

/**
 * @brief   Finish an ongoing SPI transaction by releasing the given SPI bus
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
void qspi_release(qspi_t bus);

/**
 * @brief   Initial mode to default value
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
static inline uint32_t qspi_cmd_init(void) { return 0; }

/**
 * @brief   Finish an ongoing SPI transaction by releasing the given SPI bus
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
static inline uint32_t qspi_cmd_set_cmd_lines(uint32_t mode,
                                               qspi_io_lines_t pins) {
  assert(pins != qspi_io_0lines); /* cmd lines must have one line at least*/
  return (mode & ~QUADSPI_CCR_IMODE) | (pins << QUADSPI_CCR_IMODE_Pos);
}

/**
 * @brief   Finish an ongoing SPI transaction by releasing the given SPI bus
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
static inline uint32_t qspi_cmd_set_data_lines(uint32_t mode,
                                                qspi_io_lines_t pins) {
  return (mode & ~QUADSPI_CCR_DMODE) | (pins << QUADSPI_CCR_DMODE_Pos);
}

/**
 * @brief   Set Alter Bytes Mode bytes
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to use
 */
static inline uint32_t qspi_cmd_set_abyte_lines(uint32_t mode,
                                                 qspi_io_lines_t pins) {
  return (mode & ~QUADSPI_CCR_ABMODE) | (pins << QUADSPI_CCR_ABMODE_Pos);
}

/**
 * @brief   Alter Bytes size
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
static inline uint32_t qspi_cmd_set_abyte_size(uint32_t mode, uint8_t size) {
  assert(size >= 1 && size <= 4);
  return (mode & ~QUADSPI_CCR_ABSIZE) | ((size - 1) << QUADSPI_CCR_ABSIZE_Pos);
}

/**
 * @brief   Set the Address Mode bits
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
static inline uint32_t qspi_cmd_set_addr_lines(uint32_t mode,
                                                qspi_io_lines_t pins) {
  return (mode & ~QUADSPI_CCR_ADMODE) | (pins << QUADSPI_CCR_ADMODE_Pos);
}

/**
 * @brief   Finish an ongoing SPI transaction by releasing the given SPI bus
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] mode       mode to be set to.
 * @param[in] size       size to be set to, can be set to 1 - 4
 */
static inline uint32_t qspi_cmd_set_addr_size(uint32_t mode, uint8_t size) {
  assert(size >= 1 && size <= 4);
  return (mode & ~QUADSPI_CCR_ADSIZE) | ((size - 1) << QUADSPI_CCR_ADSIZE_Pos);
}

/**
 * @brief   Dymmy Cycles
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
static inline uint32_t qspi_cmd_set_dcycs(uint32_t mode, uint8_t cycles) {
  assert(cycles < 32);
  return (mode & ~QUADSPI_CCR_DCYC) | (cycles << QUADSPI_CCR_DCYC_Pos);
}

/**
 * @brief Double Data Rate Mode
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
static inline uint32_t qspi_cmd_set_ddr_mode(uint32_t mode, bool ddr) {
  if (ddr) {
    /*!<Double data rate mode enabled*/
    return mode | QUADSPI_CCR_DDRM;
  } else {
    /*!<Double data rate mode disabled*/
    return mode & ~QUADSPI_CCR_DDRM;
  }
}

/**
 * @brief  Delay the data output by one half of system clock in DDR mode
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       QSPI device to release
 */
#if defined(QUADSPI_CCR_DHHC)
static inline uint32_t qspi_cmd_set_ddr_hhc(uint32_t mode, bool hhc) {
  if (hhc) {
    /*!<Delay the data output by one half of system clock in DDR mode*/
    return mode | QUADSPI_CCR_DHHC;
  } else {
    /*!<Delay the data output using analog delay in DDR mode*/
    return mode & ~QUADSPI_CCR_DHHC;
  }
}
#endif

/**
 * @brief   Send instruction on every transaction
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       QSPI device to use
 */
static inline uint32_t qspi_cmd_set_cmd_sioo(uint32_t mode, bool always) {
  if (always) {
    /* Send instruction on every transaction */
    return mode & ~QUADSPI_CCR_SIOO;
  } else {
    /*Send instruction only for the first command */
    return mode |= QUADSPI_CCR_SIOO;
  }
}

/**
 * @brief   Send instruction on every transaction
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       QSPI device to use
 */
static inline uint32_t qspi_cmd_set_inst(uint32_t cmd, uint8_t inst) {
  return (cmd & ~QUADSPI_CCR_INSTRUCTION) | (inst << QUADSPI_CCR_INSTRUCTION_Pos);
}

/**
 * @brief Send a command to QSPI bus
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
void qspi_command(qspi_t bus, uint32_t cmd, uint32_t addr,
                  uint32_t abytes, uint32_t len);

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
void qspi_send_bytes(qspi_t bus, const void *data);

/**
 * @brief Receive bytes from QSPI device
 *
 * @param[in] bus       QSPI device to use
 * @param[in] data      data to send out
 */
void qspi_recv_bytes(qspi_t bus, void *data);

#ifdef __cplusplus
}
#endif

#endif /* QSPI_H */
/** @} */
