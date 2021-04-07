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
#define SDIO_CMD_CMDINDEX SDMMC_CMD_CMDINDEX
#define SDIO_CMD_WAITRESP SDMMC_CMD_WAITRESP
#define SDIO_CMD_WAITRESP_Pos SDMMC_CMD_WAITRESP_Pos
#define SDIO_CMD_WAITINT SDMMC_CMD_WAITINT
#define SDIO_CMD_WAITINT_Pos SDMMC_CMD_WAITINT_Pos
#define SDIO_CMD_WAITPEND SDMMC_CMD_WAITPEND
#define SDIO_CMD_WAITPEND_Pos SDMMC_CMD_WAITPEND_Pos
#define SDIO_CMD_CPSMEN SDMMC_CMD_CPSMEN
#define SDIO_CMD_CPSMEN_Pos SDMMC_CMD_CPSMEN_Pos
#define SDIO_POWER_PWRCTRL SDMMC_POWER_PWRCTRL
#define SDIO_DCTRL_DBLOCKSIZE SDMMC_DCTRL_DBLOCKSIZE
#define SDIO_DCTRL_DBLOCKSIZE_Pos SDMMC_DCTRL_DBLOCKSIZE_Pos
#define SDIO_DCTRL_DTDIR SDMMC_DCTRL_DTDIR
#define SDIO_DCTRL_DTDIR_Pos SDMMC_DCTRL_DTDIR_Pos
#define SDIO_DCTRL_DTMODE SDMMC_DCTRL_DTMODE
#define SDIO_DCTRL_DTMODE_Pos SDMMC_DCTRL_DTMODE_Pos
#define SDIO_DCTRL_DTEN SDMMC_DCTRL_DTEN
#define SDIO_DCTRL_DTEN_Pos SDMMC_DCTRL_DTEN_Pos
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
static inline uint32_t sdio_data_init(void) { return 0; }

/**
 * @brief   Specifies the data block size for block transfer.
 *
 * This parameter can be a value of:
 *
 * @SDMMC_DATABLOCK_SIZE_1B               ((uint32_t)0x00000000U)
 * @SDMMC_DATABLOCK_SIZE_2B               SDMMC_DCTRL_DBLOCKSIZE_0
 * @SDMMC_DATABLOCK_SIZE_4B               SDMMC_DCTRL_DBLOCKSIZE_1
 * @SDMMC_DATABLOCK_SIZE_8B               (SDMMC_DCTRL_DBLOCKSIZE_0|SDMMC_DCTRL_DBLOCKSIZE_1)
 * @SDMMC_DATABLOCK_SIZE_16B              SDMMC_DCTRL_DBLOCKSIZE_2
 * @SDMMC_DATABLOCK_SIZE_32B              (SDMMC_DCTRL_DBLOCKSIZE_0|SDMMC_DCTRL_DBLOCKSIZE_2)
 * @SDMMC_DATABLOCK_SIZE_64B              (SDMMC_DCTRL_DBLOCKSIZE_1|SDMMC_DCTRL_DBLOCKSIZE_2)
 * @SDMMC_DATABLOCK_SIZE_128B             (SDMMC_DCTRL_DBLOCKSIZE_0|SDMMC_DCTRL_DBLOCKSIZE_1|SDMMC_DCTRL_DBLOCKSIZE_2)
 * @SDMMC_DATABLOCK_SIZE_256B             SDMMC_DCTRL_DBLOCKSIZE_3
 * @SDMMC_DATABLOCK_SIZE_512B             (SDMMC_DCTRL_DBLOCKSIZE_0|SDMMC_DCTRL_DBLOCKSIZE_3)
 * @SDMMC_DATABLOCK_SIZE_1024B            (SDMMC_DCTRL_DBLOCKSIZE_1|SDMMC_DCTRL_DBLOCKSIZE_3)
 * @SDMMC_DATABLOCK_SIZE_2048B            (SDMMC_DCTRL_DBLOCKSIZE_0|SDMMC_DCTRL_DBLOCKSIZE_1|SDMMC_DCTRL_DBLOCKSIZE_3)
 * @SDMMC_DATABLOCK_SIZE_4096B            (SDMMC_DCTRL_DBLOCKSIZE_2|SDMMC_DCTRL_DBLOCKSIZE_3)
 * @SDMMC_DATABLOCK_SIZE_8192B            (SDMMC_DCTRL_DBLOCKSIZE_0|SDMMC_DCTRL_DBLOCKSIZE_2|SDMMC_DCTRL_DBLOCKSIZE_3)
 * @SDMMC_DATABLOCK_SIZE_16384B           (SDMMC_DCTRL_DBLOCKSIZE_1|SDMMC_DCTRL_DBLOCKSIZE_2|SDMMC_DCTRL_DBLOCKSIZE_3)
 */
typedef enum {
  sdio_data_block_size_bit_1,
  sdio_data_block_size_bit_2,
  sdio_data_block_size_4bit,
  sdio_data_block_size_8bit,
  sdio_data_block_size_16bit,
  sdio_data_block_size_32bit,
  sdio_data_block_size_64bit,
  sdio_data_block_size_128bit,
  sdio_data_block_size_256bit,
  sdio_data_block_size_512bit,
  sdio_data_block_size_1024bit,
  sdio_data_block_size_2048bit,
  sdio_data_block_size_40966bit,
  sdio_data_block_size_8192bit,
  sdio_data_block_size_16384bit,
}sdio_data_block_size_t;

static inline uint32_t sdio_data_set_block_size(uint32_t data, sdio_data_block_size_t size) { 
  return  (data & ~SDIO_DCTRL_DBLOCKSIZE) | (size << SDIO_DCTRL_DBLOCKSIZE_Pos);
}

/**
 * Specifies the data transfer direction, whether the transfer is a read or write.
 * This parameter can be a value of:
 **/
typedef enum {
  sdio_data_set_transfer_to_card,
  sdio_data_set_transfer_to_sdmmc
} sdio_data_set_transfer_dir_t;

static inline uint32_t sdio_data_set_transfer_dir(uint32_t data, sdio_data_set_transfer_dir_t dir) {
  return (data & ~SDIO_DCTRL_DTDIR) | (dir << SDIO_DCTRL_DTDIR_Pos);
}

/**
 * Specifies whether data transfer is in stream or block mode.
 * This parameter can be a value of:
 **/
typedef enum {
  sdio_data_set_transfer_block,
  sdio_data_set_transfer_stream
} sdio_data_set_transfer_mode_t;

static inline uint32_t sdio_data_set_transfer_mode(uint32_t data, sdio_data_set_transfer_mode_t mode) {
  return (data & ~SDIO_DCTRL_DTMODE) | (mode << SDIO_DCTRL_DTMODE_Pos);
}

/**
 * Specifies whether SDMMC Data path state machine (DPSM) is enabled or disabled.
 * This parameter can be a value of:
 **/
static inline uint32_t sdio_data_set_dpsm(uint32_t data, bool enable) {
  return (data & ~SDIO_DCTRL_DTEN) | ((enable ? 1 : 0) << SDIO_DCTRL_DTEN_Pos);
}

/**
 * @brief   Initial mode to default value
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */
static inline uint32_t sdio_cmd_init(void) { return 0; }

/**
 * Specifies the SDMMC command instruction. It must be 0 to 64
 **/
static inline uint32_t sdio_cmd_set_inst(uint32_t cmd, uint8_t index) {
  assert(index <= 64);
  return (cmd & ~SDIO_CMD_CMDINDEX) | index;
}

/**
 *    Specifies the SDMMC response type, This parameter can be a value of:
 **/
typedef enum {
  sdio_cmd_wait_resp_no,
  sdio_cmd_wait_resp_short,
  sdio_cmd_wait_resp_long
} sdio_cmd_wait_resp_t;

static inline uint32_t sdio_cmd_set_wait_resp(uint32_t cmd,
                                              sdio_cmd_wait_resp_t resp) {
  return (cmd & ~SDIO_CMD_WAITRESP) | (resp << SDIO_CMD_WAITRESP_Pos);
}

/**
 * Specifies whether SDMMC wait for interrupt request is enabled or disabled.
 * This parameter can be a value of:
 **/
static inline uint32_t sdio_cmd_set_wait_int(uint32_t cmd, bool wait) {
  return (cmd & ~SDIO_CMD_WAITINT) | ((wait ? 1 : 0) << SDIO_CMD_WAITINT_Pos);
}

/**
 * Specifies whether SDMMC wait for pend request is enabled or disabled.
 * This parameter can be a value of:
 **/
static inline uint32_t sdio_cmd_set_wait_pend(uint32_t cmd, bool wait) {
  return (cmd & ~SDIO_CMD_WAITPEND) | ((wait ? 1 : 0) << SDIO_CMD_WAITPEND_Pos);
}

/**
 *  Specifies whether SDMMC Command path state machine (CPSM) is enabled or
 *disabled. This parameter can be a value of
 * @0   0
 * @1   SDMMC_CMD_CPSMEN
 **/
static inline uint32_t sdio_cmd_set_cpsm(uint32_t cmd, bool enable) {
  return (cmd & ~SDIO_CMD_CPSMEN) | ((enable ? 1 : 0) << SDIO_CMD_CPSMEN_Pos);
}

/**
 * @brief   Initial mode to default value
 *
 * After release, the given SPI bus should be fully powered down until acquired
 * again.
 *
 * @param[in] bus       SPI device to release
 */

void sdio_command(sdio_t bus, uint32_t cmd, uint32_t arg);

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
void sdio_recv_bytes(sdio_t bus, void *data, size_t count);

#ifdef __cplusplus
}
#endif

#endif /* SDIO_H */
/** @} */
