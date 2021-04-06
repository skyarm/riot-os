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

#include "sdio.h"
#include "kernel_defines.h"
#include "pm_layered.h"
#include "xtimer.h"
#include <assert.h>
#include <string.h>

#define ENABLE_DEBUG 0
#include "debug.h"

#ifdef CPU_FAM_STM32L4
#define SDIO_STA_TXACT SDMMC_STA_TXACT
#define SDIO_STA_RXACT SDMMC_STA_RXACT
#define SDIO_STA_CCRCFAIL SDMMC_STA_CCRCFAIL
#define SDIO_STA_DCRCFAIL SDMMC_STA_DCRCFAIL
#define SDIO_STA_TXUNDERR SDMMC_STA_TXUNDERR
#define SDIO_STA_RXOVERR SDMMC_STA_RXOVERR
#define SDIO_STA_CMDSENT SDMMC_STA_CMDSENT
#endif

typedef enum {
  SDIO_1B_BLOCK = 1,
  SDIO_2B_BLOCK = 2,
  SDIO_4B_BLOCK = 4,
  SDIO_8B_BLOCK = 8,
  SDIO_16B_BLOCK = 16,
  SDIO_32B_BLOCK = 32,
  SDIO_64B_BLOCK = 64,
  SDIO_128B_BLOCK = 128,
  SDIO_256B_BLOCK = 256,
  SDIO_512B_BLOCK = 512,
  SDIO_1024B_BLOCK = 1024,
  SDIO_2048B_BLOCK = 2048
} sdio_block_size_t;

static mutex_t _sdio_locks[SDIO_NUMOF];

static inline SDMMC_TypeDef *_dev(sdio_t bus) {
  assert(bus < SDIO_NUMOF);
  return sdio_config[bus].dev;
}

void sdio_init(sdio_t bus) {
  assert(bus < SDIO_NUMOF);

  mutex_init(&_sdio_locks[bus]);
  sdio_init_pins(bus);

  periph_clk_en(sdio_config[bus].apbbus, sdio_config[bus].rccmask);

  periph_clk_dis(sdio_config[bus].apbbus, sdio_config[bus].rccmask);
}

void sdio_deinit(sdio_t bus) {
  assert(bus < SDIO_NUMOF);
  /* Disable SDIO bus */
}

void sdio_init_pins(sdio_t bus) {
  assert(bus < SDIO_NUMOF);
  assert(sdio_config[bus].cmd_pin != GPIO_UNDEF);
  assert(sdio_config[bus].sclk_pin != GPIO_UNDEF);
  assert(sdio_config[bus].sclk_pin != GPIO_UNDEF);

  gpio_init(sdio_config[bus].cmd_pin, GPIO_OUT);
  gpio_init(sdio_config[bus].sclk_pin, GPIO_OUT);
  gpio_init(sdio_config[bus].data_pin_0, GPIO_IN | GPIO_OUT);
}

#ifdef MODULE_PERIPH_DMA
static inline bool _use_dma(const sdio_config_t *conf) {
  return conf->dma != DMA_STREAM_UNDEF;
}
#endif

int sdio_acquire(sdio_t bus) {
  assert(bus < SDIO_NUMOF);
  /* lock bus */
  mutex_lock(&(_sdio_locks[bus]));
#ifdef STM32_PM_STOP
  /* block POWER STOP mode */
  pm_block(STM32_PM_STOP);
#endif
  /* enable SPI device clock */
  periph_clk_en(sdio_config[bus].apbbus, sdio_config[bus].rccmask);

  return 0;
}

void sdio_release(sdio_t bus) {
  assert(bus < SDIO_NUMOF);

  /* disable device */
  periph_clk_dis(sdio_config[bus].apbbus, sdio_config[bus].rccmask);
#ifdef STM32_PM_STOP
  /* unblock POWER STOP mode */
  pm_unblock(STM32_PM_STOP);
#endif
  /* release lock */
  mutex_unlock(&(_sdio_locks[bus]));
}

void sdio_command(sdio_t bus, uint32_t cmd, uint32_t arg) {
  assert(bus < SDIO_NUMOF);
  _dev(bus)->ICR = 0xFFFFFFFF;
  _dev(bus)->ARG = arg;
  _dev(bus)->CMD = cmd;
  while ((_dev(bus)->STA & SDIO_STA_CMDSENT) == 0);
}

#ifdef MODULE_PERIPH_DMA
static void _send_bytes_dma(sdio_t bus, void *data, size_t count) {
  dma_acquire(sdio_config[bus].dma);

  dma_transfer(sdio_config[bus].dma, sdio_config[bus].dma_chan, data,
               (uint32_t *)&(_dev(bus)->FIFO), count, DMA_MEM_TO_PERIPH,
               DMA_DATA_WIDTH_BYTE | DMA_INC_SRC_ADDR);
  if ((_dev(bus)->STA & (SDIO_STA_CCRCFAIL | SDIO_STA_DCRCFAIL |
                         SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR)) != 0) {
  }
  /* Wait for STA flags is set */
  while ((_dev(bus)->STA & SDIO_STA_TXACT) != 0)
    ;
  dma_release(sdio_config[bus].dma);
}
#endif /* MODULE_PERIPH_DMA */

static void _send_bytes(sdio_t bus, const void *data, size_t count) {
  volatile uint8_t *FIFO = (volatile uint8_t *)&(_dev(bus)->FIFO);

  for (unsigned i = 0; i < count; i++) {
    /* Wait until FT flag is set to send data */
    /* Wait for STA flags is set */
    while ((_dev(bus)->STA & SDIO_STA_TXACT) != 0)
      ;
    *FIFO = ((const uint8_t *)data)[i];
  }

  /* Wait until TC flag is set to go back in idle state */
  while ((_dev(bus)->STA & SDIO_STA_TXACT) != 0)
    ;
}

void sdio_send_bytes(sdio_t bus, const void *data, size_t count) {
  assert(bus < SDIO_NUMOF);
  assert(data);
  /* Set data timer register to max*/
  _dev(bus)->DTIMER = (uint32_t)0xFFFFFFFF;
  /* Set data length */
  _dev(bus)->DLEN = count;
  _dev(bus)->DCTRL = 0; /* FIXME: */

#ifdef MODULE_PERIPH_DMA
  if (_use_dma(&sdio_config[bus])) {
    if (count < CONFIG_SDIO_DMATRANS_MINLEN) {
      _send_bytes(bus, (void *)data, count);
    } else {
      _send_bytes_dma(bus, (void *)data, count);
    }
  } else {
#endif
    _send_bytes(bus, data, count);
#ifdef MODULE_PERIPH_DMA
  }
#endif
}

#ifdef MODULE_PERIPH_DMA
static void _recv_bytes_dma(sdio_t bus, void *data, size_t count) {
  dma_acquire(sdio_config[bus].dma);

  dma_transfer(sdio_config[bus].dma, sdio_config[bus].dma_chan,
               (uint32_t *)&(_dev(bus)->FIFO), data, count, DMA_PERIPH_TO_MEM,
               DMA_DATA_WIDTH_BYTE | DMA_INC_DST_ADDR);
  if ((_dev(bus)->STA & (SDIO_STA_CCRCFAIL | SDIO_STA_DCRCFAIL |
                         SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR)) != 0) {
  }

  while ((_dev(bus)->STA & SDIO_STA_RXACT) != 0)
    ;
  dma_release(sdio_config[bus].dma);
}
#endif /* MODULE_PERIPH_DMA */

static void _recv_bytes(sdio_t bus, void *data, size_t count) {
  volatile uint8_t *FIFO = (volatile uint8_t *)&(_dev(bus)->FIFO);
  for (unsigned i = 0; i < count; i++) {
    /* Wait until FT or TC flag is set to read received data */
    while ((_dev(bus)->STA & SDIO_STA_RXACT) != 0)
      ;
    ((uint8_t *)data)[i] = *FIFO;
  }
  while ((_dev(bus)->STA & SDIO_STA_RXACT) != 0)
    ;
}

void sdio_recv_bytes(sdio_t bus, void *data, size_t count) {
  assert(bus < SDIO_NUMOF);
  assert(data);
  /* Enable QSPI DMA*/
  _dev(bus)->DTIMER = (uint32_t)0xFFFFFFFF;
  _dev(bus)->DLEN = count;
  _dev(bus)->DCTRL = 0; /* FIXME: */

#ifdef MODULE_PERIPH_DMA
  if (_use_dma(&sdio_config[bus])) {
    if (count < CONFIG_SDIO_DMATRANS_MINLEN) {
      _recv_bytes(bus, data, count);
    } else {
      _recv_bytes_dma(bus, data, count);
    }
  } else {
#endif
    _recv_bytes(bus, data, count);
#ifdef MODULE_PERIPH_DMA
  }
#endif
}
