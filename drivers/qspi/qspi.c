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

#include "qspi.h"
#include "kernel_defines.h"
#include "pm_layered.h"
#include "xtimer.h"
#include <assert.h>
#include <string.h>

#define ENABLE_DEBUG 0
#include "debug.h"

static mutex_t _qspi_locks[QSPI_NUMOF];

static inline QUADSPI_TypeDef *_dev(qspi_t bus) {
  assert(bus < QSPI_NUMOF);
  return qspi_config[bus].dev;
}

void qspi_init(qspi_t bus) {
  assert(bus < QSPI_NUMOF);

  mutex_init(&_qspi_locks[bus]);
  qspi_init_pins(bus);

  periph_clk_en(qspi_config[bus].apbbus, qspi_config[bus].rccmask);
  /* Disable QSPI bus */
  _dev(bus)->CR = 0;
  /*FIXME: what is the default value of DCR*/
  _dev(bus)->DCR = 0; /* Clear the DMA and SSOE flags */

  periph_clk_dis(qspi_config[bus].apbbus, qspi_config[bus].rccmask);
}

void qspi_deinit(qspi_t bus) {
  assert(bus < QSPI_NUMOF);
  /* Disable QSPI bus */
  _dev(bus)->CR &= ~QUADSPI_CR_EN;
}

void qspi_init_pins(qspi_t bus) {
  assert(bus < QSPI_NUMOF);
  assert(qspi_config[bus].cs_pin != GPIO_UNDEF);
  assert(qspi_config[bus].sclk_pin != GPIO_UNDEF);
  assert(qspi_config[bus].io_pin_0 != GPIO_UNDEF);
  assert(qspi_config[bus].io_pin_1 != GPIO_UNDEF);

#ifdef CPU_FAM_STM32F1
  gpio_init_af(spi_config[bus].cs_pin, GPIO_AF_OUT_PP);
  gpio_init_af(spi_config[bus].sclk_pin, GPIO_AF_OUT_PP);
  gpio_init_af(spi_config[bus].io_pin_0, GPIO_IN | GPIO_OUT);
  gpio_init_af(spi_config[bus].io_pin_1, GPIO_IN | GPIO_OUT);
  if (spi_config[bus].io_pin_2 != GPIO_UNDEF &&
      pi_config[bus].io_pin_3 != GPIO_UNDEF) {
    gpio_init_af(spi_config[bus].io_pin_2, GPIO_IN | GPIO_OUT);
    gpio_init_af(spi_config[bus].io_pin_3, GPIO_IN | GPIO_OUT);
  } else if (qspi_config[bus].io_pin_2 == GPIO_UNDEF &&
      qspi_config[bus].io_pin_3 == GPIO_UNDEF) {
      /* do nothing */  
  } else {
    assert(0) /*io_pin_2 and io_pin_3 must to be GPIO_UNDEF same time or all not */
  }
#else  /* CPU_FAM_STM32F1 */
  gpio_init(qspi_config[bus].cs_pin, GPIO_OUT);
  gpio_init(qspi_config[bus].sclk_pin, GPIO_OUT);
  gpio_init(qspi_config[bus].io_pin_0, GPIO_IN | GPIO_OUT);
  gpio_init(qspi_config[bus].io_pin_1, GPIO_IN | GPIO_OUT);
  gpio_init_af(qspi_config[bus].cs_pin, qspi_config[bus].cs_af);
  gpio_init_af(qspi_config[bus].sclk_pin, qspi_config[bus].sclk_af);
  gpio_init_af(qspi_config[bus].io_pin_0, qspi_config[bus].io_af_0);
  gpio_init_af(qspi_config[bus].io_pin_1, qspi_config[bus].io_af_1);
  if (qspi_config[bus].io_pin_2 != GPIO_UNDEF &&
      qspi_config[bus].io_pin_3 != GPIO_UNDEF) {
    gpio_init(qspi_config[bus].io_pin_2, GPIO_IN | GPIO_OUT);
    gpio_init(qspi_config[bus].io_pin_3, GPIO_IN | GPIO_OUT);
    gpio_init_af(qspi_config[bus].io_pin_2, qspi_config[bus].io_af_2);
    gpio_init_af(qspi_config[bus].io_pin_3, qspi_config[bus].io_af_3);
  } else if (qspi_config[bus].io_pin_2 == GPIO_UNDEF &&
      qspi_config[bus].io_pin_3 == GPIO_UNDEF) {
      /* do nothing */  
  } else {
    assert(0); /*io_pin_2 and io_pin_3 must to be GPIO_UNDEF same time or all not */
  }
#endif /* CPU_FAM_STM32F1 */
}

#ifdef MODULE_PERIPH_DMA
static inline bool _use_dma(const qspi_config_t *conf) {
  return conf->dma != DMA_STREAM_UNDEF;
}
#endif

int qspi_acquire(qspi_t bus, qspi_flash_id_t id) {
  assert(bus < QSPI_NUMOF);
  /* lock bus */
  mutex_lock(&(_qspi_locks[bus]));
#ifdef STM32_PM_STOP
  /* block POWER STOP mode */
  pm_block(STM32_PM_STOP);
#endif
  /* enable SPI device clock */
  periph_clk_en(qspi_config[bus].apbbus, qspi_config[bus].rccmask);

  /* Set FIFO Threshold bits */
  _dev(bus)->CR = (_dev(bus)->CR & ~QUADSPI_CR_FTHRES) |
                 ((QSPI_INIT_FIFO_THRESHOLD - 1U) << QUADSPI_CR_FTHRES_Pos);

  /* Wait till BUSY flag reset */
  while (_dev(bus)->SR & QUADSPI_SR_BUSY)
    ;

  uint32_t flash_id = 0x0U;
  uint32_t dual_mode = 0x0U;
#if defined(QUADSPI_CR_DFM)
  switch (id) {
  case qspi_flash_id_1:
    /* let flash_id and dual_mode to default value */
    break;
  case qspi_flash_id_2:
    flash_id = QUADSPI_CR_FSEL;
    break;
  case qspi_flash_id_dual:
    dual_mode = QUADSPI_CR_DFM;
    break;
  }
  _dev(bus)->CR = (_dev(bus)->CR & ~(QUADSPI_CR_PRESCALER | QUADSPI_CR_SSHIFT |
                                   QUADSPI_CR_FSEL | QUADSPI_CR_DFM)) |
                 ((QSPI_INIT_CLOCK_PRESCALER << QUADSPI_CR_PRESCALER_Pos) |
                  QSPI_INIT_SAMPLE_SHIFTING | flash_id | dual_mode);
#else /* QUADSPI_CR_DFM */
  _dev(bus)->CR = _dev(bus)->CR & ~(QUADSPI_CR_PRESCALER | QUADSPI_CR_SSHIFT) |
                 ((QSPI_INIT_CLOCK_PRESCALER << QUADSPI_CR_PRESCALER_Pos) |
                  QSPI_INIT_SAMPLE_SHIFTING);
#endif /* QUADSPI_CR_DFM */
  _dev(bus)->DCR = (_dev(bus)->DCR & ~(QUADSPI_DCR_FSIZE | QUADSPI_DCR_CSHT |
                                     QUADSPI_DCR_CKMODE)) |
                  ((QSPI_INIT_FLASH_SIZE << QUADSPI_DCR_FSIZE_Pos) |
                   QSPI_INIT_CHIP_SELECT_HIGN_TIME | QSPI_INIT_CLOCK_MODE);
  /* enable qspi bus */
  _dev(bus)->CR |= QUADSPI_CR_EN;

  return 0;
}

void qspi_release(qspi_t bus) {
  assert(bus < QSPI_NUMOF);
  /* Disable QSPI bus */
  _dev(bus)->CR = 0;
  /*FIXME: what is the default value of DCR*/
  _dev(bus)->DCR = 0; /* Clear all the flags */
  /* disable device */
  periph_clk_dis(qspi_config[bus].apbbus, qspi_config[bus].rccmask);
#ifdef STM32_PM_STOP
  /* unblock POWER STOP mode */
  pm_unblock(STM32_PM_STOP);
#endif
  /* release lock */
  mutex_unlock(&(_qspi_locks[bus]));
}

void qspi_command(qspi_t bus, uint32_t cmd, uint32_t addr,
                  uint32_t abytes, uint32_t len) {
  assert(bus < QSPI_NUMOF);

  /* Wait till BUSY flag reset */
  while (_dev(bus)->SR & QUADSPI_SR_BUSY)
    ;
  /* Data need to be transmit. */
  if (cmd & QUADSPI_CCR_DMODE) {
    /* set Data Length register */
    _dev(bus)->DLR = len - 1U;
  }
  /* Alter Bytes need to be transmit. */
  if (cmd & QUADSPI_CCR_ABMODE) {
    /* set Alter Bytes register */
    _dev(bus)->ABR = abytes;
  }

  /* Address need to be transmit. */
  if (cmd & QUADSPI_CCR_ADMODE) {
    /*---- Command with instruction, address and alternate bytes ----*/
    /* CCR register with all communications parameters */
    _dev(bus)->CCR = cmd;
    /* AR register with address value */
    _dev(bus)->AR = addr;
  } else {
    /*---- Command with instruction and alternate bytes ----*/
    /* CCR register with all communications parameters */
    _dev(bus)->CCR = cmd;
  }
  /* No data to transmit. */
  if ((cmd & QUADSPI_CCR_DMODE) == 0) {
    /* When there is no data phase, the transfer start as soon as the
       configuration is done so wait until TC flag is set to go back in idle
       state
    */
    while ((_dev(bus)->SR & QUADSPI_SR_TCF) == 0)
      ;
    /* Clear Transfer Complete bit */
    _dev(bus)->FCR = QUADSPI_SR_TCF;
  }
}
#if (defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) ||   \
     defined(STM32L485xx) || defined(STM32L486xx))
static void _transfer_abort(qspi_t bus) { assert(bus < QSPI_NUMOF); }
#endif

#ifdef MODULE_PERIPH_DMA
static void _send_bytes_dma(qspi_t bus, void *data, size_t count) {
  dma_acquire(qspi_config[bus].dma);
  /* Enable QSPI DMA*/
  _dev(bus)->CR |= QUADSPI_CR_DMAEN;
  dma_transfer(qspi_config[bus].dma, qspi_config[bus].dma_chan, data,
               (uint32_t *)&(_dev(bus)->DR), count, DMA_MEM_TO_PERIPH,
               DMA_DATA_WIDTH_BYTE | DMA_INC_SRC_ADDR);
  /* Wait for TCF flags is set */
  while ((_dev(bus)->SR & QUADSPI_SR_TCF) == 0)
    ;
  /* Wait for busy flag is clear */
  while (_dev(bus)->SR & QUADSPI_SR_BUSY)
    ;
  /* Clear TCF flag */
  _dev(bus)->FCR = QUADSPI_SR_TCF;
  /* Disable QSPI DMA*/
  _dev(bus)->CR &= ~QUADSPI_CR_DMAEN;
  dma_release(qspi_config[bus].dma);
}
#endif /* MODULE_PERIPH_DMA */

static void _send_bytes(qspi_t bus, const void *data, size_t count) {
  volatile uint8_t *DR = (volatile uint8_t *)&(_dev(bus)->DR);
  for (unsigned i = 0; i < count; i++) {
    /* Wait until FT flag is set to send data */
    while ((_dev(bus)->SR & QUADSPI_SR_FTF) == 0)
      ;
    *DR = ((const uint8_t *)data)[i];
  }

  /* Wait until TC flag is set to go back in idle state */
  while ((_dev(bus)->SR & QUADSPI_SR_TCF) == 0)
    ;

  /* Clear Transfer Complete bit */
  _dev(bus)->FCR = QUADSPI_SR_TCF;

#if (defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) ||   \
     defined(STM32L485xx) || defined(STM32L486xx))
  /* Workaround - Extra data written in the FIFO at the end of a read transfer
   */
  _transfer_abort(bus);
#endif
}

void qspi_send_bytes(qspi_t bus, const void *data) {
  assert(bus < QSPI_NUMOF);
  assert(data);

  /* Configure QSPI: CCR register with functional as indirect write */
  _dev(bus)->CCR &= ~QUADSPI_CCR_FMODE;

  /* Get data length from DLR register */
  const uint32_t count = _dev(bus)->DLR + 1;

#ifdef MODULE_PERIPH_DMA
  if (_use_dma(&qspi_config[bus])) {
    if (count < CONFIG_QSPI_DMATRANS_MINLEN) {
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
static void _recv_bytes_dma(qspi_t bus, void *data, size_t count) {
  dma_acquire(qspi_config[bus].dma);
  /* Enable QSPI DMA*/
  _dev(bus)->CR |= QUADSPI_CR_DMAEN;
  dma_transfer(qspi_config[bus].dma, qspi_config[bus].dma_chan,
               (uint32_t *)&(_dev(bus)->DR), data, count, DMA_PERIPH_TO_MEM,
               DMA_DATA_WIDTH_BYTE | DMA_INC_DST_ADDR);
  /* Wait for TCF flags is set */
  while ((_dev(bus)->SR & QUADSPI_SR_TCF) == 0)
    ;
  /* Wait for busy flag is clear */
  while (_dev(bus)->SR & QUADSPI_SR_BUSY)
    ;
  /* Clear TFC and FTF flags */
  _dev(bus)->FCR = QUADSPI_SR_TCF | QUADSPI_SR_FTF;
  /* Disable QSPI DMA*/
  _dev(bus)->CR &= ~QUADSPI_CR_DMAEN;
  dma_release(qspi_config[bus].dma);
}
#endif /* MODULE_PERIPH_DMA */

static void _recv_bytes(qspi_t bus, void *data, size_t count) {
  volatile uint8_t *DR = (volatile uint8_t *)&(_dev(bus)->DR);
  for (unsigned i = 0; i < count; i++) {
    /* Wait until FT or TC flag is set to read received data */
    while ((_dev(bus)->SR & QUADSPI_SR_TCF) == 0 &&
           (_dev(bus)->SR & QUADSPI_SR_FTF) == 0)
      ;
    ((uint8_t *)data)[i] = *DR;
  }

  /* Wait until TC flag is set to read received data */
  while ((_dev(bus)->SR & QUADSPI_SR_TCF) == 0)
    ;
  /* Clear Transfer Complete Flags bit */
  _dev(bus)->FCR = QUADSPI_SR_TCF;

#if (defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) ||   \
     defined(STM32L485xx) || defined(STM32L486xx))
  /* Workaround - Extra data written in the FIFO at the end of a read transfer
   */
  _transfer_abort(bus);
#endif
}

void qspi_recv_bytes(qspi_t bus, void *data) {
  assert(bus < QSPI_NUMOF);
  assert(data);
  uint32_t ar = _dev(bus)->AR;

  _dev(bus)->CCR = (_dev(bus)->CCR & ~QUADSPI_CCR_FMODE) | QUADSPI_CCR_FMODE_0;
  const uint32_t count = _dev(bus)->DLR + 1;
  /*
    Start the transfer by re-writing the address in AR register
    Receiving will not work if don't do this!!!
  */
  _dev(bus)->AR = ar;

#ifdef MODULE_PERIPH_DMA
  if (_use_dma(&qspi_config[bus])) {
    if (count < CONFIG_QSPI_DMATRANS_MINLEN) {
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

gpio_t qspi_pin_io_0(qspi_t bus) {
  assert(bus < QSPI_NUMOF);
  return qspi_config[bus].io_pin_0;
}

gpio_t qspi_pin_io_1(qspi_t bus) {
  assert(bus < QSPI_NUMOF);
  return qspi_config[bus].io_pin_1;
}

gpio_t qspi_pin_io_2(qspi_t bus) {
  assert(bus < QSPI_NUMOF);
  return qspi_config[bus].io_pin_2;
}
gpio_t qspi_pin_io_3(qspi_t bus) {
  assert(bus < QSPI_NUMOF);
  return qspi_config[bus].io_pin_3;
}

gpio_t qspi_pin_clk(qspi_t bus) {
  assert(bus < QSPI_NUMOF);
  return qspi_config[bus].sclk_pin;
}

gpio_t qspi_pin_cs(qspi_t bus) {
  assert(bus < QSPI_NUMOF);
  return qspi_config[bus].cs_pin;
}
