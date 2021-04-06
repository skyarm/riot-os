/*
 * Copyright (C) 2016 Eistec AB
 *               2017 OTA keys S.A.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_mtd_qspi_nor
 * @{
 *
 * @file
 * @brief       Driver for serial flash memory attached to SPI
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Vincent Dupont <vincent@otakeys.com>
 *
 * @}
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "mtd.h"
#include "qspi_nor.h"
#include "thread.h"
#include "xtimer.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#define ENABLE_TRACE 0
#define TRACE(...) DEBUG(__VA_ARGS__)

/* after power up, on an invalid JEDEC ID, wait and read N times */
#ifndef MTD_POWER_UP_WAIT_FOR_ID
#define MTD_POWER_UP_WAIT_FOR_ID (0x0F)
#endif

#define SFLASH_CMD_4_BYTE_ADDR (0xB7) /**< enable 32 bit addressing */
#define SFLASH_CMD_3_BYTE_ADDR (0xE9) /**< enable 24 bit addressing */

#define QSPI_NOR_64K (65536ul)
#define QSPI_NOR_64K_ADDR_MASK (0xFFFF)
#define QSPI_NOR_32K (32768ul)
#define QSPI_NOR_32K_ADDR_MASK (0x7FFF)
#define QSPI_NOR_4K (4096ul)
#define QSPI_NOR_4K_ADDR_MASK (0xFFF)

#define MBIT_AS_BYTES ((1024 * 1024) / 8)

#define MIN(a, b) ((a) > (b) ? (b) : (a))

/**
 * @brief   JEDEC memory manufacturer ID codes.
 *
 *          see http://www.softnology.biz/pdf/JEP106AV.pdf
 * @{
 */
#define JEDEC_BANK(n) ((n) << 8)

typedef enum {
  QSPI_NOR_JEDEC_ATMEL = 0x1F | JEDEC_BANK(1),
} jedec_manuf_t;
/** @} */

static inline qspi_t _get_qspi(const qspi_nor_t *dev) {
  return dev->params->qspi;
}

static inline void _acquire(const qspi_nor_t *dev) {
  qspi_acquire(_get_qspi(dev), dev->params->flash_id);
}

static inline void _release(const qspi_nor_t *dev) {
  qspi_release(_get_qspi(dev));
}

static inline uint8_t _opcode_dcycs(qspi_nor_t *dev, uint8_t opcode) {
  if (opcode == dev->params->opcode->read_fast) {
    return 8;
  } else {
    return 0;
  }
}
/**
 * @internal
 * @brief Send command opcode followed by address, followed by a read to buffer
 *
 * @param[in]  dev    pointer to device descriptor
 * @param[in]  opcode command opcode
 * @param[in]  addr   address (big endian)
 * @param[out] dest   read buffer
 * @param[in]  count  number of bytes to read after the address has been sent
 */
static void _cmd_address_read(qspi_nor_t *dev, uint8_t opcode,
                                   uint32_t addr, void *dest, uint32_t count) {
  TRACE("_cmd_address_read: %p, %02x, (%06" PRIx32 "), %p, %" PRIu32 "\n",
        (void *)dev, (unsigned int)opcode, addr, dest, count);

  dev->cmd = qspi_cmd_set_inst(dev->cmd, opcode);
  dev->cmd = qspi_cmd_set_addr_lines(dev->cmd, qspi_io_1lines);
  dev->cmd = qspi_cmd_set_dcycs(dev->cmd, _opcode_dcycs(dev, opcode));
  if (dest && count) {
    dev->cmd = qspi_cmd_set_data_lines(dev->cmd, qspi_io_1lines);
  } else {
    dev->cmd = qspi_cmd_set_data_lines(dev->cmd, qspi_io_0lines);
  }

  /* Send opcode followed by address */
  qspi_command(_get_qspi(dev), dev->cmd, addr, 0, count);
  if (dest && count) {
    qspi_recv_bytes(_get_qspi(dev), dest);
  }
}

/**
 * @internal
 * @brief Send command opcode followed by address, followed by a write from
 * buffer
 *
 * @param[in]  dev    pointer to device descriptor
 * @param[in]  opcode command opcode
 * @param[in]  addr   address (big endian)
 * @param[out] src    write buffer
 * @param[in]  count  number of bytes to write after the opcode has been sent
 */
static void _cmd_address_write(qspi_nor_t *dev, uint8_t opcode,
                                    uint32_t addr, const void *src,
                                    uint32_t count) {
  TRACE("_cmd_address_write: %p, %02x, (%06" PRIx32 "), %p, %" PRIu32 "\n",
        (void *)dev, (unsigned int)opcode, addr, src, count);

  dev->cmd = qspi_cmd_set_inst(dev->cmd, opcode);
  dev->cmd = qspi_cmd_set_addr_lines(dev->cmd, qspi_io_1lines);
  dev->cmd = qspi_cmd_set_dcycs(dev->cmd, _opcode_dcycs(dev, opcode));
  if (src && count) {
    dev->cmd = qspi_cmd_set_data_lines(dev->cmd, qspi_io_1lines);
  } else {
    dev->cmd = qspi_cmd_set_data_lines(dev->cmd, qspi_io_0lines);
  }

  /* Send opcode followed by address */
  qspi_command(_get_qspi(dev), dev->cmd, addr, 0, count);
  if (src && count) {
    qspi_send_bytes(_get_qspi(dev), src);
  }
}

/**
 * @internal
 * @brief Send command opcode followed by a read to buffer
 *
 * @param[in]  dev    pointer to device descriptor
 * @param[in]  opcode command opcode
 * @param[out] dest   read buffer
 * @param[in]  count  number of bytes to write after the opcode has been sent
 */
static void _cmd_read(qspi_nor_t *dev, uint8_t opcode, void *dest,
                              uint32_t count) {
  TRACE("_cmd_read: %p, %02x, %p, %" PRIu32 "\n", (void *)dev,
        (unsigned int)opcode, dest, count);

  dev->cmd = qspi_cmd_set_inst(dev->cmd, opcode);
  dev->cmd = qspi_cmd_set_addr_lines(dev->cmd, qspi_io_0lines); /*no address*/
  dev->cmd = qspi_cmd_set_dcycs(
      dev->cmd, _opcode_dcycs(dev, opcode)); /*no dummy cycles*/
  if (dest && count) {
    dev->cmd = qspi_cmd_set_data_lines(dev->cmd, qspi_io_1lines);
  } else {
    dev->cmd = qspi_cmd_set_data_lines(dev->cmd, qspi_io_0lines);
  }

  /* Send opcode */
  qspi_command(_get_qspi(dev), dev->cmd, 0, 0, count);
  if (dest && count) {
    qspi_recv_bytes(_get_qspi(dev), dest);
  }
}

/**
 * @internal
 * @brief Send command opcode followed by a write from buffer
 *
 * @param[in]  dev    pointer to device descriptor
 * @param[in]  opcode command opcode
 * @param[out] src    write buffer
 * @param[in]  count  number of bytes to write after the opcode has been sent
 */
static void __attribute__((unused))
_cmd_write(qspi_nor_t *dev, uint8_t opcode, const void *src,
                   uint32_t count) {
  TRACE("_cmd_write: %p, %02x, %p, %" PRIu32 "\n", (void *)dev,
        (unsigned int)opcode, src, count);

  dev->cmd = qspi_cmd_set_inst(dev->cmd, opcode);
  dev->cmd = qspi_cmd_set_addr_lines(dev->cmd, qspi_io_0lines); /*no address*/
  dev->cmd = qspi_cmd_set_dcycs(
      dev->cmd, _opcode_dcycs(dev, opcode)); /*no dummy cycles*/
  if (src && count) {
    dev->cmd = qspi_cmd_set_data_lines(dev->cmd, qspi_io_1lines);
  } else {
    dev->cmd = qspi_cmd_set_data_lines(dev->cmd, qspi_io_0lines);
  }

  /* Send opcode */
  qspi_command(_get_qspi(dev), dev->cmd, 0, 0, count);
  if (src && count) {
    qspi_send_bytes(_get_qspi(dev), src);
  }
}

/**
 * @internal
 * @brief Send command opcode
 *
 * @param[in]  dev    pointer to device descriptor
 * @param[in]  opcode command opcode
 */
static void _cmd_only(qspi_nor_t *dev, uint8_t opcode) {
  TRACE("mtd_qspi_cmd: %p, %02x\n", (void *)dev, (unsigned int)opcode);

  dev->cmd = qspi_cmd_set_inst(dev->cmd, opcode);
  dev->cmd = qspi_cmd_set_addr_lines(dev->cmd, qspi_io_0lines);
  dev->cmd = qspi_cmd_set_data_lines(dev->cmd, qspi_io_0lines);
  dev->cmd = qspi_cmd_set_dcycs(dev->cmd, _opcode_dcycs(dev, opcode));

  /* Send opcode */
  qspi_command(_get_qspi(dev), dev->cmd, 0, 0, 0);
}

static bool qspi_nor_manuf_match(const jedec_id_t *id, jedec_manuf_t manuf) {
  return manuf == ((id->bank << 8) | id->manuf);
}

/**
 * @internal
 * @brief Compute 8 bit parity
 */
static inline uint8_t _parity8(uint8_t x) {
  /* Taken from http://stackoverflow.com/a/21618038/1805713 */
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return (x & 1);
}

/**
 * @internal
 * @brief Read JEDEC ID
 */
static int _read_jedec_id(qspi_nor_t *dev, jedec_id_t *out) {
  uint8_t buffer[JEDEC_BANK_MAX + sizeof(jedec_id_t) - 1];

  DEBUG("_read_jedec_id: rdid=0x%02x\n",
        (unsigned int)dev->params->opcode->rdid);

  /* Send opcode */
  _cmd_read(dev, dev->params->opcode->rdid, buffer, sizeof(buffer));

  /* Manufacturer IDs are organized in 'banks'.
   * If we read the 'next bank' instead of manufacturer ID, skip
   * the byte and increment the bank counter.
   */
  uint8_t bank = 0;
  while (buffer[bank] == JEDEC_NEXT_BANK) {
    if (++bank == JEDEC_BANK_MAX) {
      DEBUG_PUTS("_read_jedec_id: bank out of bounds\n")
      return -1;
    }
  }

  if (_parity8(buffer[bank]) == 0) {
    /* saw even parity, we expected odd parity => parity error */
    DEBUG("_read_jedec_id: Parity error (0x%02x)\n", buffer[bank]);
    return -2;
  }

  if (buffer[bank] == 0xFF || buffer[bank] == 0x00) {
    DEBUG_PUTS("_read_jedec_id: failed to read manufacturer ID");
    return -3;
  }

  /* Copy manufacturer ID */
  out->bank = bank + 1;
  memcpy((uint8_t *)out + 1, &buffer[bank], 3);

  DEBUG("_read_jedec_id: bank=%u manuf=0x%02x\n",
        (unsigned int)out->bank, (unsigned int)out->manuf);

  DEBUG("_read_jedec_id: device=0x%02x, 0x%02x\n",
        (unsigned int)out->device[0], (unsigned int)out->device[1]);

  return 0;
}

/**
 * @internal
 * @brief Get Flash capacity based on JEDEC ID
 *
 * @note The way the capacity is encoded differs between vendors.
 *       This formula has been tested with flash chips from Adesto,
 *       ISSI, Micron and Spansion, but it might not cover all cases.
 *       Please extend the function if necessary.
 */
static uint32_t _get_size_by_jedec_id(const jedec_id_t *id) {
  /* old Atmel (now Adesto) parts use 5 lower bits of device ID 1 for density */
  if (qspi_nor_manuf_match(id, QSPI_NOR_JEDEC_ATMEL) &&
      /* ID 2 is used to encode the product version, usually 1 or 2 */
      (id->device[1] & ~0x3) == 0) {
    return (0x1F & id->device[0]) * MBIT_AS_BYTES;
  }

  /* everyone else seems to use device ID 2 for density */
  return 1 << id->device[1];
}

static inline void _wait_for_write_complete(qspi_nor_t *dev, uint32_t us) {
  unsigned i = 0, j = 0;
  uint32_t div = 2;
  uint32_t diff = 0;
  if (IS_ACTIVE(ENABLE_DEBUG) && IS_USED(MODULE_XTIMER)) {
    diff = xtimer_now_usec();
  }
  do {
    uint8_t status;
    _cmd_read(dev, dev->params->opcode->rdsr, &status, sizeof(status));

    TRACE("mtd_qspi_nor: wait device status = 0x%02x\n", (unsigned int)status);
    if ((status & 1) == 0) { /* TODO magic number */
      break;
    }
    i++;
#if MODULE_XTIMER
    if (us) {
      xtimer_usleep(us);
      /* reduce the waiting time quickly if the estimate was too short,
       * but still avoid busy (yield) waiting */
      if (us > 2 * XTIMER_BACKOFF) {
        us -= (us / div);
        div++;
      } else {
        us = 2 * XTIMER_BACKOFF;
      }
    } else {
      j++;
      thread_yield();
    }
#else
    (void)div;
    (void)us;
    thread_yield();
#endif
  } while (1);
  DEBUG("wait loop %u times, yield %u times", i, j);
  if (IS_ACTIVE(ENABLE_DEBUG) && IS_ACTIVE(MODULE_XTIMER)) {
    diff = xtimer_now_usec() - diff;
    DEBUG(", total wait %" PRIu32 "us", diff);
  }
  DEBUG("\n");
}

static int qspi_nor_power(mtd_dev_t *mtd, enum mtd_power_state power) {
  qspi_nor_t *dev = (qspi_nor_t *)mtd;

  _acquire(dev);
  switch (power) {
  case MTD_POWER_UP:
    _cmd_only(dev, dev->params->opcode->wake);
#if defined(MODULE_XTIMER)
    /* No sense in trying multiple times if no xtimer to wait between
       reads */
    uint8_t retries = 0;
    int res = 0;
    do {
      xtimer_usleep(dev->params->wait_chip_wake_up);
      res = _read_jedec_id(dev, &dev->jedec_id);
      retries++;
    } while (res < 0 || retries < MTD_POWER_UP_WAIT_FOR_ID);
    if (res < 0) {
      return -EIO;
    }
#endif
    /* enable 32 bit address mode */
    if (dev->params->addr_width == 4) {
      _cmd_only(dev, dev->params->opcode->wren);
      _cmd_only(dev, SFLASH_CMD_4_BYTE_ADDR);
    }

    break;
  case MTD_POWER_DOWN:
    _cmd_only(dev, dev->params->opcode->sleep);
    break;
  }
  _release(dev);

  return 0;
}

static void _init_default_mode(qspi_nor_t *dev) {
  dev->cmd = qspi_cmd_init();
  dev->cmd = qspi_cmd_set_cmd_sioo(dev->cmd, true);
  dev->cmd = qspi_cmd_set_ddr_mode(dev->cmd, false);
  dev->cmd = qspi_cmd_set_ddr_hhc(dev->cmd, false);

  dev->cmd = qspi_cmd_set_cmd_lines(dev->cmd, qspi_io_1lines);
  dev->cmd = qspi_cmd_set_addr_lines(dev->cmd, qspi_io_0lines);
  dev->cmd = qspi_cmd_set_addr_size(dev->cmd, dev->params->addr_width);
  dev->cmd = qspi_cmd_set_abyte_lines(dev->cmd, qspi_io_0lines);
  dev->cmd = qspi_cmd_set_data_lines(dev->cmd, qspi_io_0lines);
  dev->cmd = qspi_cmd_set_dcycs(dev->cmd, 0);
}

static int qspi_nor_init(mtd_dev_t *mtd) {
  DEBUG("qspi_nor_init: %p\n", (void *)mtd);
  qspi_nor_t *dev = (qspi_nor_t *)mtd;

  DEBUG("qspi_nor_init: -> qspi: %lx, opcodes: %p\n",
        (unsigned long)_get_qspi(dev), (void *)dev->params->opcode);

  /* verify configuration */
  assert(dev->params->addr_width > 0);
  assert(dev->params->addr_width <= 4);

  /* initial qspi device */
  DEBUG("qspi_nor_init: initial qspi device");
  qspi_init(dev->params->qspi);
  _init_default_mode(dev);

  /* power up the MTD device*/
  DEBUG("qspi_nor_init: power up MTD device");
  if (qspi_nor_power(mtd, MTD_POWER_UP)) {
    DEBUG("qspi_nor_init: failed to power up MTD device");
    return -EIO;
  }

  _acquire(dev);
  int res = _read_jedec_id(dev, &dev->jedec_id);
  if (res < 0) {
    _release(dev);
    return -EIO;
  }
  DEBUG("qspi_nor_init: Found chip with ID: (%d, 0x%02x, 0x%02x, 0x%02x)\n",
        dev->jedec_id.bank, dev->jedec_id.manuf, dev->jedec_id.device[0],
        dev->jedec_id.device[1]);

  /* derive density from JEDEC ID  */
  if (mtd->sector_count == 0) {
    mtd->sector_count = _get_size_by_jedec_id(&dev->jedec_id) /
                        (mtd->pages_per_sector * mtd->page_size);
  }

  DEBUG("qspi_nor_init: %" PRIu32 " bytes "
        "(%" PRIu32 " sectors, %" PRIu32 " bytes/sector, "
        "%" PRIu32 " pages, "
        "%" PRIu32 " pages/sector, %" PRIu32 " bytes/page)\n",
        mtd->pages_per_sector * mtd->sector_count * mtd->page_size,
        mtd->sector_count, mtd->pages_per_sector * mtd->page_size,
        mtd->pages_per_sector * mtd->sector_count, mtd->pages_per_sector,
        mtd->page_size);
  DEBUG("qspi_nor_init: Using %u byte addresses\n", dev->params->addr_width);

  uint8_t status;
  _cmd_read(dev, dev->params->opcode->rdsr, &status, sizeof(status));
  _release(dev);

  DEBUG("qspi_nor_init: device status = 0x%02x\n", (unsigned int)status);

  /* check whether page size and sector size are powers of two (most chips' are)
   * and compute the number of shifts needed to get the page and sector
   * addresses from a byte address */
  uint8_t shift = 0;
  uint32_t page_size = mtd->page_size;
  uint32_t mask = 0;

  if ((page_size & (page_size - 1)) == 0) {
    while ((page_size >> shift) > 1) {
      ++shift;
    }
    mask = (UINT32_MAX << shift);
  }

  dev->page_addr_mask = mask;
  dev->page_addr_shift = shift;
  DEBUG("qspi_nor_init: page_addr_mask = 0x%08" PRIx32
        ", page_addr_shift = %u\n",
        mask, (unsigned int)shift);

  mask = 0;
  shift = 0;
  uint32_t sector_size = mtd->page_size * mtd->pages_per_sector;
  if ((sector_size & (sector_size - 1)) == 0) {
    while ((sector_size >> shift) > 1) {
      ++shift;
    }
    mask = (UINT32_MAX << shift);
  }
  dev->sec_addr_mask = mask;
  dev->sec_addr_shift = shift;

  DEBUG("qspi_nor_init: sec_addr_mask = 0x%08" PRIx32 ", sec_addr_shift = %u\n",
        mask, (unsigned int)shift);

  return 0;
}

static int qspi_nor_read(mtd_dev_t *mtd, void *dest, uint32_t addr,
                         uint32_t size) {
  DEBUG("qspi_nor_read: %p, %p, 0x%" PRIx32 ", 0x%" PRIx32 "\n", (void *)mtd,
        dest, addr, size);
  qspi_nor_t *dev = (qspi_nor_t *)mtd;
  uint32_t chipsize =
      mtd->page_size * mtd->pages_per_sector * mtd->sector_count;

  if (addr > chipsize) {
    return -EOVERFLOW;
  }
  if ((addr + size) > chipsize) {
    size = chipsize - addr;
  }
  if (size == 0) {
    return 0;
  }

  _acquire(dev);
  _cmd_address_read(dev, dev->params->opcode->read, addr, dest, size);
  _release(dev);

  return 0;
}

static int qspi_nor_write(mtd_dev_t *mtd, const void *src, uint32_t addr,
                          uint32_t size) {
  uint32_t total_size =
      mtd->page_size * mtd->pages_per_sector * mtd->sector_count;

  DEBUG("qspi_nor_write: %p, %p, 0x%" PRIx32 ", 0x%" PRIx32 "\n", (void *)mtd,
        src, addr, size);
  if (size == 0) {
    return 0;
  }
  qspi_nor_t *dev = (qspi_nor_t *)mtd;
  if (size > mtd->page_size) {
    DEBUG("qspi_nor_write: ERR: page program >1 page (%" PRIu32 ")!\n",
          mtd->page_size);
    return -EOVERFLOW;
  }
  if (dev->page_addr_mask && ((addr & dev->page_addr_mask) !=
                              ((addr + size - 1) & dev->page_addr_mask))) {
    DEBUG("qspi_nor_write: ERR: page program spans page boundary!\n");
    return -EOVERFLOW;
  }
  if (addr + size > total_size) {
    return -EOVERFLOW;
  }

  _acquire(dev);

  /* write enable */
  _cmd_only(dev, dev->params->opcode->wren);

  /* Page program */
  _cmd_address_write(dev, dev->params->opcode->page_program, addr, src,
                          size);

  /* waiting for the command to complete before returning */
  _wait_for_write_complete(dev, 0);

  _release(dev);

  return 0;
}

static int qspi_nor_write_page(mtd_dev_t *mtd, const void *src, uint32_t page,
                               uint32_t offset, uint32_t size) {
  qspi_nor_t *dev = (qspi_nor_t *)mtd;

  DEBUG("qspi_nor_write_page: %p, %p, 0x%" PRIx32 ", 0x%" PRIx32 ", 0x%" PRIx32
        "\n",
        (void *)mtd, src, page, offset, size);

  uint32_t remaining = mtd->page_size - offset;
  size = MIN(remaining, size);

  uint32_t addr = page * mtd->page_size + offset;

  _acquire(dev);

  /* write enable */
  _cmd_only(dev, dev->params->opcode->wren);

  /* Page program */
  _cmd_address_write(dev, dev->params->opcode->page_program, addr, src,
                          size);

  /* waiting for the command to complete before returning */
  _wait_for_write_complete(dev, 0);

  _release(dev);

  return size;
}

static int qspi_nor_erase(mtd_dev_t *mtd, uint32_t addr, uint32_t size) {
  DEBUG("qspi_nor_erase: %p, 0x%" PRIx32 ", 0x%" PRIx32 "\n", (void *)mtd, addr,
        size);
  qspi_nor_t *dev = (qspi_nor_t *)mtd;
  uint32_t sector_size = mtd->page_size * mtd->pages_per_sector;
  uint32_t total_size = sector_size * mtd->sector_count;

  if (dev->sec_addr_mask && ((addr & ~dev->sec_addr_mask) != 0)) {
    /* This is not a requirement in hardware, but it helps in catching
     * software bugs (the erase-all-your-files kind) */
    DEBUG("addr = %" PRIx32 " ~dev->erase_addr_mask = %" PRIx32 "", addr,
          ~dev->sec_addr_mask);
    DEBUG("qspi_nor_erase: ERR: erase addr not aligned on %" PRIu32
          " byte boundary.\n",
          sector_size);
    return -EOVERFLOW;
  }
  if (addr + size > total_size) {
    return -EOVERFLOW;
  }
  if (size % sector_size != 0) {
    return -EOVERFLOW;
  }

  _acquire(dev);
  while (size) {
    uint32_t us;

    /* write enable */
    _cmd_only(dev, dev->params->opcode->wren);

    if (size == total_size) {
      _cmd_only(dev, dev->params->opcode->chip_erase);
      size -= total_size;
      us = dev->params->wait_chip_erase;
    } else if ((dev->params->flag & QSPI_NOR_F_SECT_64K) &&
               (size >= QSPI_NOR_64K) &&
               ((addr & QSPI_NOR_64K_ADDR_MASK) == 0)) {
      /* 64 KiB blocks can be erased with block erase command */
      _cmd_address_write(dev, dev->params->opcode->block_erase_64k, addr,
                              NULL, 0);
      addr += QSPI_NOR_64K;
      size -= QSPI_NOR_64K;
      us = dev->params->wait_64k_erase;
    } else if ((dev->params->flag & QSPI_NOR_F_SECT_32K) &&
               (size >= QSPI_NOR_32K) &&
               ((addr & QSPI_NOR_32K_ADDR_MASK) == 0)) {
      /* 32 KiB blocks can be erased with block erase command */
      _cmd_address_write(dev, dev->params->opcode->block_erase_32k, addr,
                              NULL, 0);
      addr += QSPI_NOR_32K;
      size -= QSPI_NOR_32K;
      us = dev->params->wait_32k_erase;
    } else if ((dev->params->flag & QSPI_NOR_F_SECT_4K) &&
               (size >= QSPI_NOR_4K) && ((addr & QSPI_NOR_4K_ADDR_MASK) == 0)) {
      /* 4 KiB sectors can be erased with sector erase command */
      _cmd_address_write(dev, dev->params->opcode->sector_erase, addr,
                              NULL, 0);
      addr += QSPI_NOR_4K;
      size -= QSPI_NOR_4K;
      us = dev->params->wait_sector_erase;
    } else {
      /* no suitable erase block found */
      assert(0);

      _release(dev);
      return -EINVAL;
    }

    /* waiting for the command to complete before continuing */
    _wait_for_write_complete(dev, us);
  }
  _release(dev);

  return 0;
}

const mtd_desc_t qspi_nor_driver = {
    .init = qspi_nor_init,
    .read = qspi_nor_read,
    .write = qspi_nor_write,
    .write_page = qspi_nor_write_page,
    .erase = qspi_nor_erase,
    .power = qspi_nor_power,
};
