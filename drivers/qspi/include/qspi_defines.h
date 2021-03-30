#ifndef QSPI_DEFINES_H
#define QSPI_DEFINES_H

#include "cpu_conf.h"

/*
 *    Specifies the threshold number of bytes in the FIFO (used only in indirect
 * mode) This parameter can be a value between 1 and 16
 */
#ifndef QSPI_INIT_FIFO_THRESHOLD
#define QSPI_INIT_FIFO_THRESHOLD (0x00000004U)
#endif

/*
 *    Specifies the prescaler factor for generating clock based on the AHB
 * clock. This parameter can be a number between 0 and 255
 */
#ifndef QSPI_INIT_CLOCK_PRESCALER
#define QSPI_INIT_CLOCK_PRESCALER (0x00000000U)
#endif

/*
 *    Specifies the Sample Shift. The data is sampled 1/2 clock cycle delay
 * later to take in account external signal delays. (It should be 0x00000000U in
 * DDR mode):
 *  @1  0x00000000U         No clock cycle shift to sample data
 *  @2  QUADSPI_CR_SSHIFT   1/2 clock cycle shift to sample data
 */
#ifndef QSPI_INIT_SAMPLE_SHIFTING
#define QSPI_INIT_SAMPLE_SHIFTING (QUADSPI_CR_SSHIFT)
#endif

/*
 *    Specifies the Flash Size. FlashSize+1 is effectively the number of address
 * bits required to address the flash memory. The flash capacity can be up to
 * 4GB (addressed using 32 bits) in indirect mode, but the addressable space in
 *  memory-mapped mode is limited to 256MB.
 */
#ifndef QSPI_INIT_FLASH_SIZE
#define QSPI_INIT_FLASH_SIZE (22)
#endif

/**
 *    Specifies the Chip Select High Time. ChipSelectHighTime+1 defines the
 * minimum number of clock cycles which the chip select must remain high between
 * commands. This parameter can be a value of:
 *  @1 0x00000000U                               nCS stay high for at least 1
 * clock cycle between commands
 *  @2 QUADSPI_DCR_CSHT_0                        nCS stay high for at least 2
 * clock cycle between commands
 *  @3 QUADSPI_DCR_CSHT_1                        nCS stay high for at least 3
 * clock cycle between commands
 *  @4 QUADSPI_DCR_CSHT_0 | QUADSPI_DCR_CSHT_1   nCS stay high for at least 4
 * clock cycle between commands
 *  @5 QUADSPI_DCR_CSHT_2                        nCS stay high for at least 5
 * clock cycle between commands
 *  @6 QUADSPI_DCR_CSHT_2 | QUADSPI_DCR_CSHT_0   nCS stay high for at least 6
 * clock cycle between commands
 *  @7 QUADSPI_DCR_CSHT_2 | QUADSPI_DCR_CSHT_1   nCS stay high for at least 7
 * clock cycle between commands
 *  @8 QUADSPI_DCR_CSHT                          nCS stay high for at least 8
 * clock cycle between commands
 */
#ifndef QSPI_INIT_CHIP_SELECT_HIGN_TIME
#define QSPI_INIT_CHIP_SELECT_HIGN_TIME (QUADSPI_DCR_CSHT_0 | QUADSPI_DCR_CSHT_1)
#endif

/*
 *    Specifies the Clock Mode. It indicates the level that clock takes between
 * commands. This parameter can be a value of:
 *  @1 0x00000000U             Clk stays low while nCS is released
 *  @2 QUADSPI_DCR_CKMODE      Clk goes high while nCS is released
 */
#ifndef QSPI_INIT_CLOCK_MODE
#define QSPI_INIT_CLOCK_MODE (0x00000000U)
#endif

#endif /*QSPI_DEFINES_H*/
