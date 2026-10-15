/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    octospi_ll.h
  * @brief   OctoSPI bare-metal (register level) driver untuk bootloader.
  *          Mode: Memory-Mapped (XIP) — CPU langsung eksekusi dari 0x90000000
  *          Flash: Macronix MX25L/MX66L Quad-SPI (4-wire)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __OCTOSPI_LL_H
#define __OCTOSPI_LL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx.h"

/* -----------------------------------------------------------------------
 * Alamat memory-mapped OctoSPI1 di STM32H5xx
 * CPU bisa baca langsung dari sini setelah mode Memory-Mapped aktif
 * ----------------------------------------------------------------------- */
#define OCTOSPI1_MM_BASE        0x90000000UL

/* -----------------------------------------------------------------------
 * Command Macronix MX25L/MX66L (Quad SPI, 1-1-4 Fast Read)
 * ----------------------------------------------------------------------- */
#define FLASH_CMD_READ_QUAD     0xEB  /* Fast Read Quad I/O              */
#define FLASH_CMD_RDSR          0x05  /* Read Status Register            */
#define FLASH_CMD_WREN          0x06  /* Write Enable                    */
#define FLASH_CMD_EQIO          0x35  /* Enter Quad Mode (Macronix)      */

/* Jumlah dummy cycles untuk Fast Read Quad I/O @ frekuensi tinggi       */
#define FLASH_DUMMY_CYCLES      6

/* Inisialisasi OctoSPI1 phase 1: GPIO + Core register (Indirect mode)      */
/* Dipanggil di awal bootloader, sebelum baca META / CRC dari flash          */
void OCTOSPI1_LL_IndirectInit(void);

/* Inisialisasi OctoSPI1 phase 2: switch ke Memory-Mapped (XIP) mode        */
/* Dipanggil tepat sebelum Jump_To_Address(), TIDAK boleh ada akses indirect */
/* lagi setelah fungsi ini dipanggil!                                        */
void OCTOSPI1_LL_MemoryMapped(void);

/* Fungsi bantu: kirim 1 command SPI sederhana (indirect write, 1-wire)     */
/* Dipakai untuk WREN, EQIO, dsb.                                           */
void OCTOSPI1_LL_SendCmd(uint8_t cmd);

/* Aktifkan Quad mode pada flash Macronix via EQIO command (0x35)           */
void OCTOSPI1_LL_EnableQuadMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __OCTOSPI_LL_H */
