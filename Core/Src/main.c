/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Bootloader STM32H523CET6
  *                   - Backup Image : Flash Internal @ 0x08002000 (Sektor 1..63)
  *                   - App Image    : Flash Eksternal OctoSPI XIP @ 0x90001000
  *
  * ALUR:
  *  1. Init clock (LL) + GPIO (LL) + CRC (LL)
  *  2. Init OctoSPI → Indirect Read mode
  *  3. Baca metadata App dari flash eksternal (size, CRC, flag)
  *  4. Verifikasi firmware App di flash eksternal:
  *     - CRC OK  : jika flag OTA 0xA5, clear flag, switch MM mode, jump ke APP (0x90001000)
  *     - CRC FAIL / App Kosong: jump ke Backup di flash internal (0x08002000)
  *
  * CATATAN:
  *  - Flash internal STM32H5 sektor = 8 KB. Sektor 0 (0x08000000) = Bootloader.
  *  - OctoSPI TIDAK boleh di-deinit saat jump ke App (flash tetap harus aktif untuk XIP).
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "octospi_ll.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Tipe function pointer ke Reset_Handler aplikasi                           */
typedef void (*pFunction)(void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* --------------------------------------------------------------------------
 * ALOKASI PARTISI FLASH INTERNAL (STM32H523 - 512 KB Total, Sektor = 8 KB)
 * -------------------------------------------------------------------------- */
/* Sektor 0   : Bootloader  (0x08000000 - 0x08001FFF, 8 KB)                  */
/* Sektor 1.. : Backup Image (0x08002000 - 0x0807FFFF, 504 KB)               */
#define BACKUP_APP_ADDRESS      (0x08002000UL)

/* --------------------------------------------------------------------------
 * ALOKASI PARTISI FLASH EKSTERNAL (OctoSPI Physical Offset)
 * -------------------------------------------------------------------------- */
#define EXTFLASH_SECTOR_SIZE    (4UL * 1024UL)          /* 4 KB (1 sector SPI flash)    */
#define FLASH_APP_META          (0x00000000UL)          /* Sektor 0 (0 KB - 4 KB): Meta */
#define FLASH_APP_BIN           (FLASH_APP_META + EXTFLASH_SECTOR_SIZE) /* 0x00001000 (4 KB)   */
#define FLASH_STAGING_META      (0x00100000UL)          /* Sektor 256 (1 MB): Staging Meta */
#define FLASH_STAGING_BIN       (FLASH_STAGING_META + EXTFLASH_SECTOR_SIZE) /* 0x00101000 (1 MB + 4 KB) */
#define FLASH_APP_MAX_SIZE      (1020UL * 1024UL)       /* ~1020 KB batas binary aplikasi   */

#define STAGING_FLAG_VALID      0xA5U

/* --------------------------------------------------------------------------
 * ALAMAT MEMORY-MAPPED (CPU SPACE @ 0x90000000)
 * -------------------------------------------------------------------------- */
#define APP_MM_ADDRESS          (OCTOSPI1_MM_BASE + FLASH_APP_BIN) /* 0x90001000 */
#define APP_ADDRESS             APP_MM_ADDRESS                     /* Alias      */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Boot_CheckAndJump(void);
static void Jump_To_Address(uint32_t address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==========================================================================
 * HELPER: Baca bytes dari flash eksternal via OctoSPI Indirect Read
 * Dipanggil SEBELUM memory-mapped mode aktif
 * ========================================================================== */
static void Flash_ReadBytes_Indirect(uint32_t flash_offset, uint8_t *buf, uint32_t len)
{
    /* Tunggu OctoSPI tidak busy                                             */
    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);

    /* Set panjang data yang akan dibaca                                     */
    OCTOSPI1->DLR = len - 1U;

    /* CCR: 1-1-4 Fast Read, Indirect Read mode (FMODE=01)
     * IMODE [4:3]   = 01 → Instruction: 1 wire
     * ADMODE[7:6]   = 01 → Address: 1 wire (SPI standar untuk indirect)
     * ADSIZE[9:8]   = 10 → Address: 24-bit
     * DMODE [27:26] = 01 → Data: 1 wire (indirect dibaca per byte)
     * DCYC  [22:17] = 0  → Tidak ada dummy untuk read biasa (0x03 cmd)     */
    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)
                  | (1U << OCTOSPI_CCR_ADMODE_Pos)
                  | (2U << OCTOSPI_CCR_ADSIZE_Pos)
                  | (1U << OCTOSPI_CCR_DMODE_Pos);

    /* FMODE = 01 → Indirect Read                                            */
    OCTOSPI1->CR = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk)
                 | (1U << OCTOSPI_CR_FMODE_Pos);

    /* IR: perintah baca normal (0x03)                                       */
    OCTOSPI1->IR = 0x03U;

    /* AR: alamat di flash yang ingin dibaca                                 */
    OCTOSPI1->AR = flash_offset;

    /* Baca data byte per byte dari FIFO                                     */
    for (uint32_t i = 0; i < len; i++)
    {
        while (!(OCTOSPI1->SR & OCTOSPI_SR_FTF) &&
               !(OCTOSPI1->SR & OCTOSPI_SR_TCF));
        buf[i] = *(__IO uint8_t *)(&OCTOSPI1->DR);
    }

    /* Tunggu Transfer Complete                                              */
    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;

    /* Kembali ke Indirect Write mode (default)                              */
    OCTOSPI1->CR = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
}

/* ==========================================================================
 * HELPER: Tunggu proses internal Flash (WIP bit = 0)
 * ========================================================================== */
static void Flash_WaitBusy_Indirect(void)
{
    uint8_t status;
    do {
        while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);
        OCTOSPI1->DLR = 0U;
        OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)
                      | (1U << OCTOSPI_CCR_DMODE_Pos);
        OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk)
                      | (1U << OCTOSPI_CR_FMODE_Pos);
        OCTOSPI1->IR  = FLASH_CMD_RDSR;
        while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF) &&
               !(OCTOSPI1->SR & OCTOSPI_SR_FTF));
        status = *(__IO uint8_t *)(&OCTOSPI1->DR);
        while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
        OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
        OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
    } while (status & 0x01U);
}

/* ==========================================================================
 * HELPER: Tulis 1 byte ke flash via OctoSPI Indirect Write
 * ========================================================================== */
static void Flash_WriteByte_Indirect(uint32_t flash_offset, uint8_t byte)
{
    Flash_WaitBusy_Indirect();
    OCTOSPI1_LL_SendCmd(FLASH_CMD_WREN);

    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);
    OCTOSPI1->DLR = 0U;
    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)
                  | (1U << OCTOSPI_CCR_ADMODE_Pos)
                  | (2U << OCTOSPI_CCR_ADSIZE_Pos)
                  | (1U << OCTOSPI_CCR_DMODE_Pos);
    OCTOSPI1->CR = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
    OCTOSPI1->IR = 0x02U; /* Page Program */
    OCTOSPI1->AR = flash_offset;

    *(__IO uint8_t *)(&OCTOSPI1->DR) = byte;

    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;

    Flash_WaitBusy_Indirect();
}

/* ==========================================================================
 * HELPER: Erase 1 sektor (4 KB) di flash eksternal
 * ========================================================================== */
static void Flash_EraseSector_Indirect(uint32_t flash_offset)
{
    Flash_WaitBusy_Indirect();
    OCTOSPI1_LL_SendCmd(FLASH_CMD_WREN);

    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);
    OCTOSPI1->DLR = 0U;
    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)
                  | (1U << OCTOSPI_CCR_ADMODE_Pos)
                  | (2U << OCTOSPI_CCR_ADSIZE_Pos);
    OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
    OCTOSPI1->IR  = 0x20U; /* Sector Erase 4KB */
    OCTOSPI1->AR  = flash_offset;

    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;

    Flash_WaitBusy_Indirect();
}

/* ==========================================================================
 * HELPER: Tulis 1 page (maks 256 byte) ke flash eksternal
 * ========================================================================== */
static void Flash_WritePage_Indirect(uint32_t flash_offset, const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > 256) return;
    Flash_WaitBusy_Indirect();
    OCTOSPI1_LL_SendCmd(FLASH_CMD_WREN);

    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);
    OCTOSPI1->DLR = (uint32_t)(len - 1U);
    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)
                  | (1U << OCTOSPI_CCR_ADMODE_Pos)
                  | (2U << OCTOSPI_CCR_ADSIZE_Pos)
                  | (1U << OCTOSPI_CCR_DMODE_Pos);
    OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
    OCTOSPI1->IR  = 0x02U; /* Page Program */
    OCTOSPI1->AR  = flash_offset;

    for (uint16_t i = 0; i < len; i++)
    {
        while (!(OCTOSPI1->SR & OCTOSPI_SR_FTF) &&
               !(OCTOSPI1->SR & OCTOSPI_SR_TCF));
        *(__IO uint8_t *)(&OCTOSPI1->DR) = data[i];
    }

    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;

    Flash_WaitBusy_Indirect();
}

/* ==========================================================================
 * HELPER: Salin firmware dari STAGING (0x00101000) ke APP (0x00001000)
 * ========================================================================== */
static void Boot_CopyStagingToApp(uint32_t size, uint32_t crc)
{
    /* 1. Hapus sektor-sektor aplikasi aktif (FLASH_APP_BIN) */
    uint32_t sectors = (size + EXTFLASH_SECTOR_SIZE - 1U) / EXTFLASH_SECTOR_SIZE;
    for (uint32_t s = 0; s < sectors; s++)
    {
        Flash_EraseSector_Indirect(FLASH_APP_BIN + (s * EXTFLASH_SECTOR_SIZE));
    }

    /* 2. Salin binary dari FLASH_STAGING_BIN ke FLASH_APP_BIN per page (256 byte) */
    uint8_t page_buf[256];
    uint32_t copied = 0;
    while (copied < size)
    {
        uint16_t chunk = (size - copied > sizeof(page_buf)) ? (uint16_t)sizeof(page_buf) : (uint16_t)(size - copied);
        Flash_ReadBytes_Indirect(FLASH_STAGING_BIN + copied, page_buf, chunk);
        Flash_WritePage_Indirect(FLASH_APP_BIN + copied, page_buf, chunk);
        copied += chunk;
    }

    /* 3. Tulis metadata aplikasi di FLASH_APP_META */
    Flash_EraseSector_Indirect(FLASH_APP_META);
    uint8_t app_meta[10] = { 0 };
    app_meta[0] = (uint8_t)(size);
    app_meta[1] = (uint8_t)(size >> 8);
    app_meta[2] = (uint8_t)(size >> 16);
    app_meta[3] = (uint8_t)(size >> 24);
    app_meta[4] = (uint8_t)(crc);
    app_meta[5] = (uint8_t)(crc >> 8);
    app_meta[6] = (uint8_t)(crc >> 16);
    app_meta[7] = (uint8_t)(crc >> 24);
    app_meta[8] = 0x00; /* Flag App Normal */
    Flash_WritePage_Indirect(FLASH_APP_META, app_meta, 10);

    /* 4. Hapus flag valid staging (tulis 0x00 ke byte ke-8 di FLASH_STAGING_META) */
    Flash_WriteByte_Indirect(FLASH_STAGING_META + 8U, 0x00U);
}

/* ==========================================================================
 * HELPER: Hitung CRC32 dari flash eksternal (indirect read, per 32-bit word)
 * Menggunakan hardware CRC unit STM32H5 (sudah di-init oleh MX_CRC_Init)
 * ========================================================================== */
static uint32_t CRC32_ComputeFromFlash(uint32_t flash_offset, uint32_t size_bytes)
{
    uint8_t  buf[64];
    uint32_t remaining = size_bytes;
    uint32_t offset    = flash_offset;

    /* Reset CRC unit                                                        */
    LL_CRC_ResetCRCCalculationUnit(CRC);

    while (remaining > 0U)
    {
        uint32_t chunk = (remaining > sizeof(buf)) ? sizeof(buf) : remaining;

        /* Bulatkan ke kelipatan 4 (karena feed per word 32-bit)            */
        uint32_t chunk_aligned = (chunk + 3U) & ~3U;

        /* Bersihkan buffer dulu agar padding 0xFF tidak merusak CRC        */
        for (uint32_t z = 0; z < chunk_aligned; z++) buf[z] = 0xFFU;

        Flash_ReadBytes_Indirect(offset, buf, chunk);

        /* Feed ke hardware CRC per 32-bit word                             */
        for (uint32_t i = 0; i < chunk_aligned; i += 4U)
        {
            uint32_t word = (uint32_t)buf[i]
                          | ((uint32_t)buf[i + 1U] << 8U)
                          | ((uint32_t)buf[i + 2U] << 16U)
                          | ((uint32_t)buf[i + 3U] << 24U);
            LL_CRC_FeedData32(CRC, word);
        }

        offset    += chunk;
        remaining -= chunk;
    }

    return LL_CRC_ReadData32(CRC);
}

/* ==========================================================================
 * HELPER: Jump ke aplikasi di alamat tertentu (Internal Flash atau XIP MM)
 * ========================================================================== */
static void Jump_To_Address(uint32_t address)
{
    uint32_t app_msp   = *(__IO uint32_t *)(address);
    uint32_t app_reset = *(__IO uint32_t *)(address + 4U);

    /* Validasi MSP: harus di range SRAM STM32H523 (0x20000000..0x20044000) */
    if ((app_msp & 0xFF000000U) != 0x20000000U)
    {
        while (1); /* Flash tidak valid, hang di sini                       */
    }

    __disable_irq();

    /* Matikan SysTick sepenuhnya                                            */
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;

    /* Clear semua IRQ pending di NVIC                                       */
    for (uint8_t i = 0; i < 8U; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }

    /* Pindahkan Vector Table ke lokasi aplikasi                             */
    SCB->VTOR = address;

    /* Reset MSPLIM (Stack Pointer Limit) untuk Cortex-M33                   */
    __set_MSPLIM(0);

    __DSB();
    __ISB();

    /* Eksekusi Jump via Assembly Murni (Bebas dari Stack Frame C)           */
    __asm volatile (
        "msr msp, %0 \n"  /* Isi Main Stack Pointer dengan MSP aplikasi */
        "bx  %1      \n"  /* Branch langsung ke Reset_Handler aplikasi  */
        :
        : "r" (app_msp), "r" (app_reset)
        : "memory"
    );

    while (1); /* Tidak seharusnya sampai sini                              */
}

/* ==========================================================================
 * MAIN LOGIC: Cek firmware App di Flash Eksternal, lalu fallback ke Backup
 *
 * Alur:
 *  1. Baca FLASH_APP_META dari OctoSPI flash:
 *     Format metadata: [0..3]=size, [4..7]=crc32, [8]=flag (0xA5 jika baru OTA)
 *  2. Jika size valid & MSP valid:
 *     - Hitung hardware CRC32 dari binary di FLASH_APP_BIN (0x00001000)
 *     - Jika CRC cocok:
 *         * Clear flag 0xA5 (jika ada) di byte ke-8
 *         * Aktifkan mode Memory-Mapped OctoSPI
 *         * Jump ke APP_MM_ADDRESS (0x90001000)
 *  3. Jika App tidak valid / corrupt:
 *     - Cek firmware Backup di Flash Internal (0x08002000)
 *     - Jika MSP valid: Jump ke Backup (0x08002000)
 *  4. Jika keduanya tidak valid: CPU hang (while 1)
 * ========================================================================== */
static void Boot_CheckAndJump(void)
{
    /* ======================================================================
     * 0. MANUAL RECOVERY MODE (TOMBOL BOOT_MODE / PA0)
     *    Jika pin BOOT_MODE ditekan ke GND (LOW saat startup/reset),
     *    langsung paksa meloncat ke program Backup di internal flash.
     * ====================================================================== */
    if (LL_GPIO_IsInputPinSet(BOOT_MODE_GPIO_Port, BOOT_MODE_Pin) == 0)
    {
        uint32_t backup_msp = *(__IO uint32_t *)(BACKUP_APP_ADDRESS);
        if ((backup_msp & 0xFF000000U) == 0x20000000U)
        {
            Jump_To_Address(BACKUP_APP_ADDRESS);
        }
    }

    uint8_t  meta[10];
    uint8_t  app_valid = 0;

    /* ======================================================================
     * 1. CEK FIRMWARE BARU DI STAGING AREA (HASIL OTA DARI APP)
     * ====================================================================== */
    Flash_ReadBytes_Indirect(FLASH_STAGING_META, meta, sizeof(meta));

    uint32_t size_stg = (uint32_t)meta[0]
                      | ((uint32_t)meta[1] << 8U)
                      | ((uint32_t)meta[2] << 16U)
                      | ((uint32_t)meta[3] << 24U);
    uint32_t crc_stg  = (uint32_t)meta[4]
                      | ((uint32_t)meta[5] << 8U)
                      | ((uint32_t)meta[6] << 16U)
                      | ((uint32_t)meta[7] << 24U);
    uint8_t  flag_stg = meta[8];

    if (flag_stg == STAGING_FLAG_VALID && size_stg > 0U && size_stg <= FLASH_APP_MAX_SIZE)
    {
        /* Validasi header binary staging (MSP & Reset Handler) */
        uint8_t hdr_stg[8];
        Flash_ReadBytes_Indirect(FLASH_STAGING_BIN, hdr_stg, 8U);
        uint32_t msp_stg = (uint32_t)hdr_stg[0]
                         | ((uint32_t)hdr_stg[1] << 8U)
                         | ((uint32_t)hdr_stg[2] << 16U)
                         | ((uint32_t)hdr_stg[3] << 24U);
        uint32_t rst_stg = (uint32_t)hdr_stg[4]
                         | ((uint32_t)hdr_stg[5] << 8U)
                         | ((uint32_t)hdr_stg[6] << 16U)
                         | ((uint32_t)hdr_stg[7] << 24U);

        /* MSP harus di SRAM (0x20xxxxxx) & 4-byte aligned; Reset vector harus di OctoSPI (0x90xxxxxx) & Thumb bit (ganjil) */
        if (((msp_stg & 0xFF000000U) == 0x20000000U) && ((msp_stg & 0x03U) == 0U) &&
            ((rst_stg & 0xFF000000U) == 0x90000000U) && ((rst_stg & 0x01U) == 1U))
        {
            uint32_t crc_calc = CRC32_ComputeFromFlash(FLASH_STAGING_BIN, size_stg);
            if (crc_calc == crc_stg)
            {
                /* Valid binary STM32! Salin dari Staging ke Active App */
                Boot_CopyStagingToApp(size_stg, crc_stg);
            }
            else
            {
                /* CRC staging tidak cocok (data rusak) -> Hapus flag staging */
                Flash_WriteByte_Indirect(FLASH_STAGING_META + 8U, 0x00U);
            }
        }
        else
        {
            /* Binary bukan untuk STM32 (misal ESP8266) -> Hapus flag staging, tolak update */
            Flash_WriteByte_Indirect(FLASH_STAGING_META + 8U, 0x00U);
        }
    }

    /* ======================================================================
     * 2. CEK FIRMWARE APLIKASI UTAMA DI FLASH EKSTERNAL (XIP OctoSPI)
     * ====================================================================== */
    Flash_ReadBytes_Indirect(FLASH_APP_META, meta, sizeof(meta));

    uint32_t size_app = (uint32_t)meta[0]
                      | ((uint32_t)meta[1] << 8U)
                      | ((uint32_t)meta[2] << 16U)
                      | ((uint32_t)meta[3] << 24U);
    uint32_t crc_app  = (uint32_t)meta[4]
                      | ((uint32_t)meta[5] << 8U)
                      | ((uint32_t)meta[6] << 16U)
                      | ((uint32_t)meta[7] << 24U);

    if (size_app > 0U && size_app <= FLASH_APP_MAX_SIZE)
    {
        /* Validasi Initial Stack Pointer (MSP) dan Reset Handler di awal binary aplikasi */
        uint8_t hdr_bytes[8];
        Flash_ReadBytes_Indirect(FLASH_APP_BIN, hdr_bytes, 8U);
        uint32_t msp_app = (uint32_t)hdr_bytes[0]
                         | ((uint32_t)hdr_bytes[1] << 8U)
                         | ((uint32_t)hdr_bytes[2] << 16U)
                         | ((uint32_t)hdr_bytes[3] << 24U);
        uint32_t rst_app = (uint32_t)hdr_bytes[4]
                         | ((uint32_t)hdr_bytes[5] << 8U)
                         | ((uint32_t)hdr_bytes[6] << 16U)
                         | ((uint32_t)hdr_bytes[7] << 24U);

        /* MSP harus di SRAM (0x20xxxxxx) & 4-byte aligned; Reset vector harus di OctoSPI (0x90xxxxxx) & Thumb bit (ganjil) */
        if (((msp_app & 0xFF000000U) == 0x20000000U) && ((msp_app & 0x03U) == 0U) &&
            ((rst_app & 0xFF000000U) == 0x90000000U) && ((rst_app & 0x01U) == 1U))
        {
            /* Hitung hardware CRC32 dari binary aplikasi */
            uint32_t crc_calc = CRC32_ComputeFromFlash(FLASH_APP_BIN, size_app);

            if (crc_calc == crc_app)
            {
                app_valid = 1;
            }
        }
    }

    if (app_valid)
    {
        /* CRC cocok → aktifkan mode Memory-Mapped → Jump ke APP */
        OCTOSPI1_LL_MemoryMapped();
        Jump_To_Address(APP_MM_ADDRESS);
    }

    /* ======================================================================
     * 3. JIKA APP GAGAL / KORUP / KOSONG: JUMP KE FIRMWARE BACKUP
     *    (Backup berada di Flash Internal @ 0x08002000, Sektor 1)
     * ====================================================================== */
    uint32_t backup_msp = *(__IO uint32_t *)(BACKUP_APP_ADDRESS);

    if ((backup_msp & 0xFF000000U) == 0x20000000U)
    {
        Jump_To_Address(BACKUP_APP_ADDRESS);
    }

    /* ======================================================================
     * 4. FLASH KOSONG ATAU DUA-DUANYA KORUP: Tahan CPU di sini
     * ====================================================================== */
    while (1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  /* --- Init OctoSPI1 core + GPIO (Indirect mode dulu, belum MM) --------- */
  OCTOSPI1_LL_IndirectInit();

  /* --- Cek staging, verifikasi CRC, lalu jump ke aplikasi (XIP) --------- */
  Boot_CheckAndJump();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }

  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL1_SetSource(LL_RCC_PLL1SOURCE_HSE);
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(2);
  LL_RCC_PLL1_SetN(40);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(2);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
  {
  }

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_1);
  LL_SetSystemCoreClock(250000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
