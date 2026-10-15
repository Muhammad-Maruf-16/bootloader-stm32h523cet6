/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Bootloader STM32H523CET6 — XIP via OctoSPI Memory-Mapped
  *
  * ALUR:
  *  1. Init clock (LL) + GPIO (LL) + CRC (LL)
  *  2. Init OctoSPI → Indirect Read mode
  *  3. Baca META dari flash eksternal (flag, size, CRC)
  *  4. Jika flag staging valid → verifikasi CRC32 firmware
  *     - CRC OK  : clear flag, switch MM mode, jump ke APP_ADDRESS
  *     - CRC FAIL: coba area backup, jump ke backup
  *  5. Jika tidak ada staging → cek MSP valid, jump ke APP_ADDRESS
  *
  * CATATAN:
  *  - Tidak ada copy ke SRAM — CPU langsung XIP dari 0x90000000
  *  - OctoSPI TIDAK boleh di-deinit setelah jump (flash tetap harus aktif)
  *  - Linker script aplikasi harus set FLASH origin = 0x90000000
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
 * UKURAN SEKTOR & ALOKASI PARTISI (INCREMENTAL)
 * -------------------------------------------------------------------------- */
#define EXTFLASH_SECTOR_SIZE    (4UL * 1024UL)          /* 4 KB (1 sector SPI flash)    */
#define FLASH_META_SIZE         EXTFLASH_SECTOR_SIZE    /* 4 KB sektor metadata         */
#define FLASH_BACKUP_MAX_SIZE   (60UL * 1024UL)         /* 60 KB batas binary backup    */
#define FLASH_APP_MAX_SIZE      (4UL * 1024UL * 1024UL) /* 4 MB batas binary aplikasi   */

/* --------------------------------------------------------------------------
 * PENGALAMATAN INCREMENTAL DI FLASH EKSTERNAL (PHYSICAL OFFSET)
 * -------------------------------------------------------------------------- */
/* 1. SLOT BACKUP (Golden Image) */
#define FLASH_BACKUP_META       (0x00000000UL)
#define FLASH_BACKUP_BIN        (FLASH_BACKUP_META + FLASH_META_SIZE) /* +4 KB = 0x1000 */

/* 2. SLOT APLIKASI UTAMA / STAGING OTA */
#define FLASH_APP_BASE          (FLASH_BACKUP_BIN + FLASH_BACKUP_MAX_SIZE) /* +60 KB = 64 KB (0x10000) */
#define FLASH_APP_META          (FLASH_APP_BASE)                           /* 0x10000                   */
#define FLASH_APP_BIN           (FLASH_APP_META + FLASH_META_SIZE)         /* +4 KB = 68 KB (0x11000)   */

#define STAGING_FLAG_VALID      0xA5U

/* --------------------------------------------------------------------------
 * ALAMAT MEMORY-MAPPED (CPU SPACE @ 0x90000000)
 * -------------------------------------------------------------------------- */
#define APP_BACKUP_MM_ADDRESS   (OCTOSPI1_MM_BASE + FLASH_BACKUP_BIN) /* 0x90001000 */
#define APP_MM_ADDRESS          (OCTOSPI1_MM_BASE + FLASH_APP_BIN)    /* 0x90011000 */
#define APP_ADDRESS             APP_MM_ADDRESS                        /* Alias      */

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
static void Jump_To_Address(uint32_t mm_address);
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
 * HELPER: Tulis 1 byte ke flash via OctoSPI Indirect Write
 * Khusus untuk clear flag staging (bit-flip 0xA5 → 0x00)
 * Membutuhkan WREN terlebih dahulu
 * ========================================================================== */
static void Flash_WriteByte_Indirect(uint32_t flash_offset, uint8_t byte)
{
    /* WREN (Write Enable)                                                   */
    OCTOSPI1_LL_SendCmd(FLASH_CMD_WREN);

    /* Tunggu tidak busy                                                     */
    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);

    OCTOSPI1->DLR = 0U; /* 1 byte data */

    /* CCR: Indirect Write, 1-1-1                                            */
    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)
                  | (1U << OCTOSPI_CCR_ADMODE_Pos)
                  | (2U << OCTOSPI_CCR_ADSIZE_Pos)
                  | (1U << OCTOSPI_CCR_DMODE_Pos);

    /* FMODE = 00 → Indirect Write                                           */
    OCTOSPI1->CR = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);

    OCTOSPI1->IR = 0x02U; /* Page Program                                   */
    OCTOSPI1->AR = flash_offset;

    /* Tulis byte                                                            */
    *(__IO uint8_t *)(&OCTOSPI1->DR) = byte;

    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;

    /* Tunggu flash selesai write (polling status register)                  */
    uint8_t status;
    do {
        Flash_ReadBytes_Indirect(0x00000000UL, &status, 0); /* dummy reset  */

        /* Baca Status Register flash (cmd 0x05)                            */
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
    } while (status & 0x01U); /* WIP bit = 1 → masih busy                   */
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
 * HELPER: Jump ke aplikasi di alamat memory-mapped tertentu
 * Dipanggil SETELAH OctoSPI sudah dalam Memory-Mapped mode
 * ========================================================================== */
static void Jump_To_Address(uint32_t mm_address)
{
    uint32_t app_msp   = *(__IO uint32_t *)(mm_address);
    uint32_t app_reset = *(__IO uint32_t *)(mm_address + 4U);

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
    SCB->VTOR = mm_address;

    /* Reset seluruh SRAM STM32H523 (272 KB: 0x20000000 .. 0x20044000)      */
    uint32_t *ram     = (uint32_t *)0x20000000UL;
    uint32_t *ram_end = (uint32_t *)(0x20000000UL + (272UL * 1024UL));
    while (ram < ram_end) *ram++ = 0U;

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
 * MAIN LOGIC: Cek staging, verifikasi CRC, lalu jump
 *
 * Alur:
 *  1. Init OctoSPI Indirect mode
 *  2. Baca STAGING_META → cek flag 0xA5
 *  3. Jika ada staging:
 *       - Hitung CRC32 dari STAGING_BIN
 *       - Cocok → clear flag → aktifkan MM mode → jump ke APP
 *       - Gagal → coba BACKUP → aktifkan MM mode → jump ke backup
 *  4. Tidak ada staging:
 *       - Cek MSP di awal flash (indirect read)
 *       - Valid → aktifkan MM mode → jump ke APP
 *       - Tidak valid → coba backup
 * ========================================================================== */
static void Boot_CheckAndJump(void)
{
    uint8_t meta[9]; /* [0]=flag, [1..4]=size, [5..8]=crc32                 */
    uint8_t msp_bytes[4];

    /* ======================================================================
     * 1. CEK FIRMWARE APLIKASI UTAMA (FLASH_APP_META & FLASH_APP_BIN)
     * ====================================================================== */
    Flash_ReadBytes_Indirect(FLASH_APP_META, meta, sizeof(meta));

    uint8_t  flag_app = meta[0];
    uint32_t size_app = (uint32_t)meta[1]
                      | ((uint32_t)meta[2] << 8U)
                      | ((uint32_t)meta[3] << 16U)
                      | ((uint32_t)meta[4] << 24U);
    uint32_t crc_app  = (uint32_t)meta[5]
                      | ((uint32_t)meta[6] << 8U)
                      | ((uint32_t)meta[7] << 16U)
                      | ((uint32_t)meta[8] << 24U);

    if (size_app > 0U && size_app <= FLASH_APP_MAX_SIZE)
    {
        /* Validasi Initial Stack Pointer (MSP) di awal binary aplikasi */
        Flash_ReadBytes_Indirect(FLASH_APP_BIN, msp_bytes, 4U);
        uint32_t msp_app = (uint32_t)msp_bytes[0]
                         | ((uint32_t)msp_bytes[1] << 8U)
                         | ((uint32_t)msp_bytes[2] << 16U)
                         | ((uint32_t)msp_bytes[3] << 24U);

        if ((msp_app & 0xFF000000U) == 0x20000000U)
        {
            /* Hitung CRC32 dari binary aplikasi via hardware CRC */
            uint32_t crc_calc = CRC32_ComputeFromFlash(FLASH_APP_BIN, size_app);

            if (crc_calc == crc_app)
            {
                /* Jika baru saja di-download OTA (flag 0xA5), clear flag */
                if (flag_app == STAGING_FLAG_VALID)
                {
                    Flash_WriteByte_Indirect(FLASH_APP_META, 0x00U);
                }

                /* CRC cocok → aktifkan mode Memory-Mapped → Jump ke APP */
                OCTOSPI1_LL_MemoryMapped();
                Jump_To_Address(APP_MM_ADDRESS);
            }
        }
    }

    /* ======================================================================
     * 2. JIKA APP GAGAL / KORUP: CEK FIRMWARE BACKUP
     * ====================================================================== */
    Flash_ReadBytes_Indirect(FLASH_BACKUP_META, meta, sizeof(meta));

    uint32_t size_backup = (uint32_t)meta[1]
                         | ((uint32_t)meta[2] << 8U)
                         | ((uint32_t)meta[3] << 16U)
                         | ((uint32_t)meta[4] << 24U);
    uint32_t crc_backup  = (uint32_t)meta[5]
                         | ((uint32_t)meta[6] << 8U)
                         | ((uint32_t)meta[7] << 16U)
                         | ((uint32_t)meta[8] << 24U);

    if (size_backup > 0U && size_backup <= FLASH_BACKUP_MAX_SIZE)
    {
        Flash_ReadBytes_Indirect(FLASH_BACKUP_BIN, msp_bytes, 4U);
        uint32_t msp_backup = (uint32_t)msp_bytes[0]
                            | ((uint32_t)msp_bytes[1] << 8U)
                            | ((uint32_t)msp_bytes[2] << 16U)
                            | ((uint32_t)msp_bytes[3] << 24U);

        if ((msp_backup & 0xFF000000U) == 0x20000000U)
        {
            uint32_t crc_calc = CRC32_ComputeFromFlash(FLASH_BACKUP_BIN, size_backup);

            if (crc_calc == crc_backup)
            {
                /* CRC Backup cocok → aktifkan MM mode → Jump ke Backup */
                OCTOSPI1_LL_MemoryMapped();
                Jump_To_Address(APP_BACKUP_MM_ADDRESS);
            }
        }
    }

    /* ======================================================================
     * 3. FLASH KOSONG ATAU DUA-DUANYA KORUP: Tahan CPU di sini
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
  /* HAL_Init() sengaja tidak dipanggil — bootloader tidak butuh SysTick.   */
  /* HAL_Init(); */

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

  /* HAL_InitTick sengaja tidak dipanggil — bootloader tidak butuh SysTick. */
  /* if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK) { Error_Handler(); }   */
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
