/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    octospi_ll.c
  * @brief   OctoSPI bare-metal driver untuk bootloader STM32H523CET6.
  *
  *          TUJUAN: Init OctoSPI1 ke Memory-Mapped mode supaya CPU bisa
  *          langsung eksekusi kode dari flash eksternal di 0x90000000
  *          (XIP — eXecute In Place), tanpa copy ke SRAM.
  *
  *          HARDWARE:
  *          - PA1  → OCTOSPI1_IO3  (AF9)
  *          - PA3  → OCTOSPI1_CLK  (AF3)
  *          - PA7  → OCTOSPI1_IO2  (AF10)
  *          - PB0  → OCTOSPI1_IO1  (AF6)
  *          - PB1  → OCTOSPI1_IO0  (AF6)
  *          - PB10 → OCTOSPI1_NCS  (AF9)
  *
  *          FLASH: Macronix MX25L/MX66L (QSPI, 4-wire Quad mode)
  *
  *          URUTAN:
  *          1. Enable clock GPIO & OctoSPI
  *          2. Konfigurasi pin GPIO sebagai AF
  *          3. Init OctoSPI1 register (prescaler, size, dll)
  *          4. Kirim EQIO command → aktifkan Quad mode di flash
  *          5. Set Memory-Mapped mode dengan Fast Read Quad I/O (0xEB)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "octospi_ll.h"

/* =========================================================================
 * Helper: tunggu bit BUSY di register SR clear
 * ========================================================================= */
static inline void wait_not_busy(void)
{
    /* Bit 5 di OCTOSPI_SR = BUSY */
    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);
}

/* =========================================================================
 * Helper: tunggu Transfer Complete Flag (TCF)
 * ========================================================================= */
static inline void wait_tcf(void)
{
    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF; /* clear TCF */
}

/* =========================================================================
 * 1. Inisialisasi GPIO untuk OctoSPI1
 *    Semua pin di-set sebagai Alternate Function, Push-Pull, Very High Speed
 * ========================================================================= */
static void OCTOSPI1_GPIO_Init(void)
{
    /* ---- Enable clock GPIOA dan GPIOB ----------------------------------- */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    __DSB(); /* data sync barrier setelah enable clock */

    /* ====================================================================
     * PA1  → OCTOSPI1_IO3  (AF9)
     * ===================================================================== */
    /* Mode: Alternate Function (10b) */
    GPIOA->MODER  = (GPIOA->MODER  & ~GPIO_MODER_MODE1_Msk)  | (2U << GPIO_MODER_MODE1_Pos);
    /* Output type: Push-Pull (0) */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;
    /* Speed: Very High (11b) */
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED1_Msk) | (3U << GPIO_OSPEEDR_OSPEED1_Pos);
    /* Pull: No pull (00b) */
    GPIOA->PUPDR  = (GPIOA->PUPDR  & ~GPIO_PUPDR_PUPD1_Msk);
    /* AF9 → AFR[0] bits [7:4] */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << 4)) | (9U << 4);

    /* ====================================================================
     * PA3  → OCTOSPI1_CLK  (AF3)
     * ===================================================================== */
    GPIOA->MODER  = (GPIOA->MODER  & ~GPIO_MODER_MODE3_Msk)  | (2U << GPIO_MODER_MODE3_Pos);
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED3_Msk) | (3U << GPIO_OSPEEDR_OSPEED3_Pos);
    GPIOA->PUPDR  = (GPIOA->PUPDR  & ~GPIO_PUPDR_PUPD3_Msk);
    /* AF3 → AFR[0] bits [15:12] */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << 12)) | (3U << 12);

    /* ====================================================================
     * PA7  → OCTOSPI1_IO2  (AF10)
     * ===================================================================== */
    GPIOA->MODER  = (GPIOA->MODER  & ~GPIO_MODER_MODE7_Msk)  | (2U << GPIO_MODER_MODE7_Pos);
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT7;
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED7_Msk) | (3U << GPIO_OSPEEDR_OSPEED7_Pos);
    GPIOA->PUPDR  = (GPIOA->PUPDR  & ~GPIO_PUPDR_PUPD7_Msk);
    /* AF10 → AFR[0] bits [31:28] */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << 28)) | (10U << 28);

    /* ====================================================================
     * PB0  → OCTOSPI1_IO1  (AF6)
     * ===================================================================== */
    GPIOB->MODER  = (GPIOB->MODER  & ~GPIO_MODER_MODE0_Msk)  | (2U << GPIO_MODER_MODE0_Pos);
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT0;
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~GPIO_OSPEEDR_OSPEED0_Msk) | (3U << GPIO_OSPEEDR_OSPEED0_Pos);
    GPIOB->PUPDR  = (GPIOB->PUPDR  & ~GPIO_PUPDR_PUPD0_Msk);
    /* AF6 → AFR[0] bits [3:0] */
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFU << 0)) | (6U << 0);

    /* ====================================================================
     * PB1  → OCTOSPI1_IO0  (AF6)
     * ===================================================================== */
    GPIOB->MODER  = (GPIOB->MODER  & ~GPIO_MODER_MODE1_Msk)  | (2U << GPIO_MODER_MODE1_Pos);
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT1;
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~GPIO_OSPEEDR_OSPEED1_Msk) | (3U << GPIO_OSPEEDR_OSPEED1_Pos);
    GPIOB->PUPDR  = (GPIOB->PUPDR  & ~GPIO_PUPDR_PUPD1_Msk);
    /* AF6 → AFR[0] bits [7:4] */
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFU << 4)) | (6U << 4);

    /* ====================================================================
     * PB10 → OCTOSPI1_NCS  (AF9)
     * ===================================================================== */
    GPIOB->MODER  = (GPIOB->MODER  & ~GPIO_MODER_MODE10_Msk) | (2U << GPIO_MODER_MODE10_Pos);
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT10;
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~GPIO_OSPEEDR_OSPEED10_Msk) | (3U << GPIO_OSPEEDR_OSPEED10_Pos);
    GPIOB->PUPDR  = (GPIOB->PUPDR  & ~GPIO_PUPDR_PUPD10_Msk);
    /* AF9 → AFR[1] bits [11:8] — PB10 ada di AFR[1] (pin 8..15) */
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFU << 8)) | (9U << 8);
}

/* =========================================================================
 * 2. Inisialisasi register OctoSPI1 (prescaler, ukuran flash, dll)
 *    Mode awal: Indirect Write (untuk kirim command ke flash)
 * ========================================================================= */
static void OCTOSPI1_Core_Init(void)
{
    /* ---- Enable clock OctoSPI1 ----------------------------------------- */
    /* STM32H523: OCTOSPI1 is on AHB4 bus                                   */
    RCC->AHB4ENR |= RCC_AHB4ENR_OCTOSPI1EN;
    __DSB();

    /* ---- Pastikan OctoSPI tidak dalam mode aktif ----------------------- */
    OCTOSPI1->CR &= ~OCTOSPI_CR_EN;

    /* ---- DCR1: Device Configuration Register 1 -------------------------
     * MTYP   [27:24] = 0000 → Micron/Macronix (Standard)
     * DEVSIZE[20:16] = 24   → 2^(24+1) = 32 MB (MX25L256)
     *                         Sesuaikan dengan ukuran flash Anda!
     *                         16MB = 23, 32MB = 24, 64MB = 25
     * CSHT   [23:20] = 1    → CS High time = 2 cycles (tSHSL)
     * CKMODE [0]     = 0    → Clock Mode 0 (CPOL=0, CPHA=0)
     * -------------------------------------------------------------------- */
    OCTOSPI1->DCR1 = (0U  << OCTOSPI_DCR1_MTYP_Pos)    /* Standard         */
                   | (24U << OCTOSPI_DCR1_DEVSIZE_Pos)  /* 32 MB            */
                   | (1U  << OCTOSPI_DCR1_CSHT_Pos)     /* CS High 2 cycles */
                   | (0U  << OCTOSPI_DCR1_CKMODE_Pos);  /* Mode 0           */

    /* ---- DCR2: Prescaler -----------------------------------------------
     * PRESCALER[7:0] = 1 → fOCTOSPI = fHCLK / (1+1) = 250/2 = 125 MHz
     * -------------------------------------------------------------------- */
    OCTOSPI1->DCR2 = (1U << OCTOSPI_DCR2_PRESCALER_Pos);

    /* ---- DCR3: CS boundary — tidak dipakai (disable) ------------------- */
    OCTOSPI1->DCR3 = 0U;

    /* ---- DCR4: Refresh — tidak dipakai (0 = disable) ------------------- */
    OCTOSPI1->DCR4 = 0U;

    /* ---- CR: Control Register ------------------------------------------
     * MSEL   [7]    = 0 → Memory 1 selected (NCS1)
     * DMM    [6]    = 0 → Dual Memory Mode disabled (single flash)
     * FMODE [29:28] = 00 → Indirect Write Mode (default)
     * FTHRES[12:8]  = 0 → FIFO threshold = 1 byte
     * -------------------------------------------------------------------- */
    OCTOSPI1->CR = (0U << OCTOSPI_CR_FMODE_Pos)
                 | (0U << OCTOSPI_CR_FTHRES_Pos)
                 | (0U << OCTOSPI_CR_DMM_Pos)
                 | (0U << OCTOSPI_CR_MSEL_Pos);

    /* ---- Enable OctoSPI ------------------------------------------------ */
    OCTOSPI1->CR |= OCTOSPI_CR_EN;
}

/* =========================================================================
 * 3. Kirim 1-byte command sederhana ke flash (Indirect Write, 1-wire)
 *    Dipakai untuk: WREN, EQIO, dsb.
 * ========================================================================= */
void OCTOSPI1_LL_SendCmd(uint8_t cmd)
{
    wait_not_busy();

    /* CCR: Communication Configuration Register
     * IMODE [4:3]  = 01 → Instruction on 1 wire (SIO)
     * ADMODE[7:6]  = 00 → No address
     * DMODE [27:26]= 00 → No data
     * ISIZE [5:4]  = 00 → Instruction 8-bit
     */
    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)   /* Instruction: 1 line */
                  | (0U << OCTOSPI_CCR_ADMODE_Pos)   /* No address          */
                  | (0U << OCTOSPI_CCR_DMODE_Pos);   /* No data             */

    /* IR: Instruction Register — tulis command byte, ini trigger transfer  */
    OCTOSPI1->IR  = cmd;

    wait_tcf();
}

/* =========================================================================
 * 4. Aktifkan Quad mode pada flash Macronix
 *    Command: EQIO (0x35) — Enter Quad I/O mode
 *    Setelah ini IO0..IO3 semua aktif untuk data
 * ========================================================================= */
void OCTOSPI1_LL_EnableQuadMode(void)
{
    OCTOSPI1_LL_SendCmd(FLASH_CMD_EQIO);

    /* Delay singkat setelah EQIO agar flash stabil masuk quad mode         */
    for (volatile uint32_t i = 0; i < 1000; i++);
}

/* =========================================================================
 * 5. Set Memory-Mapped mode (FMODE = 11b)
 *    Konfigurasi: Fast Read Quad I/O (0xEB)
 *    Format: 1-4-4 (Instruction:1wire, Address:4wire, Data:4wire)
 *
 *    Setelah fungsi ini return:
 *    → CPU bisa baca/eksekusi langsung dari 0x90000000
 *    → TIDAK BOLEH ada akses indirect lagi (DLR, DR, dll)
 * ========================================================================= */
void OCTOSPI1_LL_MemoryMapped(void)
{
    wait_not_busy();

    /* ---- DLR: Data Length Register — tidak relevan di MM mode ----------- */
    /* (diabaikan hardware saat FMODE=11)                                    */

    /* ---- CCR: Communication Configuration --------------------------------
     * IMODE [4:3]   = 01  → Instruction: 1 wire (SIO)
     * ISIZE [5:4]   = 00  → Instruction: 8-bit
     * ADMODE[7:6]   = 10  → Address: 4 wire (Quad)
     * ADSIZE[9:8]   = 10  → Address: 24-bit (3 byte), sesuaikan jika 4-byte
     * ABMODE[11:10] = 10  → Alternate bytes: 4 wire
     * ABSIZE[13:12] = 00  → Alternate bytes: 1 byte (mode byte = 0x00)
     * DMODE [27:26] = 10  → Data: 4 wire (Quad)
     * DCYC  [22:17] = 6   → Dummy cycles = 6 (sesuaikan dg flash & frekuensi)
     * -------------------------------------------------------------------- */
    OCTOSPI1->CCR = (1U  << OCTOSPI_CCR_IMODE_Pos)   /* Instr: 1 line      */
                  | (0U  << OCTOSPI_CCR_ISIZE_Pos)    /* Instr: 8-bit       */
                  | (2U  << OCTOSPI_CCR_ADMODE_Pos)   /* Addr: Quad         */
                  | (2U  << OCTOSPI_CCR_ADSIZE_Pos)   /* Addr: 24-bit       */
                  | (2U  << OCTOSPI_CCR_ABMODE_Pos)   /* AltByte: Quad      */
                  | (0U  << OCTOSPI_CCR_ABSIZE_Pos)   /* AltByte: 1 byte    */
                  | (2U  << OCTOSPI_CCR_DMODE_Pos);   /* Data: Quad         */

    /* ---- TCR: Timing Configuration (Dummy cycles) ------------------------ */
    OCTOSPI1->TCR = (FLASH_DUMMY_CYCLES << OCTOSPI_TCR_DCYC_Pos);

    /* ---- IR: Instruction — Fast Read Quad I/O command ------------------- */
    OCTOSPI1->IR  = FLASH_CMD_READ_QUAD;

    /* ---- ABR: Alternate Bytes Register
     * Mode byte = 0x00 → continuous read (mode bits off)
     * Kalau flash Anda pakai XIP continuous mode, set 0xA0
     * Untuk single-burst read tanpa continuous, gunakan 0x00
     * -------------------------------------------------------------------- */
    OCTOSPI1->ABR = 0x00U;

    /* ---- CR: Aktifkan Memory-Mapped mode (FMODE = 11b) ------------------ */
    OCTOSPI1->CR = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk)
                 | (3U << OCTOSPI_CR_FMODE_Pos);
}

/* =========================================================================
 * PUBLIC: OCTOSPI1_LL_IndirectInit()
 * Inisialisasi GPIO, Core register, dan aktifkan Quad mode pada flash.
 * Tetap dalam mode Indirect (bisa Read/Write byte register).
 * ========================================================================= */
void OCTOSPI1_LL_IndirectInit(void)
{
    OCTOSPI1_GPIO_Init();
    OCTOSPI1_Core_Init();
    OCTOSPI1_LL_EnableQuadMode();
}

/* =========================================================================
 * PUBLIC: OCTOSPI1_LL_Init()
 * Init lengkap langsung sampai Memory-Mapped (shortcut jika tidak butuh staging)
 * ========================================================================= */
void OCTOSPI1_LL_Init(void)
{
    OCTOSPI1_LL_IndirectInit();
    OCTOSPI1_LL_MemoryMapped();
}

