/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Bootloader STM32H523CET6 — XIP via OctoSPI Memory-Mapped
  *
  * ALUR:
  *  1. Init clock (LL)
  *  2. Init GPIO (LL)
  *  3. Init OctoSPI → Memory-Mapped mode (bare-metal register)
  *  4. Validasi: cek apakah MSP di 0x90000000 valid (SRAM range)
  *  5. Jump ke Reset_Handler aplikasi di flash eksternal (0x90000000)
  *
  * CATATAN:
  *  - Tidak ada copy ke SRAM — CPU langsung eksekusi dari 0x90000000 (XIP)
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

/*
 * APP_ADDRESS: alamat awal aplikasi di memory-mapped OctoSPI
 * Ini adalah awal dari flash eksternal setelah di-map ke 0x90000000
 *
 * Offset bisa diubah jika ada header/metadata di awal flash, misal:
 *   #define APP_ADDRESS  (OCTOSPI1_MM_BASE + 0x10000)  // app mulai di +64KB
 */
#define APP_ADDRESS     APP_MM_ADDRESS  /* = 0x90000000 dari octospi_ll.h   */

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
static void Jump_To_App(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Jump ke aplikasi di flash eksternal (XIP, memory-mapped OctoSPI)
  *
  *         Langkah:
  *         1. Baca MSP dari word pertama di APP_ADDRESS (offset 0)
  *         2. Baca Reset_Handler dari word kedua di APP_ADDRESS (offset +4)
  *         3. Validasi: MSP harus menunjuk ke SRAM STM32H523 (0x20000000 range)
  *         4. Set MSP baru
  *         5. Jump ke Reset_Handler
  *
  *         NOTE: Tidak ada deinit peripheral apapun sebelum jump,
  *         karena OctoSPI harus tetap aktif agar XIP berjalan!
  */
static void Jump_To_App(void)
{
    /* --- Baca vektor table dari awal flash eksternal --------------------- */
    uint32_t app_msp    = *(__IO uint32_t *)(APP_ADDRESS);        /* offset 0 */
    uint32_t app_reset  = *(__IO uint32_t *)(APP_ADDRESS + 4U);   /* offset 4 */

    /*
     * Validasi MSP:
     * STM32H523 SRAM ada di 0x20000000 .. 0x20044000 (272 KB)
     * Jika MSP di luar range ini, anggap flash kosong / korup
     */
    if ((app_msp & 0xFF000000U) != 0x20000000U)
    {
        /* Flash eksternal kosong atau tidak valid — berhenti di sini        */
        /* Tambahkan LED blink / error indicator sesuai kebutuhan           */
        while (1)
        {
            /* Tunggu — tidak ada yang bisa dilakukan tanpa aplikasi valid  */
        }
    }

    /* --- Disable semua interrupt sebelum jump ---------------------------- */
    __disable_irq();

    /* --- Reset NVIC: clear semua pending interrupt ----------------------- */
    for (uint8_t i = 0; i < 8U; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFU;  /* Disable semua IRQ                 */
        NVIC->ICPR[i] = 0xFFFFFFFFU;  /* Clear pending                     */
    }

    /* --- Set Vector Table ke lokasi aplikasi di OctoSPI ------------------ */
    SCB->VTOR = APP_ADDRESS;

    /* --- Set Main Stack Pointer ke nilai dari vektor table aplikasi ------ */
    __set_MSP(app_msp);

    /* --- Set Process Stack Pointer (untuk OS compatibility) -------------- */
    __set_PSP(app_msp);

    /* --- Ensure instruction & data sync sebelum jump --------------------- */
    __ISB();
    __DSB();

    /* --- Buat function pointer ke Reset_Handler aplikasi ----------------- */
    pFunction jump = (pFunction)(app_reset);

    /* --- JUMP! ----------------------------------------------------------- */
    jump();

    /* Tidak seharusnya sampai sini                                          */
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  /* --- Init OctoSPI1 → Memory-Mapped mode (bare-metal, tanpa HAL) ------ */
  OCTOSPI1_LL_Init();

  /* --- Langsung jump ke aplikasi di flash eksternal (XIP) -------------- */
  Jump_To_App();

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

  /*
   * HAL_InitTick sengaja tidak dipanggil — bootloader tidak butuh SysTick.
   * SystemClock_Config selesai dengan LL murni.
   */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
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
{/* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
