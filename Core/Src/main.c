/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// her eksenin pid durumunu ayrı tutmak için
typedef struct {
    float kp, ki, kd;
    float integral;      // hatanın zamana göre toplamı
    float onceki_hata;   // türev için bir önceki adımın hatası
} PID_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// servo kalibrasyonu (ölçerek buldum)
#define SERVO_MIN_US        500     // 0 derece
#define SERVO_MAX_US        2500    // 180 derece

// sanal kamera ayarları (esp32-cam qvga çözünürlüğü)
#define GORUNTU_GENISLIK    320
#define GORUNTU_YUKSEKLIK   240
#define PIKSEL_PER_DERECE   5.33f   // 320 px / ~60 derece görüş açısı

#define KONTROL_PERIYOT_MS  20      // kontrol döngüsü 50 hz
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// uart
uint8_t rx_byte;                    // hal gelen karakteri buraya yazıyor
char rx_buf[16];                    // komut burada birikiyor
volatile uint8_t rx_idx = 0;        // isr ve main ikisi de değiştiriyor
volatile uint8_t komut_hazir = 0;   // isr -> main bayrağı

// pid (şimdilik sadece p)
PID_t pid_pan  = {0.03f, 0.0f, 0.0f, 0.0f, 0.0f};
PID_t pid_tilt = {0.03f, 0.0f, 0.0f, 0.0f, 0.0f};

// servoların şu anki açısı
float pan_aci  = 90.0f;
float tilt_aci = 90.0f;

// sanal hedefin dünyadaki konumu (W komutuyla değişiyor)
float hedef_pan  = 90.0f;
float hedef_tilt = 90.0f;

// zamanlama
uint32_t son_kontrol   = 0;
uint32_t son_telemetri = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// açıyı (0-180) darbe genişliğine çevirip ilgili kanala yazar
void servo_aci_ayarla(uint32_t kanal, uint8_t aci)
{
    if (aci > 180) aci = 180;
    uint32_t darbe = SERVO_MIN_US + ((uint32_t)aci * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
    __HAL_TIM_SET_COMPARE(&htim2, kanal, darbe);
}

// pid çıkışını hesaplar, aynı fonksiyon iki eksen için de kullanılıyor
float pid_hesapla(PID_t *pid, float hata, float dt)
{
    pid->integral += hata * dt;
    float turev = (hata - pid->onceki_hata) / dt;
    pid->onceki_hata = hata;
    return pid->kp * hata + pid->ki * pid->integral + pid->kd * turev;
}

// kamera gelene kadar test için: hedef görüntüde nerede görünürdü?
void sanal_kamera(float *x, float *y)
{
    *x = GORUNTU_GENISLIK  / 2.0f + (hedef_pan  - pan_aci)  * PIKSEL_PER_DERECE;
    *y = GORUNTU_YUKSEKLIK / 2.0f + (hedef_tilt - tilt_aci) * PIKSEL_PER_DERECE;
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
  MX_TIM2_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);      // pan
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);      // tilt
  HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);   // uart dinlemeye başla
  son_kontrol = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 1) uart komutu: W<pan>,<tilt> hedefi taşır, K<kp*1000> kazancı değiştirir
    if (komut_hazir)
    {
        char *w = strchr(rx_buf, 'W');
        char *v = strchr(rx_buf, ',');
        char *k = strchr(rx_buf, 'K');
        char msg[48];
        int len;

        if (k)   // K30 -> kp = 0.030 (float parse etmemek için binde bir)
        {
            float yeni_kp = atoi(k + 1) / 1000.0f;
            pid_pan.kp  = yeni_kp;
            pid_tilt.kp = yeni_kp;
            len = snprintf(msg, sizeof(msg), "Kp: %d/1000\r\n", atoi(k + 1));
        }
        else if (w && v)
        {
            hedef_pan  = (float)atoi(w + 1);
            hedef_tilt = (float)atoi(v + 1);
            len = snprintf(msg, sizeof(msg), "Hedef: %d,%d\r\n", (int)hedef_pan, (int)hedef_tilt);
        }
        else
        {
            len = snprintf(msg, sizeof(msg), "Hata: W<pan>,<tilt> veya K<kp>\r\n");
        }

        HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, len, 100);
        rx_idx = 0;
        komut_hazir = 0;   // en son indir, yoksa isr eski indeksle yazabilir
    }

    // 2) kontrol döngüsü, 20 ms'de bir (hal_delay ile bloklamıyorum)
    if (HAL_GetTick() - son_kontrol >= KONTROL_PERIYOT_MS)
    {
        son_kontrol += KONTROL_PERIYOT_MS;
        float dt = KONTROL_PERIYOT_MS / 1000.0f;

        float x, y;
        sanal_kamera(&x, &y);

        // hata = hedefin merkezden uzaklığı (piksel)
        float hata_x = x - GORUNTU_GENISLIK  / 2.0f;
        float hata_y = y - GORUNTU_YUKSEKLIK / 2.0f;

        pan_aci  += pid_hesapla(&pid_pan,  hata_x, dt);
        tilt_aci += pid_hesapla(&pid_tilt, hata_y, dt);

        // servo sınırlarının dışına çıkmasın
        if (pan_aci  < 0)   pan_aci  = 0;
        if (pan_aci  > 180) pan_aci  = 180;
        if (tilt_aci < 0)   tilt_aci = 0;
        if (tilt_aci > 180) tilt_aci = 180;

        servo_aci_ayarla(TIM_CHANNEL_1, (uint8_t)(pan_aci  + 0.5f));   // +0.5 yuvarlama için
        servo_aci_ayarla(TIM_CHANNEL_2, (uint8_t)(tilt_aci + 0.5f));
    }

    // 3) telemetri, 200 ms'de bir durumu bas (nano printf %f desteklemiyor, int'e çeviriyorum)
    if (HAL_GetTick() - son_telemetri >= 200)
    {
        son_telemetri = HAL_GetTick();
        float x, y;
        sanal_kamera(&x, &y);
        char msg[64];
        int len = snprintf(msg, sizeof(msg), "x:%d y:%d pan:%d tilt:%d\r\n",
                           (int)x, (int)y, (int)pan_aci, (int)tilt_aci);
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, len, 100);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// uart'tan her karakter gelince çağrılıyor
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        if (!komut_hazir)   // main önceki komutu işlerken buffer'a dokunma
        {
            if (rx_byte == '\r' || rx_byte == '\n')
            {
                if (rx_idx > 0)
                {
                    rx_buf[rx_idx] = '\0';
                    komut_hazir = 1;
                }
            }
            else if (rx_idx < sizeof(rx_buf) - 1)   // taşma kontrolü
            {
                rx_buf[rx_idx++] = rx_byte;
            }
        }
        HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);   // tek seferlik, tekrar başlatıyorum
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
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
