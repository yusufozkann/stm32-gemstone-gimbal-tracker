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
// her eksen icin ayri pid
typedef struct {
    float kp, ki, kd;
    float integral;      // hata toplami
    float onceki_hata;   // turev icin
} PID_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// servo kalibrasyonu, olcerek buldum
#define SERVO_MIN_US        500     // 0 derece
#define SERVO_MAX_US        2500    // 180 derece

// kamera c270, 640x480
#define GORUNTU_GENISLIK    640
#define GORUNTU_YUKSEKLIK   480
#define PIKSEL_PER_DERECE   11.6f   // 640 / ~55 derece, sadece sanal kamera icin

#define KONTROL_PERIYOT_MS  20      // sanal mod 50 hz
#define HEDEF_ZAMAN_ASIMI   500     // ms, olcum gelmezse hedef kayip

// mekanik sinirlar, parcalar carpiyor
#define PAN_MIN   20
#define PAN_MAX   160
#define TILT_MIN  45
#define TILT_MAX  135
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// uart
uint8_t rx_byte;                    // gelen karakter
char rx_buf[16];                    // komut burada birikiyor
volatile uint8_t rx_idx = 0;        // isr ve main degistiriyor
volatile uint8_t komut_hazir = 0;   // flag

// pid, simdilik sadece p
PID_t pid_pan  = {0.03f, 0.0f, 0.0f, 0.0f, 0.0f};
PID_t pid_tilt = {0.03f, 0.0f, 0.0f, 0.0f, 0.0f};

// servolarin su anki acisi
float pan_aci  = 90.0f;
float tilt_aci = 90.0f;

// sanal hedef (W komutu)
float hedef_pan  = 90.0f;
float hedef_tilt = 90.0f;

// kameradan gelen olcum (python C<x>,<y> yolluyor)
uint8_t  gercek_mod  = 0;           // 0 sanal, 1 gercek kamera
uint8_t  yeni_olcum  = 0;           // yeni kare geldi mi
uint8_t  hedef_var   = 0;           // hedef var mi
float    kam_x = GORUNTU_GENISLIK  / 2.0f;
float    kam_y = GORUNTU_YUKSEKLIK / 2.0f;
uint32_t son_olcum_ms = 0;
uint32_t onceki_olcum_ms = 0;       // dt icin

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
// aci (0-180) -> darbe genisligi (us)
void servo_aci_ayarla(uint32_t kanal, uint8_t aci)
{
    if (aci > 180) aci = 180;
    uint32_t darbe = SERVO_MIN_US + ((uint32_t)aci * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
    __HAL_TIM_SET_COMPARE(&htim2, kanal, darbe);
}

// iki eksen de bunu kullaniyor
float pid_hesapla(PID_t *pid, float hata, float dt)
{
    pid->integral += hata * dt;
    float turev = (hata - pid->onceki_hata) / dt;
    pid->onceki_hata = hata;
    return pid->kp * hata + pid->ki * pid->integral + pid->kd * turev;
}

// kamerasiz test icin, hedef goruntude nerede olurdu
void sanal_kamera(float *x, float *y)
{
    *x = GORUNTU_GENISLIK  / 2.0f + (hedef_pan  - pan_aci)  * PIKSEL_PER_DERECE;
    *y = GORUNTU_YUKSEKLIK / 2.0f + (hedef_tilt - tilt_aci) * PIKSEL_PER_DERECE;
}

// tek pid adimi
void takip_adimi(float x, float y, float dt)
{
    // hata = merkezden uzaklik (px)
    float hata_x = x - GORUNTU_GENISLIK  / 2.0f;
    float hata_y = y - GORUNTU_YUKSEKLIK / 2.0f;

    pan_aci  += pid_hesapla(&pid_pan,  hata_x, dt);
    tilt_aci += pid_hesapla(&pid_tilt, hata_y, dt);

    // sinirla
    if (pan_aci  < PAN_MIN)  pan_aci  = PAN_MIN;
    if (pan_aci  > PAN_MAX)  pan_aci  = PAN_MAX;
    if (tilt_aci < TILT_MIN) tilt_aci = TILT_MIN;
    if (tilt_aci > TILT_MAX) tilt_aci = TILT_MAX;

    servo_aci_ayarla(TIM_CHANNEL_1, (uint8_t)(pan_aci  + 0.5f));   // +0.5 yuvarlama
    servo_aci_ayarla(TIM_CHANNEL_2, (uint8_t)(tilt_aci + 0.5f));
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
  HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);   // uart dinle
  son_kontrol = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // komutlar: C<x>,<y> kamera, N hedef yok, W<p>,<t> sanal test, K<kp> kazanc (K30 = 0.030)
    if (komut_hazir)
    {
        char cmd = rx_buf[0];
        char *v  = strchr(rx_buf, ',');
        char msg[48];
        int len = 0;

        if (cmd == 'C' && v)
        {
            kam_x = (float)atoi(rx_buf + 1);
            kam_y = (float)atoi(v + 1);
            gercek_mod   = 1;
            hedef_var    = 1;
            yeni_olcum   = 1;
            son_olcum_ms = HAL_GetTick();
        }
        else if (cmd == 'N')
        {
            gercek_mod = 1;
            hedef_var  = 0;
        }
        else if (cmd == 'K')
        {
            float yeni_kp = atoi(rx_buf + 1) / 1000.0f;
            pid_pan.kp  = yeni_kp;
            pid_tilt.kp = yeni_kp;
            len = snprintf(msg, sizeof(msg), "Kp: %d/1000\r\n", atoi(rx_buf + 1));
        }
        else if (cmd == 'W' && v)
        {
            hedef_pan  = (float)atoi(rx_buf + 1);
            hedef_tilt = (float)atoi(v + 1);
            gercek_mod = 0;
            len = snprintf(msg, sizeof(msg), "Hedef: %d,%d\r\n", (int)hedef_pan, (int)hedef_tilt);
        }
        else
        {
            len = snprintf(msg, sizeof(msg), "Hata: C x,y / N / W p,t / K kp\r\n");
        }

        if (len > 0) HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, len, 100);
        rx_idx = 0;
        komut_hazir = 0;   // en son indir, race condition
    }

    if (gercek_mod)
    {
        // pid sadece yeni kare gelince. kamera ~10 fps, eski olcumle 50 hz
        // calisirsa ayni hatayi birkac kez duzeltip hedefi geciyor
        if (yeni_olcum)
        {
            yeni_olcum = 0;
            uint32_t simdi = son_olcum_ms;
            float dt = (simdi - onceki_olcum_ms) / 1000.0f;
            if (dt <= 0.0f || dt > 0.5f) dt = 0.1f;   // ilk olcum
            onceki_olcum_ms = simdi;
            takip_adimi(kam_x, kam_y, dt);
        }

        // olcum gelmiyorsa servolar oldugu yerde kalir
        if (HAL_GetTick() - son_olcum_ms > HEDEF_ZAMAN_ASIMI)
        {
            hedef_var = 0;
        }
    }
    else if (HAL_GetTick() - son_kontrol >= KONTROL_PERIYOT_MS)
    {
        // sanal kamera, 50 hz, hal_delay kullanmadim
        son_kontrol += KONTROL_PERIYOT_MS;
        float x, y;
        sanal_kamera(&x, &y);
        takip_adimi(x, y, KONTROL_PERIYOT_MS / 1000.0f);
    }

    // telemetri 200 ms, nano printf %f basmiyor o yuzden int
    if (HAL_GetTick() - son_telemetri >= 200)
    {
        son_telemetri = HAL_GetTick();
        float x = kam_x, y = kam_y;
        if (!gercek_mod) sanal_kamera(&x, &y);
        char msg[64];
        int len = snprintf(msg, sizeof(msg), "%s x:%d y:%d pan:%d tilt:%d\r\n",
                           gercek_mod ? (hedef_var ? "KAM" : "YOK") : "SIM",
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        if (!komut_hazir)   // main isini bitirmeden yazma
        {
            if (rx_byte == '\r' || rx_byte == '\n')
            {
                if (rx_idx > 0)
                {
                    rx_buf[rx_idx] = '\0';
                    komut_hazir = 1;
                }
            }
            else if (rx_idx < sizeof(rx_buf) - 1)   // tasma kontrolu
            {
                rx_buf[rx_idx++] = rx_byte;
            }
        }
        HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);   // tekrar dinlemek icin
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
