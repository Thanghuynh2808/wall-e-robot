/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Encoder + PWM Test)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
// --- Biến Encoder tích lũy (Phục vụ lệnh 'e') ---
int32_t total_encoder_left  = 0;
int32_t total_encoder_right = 0;
int16_t last_count_left     = 0;
int16_t last_count_right    = 0;

// --- Biến Vận tốc mục tiêu (Phục vụ lệnh 'm') ---
float target_speed_left  = 0.0f; // Đơn vị: xung / chu kỳ (20ms)
float target_speed_right = 0.0f;

// --- Biến USB Parser ---
char rx_buffer[64];
uint8_t rx_index = 0;

// --- Biến Debug ---
volatile int16_t debug_delta_left  = 0;
volatile int16_t debug_delta_right = 0;
volatile int32_t debug_pwm_left    = 0;
volatile int32_t debug_pwm_right   = 0;
uint8_t debug_mode = 0;  // 1 = in liên tục delta và PWM

// --- PID cho từng bánh xe (Vòng lặp vận tốc) ---
pid_type_def pid_speed_left;
pid_type_def pid_speed_right;
fp32 speed_pid_params[3] = {600.0f, 100.0f, 20.0f}; // Kp, Ki, Kd

// --- Feedforward: PWM tối thiểu để thắng lực ma sát tĩnh ---
#define DEADZONE_LEFT  54000 // Chỉnh giảm nếu bánh trái vọt quá nhanh
#define DEADZONE_RIGHT 56000 // Chỉnh tăng nếu bánh phải bị trễ

osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
    .name       = "ChassisTask",
    .stack_size = 512 * 4,
    .priority   = (osPriority_t) osPriorityRealtime,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void stopMotors(void);
void setMotorOutput(int32_t out_l, int32_t out_r);
void StartChassisTask(void *argument);
void USB_Receive_Handler(uint8_t* Buf, uint32_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// --- Hàm gửi qua USB CDC (chuyển hướng printf) ---
int _write(int file, char *ptr, int len) {
    uint32_t timeout = 0xFFFF;
    while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY && timeout--);
    return len;
}

// --- Gửi chuỗi qua USB CDC ---
void USB_SendString(const char* str) {
    uint16_t len = strlen(str);
    uint32_t timeout = 0xFFFF;
    while (CDC_Transmit_FS((uint8_t*)str, len) == USBD_BUSY && timeout--);
}

// --- Callback nhận dữ liệu từ USB CDC ---
void USB_Receive_Handler(uint8_t* Buf, uint32_t Len) {
    for (uint32_t i = 0; i < Len; i++) {
        uint8_t data = Buf[i];
        if (data == '\r' || data == '\n') {
            rx_buffer[rx_index] = '\0';
            if (rx_index > 0) {
                // Lệnh 'm': Đặt vận tốc mục tiêu (m [speed_L] [speed_R])
                if (rx_buffer[0] == 'm') {
                    float sl = 0, sr = 0;
                    int parsed = sscanf(rx_buffer, "m %f %f", &sl, &sr);
                    if (parsed == 2) {
                        target_speed_left  = sl;
                        target_speed_right = sr;
                        USB_SendString("motor ok\n");
                    }
                }
                // Lệnh 'e': Đọc encoder tích lũy
                else if (rx_buffer[0] == 'e') {
                    char tx_buf[64];
                    int t_len = sprintf(tx_buf, "%ld %ld\n", total_encoder_left, total_encoder_right);
                    CDC_Transmit_FS((uint8_t*)tx_buf, t_len);
                }
                // Lệnh 'r': Reset encoder về 0
                else if (rx_buffer[0] == 'r') {
                    total_encoder_left  = 0;
                    total_encoder_right = 0;
                    USB_SendString("reset ok\n");
                }
                // Lệnh 's': Dừng khẩn cấp
                else if (rx_buffer[0] == 's') {
                    target_speed_left  = 0;
                    target_speed_right = 0;
                    USB_SendString("stop ok\n");
                }
                // Lệnh 'u': Cập nhật tham số PID từ ROS (u Kp:Kd:Ki:Ko)
                else if (rx_buffer[0] == 'u') {
                    int p = 0, d = 0, i = 0, o = 0;
                    int parsed = sscanf(rx_buffer, "u %d:%d:%d:%d", &p, &d, &i, &o);
                    if (parsed >= 3) { // Tối thiểu cần P, D, I
                        speed_pid_params[0] = (float)p; // Kp
                        speed_pid_params[1] = (float)i; // Ki
                        speed_pid_params[2] = (float)d; // Kd
                        
                        // Khởi tạo lại PID với bộ tham số mới
                        PID_init(&pid_speed_left, PID_POSITION, speed_pid_params, 65535, 10000);
                        PID_init(&pid_speed_right, PID_POSITION, speed_pid_params, 65535, 10000);
                    }
                    USB_SendString("pid ok\n");
                }
                // Lệnh 'd': Bật/tắt debug (in delta và PWM liên tục)
                else if (rx_buffer[0] == 'd') {
                    debug_mode = !debug_mode;
                    USB_SendString(debug_mode ? "debug on\n" : "debug off\n");
                }
            }
            rx_index = 0;
        } else {
            if (rx_index < 63) rx_buffer[rx_index++] = data;
        }
    }
}

// --- Hàm xuất PWM dựa trên giá trị PID (Xử lý chiều quay) ---
void setMotorOutput(int32_t out_l, int32_t out_r) {
    // Lưu lại cho debug
    debug_pwm_left  = out_l;
    debug_pwm_right = out_r;

    // Motor Trái (PB12, PB13)
    if (out_l >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        out_l = -out_l;
    }
    // Motor Phải (PB14, PB15)
    if (out_r >= 0) {
        HAL_GPIO_WritePin(GPIOB, BIN14_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, BIN15_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, BIN14_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, BIN15_Pin, GPIO_PIN_RESET);
        out_r = -out_r;
    }
    
    if (out_l > 65535) out_l = 65535;
    if (out_r > 65535) out_r = 65535;
    
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, out_l);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, out_r);
}

void stopMotors(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, BIN14_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, BIN15_Pin,   GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    // Khởi tạo PID vận tốc cho 2 bánh
    PID_init(&pid_speed_left, PID_POSITION, speed_pid_params, 65535, 10000);
    PID_init(&pid_speed_right, PID_POSITION, speed_pid_params, 65535, 10000);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin   = BIN12_Pin | BIN13_Pin | BIN14_Pin | BIN15_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, BIN12_Pin | BIN13_Pin | BIN14_Pin | BIN15_Pin, GPIO_PIN_RESET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void StartChassisTask(void *argument) {
    uint32_t debug_timer = 0;

    for (;;) {
        // --- 1. Cập nhật Encoder tích lũy ---
        int16_t current_left  = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        int16_t current_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

        int16_t delta_left  = current_left  - last_count_left;
        int16_t delta_right = current_right - last_count_right;
        
        // ============================================================
        // ĐẢO CHIỀU ENCODER:
        // Quy ước: Khi bánh xe quay TIẾN, delta phải là số DƯƠNG.
        // Nếu bạn thấy số âm khi quay bánh tiến, thêm/bớt dấu '-'
        // ở dòng tương ứng bên dưới.
        // ============================================================
        delta_left  = -delta_left;   // Đã kích hoạt đảo chiều bánh trái
        delta_right = -delta_right;  // Đảo chiều bánh phải

        // Lưu lại cho debug
        debug_delta_left  = delta_left;
        debug_delta_right = delta_right;

        total_encoder_left  += delta_left;
        total_encoder_right += delta_right;

        last_count_left  = current_left;
        last_count_right = current_right;

        // --- 2. Tính toán PID vận tốc (Chu kỳ 20ms) ---
        float pid_out_l = PID_calc(&pid_speed_left,  (float)delta_left,  target_speed_left);
        float pid_out_r = PID_calc(&pid_speed_right, (float)delta_right, target_speed_right);

        // --- 3. Feedforward: Bù vùng chết motor ---
        // Thêm một lượng PWM cố định để vượt qua lực ma sát tĩnh
        // QUAN TRỌNG: Chặn không cho PID đảo chiều động cơ nếu chỉ là overshoot!
        int32_t out_l, out_r;
        
        if (target_speed_left > 0) {
            out_l = (int32_t)(pid_out_l + DEADZONE_LEFT);
            if (out_l < 0) out_l = 0; // Đang muốn tiến, không cho phép lùi
        } else if (target_speed_left < 0) {
            out_l = (int32_t)(pid_out_l - DEADZONE_LEFT);
            if (out_l > 0) out_l = 0; // Đang muốn lùi, không cho phép tiến
        } else {
            out_l = 0;
        }

        if (target_speed_right > 0) {
            out_r = (int32_t)(pid_out_r + DEADZONE_RIGHT);
            if (out_r < 0) out_r = 0; // Đang muốn tiến, không cho phép lùi
        } else if (target_speed_right < 0) {
            out_r = (int32_t)(pid_out_r - DEADZONE_RIGHT);
            if (out_r > 0) out_r = 0; // Đang muốn lùi, không cho phép tiến
        } else {
            out_r = 0;
        }

        // --- 4. Thực thi Motor ---
        if (target_speed_left == 0 && target_speed_right == 0) {
            stopMotors();
            PID_clear(&pid_speed_left);
            PID_clear(&pid_speed_right);
        } else {
            setMotorOutput(out_l, out_r);
        }

        // --- 5. Debug output (mỗi 500ms) ---
        if (debug_mode) {
            debug_timer += 20;
            if (debug_timer >= 500) {
                debug_timer = 0;
                char dbuf[128];
                int dlen = sprintf(dbuf, "dL:%d dR:%d pwmL:%ld pwmR:%ld tL:%.0f tR:%.0f\n",
                    debug_delta_left, debug_delta_right,
                    debug_pwm_left, debug_pwm_right,
                    target_speed_left, target_speed_right);
                CDC_Transmit_FS((uint8_t*)dbuf, dlen);
            }
        }

        osDelay(20); // Chu kỳ điều khiển 50Hz
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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