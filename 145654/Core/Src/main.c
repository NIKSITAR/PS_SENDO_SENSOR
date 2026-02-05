/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Система измерения глубины и температуры
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Настройки датчика давления (30 psi = ~2.07 Бар)
#define VOLTAGE_DIVIDER_RATIO   1.37f      // Коэффициент делителя напряжения
#define PRESSURE_MIN_VOLTAGE    0.5f       // Минимальное напряжение датчика давления (В)
#define PRESSURE_MAX_VOLTAGE    4.5f       // Максимальное напряжение датчика давления (В)
#define PRESSURE_MIN_VALUE      0.0f       // Минимальное давление (Бар) - 0 psi
#define PRESSURE_MAX_VALUE      2.07f      // Максимальное давление (Бар) - 30 psi

// Настройки датчика температуры (TMP36)
#define TEMP_SENSOR_VREF        3.3f       // Напряжение для температуры
#define TEMP_VOLTAGE_AT_0C      0.5f       // Напряжение при 0°C
#define TEMP_MV_PER_DEGREE      10.0f      // 10 мВ/°C

// Общие настройки АЦП
#define VREF_VOLTAGE            3.3f       // Опорное напряжение АЦП (В)
#define ADC_RESOLUTION          4095.0f    // Разрешение 12-битного АЦП
#define MEASUREMENT_PERIOD_MS   100        // Период измерений 100 мс (10 Гц)

// Физические константы для расчета глубины
#define WATER_DENSITY           1000.0f    // Плотность воды (кг/м³)
#define GRAVITY                 9.80665f   // Ускорение свободного падения (м/с²)

// Калибровка
#define CALIBRATION_SAMPLES     20         // Количество измерений для калибровки

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;    // ADC1 для датчика давления (канал 0, PA0)
ADC_HandleTypeDef hadc2;    // ADC2 для датчика температуры (канал 2, PA2)
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint16_t pressure_adc_value = 0;
volatile uint16_t temp_adc_value = 0;
volatile float depth_meters = 0.0f;
volatile float temperature_c = 0.0f;
volatile float surface_pressure_pa = 0.0f;  // Давление на поверхности (Па)
volatile uint32_t measurement_counter = 0;
volatile uint8_t calibration_done = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Read_Pressure_Sensor(void);
void Read_Temperature_Sensor(void);
float Calculate_Pressure_Pa(uint16_t adc_value);
float Calculate_Temperature(uint16_t adc_value);
float Calculate_Depth(float pressure_pa);
void Send_Data_To_UART(void);
void UART_Print(const char* message);
void Auto_Calibrate_Surface_Pressure(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Функция для отправки строки по UART
void UART_Print(const char* message) {
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
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
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    // Устанавливаем PA8 в логическую 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    // Калибровка АЦП
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc2);

    // Стартовое сообщение
    UART_Print("\r\n=== Система измерения глубины и температуры ===\r\n");
    UART_Print("Автоматическая калибровка...\r\n");

    // Автоматическая калибровка давления на поверхности
    Auto_Calibrate_Surface_Pressure();

    UART_Print("Калибровка завершена. Начинаем измерения...\r\n\r\n");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint32_t last_measurement = HAL_GetTick();

    while (1)
    {
        uint32_t current_time = HAL_GetTick();

        // Измерение каждые 100 мс (10 Гц)
        if (current_time - last_measurement >= MEASUREMENT_PERIOD_MS) {
            last_measurement = current_time;

            // Чтение датчиков
            Read_Pressure_Sensor();
            Read_Temperature_Sensor();

            // Расчет значений
            float pressure_pa = Calculate_Pressure_Pa(pressure_adc_value);
            depth_meters = Calculate_Depth(pressure_pa);
            temperature_c = Calculate_Temperature(temp_adc_value);

            // Отправка данных по UART
            Send_Data_To_UART();

            measurement_counter++;
        }
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief Автоматическая калибровка давления на поверхности
  */
void Auto_Calibrate_Surface_Pressure(void)
{
    float sum_pressure = 0.0f;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        Read_Pressure_Sensor();
        float pressure_pa = Calculate_Pressure_Pa(pressure_adc_value);
        sum_pressure += pressure_pa;
        HAL_Delay(50);
    }

    // Устанавливаем давление на поверхности
    surface_pressure_pa = sum_pressure / CALIBRATION_SAMPLES;
    calibration_done = 1;
}

/**
  * @brief Чтение датчика давления (ADC1, канал 0)
  */
void Read_Pressure_Sensor(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);

    if (HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC) {
        pressure_adc_value = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);
}

/**
  * @brief Чтение датчика температуры (ADC2, канал 2)
  */
void Read_Temperature_Sensor(void)
{
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);

    if (HAL_ADC_GetState(&hadc2) & HAL_ADC_STATE_REG_EOC) {
        temp_adc_value = HAL_ADC_GetValue(&hadc2);
    }

    HAL_ADC_Stop(&hadc2);
}

/**
  * @brief Расчет давления в Паскалях из ADC значения
  */
float Calculate_Pressure_Pa(uint16_t adc_value)
{
    // 1. ADC -> напряжение после делителя
    float voltage_after_divider = (adc_value / ADC_RESOLUTION) * VREF_VOLTAGE;

    // 2. Напряжение с датчика (до делителя)
    float sensor_voltage = voltage_after_divider * VOLTAGE_DIVIDER_RATIO;

    // 3. Ограничение по напряжению
    if (sensor_voltage < PRESSURE_MIN_VOLTAGE) sensor_voltage = PRESSURE_MIN_VOLTAGE;
    if (sensor_voltage > PRESSURE_MAX_VOLTAGE) sensor_voltage = PRESSURE_MAX_VOLTAGE;

    // 4. Напряжение -> давление в Барах
    float voltage_range = PRESSURE_MAX_VOLTAGE - PRESSURE_MIN_VOLTAGE;
    float pressure_range = PRESSURE_MAX_VALUE - PRESSURE_MIN_VALUE;
    float pressure_bar = 0.0f;

    if (voltage_range > 0) {
        pressure_bar = PRESSURE_MIN_VALUE +
                       ((sensor_voltage - PRESSURE_MIN_VOLTAGE) * pressure_range / voltage_range);
    }

    // 5. Ограничение по давлению
    if (pressure_bar < PRESSURE_MIN_VALUE) pressure_bar = PRESSURE_MIN_VALUE;
    if (pressure_bar > PRESSURE_MAX_VALUE) pressure_bar = PRESSURE_MAX_VALUE;

    // 6. Перевод в Паскали
    return pressure_bar * 100000.0f;
}

/**
  * @brief Расчет температуры из ADC значения (TMP36)
  */
float Calculate_Temperature(uint16_t adc_value)
{
    // 1. ADC -> напряжение
    float voltage = (adc_value / ADC_RESOLUTION) * TEMP_SENSOR_VREF;

    // 2. Напряжение -> температура
    float temperature = (voltage - TEMP_VOLTAGE_AT_0C) * 100.0f;

    // 3. Ограничение диапазона датчика
    if (temperature < -40.0f) temperature = -40.0f;
    if (temperature > 125.0f) temperature = 125.0f;

    return temperature;
}

/**
  * @brief Расчет глубины по давлению
  */
float Calculate_Depth(float pressure_pa)
{
    if (!calibration_done) {
        return 0.0f;
    }

    // Разность давления относительно калиброванного нуля
    float delta_pressure = pressure_pa - surface_pressure_pa;

    // Если давление очень близко к нулю - глубина = 0
    if (fabsf(delta_pressure) < 100.0f) {
        return 0.0f;
    }

    // Формула для глубины: h = ΔP / (ρ * g)
    return delta_pressure / (WATER_DENSITY * GRAVITY);
}

/**
  * @brief Отправка данных по UART (глубина и температура)
  */
void Send_Data_To_UART(void)
{
    char buffer[64];

    // Формат: глубина (м) и температура (°C)
    snprintf(buffer, sizeof(buffer), "%.3f m %.1f C\r\n", depth_meters, temperature_c);
    UART_Print(buffer);
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/**
  * @brief ADC1 Initialization Function (давление, канал 0, PA0)
  */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/**
  * @brief ADC2 Initialization Function (температура, канал 2, PA2)
  */
static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc2);

    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);
}

/**
  * @brief USART1 Initialization Function
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA0 - датчик давления
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA2 - датчик температуры
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
        HAL_Delay(100);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
