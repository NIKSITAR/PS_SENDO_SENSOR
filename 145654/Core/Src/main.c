/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Система измерения глубины и температуры (СТАБИЛЬНАЯ ВЕРСИЯ)
  ******************************************************************************
*/
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ---------------- НАСТРОЙКИ ---------------- */

#define VOLTAGE_DIVIDER_RATIO   1.37f
#define PRESSURE_MIN_VOLTAGE    0.5f
#define PRESSURE_MAX_VOLTAGE    4.5f
#define PRESSURE_MIN_VALUE      0.0f
#define PRESSURE_MAX_VALUE      2.06843f

#define TEMP_SENSOR_VREF        3.3f
#define TEMP_VOLTAGE_AT_0C      0.5f

#define VREF_VOLTAGE            3.3f
#define ADC_RESOLUTION          4095.0f
#define MEASUREMENT_PERIOD_MS   100

#define WATER_DENSITY           998.2071f
#define GRAVITY                 9.80665f

#define CALIBRATION_SAMPLES     1000

/* ---------------- ПЕРЕМЕННЫЕ ---------------- */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
UART_HandleTypeDef huart1;

volatile uint16_t pressure_adc_value = 0;
volatile uint16_t temp_adc_value = 0;

volatile float depth_meters = 0;
volatile float temperature_c = 0;
volatile float surface_pressure_pa = 0;
volatile float pressure_pa = 0;

volatile uint8_t calibration_done = 0;

/* ---------------- ПРОТОТИПЫ ---------------- */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);

void Read_Pressure_Sensor(void);
void Read_Temperature_Sensor(void);
float Calculate_Pressure_Pa(uint16_t adc);
float Calculate_Temperature(uint16_t adc);
float Calculate_Depth(float pressure);
void Send_Data_To_UART(void);
void Auto_Calibrate_Surface_Pressure(void);

void UART_Print(const char* msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/* ---------------- MAIN ---------------- */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_USART1_UART_Init();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc2);

    UART_Print("\r\n=== Depth + Temp Stable ===\r\n");
    UART_Print("Auto calibration...\r\n");

    Auto_Calibrate_Surface_Pressure();

    UART_Print("Calibration OK\r\n");

    uint32_t last = HAL_GetTick();

    while (1)
    {
        if (HAL_GetTick() - last >= MEASUREMENT_PERIOD_MS)
        {
            last = HAL_GetTick();

            /* Давление сначала */
            Read_Pressure_Sensor();

            /* Температуру читаем 2 раза — берем второе значение */
            Read_Temperature_Sensor();
            Read_Temperature_Sensor();

            pressure_pa = Calculate_Pressure_Pa(pressure_adc_value);
            temperature_c = Calculate_Temperature(temp_adc_value);
            depth_meters = Calculate_Depth(pressure_pa);

            Send_Data_To_UART();
        }
    }
}

/* ---------------- КАЛИБРОВКА ---------------- */

void Auto_Calibrate_Surface_Pressure(void)
{
    float sum = 0;

    for(int i=0;i<CALIBRATION_SAMPLES;i++)
    {
        Read_Pressure_Sensor();
        sum += Calculate_Pressure_Pa(pressure_adc_value);
        HAL_Delay(50);
    }

    surface_pressure_pa = sum / CALIBRATION_SAMPLES;
    calibration_done = 1;
}

/* ---------------- ЧТЕНИЕ ДАТЧИКОВ ---------------- */

void Read_Pressure_Sensor(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    pressure_adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
}

void Read_Temperature_Sensor(void)
{
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    temp_adc_value = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
}

/* ---------------- РАСЧЁТЫ ---------------- */

float Calculate_Pressure_Pa(uint16_t adc)
{
    float v = (adc / ADC_RESOLUTION) * VREF_VOLTAGE;
    float sensor_v = v * VOLTAGE_DIVIDER_RATIO;

    if(sensor_v < PRESSURE_MIN_VOLTAGE) sensor_v = PRESSURE_MIN_VOLTAGE;
    if(sensor_v > PRESSURE_MAX_VOLTAGE) sensor_v = PRESSURE_MAX_VOLTAGE;

    float bar = (sensor_v - PRESSURE_MIN_VOLTAGE) *
                (PRESSURE_MAX_VALUE / (PRESSURE_MAX_VOLTAGE - PRESSURE_MIN_VOLTAGE));

    return bar * 100000.0f;
}

float Calculate_Temperature(uint16_t adc)
{
    float v = (adc / ADC_RESOLUTION) * TEMP_SENSOR_VREF;
    float t = (v - TEMP_VOLTAGE_AT_0C) * 100.0f;

    if(t < -40) t = -40;
    if(t > 125) t = 125;

    return t;
}

float Calculate_Depth(float pressure)
{
    if(!calibration_done) return 0;

    float dp = pressure - surface_pressure_pa;

    if(fabsf(dp) < 100) return 0;

    return dp / (WATER_DENSITY * GRAVITY);
}

/* ---------------- UART ---------------- */

void Send_Data_To_UART(void)
{
    char buf[128];
    snprintf(buf,sizeof(buf),
            "D=%.3f Pa=%.3f T=%.1f\r\n",
            depth_meters,
			pressure_pa,
            temperature_c);
    UART_Print(buf);
}

/* ---------------- CLOCK ---------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct={0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct={0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit={0};

    RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState=RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue=RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState=RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL=RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType=
        RCC_CLOCKTYPE_HCLK|
        RCC_CLOCKTYPE_SYSCLK|
        RCC_CLOCKTYPE_PCLK1|
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider=RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider=RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider=RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2);

    /* ВАЖНО — делитель ADC */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/* ---------------- ADC ---------------- */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig={0};

    hadc1.Instance=ADC1;
    hadc1.Init.ScanConvMode=ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode=DISABLE;
    hadc1.Init.ExternalTrigConv=ADC_SOFTWARE_START;
    hadc1.Init.DataAlign=ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion=1;

    HAL_ADC_Init(&hadc1);

    sConfig.Channel=ADC_CHANNEL_0;
    sConfig.Rank=ADC_REGULAR_RANK_1;
    sConfig.SamplingTime=ADC_SAMPLETIME_239CYCLES_5;   // ВАЖНО
    HAL_ADC_ConfigChannel(&hadc1,&sConfig);
}

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig={0};

    hadc2.Instance=ADC2;
    hadc2.Init.ScanConvMode=ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode=DISABLE;
    hadc2.Init.ExternalTrigConv=ADC_SOFTWARE_START;
    hadc2.Init.DataAlign=ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion=1;

    HAL_ADC_Init(&hadc2);

    sConfig.Channel=ADC_CHANNEL_2;
    sConfig.Rank=ADC_REGULAR_RANK_1;
    sConfig.SamplingTime=ADC_SAMPLETIME_239CYCLES_5;   // ВАЖНО
    HAL_ADC_ConfigChannel(&hadc2,&sConfig);
}

/* ---------------- UART ---------------- */

static void MX_USART1_UART_Init(void)
{
    huart1.Instance=USART1;
    huart1.Init.BaudRate=115200;
    huart1.Init.WordLength=UART_WORDLENGTH_8B;
    huart1.Init.StopBits=UART_STOPBITS_1;
    huart1.Init.Parity=UART_PARITY_NONE;
    huart1.Init.Mode=UART_MODE_TX_RX;
    HAL_UART_Init(&huart1);
}

/* ---------------- GPIO ---------------- */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct={0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);

    GPIO_InitStruct.Pin=GPIO_PIN_8;
    GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);

    GPIO_InitStruct.Pin=GPIO_PIN_0|GPIO_PIN_2;
    GPIO_InitStruct.Mode=GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
}

/* ---------------- ERROR ---------------- */

void Error_Handler(void)
{
    while(1)
    {
        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
        HAL_Delay(100);
    }
}
