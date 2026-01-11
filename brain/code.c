#include "stm32h7xx_hal.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

#define PI 3.14159265359f
#define DEG2RAD 0.01745329251f
#define RAD2DEG 57.2957795131f
#define G 9.81f

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1, hspi2;
UART_HandleTypeDef huart1, huart2, huart3, huart4, huart5;
TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim8;

typedef struct {
    float x, y, z;
} Vec3;

typedef struct {
    float w, x, y, z;
} Quat;

typedef struct {
    float roll, pitch, yaw;
} Euler;

typedef struct {
    float p, i, d;
    float err_prev;
    float integ;
    float out_min, out_max;
} PID;

typedef struct {
    float x[2];
    float P[2][2];
    float Q[2][2];
    float R;
} KalmanAlt;

typedef struct {
    Vec3 acc;
    Vec3 gyro;
    Vec3 mag;
    float pres;
    float temp;
    float alt;
    float alt_ms5611;
    float lat, lon;
    float gps_alt;
    uint8_t gps_fix;
} SensorData;

typedef struct {
    Quat q;
    Euler ang;
    float alt;
    float vel;
    float acc_z;
} State;

typedef struct {
    float fin[4];
    float motor[4];
} Actuators;

typedef enum {
    IDLE = 0,
    ARMED,
    BOOST,
    COAST,
    APOGEE,
    DESCENT,
    LANDED
} FlightPhase;

SensorData sens;
State state;
Actuators act;
FlightPhase phase = IDLE;
PID pid_roll, pid_pitch, pid_yaw;
KalmanAlt kf;

uint32_t t_launch = 0;
uint32_t t_apogee = 0;
float max_alt = 0;
float target_lat = 0, target_lon = 0;
uint8_t flash_buf[256];
uint32_t flash_addr = 0;

void Error_Handler(void) {
    __disable_irq();
    while(1) {}
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 120;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_0|GPIO_PIN_1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}

void I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10C0ECFF;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

void SPI1_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if(HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

void SPI2_Init(void) {
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if(HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
}

void UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
    
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart4) != HAL_OK) Error_Handler();
}

void TIM_Init(void) {
    TIM_Encoder_InitTypeDef enc_cfg = {0};
    TIM_MasterConfigTypeDef mst_cfg = {0};
    
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    enc_cfg.EncoderMode = TIM_ENCODERMODE_TI12;
    enc_cfg.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc_cfg.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc_cfg.IC1Prescaler = TIM_ICPSC_DIV1;
    enc_cfg.IC1Filter = 0;
    enc_cfg.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc_cfg.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc_cfg.IC2Prescaler = TIM_ICPSC_DIV1;
    enc_cfg.IC2Filter = 0;
    if(HAL_TIM_Encoder_Init(&htim1, &enc_cfg) != HAL_OK) Error_Handler();
    
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Encoder_Init(&htim2, &enc_cfg) != HAL_OK) Error_Handler();
    
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Encoder_Init(&htim4, &enc_cfg) != HAL_OK) Error_Handler();
    
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

void PID_Init(PID *pid, float kp, float ki, float kd, float min, float max) {
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    pid->err_prev = 0;
    pid->integ = 0;
    pid->out_min = min;
    pid->out_max = max;
}

float PID_Compute(PID *pid, float sp, float pv, float dt) {
    float err = sp - pv;
    pid->integ += err * dt;
    
    if(pid->integ > pid->out_max) pid->integ = pid->out_max;
    if(pid->integ < pid->out_min) pid->integ = pid->out_min;
    
    float deriv = (err - pid->err_prev) / dt;
    pid->err_prev = err;
    
    float out = pid->p * err + pid->i * pid->integ + pid->d * deriv;
    
    if(out > pid->out_max) out = pid->out_max;
    if(out < pid->out_min) out = pid->out_min;
    
    return out;
}

void Kalman_Init(KalmanAlt *k) {
    k->x[0] = 0;
    k->x[1] = 0;
    k->P[0][0] = 1; k->P[0][1] = 0;
    k->P[1][0] = 0; k->P[1][1] = 1;
    k->Q[0][0] = 0.1; k->Q[0][1] = 0;
    k->Q[1][0] = 0; k->Q[1][1] = 0.1;
    k->R = 2.0;
}

void Kalman_Predict(KalmanAlt *k, float acc, float dt) {
    float x0 = k->x[0] + k->x[1] * dt + 0.5f * acc * dt * dt;
    float x1 = k->x[1] + acc * dt;
    
    float P00 = k->P[0][0] + dt * (k->P[1][0] + k->P[0][1]) + dt*dt*k->P[1][1] + k->Q[0][0];
    float P01 = k->P[0][1] + dt * k->P[1][1] + k->Q[0][1];
    float P10 = k->P[1][0] + dt * k->P[1][1] + k->Q[1][0];
    float P11 = k->P[1][1] + k->Q[1][1];
    
    k->x[0] = x0;
    k->x[1] = x1;
    k->P[0][0] = P00;
    k->P[0][1] = P01;
    k->P[1][0] = P10;
    k->P[1][1] = P11;
}

void Kalman_Update(KalmanAlt *k, float z) {
    float y = z - k->x[0];
    float S = k->P[0][0] + k->R;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;
    
    k->x[0] += K0 * y;
    k->x[1] += K1 * y;
    
    float P00 = (1 - K0) * k->P[0][0];
    float P01 = (1 - K0) * k->P[0][1];
    float P10 = k->P[1][0] - K1 * k->P[0][0];
    float P11 = k->P[1][1] - K1 * k->P[0][1];
    
    k->P[0][0] = P00;
    k->P[0][1] = P01;
    k->P[1][0] = P10;
    k->P[1][1] = P11;
}

void BMI088_Init(void) {
    uint8_t tx[2], rx[2];
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    tx[0] = 0x7D; tx[1] = 0x04;
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(50);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    tx[0] = 0x7C; tx[1] = 0x00;
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    tx[0] = 0x40; tx[1] = 0x0A;
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    tx[0] = 0x0F; tx[1] = 0x00;
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    tx[0] = 0x10; tx[1] = 0x00;
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}

void BMI088_Read(Vec3 *acc, Vec3 *gyro) {
    uint8_t tx[8], rx[8];
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    tx[0] = 0x12 | 0x80;
    HAL_SPI_Transmit(&hspi2, tx, 1, 100);
    HAL_SPI_Receive(&hspi2, rx, 7, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    
    int16_t ax = (int16_t)((rx[2] << 8) | rx[1]);
    int16_t ay = (int16_t)((rx[4] << 8) | rx[3]);
    int16_t az = (int16_t)((rx[6] << 8) | rx[5]);
    
    acc->x = ax * (24.0f * G / 32768.0f);
    acc->y = ay * (24.0f * G / 32768.0f);
    acc->z = az * (24.0f * G / 32768.0f);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    tx[0] = 0x02 | 0x80;
    HAL_SPI_Transmit(&hspi2, tx, 1, 100);
    HAL_SPI_Receive(&hspi2, rx, 6, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    
    int16_t gx = (int16_t)((rx[1] << 8) | rx[0]);
    int16_t gy = (int16_t)((rx[3] << 8) | rx[2]);
    int16_t gz = (int16_t)((rx[5] << 8) | rx[4]);
    
    gyro->x = gx * (2000.0f * DEG2RAD / 32768.0f);
    gyro->y = gy * (2000.0f * DEG2RAD / 32768.0f);
    gyro->z = gz * (2000.0f * DEG2RAD / 32768.0f);
}

void LIS3MDL_Init(void) {
    uint8_t cfg[2];
    cfg[0] = 0x20; cfg[1] = 0x70;
    HAL_I2C_Master_Transmit(&hi2c1, 0x1C << 1, cfg, 2, 100);
    cfg[0] = 0x21; cfg[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, 0x1C << 1, cfg, 2, 100);
    cfg[0] = 0x22; cfg[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, 0x1C << 1, cfg, 2, 100);
    cfg[0] = 0x23; cfg[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, 0x1C << 1, cfg, 2, 100);
}

void LIS3MDL_Read(Vec3 *mag) {
    uint8_t reg = 0x28 | 0x80;
    uint8_t data[6];
    HAL_I2C_Master_Transmit(&hi2c1, 0x1C << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0x1C << 1, data, 6, 100);
    
    int16_t mx = (int16_t)((data[1] << 8) | data[0]);
    int16_t my = (int16_t)((data[3] << 8) | data[2]);
    int16_t mz = (int16_t)((data[5] << 8) | data[4]);
    
    mag->x = mx * 0.00014615f;
    mag->y = my * 0.00014615f;
    mag->z = mz * 0.00014615f;
}

void BMP388_Init(void) {
    uint8_t cfg[2];
    cfg[0] = 0x1B; cfg[1] = 0x33;
    HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, cfg, 2, 100);
    cfg[0] = 0x1C; cfg[1] = 0x05;
    HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, cfg, 2, 100);
    cfg[0] = 0x1D; cfg[1] = 0x01;
    HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, cfg, 2, 100);
}

float BMP388_Read(void) {
    uint8_t reg = 0x04;
    uint8_t data[3];
    HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0x76 << 1, data, 3, 100);
    
    uint32_t raw = (data[2] << 16) | (data[1] << 8) | data[0];
    float pres = raw / 100.0f;
    float alt = 44330.0f * (1.0f - powf(pres / 1013.25f, 0.1903f));
    return alt;
}

void TMP102_Init(void) {
    uint8_t cfg[3] = {0x01, 0x60, 0xA0};
    HAL_I2C_Master_Transmit(&hi2c1, 0x48 << 1, cfg, 3, 100);
}

float TMP102_Read(void) {
    uint8_t reg = 0x00;
    uint8_t data[2];
    HAL_I2C_Master_Transmit(&hi2c1, 0x48 << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0x48 << 1, data, 2, 100);
    
    int16_t raw = (data[0] << 4) | (data[1] >> 4);
    if(raw > 2047) raw -= 4096;
    return raw * 0.0625f;
}

void MS5611_Init(void) {
    uint8_t cmd = 0x1E;
    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &cmd, 1, 100);
    HAL_Delay(10);
}

float MS5611_Read(void) {
    uint8_t cmd = 0x48;
    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &cmd, 1, 100);
    HAL_Delay(10);
    
    cmd = 0x00;
    uint8_t data[3];
    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &cmd, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0x77 << 1, data, 3, 100);
    
    uint32_t D1 = (data[0] << 16) | (data[1] << 8) | data[2];
    
    cmd = 0x58;
    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &cmd, 1, 100);
    HAL_Delay(10);
    
    cmd = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &cmd, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0x77 << 1, data, 3, 100);
    
    uint32_t D2 = (data[0] << 16) | (data[1] << 8) | data[2];
    
    int32_t dT = D2 - (6465536);
    int64_t OFF = 4311040 * 65536LL + (dT * 188743LL) / 128LL;
    int64_t SENS = 2154176 * 32768LL + (dT * 136329LL) / 256LL;
    int32_t P = ((D1 * SENS / 2097152LL) - OFF) / 32768LL;
    
    float pres = P / 100.0f;
    float alt = 44330.0f * (1.0f - powf(pres / 1013.25f, 0.1903f));
    return alt;
}

void Flash_Write(uint32_t addr, uint8_t *data, uint16_t len) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    uint8_t cmd[4];
    cmd[0] = 0x02;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_SPI_Transmit(&hspi1, data, len, 1000);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}

void GPS_Parse(uint8_t *buf, uint16_t len) {
    for(uint16_t i = 0; i < len - 6; i++) {
        if(buf[i] == '$' && buf[i+3] == 'G' && buf[i+4] == 'G' && buf[i+5] == 'A') {
            char *tok = strtok((char*)&buf[i], ",");
            int fld = 0;
            while(tok != NULL) {
                if(fld == 2) sens.lat = atof(tok) / 100.0f;
                if(fld == 4) sens.lon = atof(tok) / 100.0f;
                if(fld == 9) sens.gps_alt = atof(tok);
                if(fld == 6) sens.gps_fix = atoi(tok);
                tok = strtok(NULL, ",");
                fld++;
            }
            break;
        }
    }
}

void Quat_FromEuler(Quat *q, Euler *e) {
    float cr = cosf(e->roll * 0.5f);
    float sr = sinf(e->roll * 0.5f);
    float cp = cosf(e->pitch * 0.5f);
    float sp = sinf(e->pitch * 0.5f);
    float cy = cosf(e->yaw * 0.5f);
    float sy = sinf(e->yaw * 0.5f);
    
    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

void Quat_ToEuler(Quat *q, Euler *e) {
    float sinr = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    e->roll = atan2f(sinr, cosr);
    
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if(fabsf(sinp) >= 1) e->pitch = copysignf(PI / 2, sinp);
    else e->pitch = asinf(sinp);
    
    float siny = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    e->yaw = atan2f(siny, cosy);
}

void Quat_Integrate(Quat *q, Vec3 *w, float dt) {
    float mag = sqrtf(w->x*w->x + w->y*w->y + w->z*w->z);
    if(mag < 0.001f) return;
    
    float ha = mag * dt * 0.5f;
    float s = sinf(ha) / mag;
    
    Quat dq;
    dq.w = cosf(ha);
    dq.x = w->x * s;
    dq.y = w->y * s;
    dq.z = w->z * s;
    
    Quat qn;
    qn.w = q->w * dq.w - q->x * dq.x - q->y * dq.y - q->z * dq.z;
    qn.x = q->w * dq.x + q->x * dq.w + q->y * dq.z - q->z * dq.y;
    qn.y = q->w * dq.y - q->x * dq.z + q->y * dq.w + q->z * dq.x;
    qn.z = q->w * dq.z + q->x * dq.y - q->y * dq.x + q->z * dq.w;
    
    float norm = sqrtf(qn.w*qn.w + qn.x*qn.x + qn.y*qn.y + qn.z*qn.z);
    q->w = qn.w / norm;
    q->x = qn.x / norm;
    q->y = qn.y / norm;
    q->z = qn.z / norm;
}

void Attitude_Update(float dt) {
    Quat_Integrate(&state.q, &sens.gyro, dt);
    
    Vec3 acc_norm;
    float acc_mag = sqrtf(sens.acc.x*sens.acc.x + sens.acc.y*sens.acc.y + sens.acc.z*sens.acc.z);
    if(acc_mag > 0.1f) {
        acc_norm.x = sens.acc.x / acc_mag;
        acc_norm.y = sens.acc.y / acc_mag;
        acc_norm.z = sens.acc.z / acc_mag;
        
        float roll_acc = atan2f(acc_norm.y, acc_norm.z);
        float pitch_acc = atan2f(-acc_norm.x, sqrtf(acc_norm.y*acc_norm.y + acc_norm.z*acc_norm.z));
        
        Euler e_gyro;
        Quat_ToEuler(&state.q, &e_gyro);
        
        float alpha = 0.98f;
        e_gyro.roll = alpha * e_gyro.roll + (1.0f - alpha) * roll_acc;
        e_gyro.pitch = alpha * e_gyro.pitch + (1.0f - alpha) * pitch_acc;
        
        Quat_FromEuler(&state.q, &e_gyro);
    }
    
    Quat_ToEuler(&state.q, &state.ang);
}

void Servo_Send(uint8_t id, float angle) {
    uint8_t buf[10];
    int16_t pos = (int16_t)((angle + 150.0f) * 4096.0f / 300.0f);
    if(pos < 0) pos = 0;
    if(pos > 4095) pos = 4095;
    
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 7;
    buf[4] = 0x03;
    buf[5] = 0x2A;
    buf[6] = pos & 0xFF;
    buf[7] = (pos >> 8) & 0xFF;
    buf[8] = 0x00;
    buf[9] = 0x00;
    
    uint8_t sum = 0;
    for(int i = 2; i < 9; i++) sum += buf[i];
    buf[9] = ~sum;
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart4, buf, 10, 100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
}

void Motor_Set(uint8_t motor, int8_t spd) {
    if(motor == 0) {
        if(spd > 0) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        } else if(spd < 0) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        }
    } else if(motor == 1) {
        if(spd > 0) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
        } else if(spd < 0) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
        }
    }
}

int32_t Encoder_Read(uint8_t enc) {
    if(enc == 0) return (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
    if(enc == 1) return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    if(enc == 2) return (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    return 0;
}

void Telemetry_Send(void) {
    char buf[128];
    snprintf(buf, 128, "A:%.1f,V:%.1f,R:%.1f,P:%.1f,Y:%.1f,PH:%d\n",
             state.alt, state.vel, state.ang.roll*RAD2DEG, 
             state.ang.pitch*RAD2DEG, state.ang.yaw*RAD2DEG, phase);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
}

void State_Machine(void) {
    static uint32_t t_coast = 0;
    
    switch(phase) {
        case IDLE:
            if(sens.acc.z > 20.0f) {
                phase = ARMED;
                t_launch = HAL_GetTick();
            }
            break;
            
        case ARMED:
            if(sens.acc.z > 30.0f) {
                phase = BOOST;
                t_launch = HAL_GetTick();
            }
            break;
            
        case BOOST:
            if(sens.acc.z < 5.0f) {
                phase = COAST;
                t_coast = HAL_GetTick();
            }
            if(HAL_GetTick() - t_launch > 5000) {
                phase = COAST;
                t_coast = HAL_GetTick();
            }
            break;
            
        case COAST:
            if(state.vel < -1.0f || (HAL_GetTick() - t_coast > 3000 && state.alt > max_alt - 5.0f)) {
                phase = APOGEE;
                t_apogee = HAL_GetTick();
                max_alt = state.alt;
            }
            if(state.alt > max_alt) max_alt = state.alt;
            break;
            
        case APOGEE:
            if(HAL_GetTick() - t_apogee > 1000) {
                phase = DESCENT;
            }
            break;
            
        case DESCENT:
            if(state.alt < 10.0f && fabsf(state.vel) < 2.0f) {
                phase = LANDED;
            }
            break;
            
        case LANDED:
            break;
    }
}

void Control_Fins(float dt) {
    if(phase != BOOST && phase != COAST) {
        act.fin[0] = 0;
        act.fin[1] = 0;
        act.fin[2] = 0;
        act.fin[3] = 0;
        return;
    }
    
    float roll_cmd = PID_Compute(&pid_roll, 0, sens.gyro.x, dt);
    float pitch_cmd = PID_Compute(&pid_pitch, 0, sens.gyro.y, dt);
    float yaw_cmd = PID_Compute(&pid_yaw, 0, sens.gyro.z, dt);
    
    act.fin[0] = pitch_cmd + yaw_cmd;
    act.fin[1] = -pitch_cmd + yaw_cmd;
    act.fin[2] = pitch_cmd - yaw_cmd;
    act.fin[3] = -pitch_cmd - yaw_cmd;
    
    for(int i = 0; i < 4; i++) {
        if(act.fin[i] > 30.0f) act.fin[i] = 30.0f;
        if(act.fin[i] < -30.0f) act.fin[i] = -30.0f;
    }
}

void Control_Parachute(void) {
    if(phase != DESCENT || sens.gps_fix < 1) {
        Motor_Set(0, 0);
        Motor_Set(1, 0);
        return;
    }
    
    float dlat = target_lat - sens.lat;
    float dlon = target_lon - sens.lon;
    float dist = sqrtf(dlat*dlat + dlon*dlon) * 111139.0f;
    float bearing = atan2f(dlon, dlat);
    
    float err_bearing = bearing - state.ang.yaw;
    while(err_bearing > PI) err_bearing -= 2*PI;
    while(err_bearing < -PI) err_bearing += 2*PI;
    
    int8_t spd = (int8_t)(err_bearing * 50.0f);
    if(spd > 100) spd = 100;
    if(spd < -100) spd = -100;
    
    if(dist < 5.0f) spd = 0;
    
    Motor_Set(0, spd);
    Motor_Set(1, spd);
}

void Log_Data(void) {
    uint8_t log[64];
    uint32_t t = HAL_GetTick();
    
    memcpy(&log[0], &t, 4);
    memcpy(&log[4], &state.alt, 4);
    memcpy(&log[8], &state.vel, 4);
    memcpy(&log[12], &sens.acc.x, 4);
    memcpy(&log[16], &sens.acc.y, 4);
    memcpy(&log[20], &sens.acc.z, 4);
    memcpy(&log[24], &sens.gyro.x, 4);
    memcpy(&log[28], &sens.gyro.y, 4);
    memcpy(&log[32], &sens.gyro.z, 4);
    memcpy(&log[36], &state.ang.roll, 4);
    memcpy(&log[40], &state.ang.pitch, 4);
    memcpy(&log[44], &state.ang.yaw, 4);
    log[48] = phase;
    
    Flash_Write(flash_addr, log, 64);
    flash_addr += 64;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    I2C1_Init();
    SPI1_Init();
    SPI2_Init();
    UART_Init();
    TIM_Init();
    
    HAL_Delay(100);
    
    BMI088_Init();
    LIS3MDL_Init();
    BMP388_Init();
    TMP102_Init();
    MS5611_Init();
    
    PID_Init(&pid_roll, 0.8f, 0.0f, 0.05f, -30.0f, 30.0f);
    PID_Init(&pid_pitch, 0.8f, 0.0f, 0.05f, -30.0f, 30.0f);
    PID_Init(&pid_yaw, 0.5f, 0.0f, 0.03f, -30.0f, 30.0f);
    
    Kalman_Init(&kf);
    
    state.q.w = 1.0f;
    state.q.x = 0.0f;
    state.q.y = 0.0f;
    state.q.z = 0.0f;
    
    target_lat = 12.9716f;
    target_lon = 77.5946f;
    
    uint32_t t_prev = HAL_GetTick();
    uint32_t t_telem = 0;
    uint8_t gps_buf[128];
    
    while(1) {
        uint32_t t_now = HAL_GetTick();
        float dt = (t_now - t_prev) / 1000.0f;
        if(dt > 0.1f) dt = 0.1f;
        t_prev = t_now;
        
        BMI088_Read(&sens.acc, &sens.gyro);
        LIS3MDL_Read(&sens.mag);
        sens.alt = BMP388_Read();
        sens.temp = TMP102_Read();
        sens.alt_ms5611 = MS5611_Read();
        
        if(HAL_UART_Receive(&huart1, gps_buf, 128, 10) == HAL_OK) {
            GPS_Parse(gps_buf, 128);
        }
        
        Attitude_Update(dt);
        
        float acc_world = sens.acc.z - G;
        Kalman_Predict(&kf, acc_world, dt);
        Kalman_Update(&kf, sens.alt_ms5611);
        
        state.alt = kf.x[0];
        state.vel = kf.x[1];
        state.acc_z = acc_world;
        
        State_Machine();
        
        Control_Fins(dt);
        Control_Parachute();
        
        for(int i = 0; i < 4; i++) {
            Servo_Send(i + 1, act.fin[i]);
        }
        
        Log_Data();
        
        if(t_now - t_telem > 500) {
            Telemetry_Send();
            t_telem = t_now;
        }
        
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);
        
        HAL_Delay(5);
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
    if(hi2c->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hspi->Instance == SPI1) {
        __HAL_RCC_SPI1_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    if(hspi->Instance == SPI2) {
        __HAL_RCC_SPI2_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    if(huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
    if(huart->Instance == UART4) {
        __HAL_RCC_UART4_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    }
    if(htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    if(htim->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}
