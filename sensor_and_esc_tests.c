/*
 * STM32F411CE (Black Pill) - bare-metal drone firmware bring-up, STM32CubeIDE + HAL.
 * These are the USER CODE sections added to the CubeMX-generated main.c.
 *
 * Peripherals configured in CubeMX:
 *   I2C1  PB6/PB7      MPU9250
 *   TIM2  PA0 (TRIG GPIO) / PA1 (ECHO, TIM2_CH2)   HC-SR04, 1 MHz timer
 *   TIM3  PB0/PB1/PA6/PA7 (CH3/CH4/CH1/CH2)        4 ESCs, 50 Hz PWM, 1 tick = 1 us
 *   USART2 PA2/PA3     GPS
 *   PB14  input        signal from ESP32-CAM (marker detected)
 *   USB   CDC          debug output to PC (PuTTY, 115200)
 */

/* ---------- 1. Check the IMU is alive: WHO_AM_I should return 0x71 ---------- */
uint8_t check;
HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x75, 1, &check, 1, 100);
char msg[50];
sprintf(msg, "WHO_AM_I = 0x%02X\r\n", check);
HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

/* ---------- 2. HC-SR04 distance (echo pulse width measured with TIM2) ---------- */
uint32_t measure_distance(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);      // trigger pulse
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET);   // wait for echo high
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET);     // time echo high
    HAL_TIM_Base_Stop(&htim2);

    uint32_t count = __HAL_TIM_GET_COUNTER(&htim2);
    return count / 58;                                       // us -> cm
}

/* ---------- 3. Main loop: raw IMU + distance streamed over USB ---------- */
while (1)
{
    uint8_t data[14];
    HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B, 1, data, 14, 100);   // accel, temp, gyro in one burst
    int16_t ax = (data[0]  << 8 | data[1]);
    int16_t ay = (data[2]  << 8 | data[3]);
    int16_t az = (data[4]  << 8 | data[5]);
    int16_t gx = (data[8]  << 8 | data[9]);
    int16_t gy = (data[10] << 8 | data[11]);
    int16_t gz = (data[12] << 8 | data[13]);

    uint32_t dist = measure_distance();

    char msg[100];
    sprintf(msg, "AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d DIST:%lucm\r\n", ax, ay, az, gx, gy, gz, dist);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(100);
}

/* ---------- 4. ESC test: PWM in microseconds on TIM3 ---------- */
void set_pwm(uint32_t pulse_us)
{
    // 16 MHz / (prescaler 15 + 1) = 1 MHz -> 1 timer tick = 1 us
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_TIM3_Init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    set_pwm(1000);           // minimum throttle so the ESC arms; power the ESC after this starts
    HAL_Delay(5000);

    for (uint32_t i = 1000; i <= 1500; i += 10) {   // slow ramp up
        set_pwm(i);
        HAL_Delay(100);
    }
    while (1) { HAL_Delay(1000); }
}
