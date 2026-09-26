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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <mpu6050.h>
#include "motor.h"
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
#include "NRF24_conf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    uint16_t header;          // 0xAA55
    int16_t vx;
    int16_t vy;
    int16_t wz;
    float target_yaw_deg;     // NEW: yaw setpoint from Pi
    uint8_t mode;             // 0 manual, 1 auto
    uint8_t checksum;
} PiCommandFrame;

typedef struct __attribute__((packed)) {
    uint16_t header;      // 0x55AA

    float rpm_FL;
    float rpm_FR;
    float rpm_RL;
    float rpm_RR;

    float yaw_angle;
    float gyro_z_dps;

    float ultra_left_cm;    // USART1
    float ultra_right_cm;   // USART3
    float ultra_front_cm;   // UART4

    float accel_x;
    float accel_y;
    float accel_z;

    float dt;
    float pid_hz;

    int16_t pwm_FL;
    int16_t pwm_FR;
    int16_t pwm_RL;
    int16_t pwm_RR;

    uint16_t change_value;

    uint8_t imu_ok;
    uint8_t status;

    uint8_t checksum;
} StmTelemetryFrame;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

float yaw_rate_measured = 0.0f;
float target_yaw = 0.0f;
float yaw_correction = 0.0f;

#define YAW_LIMIT_DEG        3.0f

// Rotate mode
#define ROT_YAW_KP           80.0f
#define ROT_YAW_KD           0.0f
#define ROT_YAW_BASE_PWM     2000.0f
#define ROT_YAW_CORR_MAX     5000.0f

// Drive straight mode
#define DRIVE_YAW_KP           70.0f
#define DRIVE_YAW_KD           0.0f
#define DRIVE_YAW_BASE_PWM     1500.0f
#define DRIVE_YAW_CORR_MAX     5000.0f

// Modes
#define MODE_ROTATE_YAW      1
#define MODE_DRIVE_HEADING   2

#define JOY_MIN        0
#define JOY_MAX        4050

#define JOY_CENTER     1975
#define JOY_DEADZONE   75

#define JOY_CENTER_LOW   (JOY_CENTER - JOY_DEADZONE)   // 1900
#define JOY_CENTER_HIGH  (JOY_CENTER + JOY_DEADZONE)   // 2050
#define PWM_MAX  8399
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

volatile uint8_t imu_recovery_request = 0;
volatile uint32_t imu_last_recovery_try = 0;
volatile uint32_t imu_last_data_tick = 0;   // updated whenever fresh IMU data arrives
uint8_t ultra1_cmd = 0x01;
uint8_t ultra3_cmd = 0x01;
uint8_t ultra4_cmd = 0x01;

uint8_t ultra1_rx_buf[3];
uint8_t ultra3_rx_buf[3];
uint8_t ultra4_rx_buf[3];

volatile uint8_t ultra1_ready = 0;
volatile uint8_t ultra3_ready = 0;
volatile uint8_t ultra4_ready = 0;

uint16_t ultra1_distance_mm = 0;
uint16_t ultra3_distance_mm = 0;
uint16_t ultra4_distance_mm = 0;

float ultra1_distance_cm = 0.0f;
float ultra3_distance_cm = 0.0f;
float ultra4_distance_cm = 0.0f;

    int32_t flR ;
    int32_t frR ;
    int32_t rlR ;
    int32_t rrR ;

int32_t fl;
int32_t fr ;
int32_t rl ;
int32_t rr;



PiCommandFrame pi_cmd;
StmTelemetryFrame stm_tel;

volatile uint8_t uart_rx_ready = 0;
volatile uint8_t uart_tx_complete = 1;

uint8_t rx_buffer[sizeof(PiCommandFrame)];

float gyro_z_bias = 0.0f;   // measured at startup
/////////////////////////////////NRF////////////////////////
#define PLD_S 12
#define TX_DS   5   // Transmit Data Sent
#define MAX_RT  4   // Maximum Retransmits
uint8_t rx_ack_pld[PLD_S] = {"OK"};
uint8_t tx_addr[5] = {0x45, 0x55, 0x67, 0x10, 0x21};


uint16_t counter =0 ;
uint16_t data = 0;
uint8_t dataR[PLD_S];
uint8_t dataT[PLD_S];
volatile uint8_t irq = 0;
uint16_t dataa = 0;


float gyro_z_p;
float gyro_z_d;
float yaw_angle = 0.0f;
/* ── IMU / Yaw filtering (from drone code) ── */
float gyro_z_p_state = 0.0f;   // light filter → P and I
float gyro_z_d_state = 0.0f;   // heavy filter → D



#define RPM_SAMPLE_TICKS  10   // compute RPM every 10 ticks → ~20ms window

static int32_t acc_FL = 0, acc_FR = 0, acc_RL = 0, acc_RR = 0;
static uint8_t rpm_tick_count = 0;
static float   acc_dt = 0.0f;


int16_t motor_pwm;
//........................................MOTORS...................................................
uint8_t imu_ok = 0;

Motor_t motor_FL;
Motor_t motor_FR;
Motor_t motor_RL;
Motor_t motor_RR;

#define ENC_COUNTS_PER_REV   1320.0f    // ← CHANGE TO YOUR REAL VALUE
//..............................imu config ..........................
uint8_t mpu_raw_data[14];
volatile uint8_t mpu_data_ready = 0;
MPU6050_Data mpuData;
volatile uint8_t nrf_data_ready = 0;
//.................................. timer calculations .......................
static uint32_t  dwt_last = 0;
static float     dt       = 0.000f;     // seconds
static float     pid_hz   = 0.0f;       // what you want to monitor

uint16_t receivedY;
uint16_t receivedX;
uint16_t servo_pwma2;
uint16_t receivedYa;
uint16_t receivedXa;
uint16_t change;
uint16_t change2;

int16_t pwm_FL, pwm_FR, pwm_RL, pwm_RR;
//.....................................

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t calc_checksum(uint8_t *data, uint16_t len);
void process_pi_command(void);
void send_telemetry_to_pi(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t calc_checksum(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for(uint16_t i = 0; i < len - 1; i++)
        sum += data[i];
    return sum;
}

float wrap_angle(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

#define I2C1_SCL_PORT   GPIOB
#define I2C1_SCL_PIN    GPIO_PIN_6      // <-- verify against MspInit
#define I2C1_SDA_PORT   GPIOB
#define I2C1_SDA_PIN    GPIO_PIN_7      // <-- verify against MspInit

static void i2c_short_delay(void)
{
    for (volatile int i = 0; i < 800; i++) __NOP();
}

void I2C1_BusRecovery(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_I2C_DeInit(&hi2c1);

    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = I2C1_SCL_PIN; HAL_GPIO_Init(I2C1_SCL_PORT, &gpio);
    gpio.Pin = I2C1_SDA_PIN; HAL_GPIO_Init(I2C1_SDA_PORT, &gpio);

    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);
    i2c_short_delay();

    for (int i = 0; i < 9; i++)
    {
        if (HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET); i2c_short_delay();
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);   i2c_short_delay();
    }

    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_RESET); i2c_short_delay();
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);   i2c_short_delay();
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);   i2c_short_delay();

    MX_I2C1_Init();
}


void calibrate_gyro_bias(void)
{
    if (!imu_ok) return;

    const int   SAMPLES  = 1000;
    const float DELAY_MS = 2.0f;   // ms between samples
    float sum = 0.0f;

    for (int i = 0; i < SAMPLES; i++)
    {
        /* wait for a fresh DMA result */
        mpu_data_ready = 0;
        MPU6050_Start_Read_All_DMA(&hi2c1, mpu_raw_data);
        uint32_t t = HAL_GetTick();
        while (!mpu_data_ready && (HAL_GetTick() - t) < 50);   // 50 ms timeout

        if (mpu_data_ready)
        {
            mpu_data_ready = 0;
            MPU6050_Process_Data(&mpuData, mpu_raw_data);
            sum += mpuData.gyro_z_dps;
        }
        HAL_Delay((uint32_t)DELAY_MS);
    }

    gyro_z_bias = sum / (float)SAMPLES;
}

float pt1_filter(float input, float cutoff, float dt, float *state)
{
    float rc = 1.0f / (2.0f * M_PI * cutoff);
    float alpha = dt / (dt + rc);
    *state = *state + alpha * (input - *state);
    return *state;
}


int _write(int32_t file, uint8_t *ptr, int32_t len)
{
    int i=0;
    for(i=0 ; i<len ; i++)
        ITM_SendChar((*ptr++));  // Send each character over SWO
    return len;
}

static int16_t joy_map(uint16_t raw, int16_t max_out)
{
    if (raw >= JOY_CENTER_LOW && raw <= JOY_CENTER_HIGH)
        return 0;

    if (raw > JOY_CENTER_HIGH)
        return (int16_t)(
            ((int32_t)(raw - JOY_CENTER_HIGH) * max_out)
            / (JOY_MAX - JOY_CENTER_HIGH)
        );
    else
        return (int16_t)(
            -((int32_t)(JOY_CENTER_LOW - raw) * max_out)
            / (JOY_CENTER_LOW - JOY_MIN)
        );
}


void read_imu()
{
    // this function is connected to timer6
    if(!imu_ok) return;

    if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY && !mpu_data_ready)
    {
        if (MPU6050_Start_Read_All_DMA(&hi2c1, mpu_raw_data) != HAL_OK)
        {
            imu_ok = 0;
            yaw_correction = 0.0f;
            mpu_data_ready = 0;
            imu_recovery_request = 1;

            HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET); // IMU problem

            return;
        }
    }

    if (!mpu_data_ready) return;

    mpu_data_ready = 0;

    MPU6050_Process_Data(&mpuData, mpu_raw_data);
    imu_last_data_tick = HAL_GetTick();

    float gz_corrected = mpuData.gyro_z_dps - gyro_z_bias;

    gyro_z_p = pt1_filter(gz_corrected, 100.0f, dt, &gyro_z_p_state);
    gyro_z_d = pt1_filter(gz_corrected,  30.0f, dt, &gyro_z_d_state);

    yaw_angle += gyro_z_p * dt;
    yaw_angle = wrap_angle(yaw_angle);

    float yaw_error = wrap_angle(target_yaw - yaw_angle);

    if (fabsf(yaw_error) <= YAW_LIMIT_DEG)
    {
        yaw_correction = 0.0f;
    }
    else
    {
        if (pi_cmd.mode == MODE_ROTATE_YAW)
        {
            float sign = (yaw_error > 0.0f) ? 1.0f : -1.0f;

            yaw_correction =
                (sign * ROT_YAW_BASE_PWM)
                + (ROT_YAW_KP * yaw_error)
                - (ROT_YAW_KD * gyro_z_d);

            if (yaw_correction > ROT_YAW_CORR_MAX)
                yaw_correction = ROT_YAW_CORR_MAX;

            if (yaw_correction < -ROT_YAW_CORR_MAX)
                yaw_correction = -ROT_YAW_CORR_MAX;
        }
        else if (pi_cmd.mode == MODE_DRIVE_HEADING)
        {
            float sign = (yaw_error > 0.0f) ? 1.0f : -1.0f;

            yaw_correction =
                (sign * DRIVE_YAW_BASE_PWM)
                + (DRIVE_YAW_KP * yaw_error)
                - (DRIVE_YAW_KD * gyro_z_d);

            if (yaw_correction > DRIVE_YAW_CORR_MAX)
                yaw_correction = DRIVE_YAW_CORR_MAX;

            if (yaw_correction < -DRIVE_YAW_CORR_MAX)
                yaw_correction = -DRIVE_YAW_CORR_MAX;
        }
        else
        {
            yaw_correction = 0.0f;
        }
    }
}

void timerfuncion(){
	// this funciton is connected to timer6 thats why its not showen that its called
	static uint8_t divider = 0;

	    if(++divider & 1) return;

	    if(divider < 2) return;   // skip one interrupt

	    divider = 0;
	//................................DT Calculations.......................................
	    uint32_t dwt_now = DWT->CYCCNT;
	    	 			  uint32_t cycles_elapsed = dwt_now - dwt_last;
	    	 			  dwt_last = dwt_now;
	    	 			  if (cycles_elapsed == 0) cycles_elapsed = 1;
	    	 			  dt = (float)cycles_elapsed / (float)SystemCoreClock;
	    	 			  pid_hz = 1.0f / dt;

	    	 			 Motor_UpdateEncoder(&motor_FL);
	    	 			 Motor_UpdateEncoder(&motor_FR);
	    	 			 Motor_UpdateEncoder(&motor_RL);
	    	 			 Motor_UpdateEncoder(&motor_RR);

	    	 			 // Accumulate counts and time
	    	 			 acc_FL += motor_FL.delta_encoder;
	    	 			 acc_FR += motor_FR.delta_encoder;
	    	 			 acc_RL += motor_RL.delta_encoder;
	    	 			 acc_RR += motor_RR.delta_encoder;
	    	 			 acc_dt += dt;
	    	 			 rpm_tick_count++;

	    	 			 if (rpm_tick_count >= RPM_SAMPLE_TICKS)
	    	 			 {
	    	 			     // Temporarily swap in accumulated values for RPM calc
	    	 			     motor_FL.delta_encoder = acc_FL;
	    	 			     motor_FR.delta_encoder = acc_FR;
	    	 			     motor_RL.delta_encoder = acc_RL;
	    	 			     motor_RR.delta_encoder = acc_RR;

	    	 			     Motor_ComputeRPM(&motor_FL, acc_dt);
	    	 			     Motor_ComputeRPM(&motor_FR, acc_dt);
	    	 			     Motor_ComputeRPM(&motor_RL, acc_dt);
	    	 			     Motor_ComputeRPM(&motor_RR, acc_dt);

	    	 			     // Reset accumulators
	    	 			     acc_FL = acc_FR = acc_RL = acc_RR = 0;
	    	 			     acc_dt = 0.0f;
	    	 			     rpm_tick_count = 0;
	    	 			 }




			        	  Motor_SetPWM(&motor_FL, pwm_FL/2);
			        	  Motor_SetPWM(&motor_FR, pwm_FR/2);
			        	  Motor_SetPWM(&motor_RL, pwm_RL/2);
			        	  Motor_SetPWM(&motor_RR, pwm_RR/2);
}



void decode_received_data(void)
{
	dataa      = nrf24_uint8_t_to_type(dataR, sizeof(dataR));
	    receivedY  = ((uint16_t)dataR[1] << 8) | dataR[2];
	    receivedX  = ((uint16_t)dataR[3] << 8) | dataR[4];
	    receivedYa = ((uint16_t)dataR[5] << 8) | dataR[6];
	    receivedXa = ((uint16_t)dataR[7] << 8) | dataR[8];
	    change     = dataR[10];
	    change2    = dataR[11];

	    int16_t throttle = joy_map(receivedYa, PWM_MAX);  // forward / back
	    int16_t strafe   = joy_map(receivedXa,  PWM_MAX);  // right / left
	    int16_t yaw      = joy_map(receivedY, PWM_MAX);  // yaw right / left

	     fl = (int32_t)throttle + strafe + yaw;
	     fr = (int32_t)throttle - strafe - yaw;
	     rl = (int32_t)throttle - strafe + yaw;
	     rr = (int32_t)throttle + strafe - yaw;

	     if(change==1){
	    fl -= yaw_correction;
	    fr += yaw_correction;
	    rl -= yaw_correction;
	    rr += yaw_correction;
	     }

	    int32_t peak = fl;
	    if (abs(fr) > abs(peak)) peak = fr;
	    if (abs(rl) > abs(peak)) peak = rl;
	    if (abs(rr) > abs(peak)) peak = rr;

	    if (abs(peak) > PWM_MAX) {
	        fl = fl * PWM_MAX / abs(peak);
	        fr = fr * PWM_MAX / abs(peak);
	        rl = rl * PWM_MAX / abs(peak);
	        rr = rr * PWM_MAX / abs(peak);
	    }

	    if(change == 0)
	    {
	        pwm_FL = (int16_t)fl;
	        pwm_FR = (int16_t)fr;
	        pwm_RL = (int16_t)rl;
	        pwm_RR = (int16_t)rr;
	    }
}

void reciver(void)
{
    nrf24_listen();
    nrf24_en_ack_pld(enable);
    nrf24_en_dyn_ack(disable);
}

void handle_irq(void)
{
	if (irq == 1) {
	        uint8_t stat = nrf24_r_status();
	        if (stat & (1 << RX_DR)) {
	            counter = (counter + 1) % 21;
	            nrf24_receive(dataR, sizeof(dataR));
	            nrf24_transmit_rx_ack_pld(0, rx_ack_pld, sizeof(rx_ack_pld));
	            nrf_data_ready = 1;  // ← set flag here
	        }
        irq = 0;
    }
}

void conf(void)
{
    csn_high();
    ce_high();

    HAL_Delay(5);

    ce_low();

    nrf24_init();

    nrf24_listen();

    nrf24_auto_ack_all(auto_ack);
    nrf24_en_ack_pld(disable);
    nrf24_dpl(disable);

    nrf24_set_crc(no_crc, _1byte);

    nrf24_tx_pwr(_0dbm);
    nrf24_data_rate(_250kbps);
    nrf24_set_channel(83);
    nrf24_set_addr_width(5);

    nrf24_set_rx_dpl(0, disable);
    nrf24_set_rx_dpl(1, disable);
    nrf24_set_rx_dpl(2, disable);
    nrf24_set_rx_dpl(3, disable);
    nrf24_set_rx_dpl(4, disable);
    nrf24_set_rx_dpl(5, disable);

    nrf24_pipe_pld_size(0, PLD_S);

    nrf24_auto_retr_delay(4);
    nrf24_auto_retr_limit(10);

    nrf24_open_tx_pipe(tx_addr);
    nrf24_open_rx_pipe(0, tx_addr);

    ce_high();
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  conf();

  I2C1_BusRecovery();

  if (MPU6050_Init(&hi2c1, ACCEL_RANGE_8G, GYRO_RANGE_500) != 0)
  {
	  HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
      imu_ok = 0;
      imu_recovery_request = 1;
  }
  else
  {
	  HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
      imu_ok = 1;
      imu_last_data_tick = HAL_GetTick();
  }

     	  /*HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
     	  HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);*/

     	  printf("main");

     	 mpuData.accel_range = ACCEL_RANGE_8G;
     	 mpuData.gyro_range  = GYRO_RANGE_500;
     	 HAL_Delay(5000);
//...................................... DT calculations.................................
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  dwt_last = DWT->CYCCNT;

	 //...............................................TIMERS........................................
	 HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	 HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	 HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	 HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);


	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


	 HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);


	 HAL_TIM_Base_Start_IT(&htim6);

	 Motor_Init(&motor_FL, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, &htim2, ENC_COUNTS_PER_REV);
	 Motor_Init(&motor_FR, &htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, &htim3, ENC_COUNTS_PER_REV);
	 Motor_Init(&motor_RL, &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, &htim4, ENC_COUNTS_PER_REV);
	 Motor_Init(&motor_RR, &htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, &htim5, ENC_COUNTS_PER_REV);

	 Motor_Start(&motor_FL);
	 Motor_Start(&motor_FR);
	 Motor_Start(&motor_RL);
	 Motor_Start(&motor_RR);



	 calibrate_gyro_bias();
	 imu_last_data_tick = HAL_GetTick();

	 HAL_UART_Receive_DMA(&huart2, rx_buffer, sizeof(PiCommandFrame));
	 HAL_UART_Receive_IT(&huart1, ultra1_rx_buf, 3);
	 HAL_UART_Receive_IT(&huart3, ultra3_rx_buf, 3);
	 HAL_UART_Receive_IT(&huart4, ultra4_rx_buf, 3);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (imu_recovery_request || (imu_ok && (HAL_GetTick() - imu_last_data_tick > 200)))
		  {
		      /* rate-limit attempts so the loop stays responsive when the IMU is absent */
		  if (imu_ok && (HAL_GetTick() - imu_last_data_tick > 200))
		      {
		          imu_ok = 0;
		          yaw_correction = 0.0f;
		          HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET); // blue LED ON
		          imu_recovery_request = 1;
		      }

		      if (HAL_GetTick() - imu_last_recovery_try > 500)
		      {
		          imu_last_recovery_try = HAL_GetTick();
		          imu_recovery_request  = 0;

		          if (hi2c1.hdmarx != NULL) HAL_DMA_Abort(hi2c1.hdmarx);
		          if (hi2c1.hdmatx != NULL) HAL_DMA_Abort(hi2c1.hdmatx);

		          I2C1_BusRecovery();

		          if (HAL_I2C_IsDeviceReady(&hi2c1, (0x68 << 1), 2, 5) == HAL_OK &&
		              MPU6050_Init(&hi2c1, ACCEL_RANGE_8G, GYRO_RANGE_500) == 0)
		          {
		              imu_ok = 1;
		              yaw_angle = 0.0f;
		              yaw_correction = 0.0f;
		              gyro_z_p_state = 0.0f;
		              gyro_z_d_state = 0.0f;
		              imu_last_data_tick = HAL_GetTick();

		              HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET); // OFF = OK
		          }
		          else
		          {
		              imu_ok = 0;
		              yaw_correction = 0.0f;

		              HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET); // ON = disconnected

		              // keep trying later
		              imu_recovery_request = 1;
		          }
		      }
		  }

	  handle_irq();
	  if (nrf_data_ready) {
	      nrf_data_ready = 0;
	      decode_received_data();
	  }

	  	  if (counter == 0) {
	  	      HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
	  	  } else {
	  	      HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
	  	  }


	  	if(uart_rx_ready)
	  	    {
	  	        uart_rx_ready = 0;
	  	        process_pi_command();
	  	    }

	  	    static uint32_t last_uart_tx = 0;
	  	    if(HAL_GetTick() - last_uart_tx >= 20)
	  	    {
	  	        last_uart_tx = HAL_GetTick();
	  	        send_telemetry_to_pi();
	  	    }

	  	  static uint32_t last_ultra_request = 0;

	  	  if(HAL_GetTick() - last_ultra_request >= 100)
	  	  {
	  	      last_ultra_request = HAL_GetTick();

	  	      HAL_UART_Transmit(&huart1, &ultra1_cmd, 1, 10);
	  	      HAL_UART_Transmit(&huart3, &ultra3_cmd, 1, 10);
	  	      HAL_UART_Transmit(&huart4, &ultra4_cmd, 1, 10);
	  	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 41;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSN_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LEDR_Pin|LEDB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDR_Pin LEDB_Pin */
  GPIO_InitStruct.Pin = LEDR_Pin|LEDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_2){
			irq = 1;
		}
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        mpu_data_ready = 1; // Set flag to indicate data is ready
    }
}



void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	 if (hi2c->Instance == I2C1)
	    {
	        mpu_data_ready = 0;
	        imu_ok = 0;
	        yaw_correction = 0.0f;

	        HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET); // blue LED ON = IMU problem

	        imu_recovery_request = 1;   // heavy recovery in main loop
	    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        memcpy(&pi_cmd, rx_buffer, sizeof(PiCommandFrame));
        uart_rx_ready = 1;

        HAL_UART_Receive_DMA(&huart2, rx_buffer, sizeof(PiCommandFrame));
    }

    if(huart->Instance == USART1)
    {
        if(ultra1_rx_buf[0] == 0xFF)
        {
            ultra1_distance_mm = ((uint16_t)ultra1_rx_buf[1] << 8) | ultra1_rx_buf[2];
            ultra1_distance_cm = ultra1_distance_mm / 10.0f;
            ultra1_ready = 1;
        }

        HAL_UART_Receive_IT(&huart1, ultra1_rx_buf, 3);
    }

    if(huart->Instance == USART3)
    {
        if(ultra3_rx_buf[0] == 0xFF)
        {
            ultra3_distance_mm = ((uint16_t)ultra3_rx_buf[1] << 8) | ultra3_rx_buf[2];
            ultra3_distance_cm = ultra3_distance_mm / 10.0f;
            ultra3_ready = 1;
        }

        HAL_UART_Receive_IT(&huart3, ultra3_rx_buf, 3);
    }

    if(huart->Instance == UART4)
    {
        if(ultra4_rx_buf[0] == 0xFF)
        {
            ultra4_distance_mm = ((uint16_t)ultra4_rx_buf[1] << 8) | ultra4_rx_buf[2];
            ultra4_distance_cm = ultra4_distance_mm / 10.0f;
            ultra4_ready = 1;
        }

        HAL_UART_Receive_IT(&huart4, ultra4_rx_buf, 3);
    }

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        uart_tx_complete = 1;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        HAL_UART_AbortReceive(&huart1);
        HAL_UART_Receive_IT(&huart1, ultra1_rx_buf, 3);
    }

    if(huart->Instance == USART3)
    {
        HAL_UART_AbortReceive(&huart3);
        HAL_UART_Receive_IT(&huart3, ultra3_rx_buf, 3);
    }

    if(huart->Instance == UART4)
    {
        HAL_UART_AbortReceive(&huart4);
        HAL_UART_Receive_IT(&huart4, ultra4_rx_buf, 3);
    }
}

void process_pi_command(void)
{
    if(pi_cmd.header != 0xAA55) return;

    uint8_t check = calc_checksum((uint8_t *)&pi_cmd, sizeof(PiCommandFrame));
    if(check != pi_cmd.checksum) return;

    target_yaw = pi_cmd.target_yaw_deg;

    if (change == 0)
    {
        target_yaw = yaw_angle;
        yaw_correction = 0.0f;
    }


     flR = pi_cmd.vx + pi_cmd.vy + pi_cmd.wz;
     frR = pi_cmd.vx - pi_cmd.vy - pi_cmd.wz;
     rlR = pi_cmd.vx - pi_cmd.vy + pi_cmd.wz;
     rrR = pi_cmd.vx + pi_cmd.vy - pi_cmd.wz;

    flR -= yaw_correction;
    frR += yaw_correction;
    rlR -= yaw_correction;
    rrR += yaw_correction;

    int32_t peak = abs(flR);
    if(abs(frR) > peak) peak = abs(frR);
    if(abs(rlR) > peak) peak = abs(rlR);
    if(abs(rrR) > peak) peak = abs(rrR);

    if(peak > PWM_MAX)
    {
        flR = flR * PWM_MAX / peak;
        frR = frR * PWM_MAX / peak;
        rlR = rlR * PWM_MAX / peak;
        rrR = rrR * PWM_MAX / peak;
    }
    if (change==1){
    pwm_FL = flR;
    pwm_FR = frR;
    pwm_RL = rlR;
    pwm_RR = rrR;
    }
}
void send_telemetry_to_pi(void)
{
    if(!uart_tx_complete) return;

    stm_tel.header = 0x55AA;

    stm_tel.rpm_FL = motor_FL.rpm;
    stm_tel.rpm_FR = motor_FR.rpm;
    stm_tel.rpm_RL = motor_RL.rpm;
    stm_tel.rpm_RR = motor_RR.rpm;

    stm_tel.yaw_angle = yaw_angle;
    stm_tel.gyro_z_dps = gyro_z_p;

    stm_tel.ultra_left_cm  = ultra1_distance_cm;
    stm_tel.ultra_right_cm = ultra3_distance_cm;
    stm_tel.ultra_front_cm = ultra4_distance_cm;

    stm_tel.accel_x = mpuData.accel_x_g;
    stm_tel.accel_y = mpuData.accel_y_g;
    stm_tel.accel_z = mpuData.accel_z_g;

    stm_tel.dt = dt;
    stm_tel.pid_hz = pid_hz;

    stm_tel.pwm_FL = pwm_FL;
    stm_tel.pwm_FR = pwm_FR;
    stm_tel.pwm_RL = pwm_RL;
    stm_tel.pwm_RR = pwm_RR;

    stm_tel.change_value = change;

    stm_tel.imu_ok = imu_ok;
    stm_tel.status = 1;

    stm_tel.checksum = calc_checksum((uint8_t *)&stm_tel, sizeof(StmTelemetryFrame));

    uart_tx_complete = 0;
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&stm_tel, sizeof(StmTelemetryFrame));
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

#ifdef  USE_FULL_ASSERT
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
