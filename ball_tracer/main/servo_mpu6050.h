#ifndef _IMU_TASK_H_
#define _IMU_TASK_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

#define IMU_MSG_LENGTH              11u
#define IMU_MSG_LENGTH_2            (IMU_MSG_LENGTH * 2.f)
#define IMU_MSG_LENGTH_3            (IMU_MSG_LENGTH * 3.f)
#define IMU_MSG_SOF                 0x55
#define IMU_CMD_ID_SEGMENT_OFFSET   0x01

#define IMU_ACCEL_CMD_ID            0x51
#define IMU_GYRO_CMD_ID             0x52
#define IMU_MAG_CMD_ID              0x53

#define ECHO_TEST_TXD  (GPIO_NUM_15)
#define ECHO_TEST_RXD  (GPIO_NUM_13)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

static uint16_t _1L = IMU_MSG_LENGTH;
static uint16_t _2L = IMU_MSG_LENGTH_2;


typedef struct {
    __packed struct {
    float x;
    float y;
    float z;
    } acc;
    
    __packed struct {
    float x;
    float y;
    float z;
    } gyro;
    
    __packed struct {
    float x;
    float y;
    float z;
    } mag;
} imu_t;

imu_t imu;
void imu_task_init(void);
void imu_download_msg_process(uint8_t * buffer);
void imu_msg_unpack_handler(uint8_t * buffer);

void mpu6050_init();
void imu_download_msg_process(uint8_t *buffer);
void imu_msg_unpack_handler(uint8_t *buffer);
static void echo_task(void *arg);
void show_data();
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation);
static void mcpwm_example_gpio_initialize(void);
float map(float value,float min,float max,float map_min,float map_max);
static void mcpwm_example_servo_control(void *arg);

void mpu6050_init(){
    char mpu_config[3] = {0xFF, 0xAA, 0x64};//configure mpu to baud 115200
    uart_write_bytes(UART_NUM_1, (const char *) mpu_config, 3);
}

void imu_download_msg_process(uint8_t * buffer)
{
    // 确认帧头
    if(buffer[0] == IMU_MSG_SOF)
    {
        imu_msg_unpack_handler(buffer);
    }
}

/**
  * @brief     MPU6050 解包函数
  * @param[in] buffer: 完整的三帧信息
  * @retval    null
  */
void imu_msg_unpack_handler(uint8_t * buffer)
{
    if (buffer[IMU_CMD_ID_SEGMENT_OFFSET] == IMU_ACCEL_CMD_ID)
    {
        // IMU_ACCEL: // 0.005f ≈ 1.f / 32768.f * 16.f * 9.8f
        imu.acc.x =  ((short)(buffer[3]<<8 | buffer[2])) * 0.005f;    // g
        imu.acc.y =  ((short)(buffer[5]<<8 | buffer[4])) * 0.005f;    // g
        imu.acc.z =  ((short)(buffer[7]<<8 | buffer[6])) * 0.005f;    // g
    }
    
    if (buffer[IMU_CMD_ID_SEGMENT_OFFSET + _1L] == IMU_GYRO_CMD_ID)
    {
        // IMU_GYRO:  // 0.061f ≈ 1.f / 32768.f * 2000.f
        imu.gyro.x = ((short)(buffer[_1L + 3]<<8 | buffer[_1L + 2])) * 0.061f;   // deg/sec
        imu.gyro.y = ((short)(buffer[_1L + 5]<<8 | buffer[_1L + 4])) * 0.061f;   // deg/sec
        imu.gyro.z = ((short)(buffer[_1L + 7]<<8 | buffer[_1L + 6])) * 0.061f;   // deg/sec
    }
    
    if (buffer[IMU_CMD_ID_SEGMENT_OFFSET + _2L] == IMU_MAG_CMD_ID)
    {
        // IMU_MAG:   // 0.005f ≈ 1.f / 32768.f * 180.f
        imu.mag.x =  ((short)(buffer[_2L + 3]<<8 | buffer[_2L + 2])) * 0.005f;    // deg
        imu.mag.y =  ((short)(buffer[_2L + 5]<<8 | buffer[_2L + 4])) * 0.005f;    // deg
        imu.mag.z =  ((short)(buffer[_2L + 7]<<8 | buffer[_2L + 6])) * 0.005f;    // deg
    }
}
static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint8_t end_data[2] = {0x00, 0x00};
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 15 / portTICK_RATE_MS);
        imu_download_msg_process(data);
        //uart_write_bytes(UART_NUM_1, (const char *) data, len);
        //uart_write_bytes(UART_NUM_1,(const char *)end_data, 2);
    }
}
void show_data(){
    while(1){
        printf("imu.acc.x=%f\nimu.acc.y=%f\n,imu.acc.z=%f\n",imu.acc.x,imu.acc.y,imu.acc.z);
        printf("imu.gyro.x=%f\nimu.gyro.y=%f\n,imu.gyro.z=%f\n",imu.gyro.x,imu.gyro.y,imu.gyro.z);
        printf("imu.mag.x=%f\nimu.mag.y=%f\n,imu.mag.z=%f\n",imu.mag.x,imu.mag.y,imu.mag.z);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 4);    //Set GPIO 18 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, 14);
    //2. initial mcpwm configuration
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings

}
float map(float value,float min,float max,float map_min,float map_max){
    float temp = (value - min) / (max - min);
    return map_min + temp * (map_max - map_min);
}
static void mcpwm_example_servo_control(void *arg)
{
    uint32_t angle[3], count[3];
    while (1) {
        count[0] = (uint32_t)map(imu.mag.x, -90, 90, 0, 90);
        count[1] = (uint32_t)map(imu.mag.y, -90, 90, 0, 90);
        count[2] = (uint32_t)map(imu.mag.z, -90, 90, 0, 90);
        for (int i = 0; i < 3;i++){
            angle[i]=servo_per_degree_init(count[i]);
        }
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle[0]);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle[1]);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, angle[2]);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
#endif