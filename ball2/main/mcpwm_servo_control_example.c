/* servo motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 400  //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2400 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180      //Maximum angle in degree upto which servo can rotate

float angle_pitch = 0, angle_yaw = 100;
#define GPIO_PWM0A_OUT 14   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 2   //Set GPIO 2 as PWM0B

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT); //Set GPIO 14 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);//Set GPIO 2 as PWM0A, to which servo is connected

}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t servo_per_degree_init(float degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + ((1.0*(SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

#define ECHO_TEST_TXD (GPIO_NUM_13)
#define ECHO_TEST_RXD (GPIO_NUM_15)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    while (1)
    {
        // Read data from the UART
        memset(data,0,BUF_SIZE*sizeof(uint8_t));
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Process the data
        uint8_t *p = data;
        if(len>0){
            printf("ok \n");
            for(int i=0;i<len;i++){
                printf("%d: %c %d \n",i,data[i],data[i]);
            }
        }
        if(p[0] == 'A' && p[6] == 'B' && p[12]=='D'){
            printf("ok \n");
            while (*p != 'B')
            {
                if (*p == 'A')
                    continue;
                angle_pitch *= 10;
                angle_pitch += *p - '0';
                
                ++p;
            }
            while (*p != 'D')
            {
                if (*p == 'B')
                    continue;
                angle_yaw *= 10;
                angle_yaw += *p - '0';
                ++p;
            }
                angle_pitch /= 100.0;
                angle_yaw /= 100.0;
            printf("angle_pitch %f\n",angle_pitch);
            printf("angle_yaw %f \n",angle_yaw);

        }
        // Write data back to the UART
        uart_write_bytes(UART_NUM_1, (const char *)data, len);
        //vTaskDelay(1);
    }
}

/**
 * @brief Configure MCPWM module
 */
void mcpwm_example_servo_control(void *arg)
{
    //uint32_t angle;
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
    //int dir = 0;
    //angle = 45;
    uint32_t pluse_width;
    while (1)
    {
        // if (dir == 0)
        // {
        //     angle += 1;
        //     printf("Angle of rotation: %d\n", angle);
        //     pluse_width = servo_per_degree_init(angle);
        //     printf("pulse width: %dus\n", pluse_width);
        //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pluse_width);
        //     vTaskDelay(1); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        //     if (angle >= SERVO_MAX_DEGREE / 2)
        //     {
        //         dir = 1;
        //     }
        // }
        // else
        // {
        //     angle -= 1;
        //     printf("Angle of rotation: %d\n", angle);
        //     pluse_width = servo_per_degree_init(angle);
        //     printf("pulse width: %dus\n", pluse_width);
        //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pluse_width);
        //     vTaskDelay(1); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        //     if (angle <= 30)
        //     {
        //         dir = 0;
        //     }
        // }
        pluse_width = servo_per_degree_init(angle_yaw);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pluse_width);
        //printf("set pitch %dS\n",pluse_width);
        // pluse_width = servo_per_degree_init(angle_pitch);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, pluse_width);
        vTaskDelay(1);
    }
}

void app_main(void)
{
    printf("Testing servo motor.......\n");
    // uart
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
    xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
}
