#include <limits.h>
//
// Created by oktet on 16.06.2020.
//

#include <stdint.h>
#include <driver/adc_common.h>
#include <driver/mcpwm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/portmacro.h>
#include <freertos/projdefs.h>
#include <freertos/task.h>
#include <math.h>

#include "vector_3d.h"
#include "data.h"

QueueHandle_t queue;

_Noreturn void read_accelerometer(void* args) {
    int32_t mid = 460;

    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

    for (;;) {
        int32_t x = adc1_get_raw(ADC1_CHANNEL_4) - mid;
        int32_t y = adc1_get_raw(ADC1_CHANNEL_5) - mid;
        int32_t z = adc1_get_raw(ADC1_CHANNEL_6) - mid;

        printf("x = %d\t y = %d\t z = %d\t\n", x, y, z);

        vector_3d v = {x, y, z};
        accelerometer_data data = {rad_to_deg(vector_phi(&v)), rad_to_deg(vector_theta(&v))};

        printf("phi = %f\t theta = %f\t\n", data.phi, data.theta);

        if (xQueueSend(queue, (void *) &data, (TickType_t) 10) != pdPASS) {
            printf("Lost data\n");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void gpio_initialize(void) {
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);
}

static int32_t servo_per_degree_init(int32_t degree, int32_t max_degree, int32_t min_pulse, int32_t max_pulse) {
    return min_pulse + (((max_pulse - min_pulse) * degree) / max_degree);
}

_Noreturn void servo_control(void *args)
{
    const int32_t MIN_PULSE = 500;
    const int32_t MAX_PULSE = 2500;
    const int32_t MAX_DEGREE = 90;

    gpio_initialize();

    printf("Configuring Initial Parameters of mcpwm......\n");

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    double prev_theta = 0;
    double alpha = 0.1;

    for (;;) {
        accelerometer_data data;
        if (xQueueReceive(queue, &data, (TickType_t) 10) == pdTRUE) {
            double theta_smoothed = alpha * data.theta + (1 - alpha) * prev_theta;
            prev_theta = isnan(theta_smoothed) ? 0 : theta_smoothed;
            printf("Angle of rotation: %f\n", theta_smoothed);
            int32_t pulse = servo_per_degree_init(theta_smoothed, MAX_DEGREE, MIN_PULSE, MAX_PULSE);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    printf("Initiating queue\n");

    queue = xQueueCreate(64, sizeof(accelerometer_data));

    printf("Starting accelerometer reader\n");
    if (xTaskCreate(read_accelerometer, "read accelerometer", 4096, NULL, 5, NULL) == pdPASS) {
        printf("Starting servo control\n");
        if (xTaskCreate(servo_control, "servo control", 4096, NULL, 5, NULL) == pdPASS) {
            printf("All tasks started\n");
        } else {
            printf("Servo control failed to start\n");
        }
    } else {
        printf("Accelerometer reader failed to start\n");
    }

    vTaskDelete(NULL);
}