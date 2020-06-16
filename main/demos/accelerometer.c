#include <limits.h>
/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <driver/adc_common.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <math.h>


typedef struct vector_3d {
    double x, y, z;
} vector_3d;

double vector_len(vector_3d* v) {
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

double vector_sc_prod(vector_3d* v1, vector_3d* v2) {
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

double vector_angle(vector_3d* v1, vector_3d* v2) {
    return acos(vector_sc_prod(v1, v2) / (vector_len(v1) * vector_len(v2)));
}

double vector_phi(vector_3d* v) {
    vector_3d i = {vector_len(v), 0, 0};
    return vector_angle(v, &i);
}

double vector_theta(vector_3d* v) {
    vector_3d i = {v->x, v->y, 0};
    return vector_angle(v, &i);
}

double rad_to_deg(double r) {
    return r / M_PI * 180;
}

void hw() {
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
}

void read_accelerometer() {
    int32_t mid = 460;

    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

    int32_t x = adc1_get_raw(ADC1_CHANNEL_4) - mid;
    int32_t y = adc1_get_raw(ADC1_CHANNEL_5) - mid;
    int32_t z = adc1_get_raw(ADC1_CHANNEL_6) - mid;

    printf("x = %d\t y = %d\t z = %d\t\n", x, y, z);

    vector_3d v = {x, y, z};

    printf("phi = %f\t theta = %f\t\n",
            rad_to_deg(vector_phi(&v)),
            rad_to_deg(vector_theta(&v)));
}

_Noreturn void app_main() {
    int tick = 1;

    for (;;) {
        read_xyz();

        for (int i = tick; i >= 0; i--) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        fflush(stdout);
    }
}