#include <stdio.h>
#include <string.h>
#include "esp_log.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "led_controller.h"
#include "DS4432U.h"
#include "EMC2101.h"
#include "INA260.h"
#include "TPS546.h"
#include "adc.h"
#include "oled.h"

#define USE_OLED 1  //Set to 1 to use OLED, 0 to not use it

static const char *TAG = "system";

void init_system(void) {
    
    //test the LEDs
    // ESP_LOGI(TAG, "Init LEDs!");
    // ledc_init();
    // led_set();

    //Setb power enable pin
    gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_10, 1);

    //Init I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ADC_init();

    //DS4432U tests
    //DS4432U_set_vcore(1.2);
    //DS4432U_set(0);
    // Initialize the core voltage regulator
    TPS546_init();

    // turn on ASIC core voltage (three domains in series)
    ESP_LOGI(TAG, "---TURNING ON VCORE---");
    TPS546_set_vout(1200);
    
    //Fan Tests
    EMC2101_init();
    EMC2101_set_fan_speed(0);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    //oled
    #ifdef USE_OLED
    if (!OLED_init()) {
        ESP_LOGI(TAG, "OLED init failed!");
    } else {
        ESP_LOGI(TAG, "OLED init success!");
        OLED_clear();
        OLED_writeString(0, 0, "The Bitaxe 402!");
    }
    #endif
}

void get_stats(void) {
    char oled_buf[20];

    uint16_t fan_speed = EMC2101_get_fan_speed();
    float chip_temp = EMC2101_get_internal_temp() + 5;
    // float current = INA260_read_current();
    // float voltage = INA260_read_voltage();
    // float power = INA260_read_power();

    float voltage = TPS546_get_vin() * 1000;
    float current = TPS546_get_iout() * 1000;

    // calculate regulator power (in milliwatts)
    float power = (TPS546_get_vout() * current) / 1000;

    uint16_t vcore = ADC_get_vcore();
    uint16_t tpscore = (TPS546_get_vout() * 1000) / 3;

    ESP_LOGI(TAG, "Fan Speed: %d RPM", fan_speed);
    ESP_LOGI(TAG, "Chip Temp: %.2f C", chip_temp);

    //Current Sensor tests
    ESP_LOGI(TAG, "Current: %.2f mA", current);
    ESP_LOGI(TAG, "Voltage: %.2f mV", voltage);
    ESP_LOGI(TAG, "Power: %.2f mW", power);

    //ESP32 ADC tests
    ESP_LOGI(TAG, "Vcore: %d mV", vcore);
    ESP_LOGI(TAG, "TPScore: %d mV\n", tpscore);

    if (OLED_status()) {
        memset(oled_buf, 0, 20);
        snprintf(oled_buf, 20, "Fan: %d RPM", fan_speed);
        OLED_clearLine(1);
        OLED_writeString(0, 1, oled_buf);

        memset(oled_buf, 0, 20);
        snprintf(oled_buf, 20, "Temp: %.2f C", chip_temp);
        OLED_clearLine(2);
        OLED_writeString(0, 2, oled_buf);

        memset(oled_buf, 0, 20);
        snprintf(oled_buf, 20, "tpsCore: %u mV", tpscore);
        OLED_clearLine(3);
        OLED_writeString(0, 3, oled_buf);
    }

}

void SysTask(void *arg) {

    init_system();

    while(1){
        get_stats();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}