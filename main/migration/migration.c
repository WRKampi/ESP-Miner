#include "global_state.h"
#include "TPS546.h"
#include "esp_log.h"
#include "nvs_config.h"
#include "nvs_flash.h"
#include "vcore.h"
#include "TPS546.h"
#include "EMC2101.h"
#include "oled.h"
#include "string.h"

static const char * TAG = "migrations";

static void do_temp_sensor_calibration(GlobalState * GLOBAL_STATE){

    EMC2101_init(nvs_config_get_u16(NVS_CONFIG_INVERT_FAN_POLARITY, 1));
    EMC2101_set_fan_speed(1);
    EMC2101_configure_ideality(EMC2101_GAMMA_DEF_IDEALITY);
    EMC2101_configure_beta_compensation(EMC2101_GAMMA_DEF_BETA);

    GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs = malloc(sizeof(bm_job *) * 128);
    GLOBAL_STATE->valid_jobs = malloc(sizeof(uint8_t) * 128);
    for (int i = 0; i < 128; i++) {
        GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[i] = NULL;
        GLOBAL_STATE->valid_jobs[i] = 0;
    }
    gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_10, 0);


    uint8_t result = VCORE_init(GLOBAL_STATE);
    VCORE_set_voltage(nvs_config_get_u16(NVS_CONFIG_ASIC_VOLTAGE, CONFIG_ASIC_VOLTAGE) / 1000.0, GLOBAL_STATE);

    SERIAL_init();


    //Setup and init is done, shut vcore down and do calibration
    (GLOBAL_STATE->ASIC_functions.init_fn)(0, GLOBAL_STATE->asic_count);
    // Set vcore to 0
    VCORE_set_voltage(0.0, GLOBAL_STATE);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    float air_temp = EMC2101_get_internal_temp();
    //Reads high
    float asic_temp_old = 256;
    float asic_temp_new = EMC2101_get_external_temp();

    //If our most recent reading is more than 0.25C colder than the last, we're still cooling down
    while(asic_temp_new + 0.25 < asic_temp_old){
        ESP_LOGI(TAG, "Cooling... %fC => %fC",asic_temp_old, asic_temp_new);
        vTaskDelay(30000 / portTICK_PERIOD_MS);
        asic_temp_old = asic_temp_new;
        asic_temp_new = EMC2101_get_external_temp();
        ESP_LOGI(TAG, "Cooling... %fC => %fC",asic_temp_old, asic_temp_new);
    }

    float offset = asic_temp_new - air_temp;
    ESP_LOGI(TAG, "Temp Offset: %f", offset);

    if(offset > 50){
        SystemModule * module = &GLOBAL_STATE->SYSTEM_MODULE;
        if (OLED_status()) {
            memset(module->oled_buf, 0, 20);
            snprintf(module->oled_buf, 20, "INVALID OFFSET");
            OLED_writeString(0, 2, module->oled_buf);
        }
        vTaskDelay(60000 / portTICK_PERIOD_MS);
        return;
    }
    //Multiply by 10 to add some precision, divide by 10 when get_u16
    nvs_config_set_u16(NVS_CONFIG_EXTERNAL_TEMP_OFFSET, offset * 10);

    esp_restart();
}

void run_migrations(void * pvParameters){

    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;



    switch (GLOBAL_STATE->device_model) {
        case DEVICE_MAX:
        case DEVICE_ULTRA:
        case DEVICE_SUPRA:
        case DEVICE_GAMMA:
            //Check if temp offset has been configured
            if(nvs_config_get_u16(NVS_CONFIG_EXTERNAL_TEMP_OFFSET, 999) != 999){
                return;
            }

             if (!OLED_init()) {
                ESP_LOGE(TAG, "OLED init failed!");
            } else {
                ESP_LOGI(TAG, "OLED init success!");
                // clear the oled screen
                OLED_fill(0);
                OLED_writeString(0, 0, "CALIBRATION...");
            }

            do_temp_sensor_calibration(GLOBAL_STATE);

            break;
        default:
    }
    
}