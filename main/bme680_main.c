#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include <bme680.h>
#include <sys/types.h>
#include "driver/gpio.h"

#define LED_GPIO GPIO_NUM_15
#define TURN_ON 0
#define TURN_OFF 1

#define SCL_IO_PIN CONFIG_I2C_MASTER_SCL
#define SDA_IO_PIN CONFIG_I2C_MASTER_SDA
#define MASTER_FREQUENCY CONFIG_I2C_MASTER_FREQUENCY
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C0_TASK_SAMPLING_RATE (10) // seconds

#define APP_TAG "BME68x"

static inline void vTaskDelaySecUntil(TickType_t *previousWakeTime, const uint sec) {
    const TickType_t xFrequency = ((sec * 1000) / portTICK_PERIOD_MS);
    vTaskDelayUntil( previousWakeTime, xFrequency );  
}

static inline void print_registers(bme680_handle_t handle) {
    /* configuration registers */
    bme680_control_measurement_register_t ctrl_meas_reg;
    bme680_control_humidity_register_t    ctrl_humi_reg;
    bme680_config_register_t              config_reg;
    bme680_control_gas0_register_t        ctrl_gas0_reg;
    bme680_control_gas1_register_t        ctrl_gas1_reg;

    /* attempt to read control humidity register */
    bme680_get_control_humidity_register(handle, &ctrl_humi_reg);

    /* attempt to read control measurement register */
    bme680_get_control_measurement_register(handle, &ctrl_meas_reg);

    /* attempt to read configuration register */
    bme680_get_configuration_register(handle, &config_reg);

    /* attempt to read control gas 0 register */
    bme680_get_control_gas0_register(handle, &ctrl_gas0_reg);

    /* attempt to read control gas 1 register */
    bme680_get_control_gas1_register(handle, &ctrl_gas1_reg);

    ESP_LOGI(APP_TAG, "Variant Id          (0x%02x): %s", handle->variant_id,uint8_to_binary(handle->variant_id));
    ESP_LOGI(APP_TAG, "Configuration       (0x%02x): %s", config_reg.reg,    uint8_to_binary(config_reg.reg));
    ESP_LOGI(APP_TAG, "Control Measurement (0x%02x): %s", ctrl_meas_reg.reg, uint8_to_binary(ctrl_meas_reg.reg));
    ESP_LOGI(APP_TAG, "Control Humidity    (0x%02x): %s", ctrl_humi_reg.reg, uint8_to_binary(ctrl_humi_reg.reg));
    ESP_LOGI(APP_TAG, "Control Gas 0       (0x%02x): %s", ctrl_gas0_reg.reg, uint8_to_binary(ctrl_gas0_reg.reg));
    ESP_LOGI(APP_TAG, "Control Gas 1       (0x%02x): %s", ctrl_gas1_reg.reg, uint8_to_binary(ctrl_gas1_reg.reg));
}

void i2c0_bme680_task() {
    // Full GPIO config
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        printf("GPIO config failed: %d\n", err);
        return;
    }
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = 23,
        .sda_io_num = 22,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    // initialize i2c device configuration
    bme680_config_t dev_cfg = I2C_BME680_CONFIG_DEFAULT;
    bme680_handle_t dev_hdl;
    //
    // init device
    bme680_init(bus_handle, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "bme680 handle init failed");
        assert(dev_hdl);
    }
    
    print_registers(dev_hdl);

    // task loop entry point
    for ( ;; ) {
        gpio_set_level(LED_GPIO, TURN_ON); // Turn on LED
        ESP_LOGI(APP_TAG, "######################## BME680 - START #########################");
        //
        // handle sensor

        bme680_data_t data;
        esp_err_t result = bme680_get_data(dev_hdl, &data);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "bme680 device read failed (%s)", esp_err_to_name(result));
        } else {
            data.barometric_pressure = data.barometric_pressure / 100;
            ESP_LOGI(APP_TAG, "air temperature:     %.2f °C", data.air_temperature);
            ESP_LOGI(APP_TAG, "dewpoint temperature:%.2f °C", data.dewpoint_temperature);
            ESP_LOGI(APP_TAG, "relative humidity:   %.2f %%", data.relative_humidity);
            ESP_LOGI(APP_TAG, "barometric pressure: %.2f hPa", data.barometric_pressure);
            ESP_LOGI(APP_TAG, "gas resistance:      %.2f kOhms", data.gas_resistance/1000);
            ESP_LOGI(APP_TAG, "iaq score:           %u (%s)", data.iaq_score, bme680_air_quality_to_string(data.iaq_score));
        }
        //
        ESP_LOGI(APP_TAG, "######################## BME680 - END ###########################");
        gpio_set_level(LED_GPIO, TURN_OFF); // Turn on LED
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    bme680_delete( dev_hdl );
    vTaskDelete( NULL );
}

void app_main(void)
{
    i2c0_bme680_task();
}