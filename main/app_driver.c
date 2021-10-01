/*  Temperature Sensor demo implementation using RGB LED and timer

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <sdkconfig.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 
#include "driver/adc_common.h"
#include <app_reset.h>
#include "app_priv.h"

/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
#define BUTTON_ACTIVE_LEVEL  0
/* This is the GPIO on which the power will be set */
#define OUTPUT_GPIO    19

static TimerHandle_t sensor_timer;

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

#define OUTPUT_GPIO_NRML    19
#define OUTPUT_GPIO_WARN    18
#define OUTPUT_GPIO_BUZZER  17

#define INPUT_GPIO_LPG      4
#define GPIO_INPUT_PIN_SEL  ((1ULL<<INPUT_GPIO_LPG) )
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<OUTPUT_GPIO_WARN) | (1ULL<<OUTPUT_GPIO_NRML))
#define ESP_INTR_FLAG_DEFAULT 0

static int lpg_detected =0; 

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void buzzer_alarm() {
    gpio_config_t io_conf;
    gpio_set_level(OUTPUT_GPIO_NRML, 0);
    gpio_set_level(OUTPUT_GPIO_WARN, 1);
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<OUTPUT_GPIO_BUZZER);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(OUTPUT_GPIO_BUZZER, 1);
}

static void sensor_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
	    if (gpio_get_level(io_num)) {
                lpg_detected =0;
            } else {
                lpg_detected = 1;
		buzzer_alarm();
		esp_rmaker_raise_alert("LPG Detected !!!");    
	    } 
        }
    }
}

static void app_sensor_update(TimerHandle_t handle)
{
    
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(lpg_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
            esp_rmaker_float(lpg_detected));	   
}

float app_get_current_lpg_level()
{
    return lpg_detected;
}

esp_err_t app_sensor_init(void)
{
    sensor_timer = xTimerCreate("app_sensor_update_tm", (REPORTING_PERIOD * 1000) / portTICK_PERIOD_MS,
                            pdTRUE, NULL, app_sensor_update);
    if (sensor_timer) {
        xTimerStart(sensor_timer, 0);
        return ESP_OK;
    }
    return ESP_FAIL;
}

void app_driver_init()
{
    app_sensor_init();
    app_reset_button_register(app_reset_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL),
                WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4  here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(INPUT_GPIO_LPG, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    //start gpio task
    xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(INPUT_GPIO_LPG, gpio_isr_handler, (void*) INPUT_GPIO_LPG);
    
    gpio_set_level(OUTPUT_GPIO_NRML, 1);

}
