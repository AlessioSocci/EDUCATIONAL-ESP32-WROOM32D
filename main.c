#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_event_loop.h"

#include "nvs_flash.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "esp_sleep.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "driver/uart.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_crc.h"

// GPIO define
#define GPIO_INPUT_PIN_SEL					(1ULL << 0)

// ADC define:
#define ADC_VREF    						1100
#define ADC_SAMPLES							64

// PWM define
#define GPIO_PWM0A_OUT 19 //Set GPIO 19 as PWM0A


#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)

// WIFI define
#define WIFI_SSID							"NETGEAR92"
#define WIFI_PASS               			"basicunit361"


//******************************************************************************//

// PWM
static void pwm_init_task(void* arg)
{
    printf("PWM init...\n");

    mcpwm_pin_config_t pin_config =
    {
        .mcpwm0a_out_num = GPIO_PWM0A_OUT,
    };

    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);

    printf("Configuring Initial Parameters of mcpwm...\n");

    mcpwm_config_t pwm_config;

    pwm_config.frequency = 1000;    //frequency = 1000Hz
    pwm_config.cmpr_a = 0.0;       //duty cycle of PWMxA = 10.0%
    pwm_config.cmpr_b = 0.0;       //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings

    vTaskDelete(NULL);
}

//void pwm_send(uint8_t sendAddress, uint8_t destinationAddress, uint8_t payload)
void pwm_send()
{
	vTaskDelay(50 / portTICK_PERIOD_MS);

	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 30);

	mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

//	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 70);

//	mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
//
//	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 30);

//	mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
//
//	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 70);

//	mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
//
//	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 30);

//	mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
//
//	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 70);

//	mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
//
//	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 30);

//	mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
//
//	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 70);

//	mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

	vTaskDelay(50 / portTICK_PERIOD_MS);
}


//GPIO
int green_led = 0;

static xQueueHandle gpio_evt_queue = NULL;

static void GPIO_task(void* arg)
{
    uint32_t io_num;

	printf("executing task attached to GPIO isr");

    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
    {

    }

 	pwm_send();

    vTaskDelete(NULL);
}


static void IRAM_ATTR GPIO_isr(void* arg)
{
	uint32_t gpio_num = (uint32_t)arg;

	xTaskCreate(GPIO_task, "GPIO_task", 2048, NULL, 10, NULL);

	gpio_set_level(GPIO_NUM_17, 0);
	gpio_set_level(GPIO_NUM_4, 1);

	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// ADC
static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static void adc_init_task(void* arg)
{
	printf("ADC init...\n");



    if (unit == ADC_UNIT_1)
    {
    	adc1_config_width(width);

    	adc1_config_channel_atten(channel, atten);
    }
    else
    {
    	adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t)); // Characterize ADC

    vTaskDelete(NULL);
}

uint32_t adc_read()
{
	uint32_t adc_reading = 0;

	for (int i = 0; i < ADC_SAMPLES; i++) // ADC Multisampling
	{
		if (unit == ADC_UNIT_1)
		{
			adc_reading += adc1_get_raw((adc1_channel_t)channel);
		}
		else
		{
			int raw;

			adc2_get_raw((adc2_channel_t)channel, width, &raw);

			adc_reading += raw;
		}
	}

	adc_reading /= ADC_SAMPLES; // the mean of read value

	uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars); // raw to mV

	printf("Raw: %d \t Voltage: %dmV \n", adc_reading, voltage);

	return adc_reading;
}

// WIFI
static EventGroupHandle_t wifi_event_group;

const static int CONNECTED_BIT = BIT0;

const static char *TAG = "WiFi_demo";


esp_err_t wifi_event_handler(void *arg, system_event_t *event)
{
    switch(event->event_id)
    {
    	case SYSTEM_EVENT_STA_START:

    		esp_wifi_connect();

        break;

    	case SYSTEM_EVENT_STA_GOT_IP:

    		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

        break;

    	case SYSTEM_EVENT_STA_DISCONNECTED:

    		esp_wifi_connect();

    		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);

        break;

    	default:

        break;
    }
    return ESP_OK;
}

void wifi_init_task(void* arg)
{
    printf("WiFi init...\n");

    tcpip_adapter_init();

    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config =
    {
        .sta =
        {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
			.threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    ESP_LOGI(TAG, "start the WIFI SSID:[%s] password:[%s]\n", WIFI_SSID, WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_start());

    vTaskDelete(NULL);
}

// SCHEDULER
int task1_counter = 0;
int task2_counter = 0;
int task3_counter = 0;
int task4_counter = 0;

TaskHandle_t task1_handle = NULL;
TaskHandle_t task2_handle = NULL;
TaskHandle_t task3_handle = NULL;
TaskHandle_t task4_handle = NULL;


void task1(void* arg)
{
	while(1)
	{
		task1_counter++;

		printf("task_1 in execution \n");

		if((task1_counter == 50))
		{
			vTaskSuspend(task2_handle);
		}

		if(task1_counter > 100)
		{
			task1_counter = 0;

			vTaskResume(task2_handle);
		}

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void task2(void* arg)
{
	while(1)
	{
		task2_counter++;

		printf("task_2 in execution \n");

		if(task2_counter > 100)
		{
			task2_counter = 0;
		}

		vTaskDelay(300 / portTICK_PERIOD_MS);
	}
}

void task3(void* arg)
{
	while(1)
	{
		task3_counter++;

		printf("task_3 in execution on core 0 \n");


		if(task3_counter > 50)
		{
			task3_counter = 0;

			gpio_set_level(GPIO_NUM_17, 0);

			esp_light_sleep_start();
		}


		vTaskDelay(300 / portTICK_PERIOD_MS);
	}
}

void task4(void* arg)
{
	while(1)
	{
		task4_counter++;

		printf("task_4 in execution on core 1 \n");

		if(task4_counter > 100)
		{
			task4_counter = 0;
		}

		vTaskDelay(300 / portTICK_PERIOD_MS);
	}
}


//******************************************************************************//

void app_main(void)
{
	// CHIP INFO
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);

	printf("This is %s chip with %d CPU core(s), WiFi%s%s, ", CONFIG_IDF_TARGET, chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
	printf("silicon revision %d, ", chip_info.revision);
	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
	printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    nvs_flash_init();

    // PWM
    xTaskCreate(pwm_init_task, "pwm_init_task", 4096, NULL, 5, NULL);

    // GPIO
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT); // red led
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT); // blue led
    gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT); // green led

    gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(GPIO_NUM_16, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(GPIO_NUM_17, GPIO_PULLDOWN_ONLY);

    gpio_config_t GPIO0_conf =
    {
    	.intr_type = GPIO_INTR_HIGH_LEVEL,
		.pin_bit_mask = GPIO_INPUT_PIN_SEL,
    	.mode = GPIO_MODE_INPUT,
    	.pull_down_en = 1
    };

    gpio_config(&GPIO0_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreate(GPIO_task, "GPIO_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_NUM_0, GPIO_isr, (void*) GPIO_NUM_0);

    printf("blu led on at start \n");

  	vTaskDelay(1000 / portTICK_PERIOD_MS);

  	gpio_set_level(GPIO_NUM_4, 0);
  	gpio_set_level(GPIO_NUM_16, 0);
  	gpio_set_level(GPIO_NUM_17, 0);

  	vTaskDelay(1000 / portTICK_PERIOD_MS);

  	gpio_set_level(GPIO_NUM_16, 1);

  	vTaskDelay(5000 / portTICK_PERIOD_MS);

  	gpio_set_level(GPIO_NUM_16, 0);

  	esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 1);

  	// SCHEDULER
    xTaskCreate(task1, "task1", 2048, NULL, 5, &task1_handle);
    xTaskCreate(task2, "task2", 2048, NULL, 3, &task2_handle);

    xTaskCreatePinnedToCore(task3, "task3", 2048, NULL, 3, &task3_handle, 0); // task3 run on core 0
    xTaskCreatePinnedToCore(task4, "task4", 2048, NULL, 3, &task4_handle, 1); // task4 run on core 1

    // ADC
    xTaskCreate(adc_init_task, "adc_init_task", 4096, NULL, 5, NULL);

    // WIFI
    xTaskCreate(wifi_init_task, "wifi_init_task", 4096, NULL, 5, NULL);

    while(true)
    {
    	printf("main loop in execution \n");

    	green_led = !green_led;

    	gpio_set_level(GPIO_NUM_17, green_led);
    	gpio_set_level(GPIO_NUM_4, 0);

    	adc_read();

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

