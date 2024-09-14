#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "driver/gpio.h"


#define BLINK_LED 2	// Built-in LED

#define ESP_OK          0       /*!< esp_err_t value indicating success (no error) */
#define ESP_FAIL        -1      /*!< Generic esp_err_t code indicating failure */

void app_main(void)
{
	char *ourTaskName = pcTaskGetName(NULL);

	ESP_LOGI(ourTaskName, "Hello starting up!\n");

	gpio_reset_pin(BLINK_LED);			// Reset pin to default state
	esp_err_t blink_pin_sts = gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);	// Set the pin to OUTPUT mode

	if (blink_pin_sts != ESP_OK){
		ESP_LOGE(ourTaskName,"Error setting Direction of Pin %d",BLINK_LED);
	}
	gpio_dump_io_configuration(stdout, (1ULL<<1 | 1ULL<<2));	// Check status of the pin

	while(1)	// Run forever
	{
		// Set the pin to high, then low. Intervals of 1 sec
		gpio_set_level(BLINK_LED, 1);
		vTaskDelay(1000);
		gpio_set_level(BLINK_LED, 0);
		vTaskDelay(1000);
	}
}
