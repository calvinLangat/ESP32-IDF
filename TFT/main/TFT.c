#include <stdio.h>
#include "esp_log.h"

#include "driver/gpio.h"

#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"



/* The TFT is mapped to SPI 2 in ESP docs*/

#define ESP_HOST	VSPI_HOST
#define CSO			15
#define SCLK		14
#define MISO		12
#define MOSI		13


/* Declaring the funtions which are used in the program */
void vSpiInit(void);


#define SPI_TAG		"spi_protocol"

esp_err_t ret;
spi_device_handle_t spi;
void app_main(void)
{
	vSpiInit();

	while(1){
		;;
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void vSpiInit(void)
{
	gpio_set_direction(CSO, GPIO_MODE_OUTPUT);

	spi_bus_config_t buscfg = {
		.miso_io_num = MISO,
		.mosi_io_num = MOSI,
		.sclk_io_num = SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 512 * 8		// 4095 bytes is the max size of data that can be sent because of hardware limitations
	};

	spi_device_interface_config_t devcfg = {

		.clock_speed_hz = 12 * 1000 * 1000,			// 12MHz
		.mode = 0,
		.spics_io_num = CSO,
		.queue_size = 2
	};

	ret = spi_bus_initialize(ESP_HOST, &buscfg, SPI_DMA_CH_AUTO);	// Initialize the SPI bus
	ESP_ERROR_CHECK(ret);

	ret = spi_bus_add_device(ESP_HOST, & devcfg, &spi);				// Attach the Slave device to SPI bus
}
