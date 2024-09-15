#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"

#include "driver/gpio.h"

#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"



/* The TFT is mapped to SPI 2 in ESP docs*/

#define ESP_HOST	VSPI_HOST
#define CSO			5
#define SCLK		18
#define MISO		19
#define MOSI		23


/* Declaring the funtions which are used in the program */
void vSpiInit(void);
void spi_write_data(uint8_t addr, uint8_t data);
void spi_read_data(uint8_t addr);

#define SPI_TAG		"spi_protocol"

esp_err_t ret;
spi_device_handle_t spi;
void app_main(void)
{
	vSpiInit();

	// LCD address and data
	uint8_t LCD_TOGGE_addr = 0x01;
	uint8_t LCD_ON_data = 0x80;

	// Set LCD screen on
	spi_write_data(LCD_TOGGE_addr, LCD_ON_data);

	// Set GPIOX on
	spi_write_data(0xC7,0x01);

	// PW1config
	spi_write_data(0x8A, 0x8A);

	// Set PWMOut to 255
	spi_write_data(0x8B, 0xFF);

	// Fill screen with white
	spi_write_data(0x60, 0xFF);



	while(1){
		spi_read_data(0x8A);
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

void spi_write_data(uint8_t addr, uint8_t data)
{
	// Configure the transaction_structure
    spi_transaction_t trans_desc = {
        .flags = SPI_TRANS_USE_TXDATA,								// Set the Tx flag
        .tx_data = {addr, data},  									// The host will sent the address first followed by data we provided
        .length = 16,  												// Length of the address + data is 16 bitss
    };

    gpio_set_level(CSO, 0);
    printf("Writing '0x%x' data at 0x%x\n", data, addr);

    ret = spi_device_polling_transmit(spi, &trans_desc);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SPI_TAG, "SPI write operation failed\n");
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(CSO, 1);
    printf("Writing operation completed\n\n");
}

void spi_read_data(uint8_t addr)               // Function to read data at given address
{
    /*If MSB of addr is set the host will read data from slave.
     *If MSB of addr is clear the host will write data on slave.
     */
    uint8_t instruction_to_read_data[2] = {0x80 | addr, 0xFF};             // This line sets the MSB of addr vatiable, as a command to read the loaction.
    spi_transaction_t trans_desc = {                                       // Configure the transaction_structure
                                    .flags = SPI_TRANS_USE_RXDATA,         // Set the Rx flag
                                    .tx_buffer = instruction_to_read_data, // Host need to first transmit the (command + address) to slave which the host wants to read
                                    .rxlength = 8,                        // 8*2 = 16 bit data transfer (MAX)
                                    .length = 16};
    gpio_set_level(CSO, 0);
    ret = spi_device_polling_transmit(spi, &trans_desc);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SPI_TAG, "SPI write operation failed\n");
    }
    printf("Data Read at 0x%x- 0x%x\n\n", addr, trans_desc.rx_data[1]);               // Host can fetch the data that received from the slave from inbuild structure member- rx_data directly
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(CSO, 1);
}
