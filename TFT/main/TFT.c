#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

/* SPI configuration */
#define ESP_HOST  VSPI_HOST
#define CSO         5
#define SCLK        18
#define MISO        19
#define MOSI 		23
#define RESET 		17

#define SPI_TAG    "spi_protocol"


#define RA8875_HDWR 0x14 ///< See datasheet

#define RA8875_HNDFTR 0x15         ///< See datasheet
#define RA8875_HNDFTR_DE_HIGH 0x00 ///< See datasheet
#define RA8875_HNDFTR_DE_LOW 0x80  ///< See datasheet

#define RA8875_HNDR 0x16      ///< See datasheet
#define RA8875_HSTR 0x17      ///< See datasheet
#define RA8875_HPWR 0x18      ///< See datasheet
#define RA8875_HPWR_LOW 0x00  ///< See datasheet
#define RA8875_HPWR_HIGH 0x80 ///< See datasheet

#define RA8875_VDHR0 0x19     ///< See datasheet
#define RA8875_VDHR1 0x1A     ///< See datasheet
#define RA8875_VNDR0 0x1B     ///< See datasheet
#define RA8875_VNDR1 0x1C     ///< See datasheet
#define RA8875_VSTR0 0x1D     ///< See datasheet
#define RA8875_VSTR1 0x1E     ///< See datasheet
#define RA8875_VPWR 0x1F      ///< See datasheet
#define RA8875_VPWR_LOW 0x00  ///< See datasheet
#define RA8875_VPWR_HIGH 0x80 ///< See datasheet

#define RA8875_HSAW0 0x30 ///< See datasheet
#define RA8875_HSAW1 0x31 ///< See datasheet
#define RA8875_VSAW0 0x32 ///< See datasheet
#define RA8875_VSAW1 0x33 ///< See datasheet

#define RA8875_HEAW0 0x34 ///< See datasheet
#define RA8875_HEAW1 0x35 ///< See datasheet
#define RA8875_VEAW0 0x36 ///< See datasheet
#define RA8875_VEAW1 0x37 ///< See datasheet

#define RA8875_MCLR 0x8E            ///< See datasheet
#define RA8875_MCLR_START 0x80      ///< See datasheet
#define RA8875_MCLR_STOP 0x00       ///< See datasheet
#define RA8875_MCLR_READSTATUS 0x80 ///< See datasheet
#define RA8875_MCLR_FULL 0x00       ///< See datasheet
#define RA8875_MCLR_ACTIVE 0x40     ///< See datasheet

esp_err_t ret;
spi_device_handle_t spi;

uint16_t _width, _height;
uint8_t _textScale;
uint8_t _rotation;
uint8_t _voffset;

/* Function declarations */
void vSpiInit(void);
void spi_write_command(uint8_t addr, uint8_t data);
void spi_read_sts(void);
void spi_read_register(uint8_t addr);
void lcd_init(void);




void app_main(void)
{
	//Hardware RESET

    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(RESET, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(RESET, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Init bus
    vSpiInit();

    lcd_init();

    spi_read_sts();
    // Set LCD screen on
    spi_write_command(0x01, 0x80);

    // Set GPIOX on
    spi_write_command(0xC7, 0x01);

    // PW1config
    spi_write_command(0x8A, 0x8A);

    // Set PWMOut to 255
    spi_write_command(0x8B, 0xFF);

    // Select RGB565
    spi_write_command(0x10, 0x08);


	// RBG565
    // Background Color Red 5bits
    spi_write_command(0x60, 0x1F);

    // Background Color Green 6bits
    spi_write_command(0x61, 0x75);	

    // Background Color Blue 5bits
    spi_write_command(0x62, 0x14);

    while (1) {
    	spi_read_sts();
        //spi_read_register(0x08);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void vSpiInit(void)
{

	gpio_set_direction(RESET, GPIO_MODE_OUTPUT);
    // Configure CSO pin as output
    gpio_set_direction(CSO, GPIO_MODE_OUTPUT);
    gpio_set_level(CSO, 1); 										// Set CS high initially

    spi_bus_config_t buscfg = {
        .miso_io_num = MISO,
        .mosi_io_num = MOSI,
        .sclk_io_num = SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5 * 1000 * 1000, 							// 12MHz
        .mode = 0,
        .spics_io_num = CSO,
        .queue_size = 7
    };

    ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO); 	// Initialize the SPI bus
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi); 			// Attach the Slave device to SPI bus
    ESP_ERROR_CHECK(ret);
}


void spi_write_command(uint8_t addr, uint8_t data)
{
    // First transaction: Send command (0x80 + address)
spi_transaction_t trans_desc1 = {
    .flags      = SPI_TRANS_USE_TXDATA,               				// Use tx_data for command + address
    .tx_data    = {0x80, addr},           							// Send 0x80 (command) and addr (address)
    .length     = 16,                                 				// Send 16 bits: 8 bits for 0x80 and 8 bits for addr
};

// Second transaction: Send 0x00 + data
spi_transaction_t trans_desc2 = {
    .flags      = SPI_TRANS_USE_TXDATA,               				// Use tx_data for 0x00 + data
    .tx_data    = {0x00, data},           							// Send 0x00 (no-op) and data
    .length     = 16,                                 				// Send 16 bits: 8 bits for 0x00 and 8 bits for data
};

    // Perform the first transaction (send command + address)
    gpio_set_level(CSO, 0);    										// Activate Chip Select (CS Low)
    ret = spi_device_polling_transmit(spi, &trans_desc1);  			// Send 0x80 + addr
    if (ret != ESP_OK) {
        ESP_LOGE(SPI_TAG, "SPI Command Write (0x80 + addr) failed\n");
    }
    gpio_set_level(CSO, 1);    										// Deactivate Chip Select (CS High)

    vTaskDelay(1 / portTICK_PERIOD_MS);      						// Short delay

    // Perform the second transaction (send 0x00 + data)
    gpio_set_level(CSO, 0);    										// Activate Chip Select (CS Low)
    ret = spi_device_polling_transmit(spi, &trans_desc2);  			// Send 0x00 + data
    if (ret != ESP_OK) {
        ESP_LOGE(SPI_TAG, "SPI Data Write (0x00 + data) failed\n");
    }
    gpio_set_level(CSO, 1);    										// Deactivate Chip Select (CS High)

    vTaskDelay(1 / portTICK_PERIOD_MS);      						// Short delay to ensure the transaction is complete
}


void spi_read_register(uint8_t addr) {
    uint8_t read_data;

    // Write Command (0x80 + addr) to transaction
    spi_transaction_t write_command_transaction = {
        .flags      = SPI_TRANS_USE_TXDATA,
        .tx_data    = {0x80, addr},
        .length     = 16   											// 8 bits for 0x80, 8 bits for addr
    };

    // Combined read command (0x40) and read response in one transaction
    spi_transaction_t read_transaction = {
        .flags      = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,  // Both TX and RX
        .tx_data    = {0x40, 0x00},  								// Send the read command (0x40) followed by a dummy byte
        .rxlength   = 16,            								// Expecting 8 bits response after 8 bits command
        .length     = 16             								// Total transfer length is 16 bits (8 for command + 8 for data)
    };

    // 1. Send the write command (0x80 + addr)
    gpio_set_level(CSO, 0);    										// Activate Chip Select (CS Low)
    ret = spi_device_polling_transmit(spi, &write_command_transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(SPI_TAG, "SPI Write Command failed\n");
    }

        // Small delay
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(CSO, 1);   										// Deactivate Chip Select (CS High)



    // 2. Send the read command (0x40) and receive the data in the same transaction
    gpio_set_level(CSO, 0);    										// Activate Chip Select (CS Low)
    ret = spi_device_polling_transmit(spi, &read_transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(SPI_TAG, "SPI Read Transaction failed\n");
    }
        // Small delay to ensure the transaction is complete
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(CSO, 1);    										// Deactivate Chip Select (CS High)

    // The response from the device will be in rx_data[1] (second byte)
    read_data = read_transaction.rx_data[1];  						// Read the second byte
    printf("Data Read 0x%x -> 0x%x\n", addr, read_data);


}



void spi_read_sts(void){
	uint8_t read_data;

	// Write Command (0x80 + addr) to transaction
    spi_transaction_t write_command_transaction = {
        .flags      = SPI_TRANS_USE_TXDATA,
        .tx_data    = {0x80, 0x01},									// Dummy address
        .length     = 16   											// 8 bits for 0x80, 8 bits for addr
    };

	// STS combined read command (0xC0) and read response in one transaction
    spi_transaction_t read_transaction = {
        .flags      = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,  // Both TX and RX
        .tx_data    = {0xC0, 0x00},  								// Send the status read command (0xC0) followed by a dummy byte
        .rxlength   = 16,            								// Expecting 8 bits response after 8 bits command
        .length     = 16             								// Total transfer length is 16 bits (8 for command + 8 for data)
    };

    // 1. Send the write command (0x80 + addr)
    gpio_set_level(CSO, 0);    										// Activate Chip Select (CS Low)
    ret = spi_device_polling_transmit(spi, &write_command_transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(SPI_TAG, "SPI Write Command failed\n");
    }

        // Small delay
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(CSO, 1);   										// Deactivate Chip Select (CS High)



    // 2. Send the read command (0x40) and receive the data in the same transaction
    gpio_set_level(CSO, 0);    										// Activate Chip Select (CS Low)
    ret = spi_device_polling_transmit(spi, &read_transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(SPI_TAG, "SPI Read Transaction failed\n");
    }


    // Small delay to ensure the transaction is complete
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(CSO, 1);    										// Deactivate Chip Select (CS High)

    // The response from the device will be in rx_data[1] (second byte)
    read_data = read_transaction.rx_data[1];  						// Read the second byte
    printf("Status Read -> 0x%x\n", read_data);

}

//Initialises the driver IC (clock setup, etc.)
void lcd_init(void)
{
	 /* Timing values */
	uint8_t pixclk;
	uint8_t hsync_start;
	uint8_t hsync_pw;
	uint8_t hsync_finetune;
	uint8_t hsync_nondisp;
	uint8_t vsync_pw;
	uint16_t vsync_nondisp;
	uint16_t vsync_start;


	hsync_nondisp = 26;
	hsync_start = 32;
	hsync_pw = 96;
	hsync_finetune = 0;
	vsync_nondisp = 32;
	vsync_start = 23;
	vsync_pw = 2;
	_voffset = 0;

	// Perform a SW-based reset
    // spi_write_command(0x01, 0x01);
    // spi_write_command(0x01, 0x00);
    // vTaskDelay(10 / portTICK_PERIOD_MS);

    // Initialise the PLL
    spi_write_command(0x88, 0x00 + 11);
    spi_write_command(0x89, 0x02);

    // Color
    spi_write_command(0x10, 0x0C | 0x00);

    // Timing values
    spi_write_command(0x04, 0x80 | 0x01);


	/* Horizontal settings registers */
	spi_write_command(RA8875_HDWR, (_width / 8) - 1); // H width: (HDWR + 1) * 8 = 480
	spi_write_command(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
	spi_write_command(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) /
	                        8); // H non-display: HNDR * 8 + HNDFTR + 2 = 10
	spi_write_command(RA8875_HSTR, hsync_start / 8 - 1); // Hsync start: (HSTR + 1)*8
	spi_write_command(RA8875_HPWR,
	       RA8875_HPWR_LOW +
	           (hsync_pw / 8 - 1)); // HSync pulse width = (HPWR+1) * 8

	/* Vertical settings registers */
	spi_write_command(RA8875_VDHR0, (uint16_t)(_height - 1 + _voffset) & 0xFF);
	spi_write_command(RA8875_VDHR1, (uint16_t)(_height - 1 + _voffset) >> 8);
	spi_write_command(RA8875_VNDR0, vsync_nondisp - 1); // V non-display period = VNDR + 1
	spi_write_command(RA8875_VNDR1, vsync_nondisp >> 8);
	spi_write_command(RA8875_VSTR0, vsync_start - 1); // Vsync start position = VSTR + 1
	spi_write_command(RA8875_VSTR1, vsync_start >> 8);
	spi_write_command(RA8875_VPWR,
	       RA8875_VPWR_LOW + vsync_pw - 1); // Vsync pulse width = VPWR + 1

	/* Set active window X */
	spi_write_command(RA8875_HSAW0, 0); // horizontal start point
	spi_write_command(RA8875_HSAW1, 0);
	spi_write_command(RA8875_HEAW0, (uint16_t)(_width - 1) & 0xFF); // horizontal end point
	spi_write_command(RA8875_HEAW1, (uint16_t)(_width - 1) >> 8);

	/* Set active window Y */
	spi_write_command(RA8875_VSAW0, 0 + _voffset); // vertical start point
	spi_write_command(RA8875_VSAW1, 0 + _voffset);
	spi_write_command(RA8875_VEAW0,
	       (uint16_t)(_height - 1 + _voffset) & 0xFF); // vertical end point
	spi_write_command(RA8875_VEAW1, (uint16_t)(_height - 1 + _voffset) >> 8);

	/* ToDo: Setup touch panel? */

	/* Clear the entire window */
	spi_write_command(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
	vTaskDelay(500 / portTICK_PERIOD_MS);
}