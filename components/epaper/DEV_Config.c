/*****************************************************************************
* | File      	:   DEV_Config.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*----------------
* |	This version:   V3.0
* | Date        :   2019-07-31
* | Info        :   
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of theex Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "DEV_Config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/spi_types.h"
#include "driver/spi_master.h"
#include "esp_log.h"

spi_device_handle_t spi;

#define SPI_TAG "SPI"

/**
 * GPIO read and write
**/
void DEV_Digital_Write(uint16_t Pin, uint8_t Value)
{
	gpio_set_level(Pin, Value);
}

uint8_t DEV_Digital_Read(uint16_t Pin)
{
	return gpio_get_level(Pin);
}

/**
 * SPI
**/
void DEV_SPI_WriteByte(uint8_t Value)
{

	esp_err_t ret;
    spi_transaction_t t;
	memset(&t, 0, sizeof(Value));
    t.length=8;                 
    t.tx_buffer=&Value;
	t.rx_buffer=NULL;
	t.rxlength = 0;

	ESP_LOGW(SPI_TAG, "Data to be send: %hhu Compared to Value:%hhu\n", *(uint8_t*) t.tx_buffer, Value);
	ESP_LOGW(SPI_TAG, "Their Size: %hhu\n", t.length);

	ret=spi_device_transmit(spi, &t);  //Transmit!
	ESP_ERROR_CHECK(ret);

	printf("Transmission finished!\n");
}

/**
 * delay x ms
**/
void DEV_Delay_ms(uint32_t xms)
{
	vTaskDelay(xms / portTICK_PERIOD_MS);
}

void DEV_GPIO_Init(void)
{
	gpio_set_direction(EPD_RST_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(EPD_DC_PIN, GPIO_MODE_OUTPUT);
	//gpio_set_direction(EPD_CS_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(EPD_BUSY_PIN, GPIO_MODE_INPUT);
	//gpio_set_direction(EPD_SPICLK_PIN, GPIO_MODE_OUTPUT);

	DEV_Digital_Write(EPD_CS_PIN, 1);
}
/******************************************************************************
function:	Module Initialize, the library and initialize the pins, SPI protocol
parameter:
Info:
******************************************************************************/
uint8_t DEV_Module_Init(void)
{	
	ESP_LOGI(SPI_TAG, "Initializing GPIO...");
	DEV_GPIO_Init();

	DEV_Delay_ms(100);

	ESP_LOGI(SPI_TAG, "Initializing bus SPI...");
	    
	spi_bus_config_t buscfg={
	.miso_io_num=-1,
	.mosi_io_num=EPD_DATA_PIN,
	.sclk_io_num=EPD_SPICLK_PIN,
	.quadwp_io_num=-1,
	.quadhd_io_num=-1,
	.max_transfer_sz=16*320*2+8
    };

	spi_device_interface_config_t devcfg={
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=EPD_CS_PIN,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    //    .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };


	esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO );
	ESP_ERROR_CHECK(ret);

	ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    return 0;
}

/******************************************************************************
function:	Module exits, closes SPI and BCM2835 library
parameter:
Info:
******************************************************************************/
void DEV_Module_Exit(void)
{
	DEV_Digital_Write(EPD_CS_PIN, 0);
	DEV_Digital_Write(EPD_DC_PIN, 0);
	DEV_Digital_Write(EPD_RST_PIN, 0);

	spi_bus_remove_device(spi);
	spi_bus_free(SPI2_HOST);
}
