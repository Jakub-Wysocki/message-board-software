// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"

#include "epaper-29-ws.h"

static const char* TAG = "ePaper Driver";

#define EPAPER_QUE_SIZE_DEFAULT 10


uint8_t Voltage_Frame_7IN5_V2[]={
	0x6, 0x3F, 0x3F, 0x11, 0x24, 0x7, 0x17,
};

uint8_t LUT_VCOM_7IN5_V2[]={	
	0x0,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0xF,	0x1,	0xF,	0x1,	0x2,	
	0x0,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
};						

uint8_t LUT_WW_7IN5_V2[]={	
	0x10,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x84,	0xF,	0x1,	0xF,	0x1,	0x2,	
	0x20,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
};

uint8_t LUT_BW_7IN5_V2[]={	
	0x10,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x84,	0xF,	0x1,	0xF,	0x1,	0x2,	
	0x20,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
};

uint8_t LUT_WB_7IN5_V2[]={	
	0x80,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x84,	0xF,	0x1,	0xF,	0x1,	0x2,	
	0x40,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
};

uint8_t LUT_BB_7IN5_V2[]={	
	0x80,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x84,	0xF,	0x1,	0xF,	0x1,	0x2,	
	0x40,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
};

static portMUX_TYPE epaper_spinlock = portMUX_INITIALIZER_UNLOCKED;
#define EPAPER_ENTER_CRITICAL(mux)    portENTER_CRITICAL(mux)
#define EPAPER_EXIT_CRITICAL(mux)     portEXIT_CRITICAL(mux)

// LCD data/command
typedef struct {
    uint8_t dc_io;
    uint8_t dc_level;
} epaper_dc_t;

typedef struct {
    spi_device_handle_t bus;
    epaper_conf_t pin;      /* EPD properties */
    epaper_paint_t paint;   /* Paint properties */
    epaper_dc_t dc;
    xSemaphoreHandle spi_mux;
} epaper_dev_t;

/* This function is called (in irq context!) just before a transmission starts.
 * It will set the D/C line to the value indicated in the user field
 */
static void iot_epaper_pre_transfer_callback(spi_transaction_t *t)
{
    epaper_dc_t *dc = (epaper_dc_t *) t->user;
    gpio_set_level((int)dc->dc_io, (int)dc->dc_level);
}

static esp_err_t _iot_epaper_spi_send(spi_device_handle_t spi, spi_transaction_t* t)
{
    return spi_device_transmit(spi, t);
}

void iot_epaper_send(spi_device_handle_t spi, const uint8_t *data, int len, epaper_dc_t *dc)
{
    esp_err_t ret;
    if (len == 0) {
        return;    // no need to send anything
    }
    spi_transaction_t t = {
        .length = len * 8,  // Len is in bytes, transaction length is in bits.
        .tx_buffer = data,
        .user = (void *) dc,
    };
    ret = _iot_epaper_spi_send(spi, &t);
    assert(ret == ESP_OK);
}

static void iot_epaper_send_command(epaper_handle_t dev, unsigned char command)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    device->dc.dc_io = device->pin.dc_pin;
    device->dc.dc_level = device->pin.dc_lev_cmd;
    iot_epaper_send(device->bus, &command, 1, &device->dc);
}

static void iot_epaper_send_byte(epaper_handle_t dev, const uint8_t data)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    device->dc.dc_io = device->pin.dc_pin;
    device->dc.dc_level = device->pin.dc_lev_data;
    iot_epaper_send(device->bus, &data, 1, &device->dc);
}

static void iot_epaper_send_data(epaper_handle_t dev, const uint8_t *data, int length)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    device->dc.dc_io = device->pin.dc_pin;
    device->dc.dc_level = device->pin.dc_lev_data;
    iot_epaper_send(device->bus, data, length, &device->dc);
    ESP_LOGI(TAG, "SPI data sent %d", length);
}

static void iot_epaper_paint_init(epaper_handle_t dev, unsigned char* image, int width, int height)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    device->paint.rotate = E_PAPER_ROTATE_0;
    device->paint.image = image;
    /* 1 byte = 8 pixels, so the width should be the multiple of 8 */
    device->paint.width = width % 8 ? width + 8 - (width % 8) : width;
    device->paint.height = height;
}

static void iot_epaper_gpio_init(epaper_conf_t * pin)
{
    gpio_set_direction(pin->reset_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin->reset_pin, pin->rst_active_level);
    gpio_set_direction(pin->dc_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin->dc_pin, 1);
    ets_delay_us(10000);
    gpio_set_level(pin->dc_pin, 0);
    gpio_set_direction(pin->busy_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin->busy_pin, GPIO_PULLUP_ONLY);
}

static esp_err_t iot_epaper_spi_init(epaper_handle_t dev, spi_device_handle_t *e_spi, epaper_conf_t *pin)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,  // MISO not used, we are transferring to the slave only
        .mosi_io_num = pin->mosi_pin,
        .sclk_io_num = pin->sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // The maximum size sent below covers the case
        // when the whole frame buffer is transferred to the slave
        .max_transfer_sz = EPD_WIDTH * EPD_HEIGHT / 8,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = pin->clk_freq_hz,
        .mode = 0,  // SPI mode 0
        .spics_io_num = pin->cs_pin,
        // To Do: clarify what does it mean
        .queue_size = EPAPER_QUE_SIZE_DEFAULT,
        // We are sending only in one direction (to the ePaper slave)
        .flags = (SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE),
        //Specify pre-transfer callback to handle D/C line
        .pre_cb = iot_epaper_pre_transfer_callback,
    };
    ret = spi_bus_initialize(pin->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(pin->spi_host, &devcfg, e_spi);
    assert(ret == ESP_OK);
    return ret;
}


static void iot_7in5_v2_lut(epaper_handle_t dev, uint8_t* lut_vcom,  uint8_t* lut_ww, uint8_t* lut_bw, uint8_t* lut_wb, uint8_t* lut_bb)
{
    uint8_t count;
    
	iot_epaper_send_command(dev, 0x20); //VCOM	
	for(count=0; count<42; count++)
		iot_epaper_send_byte(dev, lut_vcom[count]);

	iot_epaper_send_command(dev, 0x21); //LUTBW
	for(count=0; count<42; count++)
		iot_epaper_send_byte(dev, lut_ww[count]);

	iot_epaper_send_command(dev, 0x22); //LUTBW
	for(count=0; count<42; count++)
		iot_epaper_send_byte(dev, lut_bw[count]);

	iot_epaper_send_command(dev, 0x23); //LUTWB
	for(count=0; count<42; count++)
		iot_epaper_send_byte(dev, lut_wb[count]);

	iot_epaper_send_command(dev, 0x24); //LUTBB
	for(count=0; count<42; count++)
		iot_epaper_send_byte(dev, lut_bb[count]);
}


static void iot_epaper_epd_init(epaper_handle_t dev)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);

    iot_epaper_reset(dev);

    /* This part of code is ePaper module specific
     * It has been copied from the instructions as it is
     */
    iot_epaper_send_command(dev, 0x01);
    iot_epaper_send_byte(dev, 0x17);
    iot_epaper_send_byte(dev, *(Voltage_Frame_7IN5_V2+6));
    iot_epaper_send_byte(dev, *(Voltage_Frame_7IN5_V2+1));
    iot_epaper_send_byte(dev, *(Voltage_Frame_7IN5_V2+2));
    iot_epaper_send_byte(dev, *(Voltage_Frame_7IN5_V2+3));

    iot_epaper_send_command(dev, 0x82);
    iot_epaper_send_byte(dev, *(Voltage_Frame_7IN5_V2+4));

    iot_epaper_send_command(dev, 0x06);
    iot_epaper_send_byte(dev, 0x27);
    iot_epaper_send_byte(dev, 0x27);
    iot_epaper_send_byte(dev, 0x2f);
    iot_epaper_send_byte(dev, 0x17);

    iot_epaper_send_command(dev, 0x30);
    iot_epaper_send_byte(dev, *(Voltage_Frame_7IN5_V2+0));

    iot_epaper_send_command(dev, 0x04);

    vTaskDelay(100 / portTICK_RATE_MS);

    iot_epaper_wait_idle(dev);

    iot_epaper_send_command(dev, 0x00);
    iot_epaper_send_byte(dev, 0x3f);

    iot_epaper_send_command(dev, 0x61);
    iot_epaper_send_byte(dev, 0x03);
    iot_epaper_send_byte(dev, 0x20);
    iot_epaper_send_byte(dev, 0x01);
    iot_epaper_send_byte(dev, 0xe0);

    iot_epaper_send_command(dev, 0x15);
    iot_epaper_send_byte(dev, 0x00);

    iot_epaper_send_command(dev, 0x50);
    iot_epaper_send_byte(dev, 0x10);
    iot_epaper_send_byte(dev, 0x00);

    iot_epaper_send_command(dev, 0x60);
    iot_epaper_send_byte(dev, 0x22);

    iot_epaper_send_command(dev, 0x65);
    iot_epaper_send_byte(dev, 0x00);
    iot_epaper_send_byte(dev, 0x00);
    iot_epaper_send_byte(dev, 0x00);
    iot_epaper_send_byte(dev, 0x00);

    iot_7in5_v2_lut(dev, LUT_VCOM_7IN5_V2, LUT_WW_7IN5_V2, LUT_BW_7IN5_V2, LUT_WB_7IN5_V2, LUT_BB_7IN5_V2);
    xSemaphoreGiveRecursive(device->spi_mux);
}

epaper_handle_t iot_epaper_create(spi_device_handle_t bus, epaper_conf_t *epconf)
{
    ESP_LOGD(TAG, "INSIDE IOT_EPAPER_CREATE_FUN");
    epaper_dev_t* dev = (epaper_dev_t*) calloc(1, sizeof(epaper_dev_t));
    dev->spi_mux = xSemaphoreCreateRecursiveMutex();
    uint8_t* frame_buf = (unsigned char*) heap_caps_malloc(
            (epconf->width * epconf->height / 8), MALLOC_CAP_8BIT);
    if (frame_buf == NULL) {
        ESP_LOGE(TAG, "frame_buffer malloc fail");
        return NULL;
    }
    iot_epaper_gpio_init(epconf);
    ESP_LOGD(TAG, "gpio init ok");
    if (bus) {
        dev->bus = bus;
    } else {
        iot_epaper_spi_init(dev, &(dev->bus), epconf);
        ESP_LOGD(TAG, "spi init ok");
    }
    dev->pin = *epconf;
    iot_epaper_epd_init(dev);
    iot_epaper_paint_init(dev, frame_buf, epconf->width, epconf->height);
    return (epaper_handle_t) dev;
}

esp_err_t iot_epaper_delete(epaper_handle_t dev, bool del_bus)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;

    iot_epaper_sleep(dev);

    spi_bus_remove_device(device->bus);
    if (del_bus) {
        spi_bus_free(device->pin.spi_host);
    }
    vSemaphoreDelete(device->spi_mux);
    if (device->paint.image) {
        free(device->paint.image);
        device->paint.image = NULL;
    }
    free(device);
    return ESP_OK;
}

int iot_epaper_get_width(epaper_handle_t dev)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    return device->paint.width;
}

void iot_epaper_set_width(epaper_handle_t dev, int width)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    device->paint.width = width % 8 ? width + 8 - (width % 8) : width;
    xSemaphoreGiveRecursive(device->spi_mux);

}

int iot_epaper_get_height(epaper_handle_t dev)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    return device->paint.height;
}

void iot_epaper_set_height(epaper_handle_t dev, int height)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    device->paint.height = height;
    xSemaphoreGiveRecursive(device->spi_mux);
}

int iot_epaper_get_rotate(epaper_handle_t dev)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    return device->paint.rotate;
}

void iot_epaper_set_rotate(epaper_handle_t dev, int rotate)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    device->paint.rotate = rotate;
    xSemaphoreGiveRecursive(device->spi_mux);
}

/**
 *  @brief: Getters and Setters
 */
unsigned char* iot_epaper_get_image(epaper_handle_t dev)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    return device->paint.image;
}

/**
 *  @brief: this draws a pixel by absolute coordinates.
 *          this function won't be affected by the rotate parameter.
 */
static void iot_epaper_draw_absolute_pixel(epaper_handle_t dev, int x, int y, int colored)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    if (x < 0 || x >= device->paint.width || y < 0 || y >= device->paint.height) {
        return;
    }
    EPAPER_ENTER_CRITICAL(&epaper_spinlock);
    if (device->pin.color_inv) {
        if (colored) {
            device->paint.image[(x + y * device->paint.width) / 8] |= 0x80 >> (x % 8);
        } else {
            device->paint.image[(x + y * device->paint.width) / 8] &= ~(0x80 >> (x % 8));
        }
    } else {
        if (colored) {
            device->paint.image[(x + y * device->paint.width) / 8] &= ~(0x80 >> (x % 8));
        } else {
            device->paint.image[(x + y * device->paint.width) / 8] |= 0x80 >> (x % 8);
        }
    }
    EPAPER_EXIT_CRITICAL(&epaper_spinlock);
}

void iot_epaper_clean_paint(epaper_handle_t dev, int colored)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    for (int x = 0; x < device->paint.width; x++) {
        for (int y = 0; y < device->paint.height; y++) {
            iot_epaper_draw_absolute_pixel(dev, x, y, colored);
        }
    }
    xSemaphoreGiveRecursive(device->spi_mux);
}

void iot_epaper_clear(epaper_handle_t dev)
{

    uint32_t Width, Height;
    Width = EPD_WIDTH / 8 ;
    Height = EPD_HEIGHT;

    uint16_t i;
    iot_epaper_send_command(dev, 0x10);
    for(i=0; i<Height*Width; i++) {
        iot_epaper_send_byte(dev, 0xFF);
    }
    iot_epaper_send_command(dev, 0x13);
    for(i=0; i<Height*Width; i++)	{
        iot_epaper_send_byte(dev, 0x00);
    }
    iot_turn_on_display(dev);

}
void iot_turn_on_display(epaper_handle_t dev)
{
    iot_epaper_send_command(dev, 0x12);
    vTaskDelay(100/ portTICK_RATE_MS);
    iot_epaper_wait_idle(dev);
}
/**
 *  @brief: this displays a string on the frame buffer but not refresh
 */
void iot_epaper_draw_string(epaper_handle_t dev, int x, int y, const char* text, epaper_font_t* font, int colored)
{
    const char* p_text = text;
    unsigned int counter = 0;
    int refcolumn = x;
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    /* Send the string character by character on EPD */
    while (*p_text != 0) {
        /* Display one character on EPD */
        iot_epaper_draw_char(dev, refcolumn, y, *p_text, font, colored);
        /* Decrement the column position by 16 */
        refcolumn += font->width;
        /* Point on the next character */
        p_text++;
        counter++;
    }
    xSemaphoreGiveRecursive(device->spi_mux);
}

/**
 *  @brief: this draws a pixel by the coordinates
 */
void iot_epaper_draw_pixel(epaper_handle_t dev, int x, int y, int colored)
{
    int point_temp;
    epaper_dev_t* device = (epaper_dev_t*) dev;
    if (device->paint.rotate == E_PAPER_ROTATE_0) {
        if (x < 0 || x >= device->paint.width || y < 0 || y >= device->paint.height) {
            return;
        }
        iot_epaper_draw_absolute_pixel(dev, x, y, colored);
    } else if (device->paint.rotate == E_PAPER_ROTATE_90) {
        if (x < 0 || x >= device->paint.height || y < 0 || y >= device->paint.width) {
            return;
        }
        point_temp = x;
        x = device->paint.width - y;
        y = point_temp;
        iot_epaper_draw_absolute_pixel(dev, x, y, colored);
    } else if (device->paint.rotate == E_PAPER_ROTATE_180) {
        if (x < 0 || x >= device->paint.width || y < 0 || y >= device->paint.height) {
            return;
        }
        x = device->paint.width - x;
        y = device->paint.height - y;
        iot_epaper_draw_absolute_pixel(dev, x, y, colored);
    } else if (device->paint.rotate == E_PAPER_ROTATE_270) {
        if (x < 0 || x >= device->paint.height || y < 0 || y >= device->paint.width) {
            return;
        }
        point_temp = x;
        x = y;
        y = device->paint.height - point_temp;
        iot_epaper_draw_absolute_pixel(dev, x, y, colored);
    }
}

/**
 *  @brief: this draws a character on the frame buffer but not refresh
 */
void iot_epaper_draw_char(epaper_handle_t dev, int x, int y, char ascii_char, epaper_font_t* font, int colored)
{
    int i, j;
    unsigned int char_offset = (ascii_char - ' ') * font->height * (font->width / 8 + (font->width % 8 ? 1 : 0));
    const unsigned char* ptr = &font->font_table[char_offset];
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    for (j = 0; j < font->height; j++) {
        for (i = 0; i < font->width; i++) {
            if (*ptr & (0x80 >> (i % 8))) {
                iot_epaper_draw_pixel(dev, x + i, y + j, colored);
            }
            if (i % 8 == 7) {
                ptr++;
            }
        }
        if (font->width % 8 != 0) {
            ptr++;
        }
    }
    xSemaphoreGiveRecursive(device->spi_mux);
}

/**
 *  @brief: this draws a line on the frame buffer
 */
void iot_epaper_draw_line(epaper_handle_t dev, int x0, int y0, int x1, int y1,
        int colored)
{
    /* Bresenham algorithm */
    int dx = x1 - x0 >= 0 ? x1 - x0 : x0 - x1;
    int sx = x0 < x1 ? 1 : -1;
    int dy = y1 - y0 <= 0 ? y1 - y0 : y0 - y1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    while ((x0 != x1) && (y0 != y1)) {
        iot_epaper_draw_pixel(dev, x0, y0, colored);
        if (2 * err >= dy) {
            err += dy;
            x0 += sx;
        }
        if (2 * err <= dx) {
            err += dx;
            y0 += sy;
        }
    }
    xSemaphoreGiveRecursive(device->spi_mux);
}

/**
 *  @brief: this draws a horizontal line on the frame buffer
 */
void iot_epaper_draw_horizontal_line(epaper_handle_t dev, int x, int y, int width, int colored)
{
    int i;
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    for (i = x; i < x + width; i++) {
        iot_epaper_draw_pixel(dev, i, y, colored);
    }
    xSemaphoreGiveRecursive(device->spi_mux);
}

/**
 *  @brief: this draws a vertical line on the frame buffer
 */
void iot_epaper_draw_vertical_line(epaper_handle_t dev, int x, int y, int height, int colored)
{
    int i;
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    for (i = y; i < y + height; i++) {
        iot_epaper_draw_pixel(dev, x, i, colored);
    }
    xSemaphoreGiveRecursive(device->spi_mux);
}


void iot_epaper_wait_idle(epaper_handle_t dev)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    while (gpio_get_level((gpio_num_t) device->pin.busy_pin) == device->pin.busy_active_level) {
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void iot_epaper_reset(epaper_handle_t dev)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    gpio_set_level((gpio_num_t) device->pin.reset_pin, (~(device->pin.rst_active_level)) & 0x1);
    ets_delay_us(200);
    gpio_set_level((gpio_num_t) device->pin.reset_pin, (device->pin.rst_active_level) & 0x1);             //module reset
    ets_delay_us(200);
    gpio_set_level((gpio_num_t) device->pin.reset_pin, (~(device->pin.rst_active_level)) & 0x1);
    iot_epaper_wait_idle(dev);
    xSemaphoreGiveRecursive(device->spi_mux);
}

/* This transfer to the display the whole image frame
 */
void iot_epaper_display_frame(epaper_handle_t dev, const unsigned char* frame_buffer)
{   
    epaper_dev_t* device = (epaper_dev_t*) dev;
    if(frame_buffer == NULL)
    {
        frame_buffer = device->paint.image;
    }

    if (frame_buffer != NULL) {
        xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);

        iot_epaper_send_command(dev, 0x13);
        iot_epaper_send_data(dev, frame_buffer, EPD_HEIGHT*EPD_WIDTH / 8);

        iot_turn_on_display(dev);

        xSemaphoreGiveRecursive(device->spi_mux);
    }
}

void iot_epaper_sleep(epaper_handle_t dev)
{
    epaper_dev_t* device = (epaper_dev_t*) dev;
    xSemaphoreTakeRecursive(device->spi_mux, portMAX_DELAY);
    iot_epaper_send_command(dev, 0x02);
    iot_epaper_wait_idle(dev);
    iot_epaper_send_command(dev, 0x07);
    iot_epaper_send_byte(dev, 0xA5);
    xSemaphoreGiveRecursive(device->spi_mux);
}

