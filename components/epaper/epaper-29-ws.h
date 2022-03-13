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

#ifndef _IOT_E_PAPER_29_H_
#define _IOT_E_PAPER_29_H_

#include "driver/spi_master.h"

// Display orientation
typedef enum {
    E_PAPER_ROTATE_0,
    E_PAPER_ROTATE_90,
    E_PAPER_ROTATE_180,
    E_PAPER_ROTATE_270,
} epaper_rotate_t;

typedef struct
{
    uint16_t width;
    uint16_t height;
    const uint8_t *font_table;
} epaper_font_t;

#define COLORED         0
#define UNCOLORED       1

// Display resolution
#define EPD_WIDTH       800
#define EPD_HEIGHT      480

/* Paint properties */
typedef struct {
    unsigned char* image;
    epaper_rotate_t rotate;
    int width;
    int height;
} epaper_paint_t;

/* EPD properties */
typedef struct {
    int reset_pin;
    int dc_pin;
    int cs_pin;
    int busy_pin;
    int mosi_pin;
    int miso_pin;
    int sck_pin;

    uint8_t rst_active_level;
    uint8_t busy_active_level;
    uint8_t dc_lev_data;
    uint8_t dc_lev_cmd;

    int clk_freq_hz;
    spi_host_device_t spi_host;

    int width;
    int height;
    bool color_inv;
} epaper_conf_t;

typedef void* epaper_handle_t; /*handle of epaper*/


void iot_epaper_clear(epaper_handle_t dev);

void iot_turn_on_display(epaper_handle_t dev);
/**
 * @brief Create and init epaper and return a epaper handle
 *
 * @param bus handle of spi device
 * @param epconf configure struct for epaper device
 *
 * @return
 *     - handle of epaper
 */
epaper_handle_t iot_epaper_create(spi_device_handle_t bus, epaper_conf_t * epconf);

/**
 * @brief   delete epaper handle_t
 *
 * @param dev object handle of epaper
 * @param del_bus whether to delete spi bus
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_epaper_delete(epaper_handle_t dev, bool del_bus);

/**
 * @brief clear display frame buffer
 *
 * @param dev object handle of epaper
 * @param colored to set display color
 */
void iot_epaper_clean_paint(epaper_handle_t dev, int colored);

/**
 * @brief get paint width
 *
 * @param dev object handle of epaper
 *
 * @return
 *     - paint width
 */
int iot_epaper_get_width(epaper_handle_t dev);

/**
 * @brief   set paint width
 * @param dev object handle of epaper
 * @param width paint width
 */
void iot_epaper_set_width(epaper_handle_t dev, int width);

/**
 * @brief   get paint height
 * @param   dev object handle of epaper
 * @return
 *     - paint height
 */
int iot_epaper_get_height(epaper_handle_t dev);

/**
 * @brief   set paint height
 * @param   dev object handle of epaper
 * @param   paint height
 */
void iot_epaper_set_height(epaper_handle_t dev, int height);

/**
 * @brief   get paint rotate
 * @param   dev object handle of epaper
 * @return
 *     - current rotation
 */
int iot_epaper_get_rotate(epaper_handle_t dev);

/**
 * @brief set paint rotate
 * @param dev object handle of epaper
 * @param rotation
 */
void iot_epaper_set_rotate(epaper_handle_t dev, int rotate);

/**
 * @brief get display data
 * @param dev object handle of epaper
 * @return
 *     - Pointer to display data
 */
unsigned char* iot_epaper_get_image(epaper_handle_t dev);

/**
 * @brief   draw string start on point(x,y) and save on display data array,
 *          screen will display when call iot_epaper_display_frame function.
 * @param dev object handle of epaper
 * @param x  poing (x)
 * @param y  poing (y)
 * @param text  display string
 * @param font Font style
 * @param colored display color
 */
void iot_epaper_draw_string(epaper_handle_t dev, int x, int y, const char* text,
        epaper_font_t* font, int colored);

/**
 * @brief   draw pixel and save on display data array,
 *          screen will display when call iot_epaper_display_frame function.
 * @param  dev object handle of epaper
 * @param  x point (x)
 * @param  y point (y)
 * @param  colored display color
 */
void iot_epaper_draw_pixel(epaper_handle_t dev, int x, int y, int colored);

/**
 * @brief   draw char and save on display data array,
 *          screen will display when call iot_epaper_display_frame function.
 * @param   dev object handle of epaper
 * @param  x poing (x)
 * @param  y poing (y)
 * @param  ascii_char display char
 * @param  font font style
 * @param  colored display color
 */
void iot_epaper_draw_char(epaper_handle_t dev, int x, int y, char ascii_char,
        epaper_font_t* font, int colored);

/**
 * @brief   draw line start on point(x0,y0) end on point(x1,y1) and save on display data array,
 *          screen will display when call iot_epaper_display_frame function.
 *
 * @param   dev object handle of epaper
 * @param  x0 poing (x0)
 * @param  y0 poing (y0)
 * @param  x1 poing (x1)
 * @param  y1 poing (y1)
 * @param  colored display color
 */
void iot_epaper_draw_line(epaper_handle_t dev, int x0, int y0, int x1, int y1,
        int colored);

/**
 * @brief   draw horizontal line start on point(x,y) and save on display data array,
 *          screen will display when call iot_epaper_display_frame function.
 * @param   dev object handle of epaper
 * @param  x poing (x)
 * @param  y poing (y)
 * @param  width line width
 * @param  colored display color
 */
void iot_epaper_draw_horizontal_line(epaper_handle_t dev, int x, int y,
        int width, int colored);

/**
 * @brief   draw vertical line start on point(x,y) and save on display data array,
 *          screen will display when call iot_epaper_display_frame function.
 *
 * @param   dev object handle of epaper
 * @param  x poing (x)
 * @param  y poing (y)
 * @param   line height
 * @param   display color
 */
void iot_epaper_draw_vertical_line(epaper_handle_t dev, int x, int y,
        int height, int colored);
/**
 * @brief  wait until idle
 * @param  dev object handle of epaper
 */
void iot_epaper_wait_idle(epaper_handle_t dev);

/**
 * @brief  reset device
 *
 * @param  dev object handle of epaper
 */
void iot_epaper_reset(epaper_handle_t dev);

/**
 * @brief dispaly frame, refresh screen
 *
 * @param dev object handle of epaper
 */
void iot_epaper_display_frame(epaper_handle_t dev, const unsigned char* frame_buffer);

/**
 * @brief   After this command is transmitted, the chip would enter the deep-sleep mode to save power.
 * The deep sleep mode would return to standby by hardware reset. The only one parameter is a
 * check code, the command would be executed if check code = 0xA5.
 * You can use iot_epaper_reset() to awaken and EPD_Init() to initialize
 *
 * @param   dev object handle of epaper
 */
void iot_epaper_sleep(epaper_handle_t dev);



/**
 * @brief   This function sets frame_buffer array as drawing background
 *
 * @param   dev object handle of epaper
 * @param   frame_buffer array containg image to be displayed in background
 */
void iot_set_background(epaper_handle_t dev, const unsigned char* frame_buffer);

int iot_center_text(int length, int letter_width, int screen_width);


#endif  //_IOT_E_PAPER_29_H_

