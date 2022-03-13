/* 2.9" Waveshare ePaper Driver Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unl` required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_log.h"
#include "epaper-29-ws.h"
#include "epaper_fonts.h"
#include "driver/gpio.h"
#include "app_main.h"
#include "epaper-29-ws.h"

static const char *TAG = "ePaper Example";
extern const unsigned char IMAGE_DATA[];

// Pin definition of the ePaper module
#define MOSI_PIN 7
#define MISO_PIN -1
#define SCK_PIN 6
#define BUSY_PIN 3
#define DC_PIN 8
#define RST_PIN 9
#define CS_PIN 10

void e_paper_task(void *pvParameter)
{
    epaper_handle_t device = NULL;
    ms_board_configuration *data_ptr = (ms_board_configuration *)pvParameter;
    char current_message[128] = {0};

    epaper_conf_t epaper_conf = {
        .busy_pin = BUSY_PIN,
        .cs_pin = CS_PIN,
        .dc_pin = DC_PIN,
        .miso_pin = MISO_PIN,
        .mosi_pin = MOSI_PIN,
        .reset_pin = RST_PIN,
        .sck_pin = SCK_PIN,

        .rst_active_level = 0,
        .busy_active_level = 0,

        .dc_lev_data = 1,
        .dc_lev_cmd = 0,

        .clk_freq_hz = 10000000,
        .spi_host = SPI2_HOST,

        .width = EPD_WIDTH,
        .height = EPD_HEIGHT,
        .color_inv = 0,
    };


    

    ESP_LOGI(TAG, "Before ePaper driver init, heap: %d", esp_get_free_heap_size());
    device = iot_epaper_create(NULL, &epaper_conf);
    iot_epaper_clear(device);

    strcpy(current_message, data_ptr->data);
    iot_set_background(device, IMAGE_DATA);
    int position = iot_center_text(strlen(current_message), (*(epaper_font_array+data_ptr->font))->width, epaper_conf.width );
    iot_epaper_draw_string(device, position , 240, current_message, *(epaper_font_array+data_ptr->font) , COLORED);
    iot_epaper_display_frame(device, NULL);
    
    while (1)
    {

        if(strcmp(current_message, data_ptr->data )) //message changed
        {
            iot_epaper_clear(device);
            iot_set_background(device, IMAGE_DATA);

            memset(current_message, 0, 128);
            strcpy(current_message, data_ptr->data);
            position = iot_center_text(strlen(current_message), (*(epaper_font_array+data_ptr->font))->width, epaper_conf.width );
            iot_epaper_draw_string(device, position, 240, current_message, *(epaper_font_array+data_ptr->font) , COLORED);
            iot_epaper_display_frame(device, NULL);
            ESP_LOGI(TAG, "Changed message");
        }

        vTaskDelay(1000 * data_ptr->display_time / portTICK_PERIOD_MS);        
    }
}
