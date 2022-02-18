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

static const char *TAG = "ePaper Example";
extern const unsigned char IMAGE_DATA[];

// Pin definition of the ePaper module
#define MOSI_PIN    7
#define MISO_PIN    -1
#define SCK_PIN     6
#define BUSY_PIN    3
#define DC_PIN      8
#define RST_PIN     9
#define CS_PIN      10

// Color inverse. 1 or 0 = set or reset a bit if set a colored pixel
#define IF_INVERT_COLOR 1

void e_paper_task(void *pvParameter)
{
    epaper_handle_t device = NULL;

    epaper_conf_t epaper_conf = {
        .busy_pin = BUSY_PIN,
        .cs_pin = CS_PIN,
        .dc_pin = DC_PIN,
        .miso_pin = MISO_PIN,
        .mosi_pin = MOSI_PIN,
        .reset_pin = RST_PIN,
        .sck_pin = SCK_PIN,

        .rst_active_level = 0,
        .busy_active_level = 1,

        .dc_lev_data = 1,
        .dc_lev_cmd = 0,

        .clk_freq_hz = 10000000,
        .spi_host = SPI2_HOST,

        .width = EPD_WIDTH,
        .height = EPD_HEIGHT,
        .color_inv = 0,
    };

    while(1){
        ESP_LOGI(TAG, "Before ePaper driver init, heap: %d", esp_get_free_heap_size());
        device = iot_epaper_create(NULL, &epaper_conf);
                    
        iot_epaper_clean_paint(device, UNCOLORED);
        iot_epaper_draw_string(device, 50, 50, "Lodz Univeristy of Technology", &epaper_font_24, COLORED);
        
        
        iot_epaper_display_frame(device, NULL); 
        ESP_LOGI(TAG, "Done");
        iot_epaper_delete(device, true);
        vTaskDelay(1000 * 60 / portTICK_PERIOD_MS);
    }
}
