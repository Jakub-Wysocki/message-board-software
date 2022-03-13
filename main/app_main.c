#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "app_main.h"

static const char *TAG = "MQTT_EXAMPLE";

ms_board_configuration mqtt_data;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, "/eink/message", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "/eink/time", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "/eink/font-size", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        strcpy(mqtt_data.data, "Device connected with server. No message");
        mqtt_data.data_len = strlen("Device connected. No message");

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        strcpy(mqtt_data.data, "Display not connected to a mqtt server");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        if (strncmp(event->topic, "/eink/message", event->topic_len) == 0)
        {
                memset(mqtt_data.data, 0, 128);
                strncpy(mqtt_data.data, event->data, event->data_len);
                mqtt_data.data_len = event->data_len;
                ESP_LOGI(TAG, "CHANGED MESSAGE");
        }
        else if (strncmp(event->topic, "/eink/time", event->topic_len) == 0)
        {
                mqtt_data.display_time = (int)strtol(event->data, NULL, 10);
                ESP_LOGI(TAG, "CHANGED TIME");
        }
        else if (strncmp(event->topic, "/eink/font-size", event->topic_len) == 0)
        {
                mqtt_data.font = (short)strtol(event->data, NULL, 10) % 5;
                ESP_LOGI(TAG, "CHANGED FONT - TODO");
        }
        else
            ESP_LOGI(TAG, "WRONG MQTT DATA");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
    return ESP_OK;
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = URI,
        .event_handle = mqtt_event_handler,
        .username = "User",
        .password = "Password",
    };

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    mqtt_data = (ms_board_configuration){
        .data = "Display not connected to a mqtt server",
        .data_len = strlen("Display not connected to a mqtt server"),
        .display_time = 20,
        .font = 3,
    };

    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

    xTaskCreate(&e_paper_task, "epaper_task", 4 * 1024, &mqtt_data, 5, NULL);
}
