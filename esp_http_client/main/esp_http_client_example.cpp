/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "app_wifi.h"

#include "esp_http_client.h"

static const char *TAG = "HTTP_CLIENT";

class HttpClient {
    esp_http_client_config_t client_config;
    esp_http_client_handle_t client;

public:
    HttpClient() : client_config { .url = "http://httpbin.org/get" } {
        client = esp_http_client_init(&client_config);
    }

    void doGet() {
        esp_http_client_set_url(client, "http://httpbin.org/get");
        esp_err_t err = esp_http_client_perform(client);
        esp_http_client_set_method(client, HTTP_METHOD_GET);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
                     esp_http_client_get_status_code(client),
                     esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
        }
    }

    void doPost() {
        // POST
        const char *post_data = "field1=value1&field2=value2";
        esp_http_client_set_url(client, "http://httpbin.org/post");
        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_post_field(client, post_data, strlen(post_data));
        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                     esp_http_client_get_status_code(client),
                     esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
        }
    }

    void doPut() {
        //PUT
        esp_http_client_set_url(client, "http://httpbin.org/put");
        esp_http_client_set_method(client, HTTP_METHOD_PUT);
        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HTTP PUT Status = %d, content_length = %d",
                     esp_http_client_get_status_code(client),
                     esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE(TAG, "HTTP PUT request failed: %s", esp_err_to_name(err));
        }
    }

    void doPatch() {
        //PATCH
        esp_http_client_set_url(client, "http://httpbin.org/patch");
        esp_http_client_set_method(client, HTTP_METHOD_PATCH);
        esp_http_client_set_post_field(client, nullptr, 0);
        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HTTP PATCH Status = %d, content_length = %d",
                     esp_http_client_get_status_code(client),
                     esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE(TAG, "HTTP PATCH request failed: %s", esp_err_to_name(err));
        }
    }

    void doDelete() {
        //DELETE
        esp_http_client_set_url(client, "http://httpbin.org/delete");
        esp_http_client_set_method(client, HTTP_METHOD_DELETE);
        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HTTP DELETE Status = %d, content_length = %d",
                     esp_http_client_get_status_code(client),
                     esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE(TAG, "HTTP DELETE request failed: %s", esp_err_to_name(err));
        }
    }

    void doHead() {
        //HEAD
        esp_http_client_set_url(client, "http://httpbin.org/get");
        esp_http_client_set_method(client, HTTP_METHOD_HEAD);
        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HTTP HEAD Status = %d, content_length = %d",
                     esp_http_client_get_status_code(client),
                     esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE(TAG, "HTTP HEAD request failed: %s", esp_err_to_name(err));
        }
    }

    ~HttpClient() {
        esp_http_client_cleanup(client);
    }
};

extern "C" {

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    app_wifi_initialise();
    app_wifi_wait_connected();
//    tcpip_adapter_dns_info_t dnsInfo;
//    tcpip_adapter_get_dns_info(TCPIP_ADAPTER_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &dnsInfo);
//    ESP_LOGI(TAG, "DNS address: " IPSTR, IP2STR(&dnsInfo.ip.u_addr.ip4));
    ESP_LOGI(TAG, "Creating HttpClient");

    while(1) {
        HttpClient client;
        ESP_LOGI(TAG, "Doing GET");
        client.doGet();
        ESP_LOGI(TAG, "Doing POST");
        client.doPost();
        ESP_LOGI(TAG, "Doing PUT");
        client.doPut();
        ESP_LOGI(TAG, "Doing PATCH");
        client.doPatch();
        ESP_LOGI(TAG, "Doing DELETE");
        client.doDelete();
        ESP_LOGI(TAG, "Doing HEAD");
        client.doHead();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

}
