/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "esp_netif.h"
#include <time.h>
#include <sys/time.h>


#define SLEEP_TIME_SEC 60  // 슬립 시간 (초 단위)
#define RUN_TIME_SEC   60  // 런타임 (초 단위)
#define INTERVAL_MS    1000 // 힙 메모리 표시 주기 (밀리초 단위)

#define DEF_SSID "nurirobot"
#define DEF_PW "nuri0625"

static const char *TAG = "main";

// Wi-Fi 이벤트 그룹
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// Wi-Fi 이벤트 핸들러
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "AP 연결 재시도 중...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP 주소 획득: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void initialize_sntp(void)
{
    ESP_LOGI(TAG, "SNTP 초기화 중");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
}

void obtain_time(void)
{
    initialize_sntp();

    // 시간 동기화가 완료될 때까지 대기
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "시간 동기화 대기 중... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    time(&now);
    localtime_r(&now, &timeinfo);

    if (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET)
    {
        ESP_LOGE(TAG, "시간 동기화 실패");
    }
    else
    {
        ESP_LOGI(TAG, "시간 동기화 성공");
    }

    esp_sntp_stop(); // SNTP 클라이언트 중지
}

bool is_time_set(void)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // tm_year가 1900 이후의 연도인지 확인하여 시간 설정 여부 판단
    if (timeinfo.tm_year < (2016 - 1900))
    {
        ESP_LOGI(TAG, "RTC에 시간이 설정되지 않았습니다.");
        return false;
    }
    else
    {
        ESP_LOGI(TAG, "RTC에 시간이 이미 설정되어 있습니다.");
        return true;
    }
}

void wifi_init(void)
{
    ESP_LOGI(TAG, "Wi-Fi 초기화 중");

    // 이벤트 그룹 생성
    s_wifi_event_group = xEventGroupCreate();

    // TCP/IP 네트워크 인터페이스 초기화
    ESP_ERROR_CHECK(esp_netif_init());

    // 기본 이벤트 루프 생성
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 기본 Wi-Fi STA 생성
    esp_netif_create_default_wifi_sta();

    // Wi-Fi 초기화
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 이벤트 핸들러 등록
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    // Wi-Fi 모드를 STA로 설정
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Wi-Fi 설정
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEF_SSID,       // 여기서 SSID를 입력하세요
            .password = DEF_PW // 여기서 패스워드를 입력하세요
            // .threshold.authmode = WIFI_AUTH_WPA2_PSK, // 필요한 경우 인증 모드 설정
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Wi-Fi 시작
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta 완료");

    // 연결 대기
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "AP에 연결되었습니다. SSID:%s", wifi_config.sta.ssid);
    }
    else
    {
        ESP_LOGE(TAG, "AP에 연결 실패. SSID:%s", wifi_config.sta.ssid);
    }
}

void wifi_cleanup(void)
{
    // Wi-Fi 연결 해제 및 종료
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());

    // 이벤트 루프 삭제
    ESP_ERROR_CHECK(esp_event_loop_delete_default());

    // 이벤트 그룹 삭제
    vEventGroupDelete(s_wifi_event_group);
}

void app_main(void)
{
    // NVS 초기화 (Wi-Fi 사용을 위해 필요)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 시간대 설정 (한국 시간)
    setenv("TZ", "KST-9", 1);
    tzset();

    // RTC에 시간이 설정되어 있는지 확인
    if (!is_time_set())
    {
        // Wi-Fi 초기화 및 연결
        wifi_init();

        // 시간 동기화
        obtain_time();

        // Wi-Fi 연결 해제 및 정리
        wifi_cleanup();
    }

    // 현재 시간 출력
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "현재 시간: %s", asctime(&timeinfo));

    // 깨어난 원인 출력
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch(wakeup_reason)
    {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "타이머에 의해 깨어남");
            break;
        default:
            ESP_LOGI(TAG, "깨어난 원인: %d", wakeup_reason);
            break;
    }

    // 런타임 동안 100ms마다 힙 메모리 표시
    int run_time_ms = RUN_TIME_SEC * 1000;
    int elapsed_time_ms = 0;

    while(elapsed_time_ms < run_time_ms)
    {
        size_t free_heap_size = esp_get_free_heap_size();
        ESP_LOGI(TAG, "남은 힙 메모리: %d 바이트", free_heap_size);

        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MS)); // 100ms 대기

        elapsed_time_ms += INTERVAL_MS;
    }

    // 슬립 타이머 설정
    esp_sleep_enable_timer_wakeup(SLEEP_TIME_SEC * 1000000ULL); // 마이크로초 단위

    ESP_LOGI(TAG, "%d초 동안 깊은 슬립에 들어갑니다.", SLEEP_TIME_SEC);

    // 깊은 슬립 시작
    esp_deep_sleep_start();
}