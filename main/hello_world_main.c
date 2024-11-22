#include <stdio.h> 
#include <string.h>
#include <stdlib.h>
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
// #include "driver/adc.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_vfs.h"
#include <esp_spiffs.h>
#include <esp_http_server.h>

// #include "driver/sdmmc_host.h"
// #include "driver/sdmmc_defs.h"
#include "esp_vfs_fat.h"

#define SLEEP_TIME_SEC 60  // 슬립 시간 (초 단위)
#define RUN_TIME_SEC   60  // 런타임 (초 단위)
#define INTERVAL_MS    10000 // 힙 메모리 표시 주기 (밀리초 단위)

#define DEF_SSID "nurirobot"
#define DEF_PW "nuri0625"
#define STORAGE_PARTITION_LABEL "storage"

static const char *TAG = "main";

// Wi-Fi 이벤트 그룹
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// Wi-Fi 초기화 상태 변수
static bool wifi_initialized = false;

// ADC 채널 및 GPIO 핀 설정 (GPIO2 사용)
#define BATTERY_ADC_CHANNEL ADC2_CHANNEL_2 // GPIO2

#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)
#define MOUNT_POINT "/sdcard"
#define MAX_FILE_SIZE   (200*1024) // 200 KB
#define MAX_FILE_SIZE_STR "200KB"
#define SCRATCH_BUFSIZE  1024

struct file_server_data {
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
};

void init_sd_card()
{
    ESP_LOGI(TAG, "SD 카드(SPI) 초기화 중...");

    // SPI 호스트 초기화
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 13, // MOSI 핀
        .miso_io_num = 12, // MISO 핀
        .sclk_io_num = 14, // CLK 핀
        .quadwp_io_num = -1, // 사용하지 않음
        .quadhd_io_num = -1, // 사용하지 않음
        .max_transfer_sz = 4 * 1024,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI 버스 초기화 실패: %s", esp_err_to_name(ret));
        return;
    }

    // SD 카드 슬롯 설정
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = 15; // CS 핀
    slot_config.host_id = host.slot;

    // FATFS 마운트 설정
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_card_t *card;
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "SD 카드 마운트 실패. FATFS 포맷 오류.");
        } else {
            ESP_LOGE(TAG, "SD 카드 초기화 실패: %s", esp_err_to_name(ret));
        }
        spi_bus_free(host.slot);
        return;
    }

    ESP_LOGI(TAG, "SD 카드가 성공적으로 마운트되었습니다.");
    ESP_LOGI(TAG, "카드 이름: %s", card->cid.name);

    // SD 카드 기본 정보 출력
    ESP_LOGI(TAG, "파일 시스템 크기: %lluMB", ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
}

void write_to_sdcard()
{
    const char *file_path = MOUNT_POINT "/test.txt";
    FILE *file = fopen(file_path, "w");
    if (!file) {
        ESP_LOGE(TAG, "파일 열기 실패: %s", file_path);
        return;
    }
    fprintf(file, "Hello, SPI SD card!\n");
    fclose(file);
    ESP_LOGI(TAG, "파일 작성 완료: %s", file_path);
}

// void read_from_sdcard()
// {
//     const char *file_path = MOUNT_POINT "/test.txt";
//     FILE *file = fopen(file_path, "r");
//     if (!file) {
//         ESP_LOGE(TAG, "파일 열기 실패: %s", file_path);
//         return;
//     }
//     char line[128];
//     while (fgets(line, sizeof(line), file) != NULL) {
//         printf("%s", line);
//     }
//     fclose(file);
//     ESP_LOGI(TAG, "파일 읽기 완료: %s", file_path);
// }

// SPIFFS 초기화
void init_spiffs()
{
    ESP_LOGI(TAG, "SPIFFS 초기화 중...");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = STORAGE_PARTITION_LABEL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS 초기화 실패: %s", esp_err_to_name(ret));
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS 정보 가져오기 실패: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SPIFFS 마운트 완료 - 총 크기: %d 바이트, 사용량: %d 바이트", total, used);

    // SPIFFS 파일 목록 출력
    struct dirent *entry;
    DIR *dir = opendir("/spiffs");
    if (dir == NULL) {
        ESP_LOGE(TAG, "SPIFFS 디렉토리 열기 실패");
        return;
    }
    ESP_LOGI(TAG, "SPIFFS 파일 목록:");
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "  %s", entry->d_name);
    }
    closedir(dir);
}

esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".pdf")) {
        return httpd_resp_set_type(req, "application/pdf");
    } else if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".jpeg")) {
        return httpd_resp_set_type(req, "image/jpeg");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    } else if (IS_FILE_EXT(filename, ".js.gz")) {
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_set_type(req, "text/javascript");
    } else if (IS_FILE_EXT(filename, ".css.gz")) {
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_set_type(req, "text/css");
    } else if (IS_FILE_EXT(filename, ".js")) {
        return httpd_resp_set_type(req, "text/javascript");
    } else if (IS_FILE_EXT(filename, ".css")) {
        return httpd_resp_set_type(req, "text/css");
    } else if (IS_FILE_EXT(filename, ".png")) {
        return httpd_resp_set_type(req, "image/png");
    }

    return httpd_resp_set_type(req, "text/plain");
}

const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest) {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash) {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize) {
        /* Full path string won't fit into destination buffer */
        return NULL;
    }

    /* Construct full path (base + path) */
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;
}

/* Handler to upload a file onto the server */
// esp_err_t upload_post_handler(httpd_req_t *req)
// {
//     char filepath[512];
//     FILE *file = NULL;
//     int received;

//     // MOUNT_POINT와 업로드된 파일 경로를 결합
//     snprintf(filepath, sizeof(filepath), MOUNT_POINT "%s", req->uri + 7); // "/upload" 제거


//     ESP_LOGI(TAG, "파일 생성 : %s %d", filepath, req->content_len);
//     file = fopen(filepath, "w");
//     if (!file) {
//         ESP_LOGE(TAG, "파일 생성 실패: %s", filepath);
//         httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "파일 생성 실패");
//         return ESP_FAIL;
//     }

//     char scratch[SCRATCH_BUFSIZE];
//     int remaining = req->content_len;

//     // ESP_LOGI(TAG, "파일 생성 start : %d", remaining);

//     while (remaining > 0) {
//         if ((received = httpd_req_recv(req, scratch, MIN(remaining, SCRATCH_BUFSIZE))) <= 0) {
//             fclose(file);
//             unlink(filepath); // 실패 시 파일 삭제
//             ESP_LOGE(TAG, "파일 수신 실패");
//             httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "파일 수신 실패");
//             return ESP_FAIL;
//         }

//         // ESP_LOGI(TAG, "파일 생성 : %d", remaining);
//         if (received && (fwrite(scratch, 1, received, file) != received)) {
//             ESP_LOGE(TAG, "파일 쓰기 실패1");
//             fclose(file);
//             ESP_LOGE(TAG, "파일 쓰기 실패2");
//             unlink(filepath); // 실패 시 파일 삭제
//             ESP_LOGE(TAG, "파일 쓰기 실패");
//             httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "파일 쓰기 실패");
//             return ESP_FAIL;
//         }

//         remaining -= received;
//     }

//     fclose(file);
//     // ESP_LOGI(TAG, "파일 업로드 완료: %s", filepath);
//     // write_to_sdcard();
//     httpd_resp_sendstr(req, "파일 업로드 성공");
//     return ESP_OK;
// }

esp_err_t upload_post_handler(httpd_req_t *req) {
    char filepath[512];
    FILE *file = NULL;
    char *boundary = NULL;
    char scratch[SCRATCH_BUFSIZE];
    char content_type[128];  // Content-Type 헤더 값 저장
    int received;
    bool is_file_field = false;
    size_t content_length = req->content_len;

    // Content-Type 헤더에서 boundary 추출
    if (httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof(content_type)) == ESP_OK) {
        char *boundary_str = strstr(content_type, "boundary=");
        if (boundary_str) {
            boundary = strdup(boundary_str + 9); // "boundary=" 이후 부분 복사
        }
    }

    if (!boundary) {
        ESP_LOGE(TAG, "Boundary를 찾을 수 없습니다.");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid multipart request");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Boundary: %s", boundary);

    int remaining = content_length; // 남은 본문 크기
    while (remaining > 0) {
        received = httpd_req_recv(req, scratch, MIN(SCRATCH_BUFSIZE, remaining));
        if (received <= 0) {
            ESP_LOGE(TAG, "데이터 수신 실패");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive data");
            if (file) fclose(file);
            if (boundary) free(boundary);
            return ESP_FAIL;
        }
        remaining -= received;
        ESP_LOGI(TAG, "remaining: %d", remaining);

        char *data_start = scratch;
        char *data_end = scratch + received;

        // Boundary 처리
        if (strstr(data_start, boundary)) {
            if (file) {
                fclose(file);
                ESP_LOGI(TAG, "파일 저장 완료");
                file = NULL;
            }
            is_file_field = false; // 새로운 part가 시작됨
        }

        // Content-Disposition 확인
        char *content_disposition = strstr(data_start, "Content-Disposition:");
        if (content_disposition) {
            char *filename_start = strstr(content_disposition, "filename=\"");
            if (filename_start) {
                filename_start += 10; // "filename=\"" 건너뜀
                char *filename_end = strchr(filename_start, '"');
                if (filename_end) {
                    snprintf(filepath, sizeof(filepath), MOUNT_POINT "/%.*s",
                             (int)(filename_end - filename_start), filename_start);
                    ESP_LOGI(TAG, "파일 경로: %s", filepath);
                    file = fopen(filepath, "w");
                    if (!file) {
                        ESP_LOGE(TAG, "파일 생성 실패: %s", filepath);
                        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");
                        if (boundary) free(boundary);
                        return ESP_FAIL;
                    }
                    is_file_field = true; // 파일 파트 시작
                }
            }
        }

        // 파일 내용 쓰기
        if (is_file_field && file) {
            char *body_start = strstr(data_start, "\r\n\r\n");
            if (body_start) {
                body_start += 4; // 헤더 끝 ("\r\n\r\n") 건너뜀
                size_t body_len = data_end - body_start;

                if (fwrite(body_start, 1, body_len, file) != body_len) {
                    ESP_LOGE(TAG, "파일 쓰기 실패");
                    fclose(file);
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to write to file");
                    if (boundary) free(boundary);
                    return ESP_FAIL;
                }
            }
        }
    }

    if (file) fclose(file);
    if (boundary) free(boundary);

    ESP_LOGI(TAG, "멀티파트 데이터 처리 완료");
    httpd_resp_sendstr(req, "File uploaded successfully");
    return ESP_OK;
}


/* Handler to delete a file from the server */
esp_err_t delete_post_handler(httpd_req_t *req)
{
    char filepath[512];

    // MOUNT_POINT와 삭제할 파일 경로를 결합
    snprintf(filepath, sizeof(filepath), MOUNT_POINT "/%s", req->uri + 8); // "/delete/" 제거

    if (unlink(filepath) == 0) {
        ESP_LOGI(TAG, "파일 삭제 성공: %s", filepath);
        httpd_resp_sendstr(req, "파일 삭제 성공");
    } else {
        ESP_LOGE(TAG, "파일 삭제 실패: %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "파일 삭제 실패");
    }
    return ESP_OK;
}


// HTTP GET 요청 핸들러
esp_err_t http_get_handler(httpd_req_t *req)
{
    char filepath[64 + 1024];
    snprintf(filepath, sizeof(filepath), "/spiffs%s", req->uri);

    // 파일 열기
    FILE *file = fopen(filepath, "r");
    if (!file) {
        ESP_LOGE(TAG, "파일 열기 실패: %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "파일을 찾을 수 없습니다.");
        return ESP_FAIL;
    }

    char buffer[512];
    size_t read_bytes;
    set_content_type_from_file(req, filepath);

    // 파일 내용 전송
    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    fclose(file);

    // 응답 종료
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

esp_err_t http_get_image_handler(httpd_req_t *req)
{
    char filepath[64 + 1024];
    snprintf(filepath, sizeof(filepath), "/sdcard%s", req->uri + 6);

    // 파일 열기
    FILE *file = fopen(filepath, "r");
    if (!file) {
        ESP_LOGE(TAG, "파일 열기 실패: %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "파일을 찾을 수 없습니다.");
        return ESP_FAIL;
    }

    char buffer[512];
    size_t read_bytes;
    set_content_type_from_file(req, filepath);

    // 파일 내용 전송
    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    fclose(file);

    // 응답 종료
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// HTTP 서버 시작
void start_web_server()
{
    httpd_handle_t server = NULL;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.recv_wait_timeout = 30; 
    ESP_LOGI(TAG, "HTTP 서버 시작 중...");

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t get_image_uri = {
            .uri = "/image/*", // 모든 요청 처리
            .method = HTTP_GET,
            .handler = http_get_image_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &get_image_uri);


        /* URI handler for uploading files to server */
        httpd_uri_t file_upload = {
            .uri       = "/upload",   // Match all URIs of type /upload/path/to/file
            .method    = HTTP_POST,
            .handler   = upload_post_handler,
            .user_ctx  = NULL    // Pass server data as context
        };
        httpd_register_uri_handler(server, &file_upload);

        /* URI handler for deleting files from server */
        httpd_uri_t file_delete = {
            .uri       = "/delete/*",   // Match all URIs of type /delete/path/to/file
            .method    = HTTP_POST,
            .handler   = delete_post_handler,
            .user_ctx  = NULL    // Pass server data as context
        };
        httpd_register_uri_handler(server, &file_delete);        

        httpd_uri_t get_uri = {
            .uri = "/*", // 모든 요청 처리
            .method = HTTP_GET,
            .handler = http_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &get_uri);        
        ESP_LOGI(TAG, "HTTP 서버가 실행 중입니다.");
    } else {
        ESP_LOGE(TAG, "HTTP 서버 시작 실패");
    }
}

// HTTP 서버 타스크
void web_server_task(void *pvParameters)
{
    init_spiffs();
    init_sd_card();
    // write_to_sdcard();
    // read_from_sdcard();
    start_web_server();
    vTaskDelete(NULL);
}

float read_battery_voltage(void)
{
    int adc_raw = 0;
    esp_err_t r;

    // Wi-Fi가 활성화되지 않았으므로 ADC2를 바로 사용할 수 있습니다.
    // r = adc2_get_raw(BATTERY_ADC_CHANNEL, ADC_WIDTH_BIT_12, &adc_raw);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    adc_oneshot_unit_handle_t adc2_handle;
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //-------------ADC2 Config---------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_2, &config));

    r = adc_oneshot_read(adc2_handle, ADC_CHANNEL_2, &adc_raw);

    if (r == ESP_OK)
    {
        // ADC의 최대 값
        const int adc_max = 4095; // 12비트 해상도

        // ADC 입력 전압 (0 ~ 3.3V)
        float adc_voltage = ((float)adc_raw / adc_max) * 3.3;

        ESP_LOGI(TAG, "ADC 측정 전압: %.2f V", adc_voltage);

        float battery_voltage = adc_voltage * ((10.0 + 10.0) / 10.0); // 배터리 전압 = adc_voltage * 2

        return battery_voltage;
    }
    else
    {
        ESP_LOGE(TAG, "ADC2 읽기 오류");
        return 0.0;
    }
}

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

    while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "시간 동기화 대기 중... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    time(&now);
    localtime_r(&now, &timeinfo);

    if (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET)
    {
        ESP_LOGE(TAG, "시간 동기화 실패");
    }
    else
    {
        ESP_LOGI(TAG, "시간 동기화 성공");
    }

    esp_sntp_stop(); // SNTP 클라이언트 중지
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
            .ssid = DEF_SSID,
            .password = DEF_PW
            // .threshold.authmode = WIFI_AUTH_WPA2_PSK, // 필요한 경우 인증 모드 설정
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Wi-Fi 시작
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta 완료");

    // Wi-Fi 초기화 상태 업데이트
    wifi_initialized = true;

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
    if (wifi_initialized)
    {
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_stop());
        ESP_ERROR_CHECK(esp_wifi_deinit());

        // 이벤트 루프 삭제
        ESP_ERROR_CHECK(esp_event_loop_delete_default());

        // 이벤트 그룹 삭제
        vEventGroupDelete(s_wifi_event_group);

        wifi_initialized = false;
    }
}

void check_battery_and_control_wifi(void)
{
    float battery_voltage = read_battery_voltage();
    ESP_LOGI(TAG, "배터리 전압: %.2f V", battery_voltage);

    if (battery_voltage < 2.0)
    {
        ESP_LOGI(TAG, "배터리가 없습니다. 슬립 모드로 진입하지 않습니다.");
    }
    else if (battery_voltage <= 4.1)
    {
        ESP_LOGI(TAG, "배터리 전압이 낮아 슬립 모드로 진입합니다.");
        // 슬립 타이머 설정
        esp_sleep_enable_timer_wakeup(SLEEP_TIME_SEC * 1000000ULL); // 마이크로초 단위

        ESP_LOGI(TAG, "%d초 동안 깊은 슬립에 들어갑니다.", SLEEP_TIME_SEC);

        // 깊은 슬립 시작
        esp_deep_sleep_start();
    }
    else
    {
        ESP_LOGI(TAG, "배터리 전압이 충분하여 Wi-Fi를 활성화합니다.");

        // Wi-Fi 초기화 및 연결
        wifi_init();

        // 시간 동기화
        obtain_time();

        // Wi-Fi 연결 해제 및 정리
        // wifi_cleanup();
    }
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

    // 배터리 전압 확인 및 Wi-Fi 활성화 결정
    check_battery_and_control_wifi();

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

    xTaskCreate(web_server_task, "web_server_task", 1024 * 32, NULL, 5, NULL);

    // 메인 루프
    while (1)
    {
        size_t free_heap_size = esp_get_free_heap_size();
        ESP_LOGI(TAG, "남은 힙 메모리: %d 바이트", free_heap_size);

        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MS)); // INTERVAL_MS 대기
    }
}
