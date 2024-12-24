#include <stdio.h>
#include <unistd.h>
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

#include "multipart_parser.h"
#include "driver/spi_common.h"

#include "Debug.h"
#include "fonts.h"
#include "GUI_Paint.h"

#define SLEEP_TIME_SEC 60  // 슬립 시간 (초 단위)
#define RUN_TIME_SEC   60  // 런타임 (초 단위)
#define INTERVAL_MS    10000 // 힙 메모리 표시 주기 (밀리초 단위)

// #define DEF_SSID "nurirobot"
// #define DEF_PW "nuri0625"
#define DEF_SSID "ckhome"
#define DEF_PW "akwldrkz!1"

#define SEC_SSID "nurirobot"
#define SEC_PW "nuri0625"
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
#define PARALLEL_LINES 16

#define MAX_BOUNDARY_LENGTH 256
#define ERR(...) fprintf(stderr, __VA_ARGS__)
#define INF(...) fprintf(stderr, __VA_ARGS__)
#define DBG(...)

#define EPD_SCK_PIN     14
#define EPD_MOSI_PIN    13
#define SPI_MISO_PIN    12
#define EPD_CS_PIN      5
#define EPD_DC_PIN      0
#define EPD_RST_PIN     19
#define EPD_BUSY_PIN    4
#define EPD_PWR_PIN     26
#define SD_CS_PIN       15

sdmmc_host_t host = SDSPI_HOST_DEFAULT();
static spi_device_handle_t epd_spi;
static spi_device_handle_t sd_spi;

#define EPD_4IN0E_WIDTH       400
#define EPD_4IN0E_HEIGHT      600

#define EPD_4IN0E_BLACK   0x0   /// 000
#define EPD_4IN0E_WHITE   0x1   /// 001
#define EPD_4IN0E_YELLOW  0x2   /// 010
#define EPD_4IN0E_RED     0x3   /// 011
#define EPD_4IN0E_BLUE    0x5   /// 101
#define EPD_4IN0E_GREEN   0x6   /// 110

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

DRAM_ATTR static const lcd_init_cmd_t epd_init_cmds[] = {
    {0xAA, {0x49, 0x55, 0x20, 0x08, 0x09, 0x18}, 6},
    {0x01, {0x3f}, 1},
    {0x00, {0x5f, 0x69}, 2},
    {0x05, {0x40, 0x1f, 0x1f, 0x2c}, 4},
    {0x08, {0x6f, 0x1f, 0x1f, 0x22}, 4},
    {0x06, {0x6f, 0x1f, 0x17, 0x17}, 4},
    {0x03, {0x00, 0x54, 0x00, 0x44}, 4},
    {0x60, {0x02, 0x00}, 2},
    {0x30, {0x08}, 1},
    {0x50, {0x3f}, 1},
    {0x61, {0x01, 0x90, 0x02, 0x58}, 4},
    {0xe3, {0x2f}, 1},
    {0x84, {0x01}, 1},
    {0, {0}, 0xff},
};

DRAM_ATTR static const lcd_init_cmd_t epd_utils_cmds[] = {
    {0x06, {0x6f, 0x1f, 0x17, 0x27}, 4},
    {0x12, {0x00}, 1},
    {0x02, {0x00}, 1},
    {0x07, {0x00}, 1},
    {0, {0}, 0xff},
};

struct file_server_data {
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
};
static FILE *current_file = NULL;  // 현재 처리 중인 파일
static char file_path[256];        // 저장할 파일 경로

static wifi_config_t wifi_config = {
    .sta = {
        .ssid = DEF_SSID,
        .password = DEF_PW
        // .threshold.authmode = WIFI_AUTH_WPA2_PSK, // 필요한 경우 인증 모드 설정
    },
};

static wifi_config_t wifi_config2 = {
    .sta = {
        .ssid = SEC_SSID,
        .password = SEC_PW
        // .threshold.authmode = WIFI_AUTH_WPA2_PSK, // 필요한 경우 인증 모드 설정
    },
};

void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;                   //Command is 8 bits
    t.tx_buffer = &cmd;             //The data is the cmd itself
    t.user = (void*)0;              //D/C needs to be set to 0
    if (keep_cs_active) {
        t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    }
    ret = spi_device_polling_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);          //Should have had no issues.
}

void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) {
        return;    //no need to send anything
    }
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = len * 8;             //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;             //Data
    t.user = (void*)1;              //D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);          //Should have had no issues.
}

// 콜백 함수: 헤더 필드 처리
static int handle_header_field(multipart_parser *p, const char *at, size_t length)
{
    char field[256];
    snprintf(field, sizeof(field), "%.*s", (int)length, at);
    ESP_LOGI(TAG, "Header Field: %s", field);
    return 0;
}

// 콜백 함수: 헤더 값 처리
static int handle_header_value(multipart_parser *p, const char *at, size_t length)
{
    char value[256];
    snprintf(value, sizeof(value), "%.*s", (int)length, at);

    // 파일 이름 추출
    if (strstr(value, "filename=\"")) {
        char *start = strstr(value, "filename=\"") + 10;
        char *end = strchr(start, '\"');
        if (start && end) {
            snprintf(file_path, sizeof(file_path), MOUNT_POINT "/%.*s", (int)(end - start), start);
            ESP_LOGI(TAG, "Parsed File Name: %s", file_path);

            // 기존 파일 닫기
            if (current_file) {
                fclose(current_file);
                current_file = NULL;
            }

            // 새로운 파일 열기
            current_file = fopen(file_path, "w");
            if (!current_file) {
                ESP_LOGE(TAG, "Failed to open file: %s", file_path);
                return -1;
            }
        }
    }
    ESP_LOGI(TAG, "Header Value: %s", value);
    return 0;
}

// 콜백 함수: 파트 데이터 시작
static int handle_part_data_begin(multipart_parser *p)
{
    ESP_LOGI(TAG, "Part Data Begin");
    return 0;
}

// 콜백 함수: 파트 데이터 처리
static int handle_part_data(multipart_parser *p, const char *at, size_t length)
{
    if (current_file) {
        if (fwrite(at, 1, length, current_file) != length) {
            ESP_LOGE(TAG, "Failed to write data to file");
            fclose(current_file);
            current_file = NULL;
            return -1;
        }
    }
    return 0;
}

// 콜백 함수: 파트 데이터 끝
static int handle_part_data_end(multipart_parser *p)
{
    ESP_LOGI(TAG, "Part Data End");
    if (current_file) {
        fclose(current_file);
        current_file = NULL;
    }
    return 0;
}

// 콜백 함수: 본문 끝
static int handle_body_end(multipart_parser *p)
{
    ESP_LOGI(TAG, "Body End");
    return 0;
}

// 멀티파트 콜백 설정
static struct multipart_parser_settings callbacks = {
    .on_header_field = handle_header_field,
    .on_header_value = handle_header_value,
    .on_part_data_begin = handle_part_data_begin,
    .on_part_data = handle_part_data,
    .on_part_data_end = handle_part_data_end,
    .on_body_end = handle_body_end,
};

void init_sd_card()
{
    ESP_LOGI(TAG, "SD 카드(SPI) 초기화 중...");

    // SPI 호스트 초기화
    // sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    // spi_bus_config_t bus_cfg = {
    //     .miso_io_num = SPI_MISO_PIN,
    //     .mosi_io_num = EPD_MOSI_PIN,
    //     .sclk_io_num = EPD_SCK_PIN,
    //     .quadwp_io_num = -1,
    //     .quadhd_io_num = -1,
    //     .max_transfer_sz = PARALLEL_LINES * 320 * 2 + 8
    // };

    // SD 카드 슬롯 설정
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN; // CS 핀
    slot_config.host_id = host.slot;

    // FATFS 마운트 설정
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

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

void gpio_init() 
{
    gpio_set_direction(EPD_BUSY_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(EPD_RST_PIN,  GPIO_MODE_OUTPUT);
    gpio_set_direction(EPD_DC_PIN,   GPIO_MODE_OUTPUT);
    gpio_set_direction(EPD_PWR_PIN,  GPIO_MODE_OUTPUT);
    gpio_set_direction(EPD_CS_PIN,   GPIO_MODE_OUTPUT);
    gpio_set_direction(SD_CS_PIN,   GPIO_MODE_OUTPUT);

    gpio_set_level(EPD_PWR_PIN, 1);
    gpio_set_level(EPD_CS_PIN,  1);
    gpio_set_level(SD_CS_PIN,   0);
}

void epd_ReadBusyH() {
    ESP_LOGI(TAG, "e-Paper busy H");
    // LOW: busy, HIGH: idle
    // 바뀐 논리에 따라, Busy 핀이 HIGH(1)가 될 때까지 대기
    while (gpio_get_level(EPD_BUSY_PIN) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms 대기
    }

    // Busy pin이 HIGH가 된 후 추가로 200ms 정도 대기
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "e-Paper busy H release");    
}

void epd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(EPD_DC_PIN, dc);
}

// void epd_reset_Ready() {
//     ESP_LOGI(TAG, "e-Paper busy H");
//     // LOW: busy, HIGH: idle
//     // 바뀐 논리에 따라, Busy 핀이 HIGH(1)가 될 때까지 대기
//     int elapsed_ms = 0;
//     const int timeout_ms = 1000; 

//     while (gpio_get_level(EPD_BUSY_PIN) != 0) { 
//         // HIGH 이면 아직 Busy가 아님(또는 상태 이상)
//         vTaskDelay(pdMS_TO_TICKS(10));
//         elapsed_ms += 10;
//         if (elapsed_ms == 500) {
//            ESP_LOGE(TAG, "Busy pin did not go LOW within 1s. Forcing hardware reset.");

//             // 리셋 신호
//             gpio_set_level(EPD_RST_PIN, 0);
//             vTaskDelay(pdMS_TO_TICKS(2));
//             gpio_set_level(EPD_RST_PIN, 1);
//             vTaskDelay(pdMS_TO_TICKS(20));            
//         }

//         if (elapsed_ms >= timeout_ms) esp_restart();
//     }

//     // Busy pin이 HIGH가 된 후 추가로 200ms 정도 대기
//     vTaskDelay(pdMS_TO_TICKS(200));
//     ESP_LOGI(TAG, "e-Paper busy H release");    
// }    

void epd_reset() {
    gpio_set_level(EPD_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));    // 2ms 지연    
    gpio_set_level(EPD_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(20));   // 20ms 지연
    gpio_set_level(EPD_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));    // 2ms 지연
    gpio_set_level(EPD_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(20));   // 20ms 지연    
    epd_ReadBusyH();
}

void epd_init() {
    gpio_set_level(EPD_PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(EPD_PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    epd_reset();
    // epd_ReadBusyH();
    vTaskDelay(pdMS_TO_TICKS(30));
    int cmd = 0;
    while (epd_init_cmds[cmd].databytes != 0xff) {
        lcd_cmd(epd_spi, epd_init_cmds[cmd].cmd, false);
        lcd_data(epd_spi, epd_init_cmds[cmd].data, epd_init_cmds[cmd].databytes & 0x1F);
        if (epd_init_cmds[cmd].databytes & 0x80) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        cmd++;
    }
    epd_ReadBusyH();
}

void epd_turnondisplay() {
    lcd_cmd(epd_spi, 0x04, false);
    epd_ReadBusyH();
    vTaskDelay(pdMS_TO_TICKS(200));

    lcd_cmd(epd_spi, epd_utils_cmds[0].cmd, false);
    lcd_data(epd_spi, epd_utils_cmds[0].data, epd_utils_cmds[0].databytes & 0x1F);
    vTaskDelay(pdMS_TO_TICKS(200));

    lcd_cmd(epd_spi, epd_utils_cmds[1].cmd, false);
    lcd_data(epd_spi, epd_utils_cmds[1].data, epd_utils_cmds[1].databytes & 0x1F);
    vTaskDelay(pdMS_TO_TICKS(200));

    lcd_cmd(epd_spi, epd_utils_cmds[2].cmd, false);
    lcd_data(epd_spi, epd_utils_cmds[2].data, epd_utils_cmds[2].databytes & 0x1F);
    epd_ReadBusyH();
    vTaskDelay(pdMS_TO_TICKS(200));    
}

void epd_sleep() {
    lcd_cmd(epd_spi, epd_utils_cmds[3].cmd, false);
    lcd_data(epd_spi, epd_utils_cmds[3].data, epd_utils_cmds[3].databytes & 0x1F);
    epd_ReadBusyH();
}

void epd_display(const UBYTE *Image) 
{
    uint16_t Width = (EPD_4IN0E_WIDTH % 2 == 0)
                   ? (EPD_4IN0E_WIDTH / 2)
                   : (EPD_4IN0E_WIDTH / 2 + 1);
    uint16_t Height = EPD_4IN0E_HEIGHT;

    size_t buffer_size = Width * Height;
    lcd_cmd(epd_spi, 0x10, false);
    size_t offset = 0;
    while (offset < buffer_size) {
        // 남은 데이터 중에서 chunk 크기 결정
        size_t remain = buffer_size - offset;
        size_t chunk_size = (remain > SOC_SPI_MAXIMUM_BUFFER_SIZE) 
                            ? SOC_SPI_MAXIMUM_BUFFER_SIZE 
                            : remain;

        // color_buffer + offset 위치부터 chunk_size 바이트 전송
        lcd_data(epd_spi, Image + offset, chunk_size);

        offset += chunk_size;
    }    
}

void epd_displaypart(const UBYTE *Image, UWORD xstart, UWORD ystart, UWORD image_width, UWORD image_heigh)
{
    uint16_t Width = (EPD_4IN0E_WIDTH % 2 == 0)
                   ? (EPD_4IN0E_WIDTH / 2)
                   : (EPD_4IN0E_WIDTH / 2 + 1);
    uint16_t Height = EPD_4IN0E_HEIGHT;

    size_t buffer_size = Width * Height;
    ESP_LOGI("EPD", "buffer_size: %d", buffer_size);
    uint8_t *color_buffer = (uint8_t*)malloc(buffer_size);
    if (!color_buffer) {
        ESP_LOGE("EPD", "Failed to allocate color_buffer");
        return;
    }

    for(int i=0; i<Height; i++) {
		for(int j=0; j<Width; j++) {
            size_t idx = i * Width + j;
			if((i<(image_heigh+ystart)) && (i>=ystart) && (j<((image_width+xstart)/2)) && (j>=(xstart/2))) {
                color_buffer[idx] = Image[(j-xstart/2) + (image_width/2*(i-ystart))];
			}
			else {
                color_buffer[idx] = 0x11;
			}
		}
	}    

    lcd_cmd(epd_spi, 0x10, false);
    size_t offset = 0;
    while (offset < buffer_size) {
        // 남은 데이터 중에서 chunk 크기 결정
        size_t remain = buffer_size - offset;
        size_t chunk_size = (remain > SOC_SPI_MAXIMUM_BUFFER_SIZE) 
                            ? SOC_SPI_MAXIMUM_BUFFER_SIZE 
                            : remain;

        // color_buffer + offset 위치부터 chunk_size 바이트 전송
        lcd_data(epd_spi, color_buffer + offset, chunk_size);

        offset += chunk_size;
    }

    free(color_buffer);
    epd_turnondisplay();
}

void epd_clear(uint8_t color)
{
    // Width: 실제 픽셀의 절반 (2픽셀당 1바이트 구조 가정)
    uint16_t Width = (EPD_4IN0E_WIDTH % 2 == 0)
                   ? (EPD_4IN0E_WIDTH / 2)
                   : (EPD_4IN0E_WIDTH / 2 + 1);
    uint16_t Height = EPD_4IN0E_HEIGHT;

    size_t buffer_size = Width * Height;
    ESP_LOGI("EPD", "buffer_size: %d", buffer_size);
    uint8_t *color_buffer = (uint8_t*)malloc(buffer_size);
    if (!color_buffer) {
        ESP_LOGE("EPD", "Failed to allocate color_buffer");
        return;
    }

    // 버퍼 전체를 (color<<4 | color)로 채움
    uint8_t fill_value = (color << 4) | color;
    for (size_t i = 0; i < buffer_size; i++) {
        color_buffer[i] = fill_value;
    }

    // 버퍼 전체를 한 번에 전송
    // SOC_SPI_MAXIMUM_BUFFER_SIZE로 나누어 전송
    lcd_cmd(epd_spi, 0x10, false);
    size_t offset = 0;
    while (offset < buffer_size) {
        // 남은 데이터 중에서 chunk 크기 결정
        size_t remain = buffer_size - offset;
        size_t chunk_size = (remain > SOC_SPI_MAXIMUM_BUFFER_SIZE) 
                            ? SOC_SPI_MAXIMUM_BUFFER_SIZE 
                            : remain;

        // color_buffer + offset 위치부터 chunk_size 바이트 전송
        lcd_data(epd_spi, color_buffer + offset, chunk_size);

        offset += chunk_size;
    }

    // 버퍼 해제
    free(color_buffer);
    // 디스플레이 갱신 명령
    epd_turnondisplay();
}

void spi_init() 
{
    
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = EPD_MOSI_PIN,
        .sclk_io_num = EPD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PARALLEL_LINES * 320 * 2 + 8
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000,     //Clock out at 10 MHz
        .mode = 0,                              //SPI mode 0
        .spics_io_num = EPD_CS_PIN,             //CS pin
        .queue_size = 7,                        //We want to be able to queue 7 transactions at a time
        .pre_cb = epd_spi_pre_transfer_callback,
    };

    //Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(host.slot, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(host.slot, &devcfg, &epd_spi);
    ESP_ERROR_CHECK(ret);    

    spi_device_interface_config_t devcfg1 = {
        .clock_speed_hz = 40 * 1000 * 1000,     //Clock out at 10 MHz
        .mode = 0,                              //SPI mode 0
        .spics_io_num = SD_CS_PIN,             //CS pin
        .queue_size = 7,                        //We want to be able to queue 7 transactions at a time
    };

    ESP_ERROR_CHECK(spi_bus_add_device(host.slot, &devcfg1, &sd_spi));    
}

void epad_init()
{
    //Attach the LCD to the SPI bus
    epd_init();
    // epd_clear(EPD_4IN0E_WHITE);
    // vTaskDelay(pdMS_TO_TICKS(500));

    UBYTE *Image;
    // // UWORD Imagesize = ((EPD_4IN0E_WIDTH % 2 == 0)? (EPD_4IN0E_WIDTH / 2 ): (EPD_4IN0E_WIDTH / 2 + 1)) * EPD_4IN0E_HEIGHT;
    // // Image = (UBYTE *)malloc(Imagesize/6);
    // // Paint_NewImage(Image, EPD_4IN0E_WIDTH/2, EPD_4IN0E_HEIGHT/3, 0, EPD_4IN0E_WHITE);
    // UWORD Imagesize = 200 * 200;
    // Image = (UBYTE *)malloc(Imagesize);
    // Paint_NewImage(Image, 200, 200, 0, EPD_4IN0E_WHITE);    
    // Paint_SetScale(6);
    // Paint_SelectImage(Image);
    // Paint_Clear(EPD_4IN0E_WHITE);    
    // Paint_DrawPoint(10, 80, EPD_4IN0E_RED, DOT_PIXEL_1X1, DOT_STYLE_DFT);
    // Paint_DrawPoint(10, 90, EPD_4IN0E_BLUE, DOT_PIXEL_2X2, DOT_STYLE_DFT);
    // Paint_DrawPoint(10, 100, EPD_4IN0E_GREEN, DOT_PIXEL_3X3, DOT_STYLE_DFT);
    // Paint_DrawRectangle(1, 1, 200, 200, EPD_4IN0E_BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    // Paint_DrawLine(20, 70, 70, 120, EPD_4IN0E_YELLOW, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    // Paint_DrawLine(70, 70, 20, 120, EPD_4IN0E_YELLOW, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    // Paint_DrawRectangle(20, 70, 70, 120, EPD_4IN0E_BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    // Paint_DrawRectangle(80, 70, 130, 120, EPD_4IN0E_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    // Paint_DrawCircle(45, 95, 20, EPD_4IN0E_BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    // Paint_DrawCircle(105, 95, 20, EPD_4IN0E_WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    // Paint_DrawLine(85, 95, 125, 95, EPD_4IN0E_YELLOW, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    // Paint_DrawLine(105, 75, 105, 115, EPD_4IN0E_YELLOW, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    // Paint_DrawNum(10, 33, 123456789, &Font12, EPD_4IN0E_BLACK, EPD_4IN0E_WHITE);
    // Paint_DrawNum(10, 50, 987654321, &Font16, EPD_4IN0E_WHITE, EPD_4IN0E_BLACK);
    
    // Paint_DrawString_EN(145, 0, "Waveshare", &Font16, EPD_4IN0E_BLACK, EPD_4IN0E_WHITE);
    // Paint_DrawString_EN(145, 35, "Waveshare", &Font16, EPD_4IN0E_BLACK, EPD_4IN0E_WHITE);
    // Paint_DrawString_EN(145, 70, "Waveshare", &Font16, EPD_4IN0E_BLACK, EPD_4IN0E_WHITE);
    // Paint_DrawString_EN(145, 104, "Waveshare", &Font16, EPD_4IN0E_BLACK, EPD_4IN0E_WHITE);
    // Paint_DrawString_EN(145, 140, "Waveshare", &Font16, EPD_4IN0E_BLACK, EPD_4IN0E_WHITE);
    // epd_displaypart(Image, 100, 150, 200, 200);

    UWORD Imagesize = EPD_4IN0E_WIDTH * EPD_4IN0E_HEIGHT;
    Image = (UBYTE *)malloc(Imagesize);
    Paint_NewImage(Image, EPD_4IN0E_WIDTH, EPD_4IN0E_HEIGHT, 0, EPD_4IN0E_WHITE);   
    Paint_SetScale(6);
    Paint_SelectImage(Image);
    Paint_Clear(EPD_4IN0E_BLACK);
    epd_display(Image);
    epd_sleep();

    free(Image);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void read_from_sdcard()
{
    const char *file_path = MOUNT_POINT "/test.txt";
    FILE *file = fopen(file_path, "r");
    if (!file) {
        ESP_LOGE(TAG, "파일 열기 실패: %s", file_path);
        return;
    }
    char line[128];
    while (fgets(line, sizeof(line), file) != NULL) {
        printf("%s", line);
    }
    fclose(file);
    ESP_LOGI(TAG, "파일 읽기 완료: %s", file_path);
}

void setSDCardMODE(bool arg) 
{
    if (arg) {
        gpio_set_level(EPD_CS_PIN,  0);
        gpio_set_level(SD_CS_PIN,   1);
    } else {
        gpio_set_level(EPD_CS_PIN,  1);
        gpio_set_level(SD_CS_PIN,   0);
    }
}

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
    // ESP_LOGI(TAG, "SPIFFS 파일 목록:");
    // while ((entry = readdir(dir)) != NULL) {
    //     ESP_LOGI(TAG, "  %s", entry->d_name);
    // }
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

// HTTP POST 핸들러
esp_err_t upload_post_handler(httpd_req_t *req)
{
    char boundary[MAX_BOUNDARY_LENGTH];
    char scratch[SCRATCH_BUFSIZE];
    int received;

    // Content-Type 헤더에서 boundary 추출
    if (httpd_req_get_hdr_value_str(req, "Content-Type", boundary, sizeof(boundary)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get Content-Type header");
        return ESP_FAIL;
    }

    char *boundary_start = strstr(boundary, "boundary=");
    if (!boundary_start) {
        ESP_LOGE(TAG, "Boundary not found in Content-Type");
        return ESP_FAIL;
    }
    boundary_start += 9;  // "boundary=" 건너뜀
    ESP_LOGI(TAG, "Boundary: %s", boundary_start);

    // 멀티파트 파서 초기화
    multipart_parser *parser = multipart_parser_init(boundary_start, &callbacks);
    if (!parser) {
        ESP_LOGE(TAG, "Failed to initialize multipart parser");
        return ESP_FAIL;
    }

    // 본문 처리
    size_t remaining = req->content_len;
    while (remaining > 0) {
        received = httpd_req_recv(req, scratch, MIN(remaining, SCRATCH_BUFSIZE));
        if (received <= 0) {
            ESP_LOGE(TAG, "Failed to receive body");
            multipart_parser_free(parser);
            return ESP_FAIL;
        }

        // ESP_LOG_BUFFER_HEXDUMP(__FUNCTION__, &scratch, received, ESP_LOG_INFO);            

        multipart_parser_execute(parser, scratch, received);
        remaining -= received;
        ESP_LOGI(TAG, "remaining: %d", remaining);
    }

    // 멀티파트 파서 정리
    multipart_parser_free(parser);

    ESP_LOGI(TAG, "File upload complete");
    httpd_resp_sendstr(req, "File upload successful");
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
    start_web_server();

    gpio_init();
    spi_init();

    init_spiffs();

    setSDCardMODE(false);
    init_sd_card();
    // write_to_sdcard();
    // read_from_sdcard();
    setSDCardMODE(true);
    epad_init();

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
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config2);
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
    else if (battery_voltage <= 3.1)
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
