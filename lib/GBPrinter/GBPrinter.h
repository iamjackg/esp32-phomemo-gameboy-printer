//
// Created by jack on 2021-11-23.
//

#ifndef ESP32_PHOMEMO_GB_GBPRINTER_H
#define ESP32_PHOMEMO_GB_GBPRINTER_H

#include <Arduino.h>

#define IMG_SIZE        5760
#define IMG_WIDTH       160 // Images are always 160 pixels wide
#define IMG_HEIGHT      144 // Maximum image height, may be less
#define MAX_DATA_SIZE   640
#define PR_PARAM_SIZE   4
struct pr_data {
    uint8_t data[IMG_SIZE];
    size_t data_len;
    uint8_t params[PR_PARAM_SIZE];
    uint8_t status;
    unsigned int busy_cnt;
    unsigned int printed;
    size_t width;
    size_t height;
};

void initialize_printer(void *pvParameters);
void sclk_isr_handler(void*);
[[noreturn]] void IRAM_ATTR packet_proto_task(void* pvParameters);
esp_err_t draw_bitmap(struct pr_data *data);

#endif //ESP32_PHOMEMO_GB_GBPRINTER_H
