#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "audio/vs1053.h"

static const char* TAG = "AUDIO";

void sine_test(audio_bus_t spi, uint8_t n, uint16_t ms)
{
    audio_reset(spi);
    uint16_t mode = sci_read(spi, VS1053_REG_MODE);
    mode |= 0x0020;
    sci_write(spi, VS1053_REG_MODE, mode);
    
    uint8_t sine_test_data[] = {0x53, 0xEF, 0x6E, n, 0x00, 0x00, 0x00, 0x00};
    sdi_write(spi, sizeof(sine_test_data), sine_test_data);

    vTaskDelay(ms / portTICK_PERIOD_MS);

    uint8_t sine_stop_data[] = {0x45, 0x78, 0x69, 0x74, 0x00, 0x00, 0x00, 0x00};
    sdi_write(spi, sizeof(sine_stop_data), sine_stop_data);    
}

void app_main()
{
    audio_spi_t spi = audio_spi_init();
    audio_reset(spi.spi1);
    // audio_reset(spi.spi2);
    ESP_LOGI(TAG, "MODE: 0x%04x", sci_read(spi.spi1, VS1053_REG_MODE));
    ESP_LOGI(TAG, "STATUS: 0x%04x", sci_read(spi.spi1, VS1053_REG_STATUS));
    ESP_LOGI(TAG, "CLOCK: 0x%04x", sci_read(spi.spi1, VS1053_REG_CLOCKF));
    ESP_LOGI(TAG, "VOLUME: 0x%04x", sci_read(spi.spi1, VS1053_REG_VOLUME));

    for (;;) {
        sine_test(spi.spi1, 126, 1000);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}