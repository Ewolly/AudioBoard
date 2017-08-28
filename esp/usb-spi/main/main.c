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

void app_main()
{
    audio_spi_t spi = spi_init();
    audio_reset(spi.spi1);
    audio_reset(spi.spi2);
    ESP_LOGI("0x%04x", sci_read(spi.spi1.control, VS1053_REG_MODE));
    ESP_LOGI("0x%04x", sci_read(spi.spi1.control, VS1053_REG_STATUS));
    ESP_LOGI("0x%04x", sci_read(spi.spi1.control, VS1053_REG_CLOCKF));
    ESP_LOGI("0x%04x", sci_read(spi.spi1.control, VS1053_REG_VOLUME));
}