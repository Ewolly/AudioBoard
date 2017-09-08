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

extern const uint8_t xfiles_ogg_start[] asm("_binary_xfiles_ogg_start");
extern const uint8_t xfiles_ogg_end[]   asm("_binary_xfiles_ogg_end");

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

void xfiles_theme(audio_bus_t spi)
{
    uint8_t *audio_buf = xfiles_ogg_start;
    uint8_t audio_diff = 32;
    bool playing = true;

    audio_start_playback(spi);

    while (playing) {
        while (!audio_ready_for_data(spi));
        if ((xfiles_ogg_end - audio_buf) < 32) {
            audio_diff = xfiles_ogg_end - audio_buf;
            playing = false;
        }

        sdi_write(spi, audio_diff, audio_buf);
        audio_buf += audio_diff;
    }
}

void audio_record(audio_bus_t speaker, audio_bus_t mic)
{
    uint8_t audio_data[32] = {0};
    uint16_t word, words_waiting;

    audio_start_playback(speaker);
    
    if (!audio_prepare_ogg(mic))
        ESP_LOGE(TAG, "error loading ogg plugin");

    audio_start_record(mic, false);
    ESP_LOGI(TAG, "audio started recording");
    
    while (1) {
        words_waiting = audio_recorded_words_waiting(mic);
        while (words_waiting >= 16) {
            for (int x = 0; x < 16; x++) {
                word = audio_recorded_read_word(mic);
                audio_data[x*2] = word >> 8;
                audio_data[x*2 + 1] = word & 0xFF;
            }
            sdi_write(speaker, 32, audio_data);
        }
    }
}

void hunter_audio_record(audio_bus_t speaker, audio_bus_t mic)
{
    uint16_t written = 0;
    uint8_t audio_data[32] = {0};

    uint16_t wordswaiting = audio_recorded_words_waiting(mic);

    while (wordswaiting > 256) {
        for (int x = 0; x < 512/128; x++){
            for (uint16_t addr = 0; addr < 128; addr+=2){
                //audio_data[addr] = word >> 8;
                //audio_data[addr + 1] = word;
            }
        }
    }
}

void app_main()
{
    audio_spi_t spi = audio_spi_init();
    audio_reset(spi.spi1);
    audio_reset(spi.spi2);
    ESP_LOGI(TAG, "MODE: 0x%04x", sci_read(spi.spi1, VS1053_REG_MODE));
    ESP_LOGI(TAG, "STATUS: 0x%04x", sci_read(spi.spi1, VS1053_REG_STATUS));
    ESP_LOGI(TAG, "CLOCK: 0x%04x", sci_read(spi.spi1, VS1053_REG_CLOCKF));
    ESP_LOGI(TAG, "VOLUME: 0x%04x", sci_read(spi.spi1, VS1053_REG_VOLUME));
    ESP_LOGI(TAG, "MODE: 0x%04x", sci_read(spi.spi2, VS1053_REG_MODE));
    ESP_LOGI(TAG, "STATUS: 0x%04x", sci_read(spi.spi2, VS1053_REG_STATUS));
    ESP_LOGI(TAG, "CLOCK: 0x%04x", sci_read(spi.spi2, VS1053_REG_CLOCKF));
    ESP_LOGI(TAG, "VOLUME: 0x%04x", sci_read(spi.spi2, VS1053_REG_VOLUME));

    for (;;) {
        audio_record(spi.spi1, spi.spi2);
        //xfiles_theme(spi.spi1);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}