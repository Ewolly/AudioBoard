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
            //while(!audio_ready_for_data(speaker));
            sdi_write(speaker, 32, audio_data);
        }
    }
}

void hunter_audio_record(audio_bus_t speaker, audio_bus_t mic)
{
    rb_t audio_rb = rb_init(65536);
    uint8_t little_piss[32] = {0};
    uint16_t word, words_waiting;
    uint8_t shift_amount = 0;
    bool playing = true;

    if (!audio_prepare_ogg(mic))
        ESP_LOGE(TAG, "error loading ogg plugin");

    audio_start_record(mic, false);
    ESP_LOGI(TAG, "audio started recording");

    // sci_write(speaker, VS1053_REG_MODE, sci_read(speaker, VS1053_REG_MODE) | VS1053_MODE_SM_STREAM);

    // initial feed buffer
    while (rb_size(&audio_rb) < 8192) {
        words_waiting = audio_recorded_words_waiting(mic);
        if (words_waiting > 256) {
            //pretty sure this will need fixing
            for (int x = 0; x < words_waiting; x++) {
                word = audio_recorded_read_word(mic);
                
                rb_push(&audio_rb, word >> 8);
                rb_push(&audio_rb, word  & 0xFF);
            }
        }
    }
    ESP_LOGI(TAG, "half-done");


    while (rb_size(&audio_rb) > 0) {
        
        if (rb_size(&audio_rb) >= 32)
            shift_amount = 32;
        else
            shift_amount = rb_size(&audio_rb);

        // shift_amount = rb_size(&audio_rb) >= 32 ? 32 : rb_size(&audio_rb);
        for (uint8_t i = 0; i < shift_amount; ++i) {
            little_piss[i] = rb_shift(&audio_rb);
            printf("%c", little_piss[i]);
        }
 
        while (!audio_ready_for_data(speaker));
        sdi_write(speaker, shift_amount, little_piss);
    }
    ESP_LOGI(TAG, "full-done");

    rb_free(&audio_rb);
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
        hunter_audio_record(spi.spi1, spi.spi2);
        //audio_record(spi.spi1, spi.spi2);
        //xfiles_theme(spi.spi1);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}