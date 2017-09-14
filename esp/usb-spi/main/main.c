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

audio_spi_t spi;
rb_t audio_rb;

// void sine_test(audio_bus_t spi, uint8_t n, uint16_t ms)
// {
//     audio_reset(spi);
//     uint16_t mode = sci_read(spi, VS1053_REG_MODE);
//     mode |= 0x0020;
//     sci_write(spi, VS1053_REG_MODE, mode);
    
//     uint8_t sine_test_data[] = {0x53, 0xEF, 0x6E, n, 0x00, 0x00, 0x00, 0x00};
//     sdi_write(spi, sizeof(sine_test_data), sine_test_data);

//     vTaskDelay(ms / portTICK_PERIOD_MS);

//     uint8_t sine_stop_data[] = {0x45, 0x78, 0x69, 0x74, 0x00, 0x00, 0x00, 0x00};
//     sdi_write(spi, sizeof(sine_stop_data), sine_stop_data);    
// }

// void xfiles_theme(audio_bus_t spi)
// {
//     uint8_t *audio_buf = xfiles_ogg_start;
//     uint8_t audio_diff = 32;
//     bool playing = true;

//     audio_start_playback(spi);

//     while (playing) {
//         while (!audio_ready_for_data(spi));
//         if ((xfiles_ogg_end - audio_buf) < 32) {
//             audio_diff = xfiles_ogg_end - audio_buf;
//             playing = false;
//         }

//         sdi_write(spi, audio_diff, audio_buf);
//         audio_buf += audio_diff;
//     }
// }

// void audio_record(audio_bus_t speaker, audio_bus_t mic)
// {
//     uint8_t audio_data[32] = {0};
//     uint16_t word, words_waiting;

//     audio_start_playback(speaker);
    
//     if (!audio_prepare_ogg(mic))
//         ESP_LOGE(TAG, "error loading ogg plugin");

//     audio_start_record(mic, false);
//     ESP_LOGI(TAG, "audio started recording");
    
//     while (1) {
//         words_waiting = audio_recorded_words_waiting(mic);
//         while (words_waiting >= 16) {
//             for (int x = 0; x < 16; x++) {
//                 word = audio_recorded_read_word(mic);
//                 audio_data[x*2] = word >> 8;
//                 audio_data[x*2 + 1] = word & 0xFF;
//             }
//             //while(!audio_ready_for_data(speaker));
//             sdi_write(speaker, 32, audio_data);
//         }
//     }
// }

void audio_playback(void *pvParameters)
{
    uint32_t audio_size;
    uint8_t little_piss[32];
    double delay;
    uint16_t i, count;
    
    audio_prepare_playback_ogg(spi.spi1);
        //ESP_LOGE(TAG, "error loading ogg plugin");

    // wait for buffer to fill up
    while (rb_size(&audio_rb) < (32768 + 16384));
    ESP_LOGI(TAG, "starting playback");

    audio_start_playback(spi.spi1);
    
    sci_write(spi.spi1, VS1053_REG_MODE, sci_read(spi.spi1, VS1053_REG_MODE) | VS1053_MODE_SM_STREAM);
    
    ESP_LOGI(TAG, "bitrate: %d kbit/s", sci_read(spi.spi1, VS1053_REG_HDAT0)*8);
    delay = 32000.0f/sci_read(spi.spi1, VS1053_REG_HDAT0);
    count = 0;
    delay = 8.0f;
    while (1) {
        audio_size = rb_size(&audio_rb);

        if (audio_size < 32)
            continue;

        count++;
        for (i = 0; i < 32; ++i)
            little_piss[i] = rb_shift(&audio_rb);
        sdi_write(spi.spi1, 32, little_piss);
        
        while (!audio_ready_for_data(spi.spi1));
        //ESP_LOGI(TAG, "HDAT0: %d", sci_read(spi.spi1, VS1053_REG_HDAT0));
        //delay = (32.0f * 8.0f)/sci_read(spi.spi1, VS1053_REG_HDAT0);
        sci_write(spi.spi1, VS1053_REG_WRAMADDR, 0x1e05);
        delay = 32000.0f / sci_read(spi.spi1, VS1053_REG_WRAM);
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }

    rb_free(&audio_rb);
}

void audio_record(void *pvParameters)
{
    uint16_t word, words_waiting;
    spi = audio_spi_init();
    audio_reset(spi.spi1);
    audio_reset(spi.spi2);
    
    audio_rb = rb_init(65536);
    xTaskCreate(&audio_playback, "audio_playback", 16384, NULL, 5, NULL);
    
    if (!audio_prepare_record_ogg(spi.spi2))
        ESP_LOGE(TAG, "error loading ogg plugin");

    audio_start_record(spi.spi2, false);
    ESP_LOGI(TAG, "audio started recording");
    
    while (!rb_full(&audio_rb))
    {
        words_waiting = audio_recorded_words_waiting(spi.spi2);
        if (words_waiting > 256) {
            for (int x = 0; x < words_waiting; x++) {
                word = audio_recorded_read_word(spi.spi2);
                
                rb_push(&audio_rb, word >> 8);
                rb_push(&audio_rb, word & 0x00FF);
            }
        }
    }
    ESP_LOGE(TAG, "buffer full!");
    rb_free(&audio_rb);
    abort();
}

void app_main()
{
    xTaskCreate(&audio_record, "audio_record", 8192, NULL, 5, NULL);
}