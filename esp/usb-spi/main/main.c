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
    audio_packet_t packet = {};    
    uint32_t audio_size, sample_count;
    double delay;
    uint16_t i, count;
    
    // audio_prepare_playback_ogg(spi.spi1);

    // wait for buffer to fill up
    while (rb_size(&audio_rb) < 512);
    ESP_LOGI(TAG, "starting playback");

    audio_start_playback(spi.spi1);
    
    //sci_write(spi.spi1, VS1053_REG_MODE, sci_read(spi.spi1, VS1053_REG_MODE) | VS1053_MODE_SM_STREAM);
    
    while (1) {
        audio_size = rb_size(&audio_rb);

        if (audio_size > 0) {
            packet = rb_shift(&audio_rb);
            // packet.data[31] = '\0';
            // ESP_LOGI(TAG, "%d: %s", packet.count, packet.data);
            sdi_write(spi.spi1, 32, packet.data);
            // sample_count = sci_read_32(spi.spi1, 0x1800);
            // ESP_LOGI(TAG, "sample count: %d", sample_count);
        }
    }

    rb_free(&audio_rb);
}

void audio_record(void *pvParameters)
{
    audio_packet_t *packet, new_packet;
    uint16_t word, words_waiting;

    packet = (audio_packet_t *) malloc(sizeof(packet));
    spi = audio_spi_init();
    audio_reset(spi.spi1);
    audio_reset(spi.spi2);
    
    audio_rb = rb_init(1024);
    xTaskCreate(&audio_playback, "audio_playback", 16384, NULL, 5, NULL);
    
    if (!audio_prepare_record_ogg(spi.spi2))
        ESP_LOGE(TAG, "error loading ogg plugin");

    audio_start_record(spi.spi2, false);
    ESP_LOGI(TAG, "audio started recording");
    
    while (!rb_full(&audio_rb))
    {
        words_waiting = audio_recorded_words_waiting(spi.spi2);
        if (words_waiting > 16) {
            for (int x = 0; x < 16; x += 2) {
                word = audio_recorded_read_word(spi.spi2);
                packet->data[x] = word >> 8;
                packet->data[x+1] = word & 0x00FF;
            }
            packet->count = sci_read_32(spi.spi2, 0x1800);
            rb_push(&audio_rb, packet);
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