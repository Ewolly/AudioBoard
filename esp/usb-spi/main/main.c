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

void audio_playback(void *pvParameters)
{
    audio_packet_t packet = {};    
    uint32_t audio_size, sample_count, last_sample_count = 0;
    double delay;
    uint16_t i, x, count = 0;
    
    audio_prepare_playback_ogg(spi.spi1);

    // wait for buffer to fill up
    while (rb_size(&audio_rb) < 64);
    ESP_LOGI(TAG, "starting playback");

    audio_start_playback(spi.spi1);
    
    // sci_write(spi.spi1, VS1053_REG_MODE, sci_read(spi.spi1, VS1053_REG_MODE) | VS1053_MODE_SM_STREAM);
    
    while (1) {
        audio_size = rb_size(&audio_rb);

        if (audio_size > 0) {
            count++;
            packet = rb_shift(&audio_rb);
            for (i = 0; i < packet.packet_size; i += 32)
            {
                if (packet.packet_size - i < 32)
                    x = packet.packet_size - i;
                else
                    x = 32;

                while (!audio_ready_for_data(spi.spi1));
                sdi_write(spi.spi1, x, packet.data + i);
            }

            while (!audio_ready_for_data(spi.spi1));
            sample_count = sci_read_32(spi.spi1, 0x1800);
        }

        if (count > 64) {
            count = 0;
            
            ESP_LOGI(TAG, "%d: %u - %u = %u", 
                audio_size, 
                packet.count, 
                sample_count, 
                packet.count - sample_count);
        }
    }

    rb_free(&audio_rb);
}

void audio_record(void *pvParameters)
{
    audio_packet_t packet = {0};
    uint16_t word, words_waiting;
    bool offset = false;

    spi = audio_spi_init();
    audio_reset(spi.spi1);
    audio_reset(spi.spi2);
    
    audio_rb = rb_init(128);
    xTaskCreate(&audio_playback, "audio_playback", 16384, NULL, 5, NULL);
    
    if (!audio_prepare_record_ogg(spi.spi2))
        ESP_LOGE(TAG, "error loading ogg plugin");

    audio_start_record(spi.spi2, false);
    ESP_LOGI(TAG, "audio started recording");
    
    while (!rb_full(&audio_rb))
    {
        words_waiting = audio_recorded_words_waiting(spi.spi2);
        while (words_waiting > 256) {
            if (offset) {
                packet.data[0] = audio_recorded_read_word(spi.spi2) & 0x00FF;
            }
            
            for (int x = 0; x < 255; x++) {
                word = audio_recorded_read_word(spi.spi2);
                if (offset) {
                    packet.data[2*x+1] = word >> 8;
                    packet.data[2*x+2] = word & 0x00FF;
                } else {
                    packet.data[2*x] = word >> 8;
                    packet.data[2*x+1] = word & 0x00FF;
                }
            }
            sci_read(spi.spi2, VS1053_SCI_AICTRL3);
            word = sci_read(spi.spi2, VS1053_SCI_AICTRL3);
            if (word & 0x0002) {
                if (offset) {
                    packet.data[511] = word >> 8;
                    packet.packet_size = 512;
                }
                else {
                    packet.data[510] = word >> 8;
                    packet.packet_size = 511;
                }
                offset = !offset;
            } else {
                if (offset)
                    packet.packet_size = 511;
                else
                    packet.packet_size = 510;
            }
            packet.count = sci_read_32(spi.spi2, 0x1800);
            rb_push(&audio_rb, &packet);
            words_waiting -= 256;
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