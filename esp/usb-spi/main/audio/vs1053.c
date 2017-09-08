#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "vs1053.h"

static const char* TAG = "VS1053";

audio_spi_t audio_spi_init()
{
    esp_err_t ret;
    audio_spi_t audio_piss;
    
    spi_bus_config_t spi1_bus_cfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t spi1_control_cfg = {
        .clock_speed_hz = 250000,
        .mode = 0,
        .spics_io_num = 5,
        .queue_size = 4
    };

    spi_device_interface_config_t spi1_data_cfg = {
        .clock_speed_hz = 8000000,
        .mode = 0,
        .spics_io_num = 25,
        .queue_size = 4
    };

    spi_bus_config_t spi2_bus_cfg = {
        .mosi_io_num = 13,
        .miso_io_num = 12,
        .sclk_io_num = 14,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t spi2_control_cfg = {
        .clock_speed_hz = 250000,
        .mode = 0,
        .spics_io_num = 15,
        .queue_size = 4
    };

    spi_device_interface_config_t spi2_data_cfg = {
        .clock_speed_hz = 8000000,
        .mode = 0,
        .spics_io_num = 26,
        .queue_size = 4
    };

    ret = spi_bus_initialize(VSPI_HOST, &spi1_bus_cfg, 1);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_initialize(HSPI_HOST, &spi2_bus_cfg, 1);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(VSPI_HOST, &spi1_control_cfg, 
        &audio_piss.spi1.control);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(VSPI_HOST, &spi1_data_cfg, 
        &audio_piss.spi1.data);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(HSPI_HOST, &spi2_control_cfg, 
        &audio_piss.spi2.control);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(HSPI_HOST, &spi2_data_cfg, 
        &audio_piss.spi2.data);
    ESP_ERROR_CHECK(ret);

    audio_piss.spi1.reset = VS1053_1_RESET;
    audio_piss.spi1.dreq = VS1053_1_DREQ;
    audio_piss.spi2.reset = VS1053_2_RESET;
    audio_piss.spi2.dreq = VS1053_2_DREQ;

    gpio_set_direction(audio_piss.spi1.reset, GPIO_MODE_DEF_OUTPUT);
    gpio_set_level(audio_piss.spi1.reset, 0);
    gpio_set_direction(audio_piss.spi1.dreq, GPIO_MODE_DEF_INPUT);

    gpio_set_direction(audio_piss.spi2.reset, GPIO_MODE_DEF_OUTPUT);
    gpio_set_level(audio_piss.spi2.reset, 0);
    gpio_set_direction(audio_piss.spi2.dreq, GPIO_MODE_DEF_INPUT);

    return audio_piss;
}

void sci_write(audio_bus_t spi, uint8_t addr, uint16_t data)
{
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.length = 32;
    t.tx_data[0] = VS1053_SCI_WRITE;
    t.tx_data[1] = addr;
    t.tx_data[2] = data >> 8;
    t.tx_data[3] = data & 0xFF;
    t.flags = SPI_TRANS_USE_TXDATA;

    ret = spi_device_transmit(spi.control, &t);
    ESP_ERROR_CHECK(ret);
}

void sdi_write(audio_bus_t spi, uint16_t num_bytes, uint8_t *data)
{
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.length = 8*num_bytes;
    t.tx_buffer = data;
    
    ret = spi_device_transmit(spi.data, &t);
    ESP_ERROR_CHECK(ret);
}

uint16_t sci_read(audio_bus_t spi, uint8_t addr)
{
    esp_err_t ret;
    spi_transaction_t t;
    uint8_t rx_buf[4] = {0};

    memset(&t, 0, sizeof(t));
    t.length = 32;
    t.rxlength = 32;
    t.tx_data[0] = VS1053_SCI_READ;
    t.tx_data[1] = addr;
    t.rx_buffer = &rx_buf;
    t.flags = SPI_TRANS_USE_TXDATA;
    
    ret = spi_device_transmit(spi.control, &t);
    ESP_ERROR_CHECK(ret);

    return (rx_buf[2] << 8) | rx_buf[3];
}

void audio_soft_reset(audio_bus_t spi) 
{
    sci_write(spi, VS1053_REG_MODE, VS1053_MODE_SM_SDINEW | VS1053_MODE_SM_RESET);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}


void audio_reset(audio_bus_t spi_bus)
{
    gpio_set_level(spi_bus.reset, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(spi_bus.reset, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    audio_soft_reset(spi_bus);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    sci_write(spi_bus, VS1053_REG_CLOCKF, 0x6000);
    sci_write(spi_bus, VS1053_REG_VOLUME, 0x4040);
}

inline bool audio_ready_for_data(audio_bus_t spi)
{
    return gpio_get_level(spi.dreq);
}

uint16_t audio_load_plugin(audio_bus_t spi, uint16_t num_bytes, const uint8_t *data)
{
    uint16_t i = 0;
    uint16_t offsets[] = {0x8000UL, 0x0, 0x4000UL};
    uint16_t type, addr, len, info;

    if (data[0] != 'P' 
        || data[1] != '&'
        || data[2] != 'H')
        return 0xFFFF;
    i = 3;

    while (i < num_bytes) {
        type = data[i++];
        if (type >= 4)
            return 0xFFFF;
        
        len = data[i++];
        len <<= 8;
        len |= data[i++] & ~1;
        addr = data[i++];
        addr <<= 8;
        addr |= data[i++];

        if (type == 3)
            return addr;

        sci_write(spi, VS1053_REG_WRAMADDR, addr + offsets[type]);
        do {
            info = data[i++];
            info <<= 8;
            info |= data[i++];
            sci_write(spi, VS1053_REG_WRAM, info);
        } while (len -= 2);
    }
    return 0xFFFF;
}

// void audio_apply_patch(audio_bus_t spi, const uint16_t *patch, uint16_t patchsize)
// {
//     uint16_t i = 0;
//     uint16_t addr, n, val;

//     while (i < patchsize) {

//         addr = patch[i++];
//         n = patch[i++];

//         if (n & 0x8000U) {
//             n &= 0x7FFF;
//             val = patch[i++];
//             while (n--)
//                 sci_write(spi, addr, val);
//         } else {
//             while (n--) {
//                 val = patch[i++];
//                 sci_write(spi, addr,val);
//             }
//         }
//     }
// }


bool audio_prepare_ogg(audio_bus_t spi)
{
    sci_write(spi, VS1053_REG_CLOCKF, 0xC000);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    while(!audio_ready_for_data(spi));
    ESP_LOGI(TAG, "got here");
    sci_write(spi, VS1053_REG_BASS, 0);
    audio_soft_reset(spi);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    while(!audio_ready_for_data(spi));

    sci_write(spi, VS1053_SCI_AIADDR, 0);
    // not sure if need to add interuppt stuff?
    sci_write(spi, VS1053_REG_WRAMADDR, VS1053_INT_ENABLE);
    sci_write(spi, VS1053_REG_WRAM, 0x02);
    int patch_start_addr = audio_load_plugin(spi, 
        v441q05_img_end - v441q05_img_start, v441q05_img_start);
    
    if (patch_start_addr == 0xFFFF)
        return false;
    if (patch_start_addr != 0x34)
        return false;
    return true;
}

uint16_t audio_recorded_words_waiting(audio_bus_t spi)
{
    return sci_read(spi, VS1053_REG_HDAT1);
}

uint16_t audio_recorded_read_word(audio_bus_t spi)
{
  return sci_read(spi, VS1053_REG_HDAT0);
}

uint32_t audio_get_recording_time(audio_bus_t spi)
{
    uint32_t time1, time2;
    
    sci_write(spi, VS1053_REG_WRAMADDR, 0x8);
    time1 = sci_read(spi, VS1053_REG_WRAM);
    time2 = sci_read(spi, VS1053_REG_WRAM);

    return (time2 << 16) | time1;
}

void audio_stop_record(audio_bus_t spi)
{
    sci_write(spi, VS1053_SCI_AICTRL3, 1);
}

void audio_start_record(audio_bus_t spi, bool mic)
{
    if (mic){
        sci_write(spi, VS1053_REG_MODE, VS1053_MODE_SM_ADPCM | VS1053_MODE_SM_SDINEW);
    }
    else{
        sci_write(spi, VS1053_REG_MODE, VS1053_MODE_SM_LINE1 | VS1053_MODE_SM_ADPCM | VS1053_MODE_SM_SDINEW);
    }
    sci_write(spi, VS1053_SCI_AICTRL0, 1024);
    /* Rec level: 1024 = 1. If 0, use AGC */
    sci_write(spi, VS1053_SCI_AICTRL1, 1024);
    /* Maximum AGC level: 1024 = 1. Only used if SCI_AICTRL1 is set to 0. */
    sci_write(spi, VS1053_SCI_AICTRL2, 0);
    /* Miscellaneous bits that also must be set before recording. */
    sci_write(spi, VS1053_SCI_AICTRL3, 0);

    sci_write(spi, VS1053_SCI_AIADDR, 0x34);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (!audio_ready_for_data(spi));
}

void audio_start_playback(audio_bus_t spi)
{
    //here
    sci_write(spi, VS1053_REG_MODE, VS1053_MODE_SM_LINE1 | VS1053_MODE_SM_SDINEW);
    sci_write(spi, VS1053_REG_WRAMADDR, 0x1e29);
    sci_write(spi, VS1053_REG_WRAM, 0);
    sci_write(spi, VS1053_REG_DECODETIME, 0x00);
    sci_write(spi, VS1053_REG_DECODETIME, 0x00);
}

rb_t rb_init(uint32_t capacity)
{
    rb_t rb;

    rb.write = 0;
    rb.read = 0;
    rb.capacity = capacity;
    rb.ring_buffer = (uint8_t) malloc(capacity);

    return rb;
}

void rb_free(rb_t *rb)
{
    free(rb->ring_buffer);
    rb->capacity = 0;
    rb->ring_buffer = NULL;
}

uint8_t rb_mask(rb_t *rb, uint8_t val)
{
    return val & (rb->capacity - 1);
}

void rb_push(rb_t *rb, uint8_t push)
{
    assert(!rb_full(rb));
    rb->ring_buffer[rb_mask(rb, rb->write++)] = push;    
}

uint8_t rb_shift(rb_t *rb)
{
    assert(!rb_empty(rb));
    return rb->ring_buffer[rb_mask(rb, rb->read++)];
}

inline bool rb_empty(rb_t *rb)
{
    return rb->read == rb->write;
}

inline bool rb_full(rb_t *rb)
{
    return rb_size(rb) == rb->capacity;
}

inline uint32_t rb_size(rb_t *rb)
{
    return rb->write - rb->read;
}
