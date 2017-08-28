#include <string.h>
#include "esp_log.h"
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
        .spics_io_num = 3,
        .queue_size = 4
    };

    spi_device_interface_config_t spi1_data_cfg = {
        .clock_speed_hz = 8000000,
        .mode = 0,
        .spics_io_num = 5,
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
        .spics_io_num = 35,
        .queue_size = 4
    };

    spi_device_interface_config_t spi2_data_cfg = {
        .clock_speed_hz = 8000000,
        .mode = 0,
        .spics_io_num = 34,
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

void sci_write(spi_device_handle_t spi, uint8_t addr, uint16_t data)
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

    ret = spi_device_transmit(spi, &t);
    ESP_ERROR_CHECK(ret);
}

uint16_t sci_read(spi_device_handle_t spi, uint8_t addr)
{
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.length = 32;
    t.tx_data[0] = VS1053_SCI_READ;
    t.tx_data[1] = addr;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    
    ret = spi_device_transmit(spi, &t);
    ESP_ERROR_CHECK(ret);

    return (t.rx_data[3] << 8) | t.rx_data[4];
}

void audio_soft_reset(spi_device_handle_t spi) 
{
    sci_write(spi, VS1053_REG_MODE, VS1053_MODE_SM_SDINEW | VS1053_MODE_SM_RESET);
}


void audio_reset(audio_bus_t spi_bus)
{
    gpio_set_level(spi_bus.reset, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(spi_bus.reset, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    audio_soft_reset(spi_bus.control);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    sci_write(spi_bus.control, VS1053_REG_CLOCKF, 0x6000);
    sci_write(spi_bus.control, VS1053_REG_VOLUME, 0x4040);
}