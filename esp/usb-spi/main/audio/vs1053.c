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

uint32_t sci_read_32(audio_bus_t spi, uint8_t addr)
{
    uint16_t msbv1, lsb, msbv2;
    
    sci_write(spi, VS1053_REG_WRAMADDR, addr+1);
    msbv1 = sci_read(spi, VS1053_REG_WRAM);
    sci_write(spi,VS1053_REG_WRAMADDR, addr);
    lsb = (uint32_t) sci_read(spi, VS1053_REG_WRAM);
    msbv2 = sci_read(spi, VS1053_REG_WRAM);
    if (lsb < 0x8000U){
        msbv1 = msbv2;
    }
    return ((uint32_t)msbv1 << 16) | lsb;
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

void audio_load_plg(audio_bus_t spi, uint16_t num_bytes,  const uint16_t *patch) {
  int i = 0;

  while (i<num_bytes) {
    uint16_t addr, n, val;
    addr = patch[i++];
    n = patch[i++];
    if (n & 0x8000U) { /* RLE run, replicate n samples */
      n &= 0x7FFF;
      val = patch[i++];
      while (n--) {
        sci_write(spi, addr, val);
      }
    } else {           /* Copy run, copy n samples */
      while (n--) {
        val = patch[i++];
        sci_write(spi, addr, val);
      }
    }
  }
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


bool audio_prepare_record_ogg(audio_bus_t spi)
{
    sci_write(spi, VS1053_REG_CLOCKF, 0xC000);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    while(!audio_ready_for_data(spi));

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

void audio_prepare_playback_ogg(audio_bus_t spi)
{

    const uint16_t vs1053b_decoding_patch[] = { /* Compressed plugin */
        0x0007,0x0001, /*copy 1*/
        0x8050,
        0x0006,0x0002, /*copy 2*/
        0x2a00,0xc000,
        0x0006, 0x801e, 0x0000, /*Rle(30)*/
        0x0006,0x04a2, /*copy 1186*/
        0xf400,0x4095,0x0000,0x02c2,0x6124,0x0024,0x0000,0x0024,
        0x2800,0x1ac5,0x4192,0x4542,0x0000,0x0041,0x2000,0x0015,
        0x0030,0x0317,0x2000,0x0000,0x3f00,0x4024,0x2000,0x0000,
        0x0000,0x0000,0x3e12,0x3800,0x3e00,0xb804,0x0030,0x0015,
        0x0007,0x8257,0x3700,0x984c,0xf224,0x1444,0xf224,0x0024,
        0x0008,0x0002,0x2910,0x0181,0x0000,0x1bc8,0xb428,0x1402,
        0x0000,0x8004,0x2910,0x0195,0x0000,0x1bc8,0xb428,0x0024,
        0x0006,0x0095,0x2800,0x2945,0x3e13,0x780e,0x3e11,0x7803,
        0x3e13,0xf806,0x3e11,0xf801,0x3510,0xb808,0x003f,0xe004,
        0xfec4,0x3800,0x48be,0x17c3,0xfe46,0x41c2,0x48be,0x4497,
        0x4090,0x1c46,0xf06c,0x0024,0x2400,0x2580,0x6090,0x41c3,
        0x6628,0x1c47,0x0000,0x0024,0x2800,0x2449,0xf07e,0x0024,
        0xf400,0x4182,0x673a,0x1c46,0x0000,0x0024,0x2800,0x2589,
        0xf06c,0x0024,0xf400,0x41c3,0x0000,0x0024,0x4224,0x3442,
        0x2900,0xa540,0x4336,0x37c3,0x0000,0x1805,0x2900,0xa540,
        0x4508,0x40c2,0x450a,0x9808,0x0000,0x0207,0xa478,0x1bc0,
        0xc45a,0x1807,0x0030,0x03d5,0x3d01,0x5bc1,0x36f3,0xd806,
        0x3601,0x5803,0x36f3,0x0024,0x36f3,0x580e,0x0007,0x8257,
        0x0000,0x6004,0x3730,0x8024,0xb244,0x1c04,0xd428,0x3c02,
        0x0006,0xc717,0x2800,0x2d05,0x4284,0x0024,0x3613,0x3c02,
        0x0006,0xc357,0x2901,0x6640,0x3e11,0x5c05,0x4284,0x1bc5,
        0x0000,0x0024,0x2800,0x2fc5,0x0000,0x0024,0x3613,0x0024,
        0x3e10,0x3813,0x3e14,0x8024,0x3e04,0x8024,0x2900,0x4f40,
        0x0006,0x02d3,0x36e3,0x0024,0x3009,0x1bd3,0x0007,0x8257,
        0x3700,0x8024,0xf224,0x0024,0x0000,0x0024,0x2800,0x31d1,
        0x3600,0x9844,0x2900,0x3780,0x0000,0x3248,0x2911,0xf140,
        0x0000,0x0024,0x0030,0x0057,0x3700,0x0024,0xf200,0x4595,
        0x0fff,0xfe02,0xa024,0x164c,0x8000,0x17cc,0x3f00,0x0024,
        0x3500,0x0024,0x0021,0x6d82,0xd024,0x44c0,0x0006,0xa402,
        0x2800,0x3695,0xd024,0x0024,0x0000,0x0000,0x2800,0x3695,
        0x000b,0x6d57,0x3009,0x3c00,0x36f0,0x8024,0x36f2,0x1800,
        0x2000,0x0000,0x0000,0x0024,0x3e14,0x7810,0x3e13,0xb80d,
        0x3e13,0xf80a,0x3e10,0xb803,0x3e11,0x3805,0x3e11,0xb807,
        0x3e14,0xf801,0x3e15,0x3815,0x0001,0x000a,0x0006,0xc4d7,
        0xbf8e,0x9c42,0x3e01,0x9c03,0x0006,0xa017,0x0023,0xffd1,
        0x0007,0x8250,0x0fff,0xfd85,0x3001,0x0024,0xa45a,0x4494,
        0x0000,0x0093,0x2800,0x3dd1,0xf25a,0x104c,0x34f3,0x0024,
        0x2800,0x3dd1,0x0000,0x0024,0x3413,0x084c,0x0000,0x0095,
        0x3281,0xf806,0x4091,0x4d64,0x2400,0x4000,0x4efa,0x9c10,
        0xf1eb,0x6061,0xfe55,0x2f66,0x5653,0x4d64,0x48b2,0xa201,
        0x4efa,0xa201,0x36f3,0x3c10,0x36f5,0x1815,0x36f4,0xd801,
        0x36f1,0x9807,0x36f1,0x1805,0x36f0,0x9803,0x36f3,0xd80a,
        0x36f3,0x980d,0x2000,0x0000,0x36f4,0x5810,0x3e12,0xb817,
        0x3e14,0xf812,0x3e01,0xb811,0x0007,0x9717,0x0020,0xffd2,
        0x0030,0x11d1,0x3111,0x8024,0x3704,0xc024,0x3b81,0x8024,
        0x3101,0x8024,0x3b81,0x8024,0x3f04,0xc024,0x2808,0x4800,
        0x36f1,0x9811,0x36f3,0x0024,0x3009,0x3848,0x3e14,0x3811,
        0x3e00,0x0024,0x0000,0x4000,0x0001,0x0010,0x2915,0x94c0,
        0x0001,0xcc11,0x36f0,0x0024,0x2927,0x9e40,0x3604,0x1811,
        0x3613,0x0024,0x3e14,0x3811,0x3e00,0x0024,0x0000,0x4000,
        0x0001,0x0010,0x2915,0x94c0,0x0001,0xcc11,0x36f0,0x0024,
        0x36f4,0x1811,0x3009,0x1808,0x2000,0x0000,0x0000,0x190d,
        0x3600,0x3840,0x3e13,0x780e,0x3e13,0xf808,0x3e00,0x0024,
        0x0000,0x464e,0x0027,0x9e0f,0x2922,0xb680,0x0000,0x190d,
        0x36f3,0x0024,0x36f3,0xd808,0x36f3,0x580e,0x2000,0x0000,
        0x3009,0x1800,0x3613,0x0024,0x3e22,0xb815,0x3e05,0xb814,
        0x3615,0x0024,0x0000,0x800a,0x3e13,0x7801,0x3e10,0xb803,
        0x3e11,0x3805,0x3e11,0xb807,0x3e14,0x3811,0x3e14,0xb813,
        0x3e03,0xf80e,0xb488,0x44d5,0x3543,0x134c,0x34e5,0xc024,
        0x3524,0x8024,0x35a4,0xc024,0x3710,0x8a0c,0x3540,0x4a0c,
        0x3d44,0x8024,0x3a10,0x8024,0x3590,0x0024,0x4010,0x15c1,
        0x6010,0x3400,0x3710,0x8024,0x2800,0x5b04,0x3af0,0x8024,
        0x3df0,0x0024,0x3591,0x4024,0x3530,0x4024,0x4192,0x4050,
        0x6100,0x1482,0x4020,0x1753,0xbf8e,0x1582,0x4294,0x4011,
        0xbd86,0x408e,0x2400,0x590e,0xfe6d,0x2819,0x520e,0x0a00,
        0x5207,0x2819,0x4fbe,0x0024,0xad56,0x904c,0xaf5e,0x1010,
        0xf7d4,0x0024,0xf7fc,0x2042,0x6498,0x2046,0x3cf4,0x0024,
        0x3400,0x170c,0x4090,0x1492,0x35a4,0xc024,0x2800,0x5395,
        0x3c00,0x0024,0x4480,0x914c,0x36f3,0xd80e,0x36f4,0x9813,
        0x36f4,0x1811,0x36f1,0x9807,0x36f1,0x1805,0x36f0,0x9803,
        0x36f3,0x5801,0x3405,0x9014,0x36e3,0x0024,0x2000,0x0000,
        0x36f2,0x9815,0x3e12,0xb817,0x3e12,0x3815,0x3e05,0xb814,
        0x3625,0x0024,0x0000,0x800a,0x3e10,0x3801,0x3e10,0xb803,
        0x3e11,0x3805,0x3e11,0xb807,0x3e14,0x3811,0x0006,0xa090,
        0x2912,0x0d00,0x3e14,0xc024,0x4088,0x8000,0x4080,0x0024,
        0x0007,0x90d1,0x2800,0x6505,0x0000,0x0024,0x0007,0x9051,
        0x3100,0x4024,0x4100,0x0024,0x3900,0x0024,0x0007,0x90d1,
        0x0004,0x0000,0x31f0,0x4024,0x6014,0x0400,0x0000,0x0024,
        0x2800,0x6951,0x4080,0x0024,0x0000,0x0000,0x2800,0x68c5,
        0x0000,0x0024,0x0007,0x9053,0x3300,0x0024,0x4080,0x0024,
        0x0000,0x0000,0x2800,0x6958,0x0000,0x0024,0x0007,0x9051,
        0x3900,0x0024,0x3200,0x504c,0x6410,0x0024,0x3cf0,0x0000,
        0x4080,0x0024,0x0006,0xc691,0x2800,0x8205,0x3009,0x0400,
        0x0007,0x9051,0x0000,0x1001,0x3100,0x0024,0x6012,0x0024,
        0x0006,0xc6d0,0x2800,0x7649,0x003f,0xe000,0x0006,0xc693,
        0x3900,0x0c00,0x3009,0x0001,0x6014,0x0024,0x0007,0x1ad0,
        0x2800,0x7655,0x3009,0x0000,0x4080,0x0024,0x0000,0x0301,
        0x2800,0x7045,0x4090,0x0024,0x0000,0x0024,0x2800,0x7155,
        0x0000,0x0024,0x3009,0x0000,0xc012,0x0024,0x2800,0x7640,
        0x3009,0x2001,0x3009,0x0000,0x6012,0x0024,0x0000,0x0341,
        0x2800,0x7355,0x0000,0x0024,0x6190,0x0024,0x2800,0x7640,
        0x3009,0x2000,0x6012,0x0024,0x0000,0x0381,0x2800,0x7515,
        0x0000,0x0024,0x6190,0x0024,0x2800,0x7640,0x3009,0x2000,
        0x6012,0x0024,0x0000,0x00c0,0x2800,0x7655,0x0000,0x0024,
        0x3009,0x2000,0x0006,0xa090,0x3009,0x0000,0x4080,0x0024,
        0x0000,0x0081,0x2800,0x7b15,0x0007,0x8c13,0x3300,0x104c,
        0xb010,0x0024,0x0002,0x8001,0x2800,0x7d85,0x34f0,0x0024,
        0x2800,0x7b00,0x0000,0x0024,0x0006,0xc351,0x3009,0x0000,
        0x6090,0x0024,0x3009,0x2000,0x2900,0x0b80,0x3009,0x0405,
        0x0006,0xc690,0x0006,0xc6d1,0x3009,0x0000,0x3009,0x0401,
        0x6014,0x0024,0x0006,0xa093,0x2800,0x7991,0xb880,0x0024,
        0x2800,0x8ac0,0x3009,0x2c00,0x4040,0x0024,0x6012,0x0024,
        0x0006,0xc6d0,0x2800,0x8ad8,0x0000,0x0024,0x0006,0xc693,
        0x3009,0x0c00,0x3009,0x0001,0x6014,0x0024,0x0006,0xc350,
        0x2800,0x8ac1,0x0000,0x0024,0x6090,0x0024,0x3009,0x2c00,
        0x3009,0x0005,0x2900,0x0b80,0x0000,0x8ac8,0x3009,0x0400,
        0x4080,0x0024,0x0003,0x8000,0x2800,0x8ac5,0x0000,0x0024,
        0x6400,0x0024,0x0000,0x0081,0x2800,0x8ac9,0x0000,0x0024,
        0x0007,0x8c13,0x3300,0x0024,0xb010,0x0024,0x0006,0xc650,
        0x2800,0x8ad5,0x0000,0x0024,0x0001,0x0002,0x3413,0x0000,
        0x3009,0x0401,0x4010,0x8406,0x0000,0x0281,0xa010,0x13c1,
        0x4122,0x0024,0x0000,0x03c2,0x6122,0x8002,0x462c,0x0024,
        0x469c,0x0024,0xfee2,0x0024,0x48be,0x0024,0x6066,0x8400,
        0x0006,0xc350,0x2800,0x8ac1,0x0000,0x0024,0x4090,0x0024,
        0x3009,0x2400,0x2900,0x0b80,0x3009,0x0005,0x0007,0x1b50,
        0x2912,0x0d00,0x3613,0x0024,0x3a00,0x0380,0x4080,0x0024,
        0x0000,0x00c1,0x2800,0x9385,0x3009,0x0000,0xb010,0x008c,
        0x4192,0x0024,0x6012,0x0024,0x0006,0xf051,0x2800,0x9198,
        0x3009,0x0400,0x0007,0x1fd1,0x30e3,0x0400,0x4080,0x0024,
        0x0000,0x0301,0x2800,0x9385,0x3009,0x0000,0xb010,0x0024,
        0x0000,0x0101,0x6012,0x0024,0x0006,0xf051,0x2800,0x9395,
        0x0000,0x0024,0x3023,0x0400,0xf200,0x184c,0xb880,0xa400,
        0x3009,0x2000,0x3009,0x0441,0x3e10,0x4402,0x2909,0xa9c0,
        0x3e10,0x8024,0x36e3,0x0024,0x36f4,0xc024,0x36f4,0x1811,
        0x36f1,0x9807,0x36f1,0x1805,0x36f0,0x9803,0x36f0,0x1801,
        0x3405,0x9014,0x36f3,0x0024,0x36f2,0x1815,0x2000,0x0000,
        0x36f2,0x9817,0x3613,0x0024,0x3e12,0xb817,0x3e12,0x3815,
        0x3e05,0xb814,0x3615,0x0024,0x0000,0x800a,0x3e10,0xb803,
        0x0012,0x5103,0x3e11,0x3805,0x3e11,0xb807,0x3e14,0x380d,
        0x0030,0x0250,0x3e13,0xf80e,0xbe8b,0x83e0,0x290c,0x4840,
        0x3613,0x0024,0x290c,0x4840,0x4086,0x984c,0x0000,0x00ce,
        0x2400,0x9d8e,0x3009,0x1bc0,0x0000,0x01c3,0xae3a,0x184c,
        0x0000,0x0043,0x3009,0x3842,0x290c,0x4840,0x3009,0x3840,
        0x4084,0x9bc0,0xfe26,0x9bc2,0xceba,0x0024,0x4e8e,0x0024,
        0x4e9a,0x0024,0x4f8e,0x0024,0x0000,0x0102,0x2800,0xa2c5,
        0x0030,0x0010,0x0000,0x0206,0x3613,0x0024,0x290c,0x4840,
        0x3009,0x3840,0x3000,0xdbc0,0xb366,0x0024,0x0000,0x0024,
        0x2800,0xa2d5,0x4e8e,0x0024,0x4e9a,0x0024,0x4f8e,0x0024,
        0x0030,0x0010,0x2800,0x9f95,0x0000,0x0206,0x36f3,0xd80e,
        0x36f4,0x180d,0x36f1,0x9807,0x36f1,0x1805,0x36f0,0x9803,
        0x3405,0x9014,0x36f3,0x0024,0x36f2,0x1815,0x2000,0x0000,
        0x36f2,0x9817,0xb386,0x40d7,0x4284,0x184c,0x0000,0x05c0,
        0x2800,0xa6d5,0xf5d8,0x3804,0x0000,0x0984,0x6400,0xb84a,
        0x3e13,0xf80d,0xa204,0x380e,0x0000,0x800a,0x0000,0x00ce,
        0x2400,0xaa0e,0xffa4,0x0024,0x48b6,0x0024,0x0000,0x0024,
        0x2800,0xaa04,0x4000,0x40c2,0x4224,0x0024,0x6090,0x0024,
        0xffa4,0x0024,0x0fff,0xfe83,0xfe86,0x1bce,0x36f3,0xd80d,
        0x48b6,0x0024,0x0fff,0xff03,0xa230,0x45c3,0x2000,0x0000,
        0x36f1,0x180a,
        0x0007,0x0001, /*copy 1*/
        0x8300,
        0x0006,0x05d6, /*copy 1494*/
        0x0030,0x0055,0xb080,0x1402,0x0fdf,0xffc1,0x0007,0x9257,
        0xb212,0x3c00,0x3d00,0x4024,0x0030,0x0297,0x3f00,0x0024,
        0x0007,0x9017,0x3f00,0x0024,0x0007,0x81d7,0x3f10,0x0024,
        0xc090,0x3c00,0x0006,0x0297,0xb080,0x3c00,0x0000,0x0401,
        0x000a,0x1055,0x0006,0x0017,0x3f10,0x3401,0x000a,0x2795,
        0x3f00,0x3401,0x0001,0x6617,0xf400,0x55c0,0x0000,0x0817,
        0xb080,0x57c0,0x0006,0x01d7,0x3f00,0x0024,0x0000,0x190d,
        0x000f,0xf94f,0x0000,0xca0e,0x280f,0xe100,0x0006,0x2016,
        0x0000,0x0080,0x0005,0x4f92,0x2909,0xf840,0x3613,0x2800,
        0x0006,0x0197,0x0006,0xa115,0xb080,0x0024,0x3f00,0x3400,
        0x0007,0x8a57,0x3700,0x0024,0x4080,0x0024,0x0000,0x0040,
        0x2800,0xcbd5,0x0006,0xa2d7,0x3009,0x3c00,0x0006,0xa157,
        0x3009,0x1c00,0x0006,0x01d7,0x0000,0x190d,0x000a,0x708f,
        0x0000,0xd4ce,0x290b,0x1a80,0x3f00,0x184c,0x0030,0x0017,
        0x4080,0x1c01,0x0000,0x0200,0x2800,0xc815,0xb102,0x0024,
        0x0000,0xca08,0x2800,0xc815,0x0000,0xd0ce,0x0011,0x210f,
        0x0000,0x190d,0x280f,0xcb00,0x3613,0x0024,0x0006,0xa115,
        0x0006,0x01d7,0x37f0,0x1401,0x6100,0x1c01,0x4012,0x0024,
        0x0000,0x8000,0x6010,0x0024,0x34f3,0x0400,0x2800,0xd398,
        0x0000,0x0024,0x0000,0x8001,0x6010,0x3c01,0x0000,0x000d,
        0x2811,0x8259,0x0000,0x0024,0x2a11,0x2100,0x0030,0x0257,
        0x3700,0x0024,0x4080,0x0024,0x0000,0x0024,0x2800,0xd6d5,
        0x0006,0x0197,0x0006,0xa115,0x3f00,0x3400,0x003f,0xc000,
        0xb600,0x41c1,0x0012,0x5103,0x000c,0xc002,0xdcd6,0x0024,
        0x0019,0xd4c2,0x2800,0x9645,0x0001,0x0d08,0x0013,0xd9c3,
        0x6fd6,0x0024,0x0000,0x190d,0x2800,0xdc95,0x0014,0x1b01,
        0x0020,0x480f,0x0000,0xdb4e,0x0000,0x190d,0x2820,0x41c0,
        0x0001,0x0d08,0x0039,0x324f,0x0001,0x3a0e,0x2820,0x4a18,
        0xb882,0x0024,0x2a20,0x48c0,0x003f,0xfd00,0xb700,0x0024,
        0x003f,0xf901,0x6010,0x0024,0x0000,0x0024,0x280a,0xc505,
        0x0000,0x190d,0x0014,0x1b01,0x0015,0x59c0,0x6fc2,0x0024,
        0x0000,0x0024,0x2800,0xe6d5,0x0000,0x0024,0x290c,0x4840,
        0x3613,0x0024,0x290c,0x4840,0x4086,0x184c,0x0000,0x18c2,
        0x6234,0x0024,0x0000,0x1d02,0x2800,0xe2d5,0x6234,0x0024,
        0x0030,0x0317,0x2800,0xe6c0,0x3f00,0x0024,0x0000,0x1d82,
        0x2800,0xe555,0x6234,0x0024,0x2912,0x0d00,0x4084,0x184c,
        0xf200,0x0024,0x6200,0x0024,0x0006,0x0017,0x2800,0xe240,
        0xb080,0x3c40,0x0000,0x0202,0x2800,0xe6d5,0xa024,0x0024,
        0xc020,0x0024,0x2800,0xe240,0x0030,0x02d7,0x000a,0x8c8f,
        0x0000,0xe80e,0x000c,0x0981,0x280a,0x71c0,0x002c,0x9d40,
        0x000a,0x708f,0x0000,0xd4ce,0x280a,0xc0d5,0x0012,0x5182,
        0x6fd6,0x0024,0x003f,0xfd81,0x280a,0x8e45,0xb710,0x0024,
        0xb710,0x0024,0x003f,0xfc01,0x6012,0x0024,0x0000,0x0101,
        0x2801,0x03d5,0xffd2,0x0024,0x48b2,0x0024,0x4190,0x0024,
        0x0000,0x190d,0x2801,0x03d5,0x0030,0x0250,0xb880,0x104c,
        0x3cf0,0x0024,0x0010,0x5500,0xb880,0x23c0,0xb882,0x2000,
        0x0007,0x8590,0x2914,0xbec0,0x0000,0x0440,0x0007,0x8b50,
        0xb880,0x0024,0x2920,0x0100,0x3800,0x0024,0x2920,0x0000,
        0x0006,0x8a91,0x0000,0x0800,0xb880,0xa440,0x003f,0xfd81,
        0xb710,0xa7c0,0x003f,0xfc01,0x6012,0x0024,0x0000,0x0101,
        0x2801,0x0d15,0x0000,0x0024,0xffe2,0x0024,0x48b2,0x0024,
        0x4190,0x0024,0x0000,0x0024,0x2801,0x0d15,0x0000,0x0024,
        0x2912,0x2d80,0x0000,0x0780,0x4080,0x0024,0x0006,0x8a90,
        0x2801,0x0d15,0x0000,0x01c2,0xb886,0x8040,0x3613,0x03c1,
        0xbcd2,0x0024,0x0030,0x0011,0x2800,0xf995,0x003f,0xff42,
        0xb886,0x8040,0x3009,0x03c1,0x0000,0x0020,0xac22,0x0024,
        0x0000,0x0102,0x6cd2,0x0024,0x3e10,0x0024,0x2909,0x8c80,
        0x3e00,0x4024,0x36f3,0x0024,0x3e11,0x8024,0x3e01,0xc024,
        0x2901,0x30c0,0x0000,0x0201,0xf400,0x4512,0x2900,0x0c80,
        0x3213,0x1b8c,0x3100,0x0024,0xb010,0x0024,0x0000,0x0024,
        0x2801,0x0d15,0x0000,0x0024,0x291a,0x8a40,0x0000,0x0100,
        0x2920,0x0200,0x3633,0x0024,0x2920,0x0280,0x0000,0x0401,
        0x408e,0x0024,0x2920,0x0280,0x0000,0x0401,0x003f,0xfd81,
        0xb710,0x4006,0x003f,0xfc01,0x6012,0x0024,0x0000,0x0101,
        0x2801,0x0d15,0x0000,0x0024,0xffe2,0x0024,0x48b2,0x0024,
        0x4190,0x0024,0x0000,0x0024,0x2801,0x0d15,0x0000,0x0024,
        0x2912,0x2d80,0x0000,0x0780,0x4080,0x0024,0x0000,0x01c2,
        0x2800,0xf585,0x0006,0x8a90,0x2a01,0x0d00,0x2920,0x0100,
        0x0000,0x0401,0x0000,0x0180,0x2920,0x0200,0x3613,0x0024,
        0x2920,0x0280,0x3613,0x0024,0x0000,0x0401,0x2920,0x0280,
        0x4084,0x984c,0x0019,0x9d01,0x6212,0x0024,0x001e,0x5c01,
        0x2801,0x0855,0x6012,0x0024,0x0000,0x0024,0x2801,0x0a45,
        0x0000,0x0024,0x001b,0x5bc1,0x6212,0x0024,0x001b,0xdd81,
        0x2801,0x0e15,0x6012,0x0024,0x0000,0x0024,0x2801,0x0e15,
        0x0000,0x0024,0x0000,0x004d,0x000a,0xbf4f,0x280a,0xb880,
        0x0001,0x0b4e,0x0020,0xfb4f,0x0000,0x190d,0x0001,0x124e,
        0x2920,0x0480,0x3009,0x2bc1,0x291a,0x8a40,0x36e3,0x0024,
        0x0000,0x190d,0x000a,0x708f,0x280a,0xcac0,0x0000,0xd4ce,
        0x0030,0x0017,0x3700,0x4024,0x0000,0x0200,0xb102,0x0024,
        0x0000,0x00c0,0x2801,0x1145,0x0005,0x4f92,0x2909,0xf840,
        0x3613,0x2800,0x0006,0x0197,0x0006,0xa115,0xb080,0x0024,
        0x3f00,0x3400,0x0000,0x190d,0x000a,0x708f,0x280a,0xc0c0,
        0x0000,0xd4ce,0x0000,0x004d,0x0020,0xfe0f,0x2820,0xfb40,
        0x0001,0x134e,0x2801,0x1515,0x3009,0x1000,0x6012,0x93cc,
        0x0000,0x0024,0x2801,0x2fc5,0x0000,0x0024,0x3413,0x0024,
        0x34b0,0x0024,0x4080,0x0024,0x0000,0x0200,0x2801,0x1815,
        0xb882,0x0024,0x3453,0x0024,0x3009,0x13c0,0x4080,0x0024,
        0x0000,0x0200,0x2801,0x2fc5,0x0000,0x0024,0xb882,0x130c,
        0x0000,0x004d,0x0021,0x058f,0x2821,0x0340,0x0001,0x190e,
        0x2801,0x2955,0x6012,0x0024,0x0000,0x0024,0x2801,0x2955,
        0x0000,0x0024,0x34c3,0x184c,0x3e13,0xb80f,0xf400,0x4500,
        0x0026,0x9dcf,0x0001,0x1d0e,0x0000,0xfa0d,0x2926,0x8e80,
        0x3e10,0x110c,0x36f3,0x0024,0x2801,0x2940,0x36f3,0x980f,
        0x001c,0xdd00,0x001c,0xd901,0x6ec2,0x0024,0x001c,0xdd00,
        0x2801,0x2015,0x0018,0xdbc1,0x3413,0x184c,0xf400,0x4500,
        0x2926,0xc640,0x3e00,0x13cc,0x2801,0x2700,0x36f3,0x0024,
        0x6ec2,0x0024,0x003f,0xc000,0x2801,0x2295,0x002a,0x4001,
        0x3413,0x184c,0xf400,0x4500,0x2926,0xafc0,0x3e00,0x13cc,
        0x2801,0x2700,0x36f3,0x0024,0xb400,0x0024,0xd100,0x0024,
        0x0000,0x0024,0x2801,0x2705,0x0000,0x0024,0x3613,0x0024,
        0x3e11,0x4024,0x2926,0x8540,0x3e01,0x0024,0x4080,0x1b8c,
        0x0000,0x0024,0x2801,0x2705,0x0000,0x0024,0x3413,0x184c,
        0xf400,0x4500,0x2926,0x8e80,0x3e10,0x13cc,0x36f3,0x0024,
        0x3110,0x8024,0x31f0,0xc024,0x0000,0x4000,0x0000,0x0021,
        0x6d06,0x0024,0x3110,0x8024,0x2826,0xa8c4,0x31f0,0xc024,
        0x2a26,0xad00,0x34c3,0x184c,0x3410,0x8024,0x3430,0xc024,
        0x0000,0x4000,0x0000,0x0021,0x6d06,0x0024,0x0000,0x0024,
        0x2801,0x2fd4,0x4d06,0x0024,0x0000,0x0200,0x2922,0x1885,
        0x0001,0x2e48,0x0000,0x0200,0x3e10,0x8024,0x2921,0xca80,
        0x3e00,0xc024,0x291a,0x8a40,0x0000,0x0024,0x2922,0x1880,
        0x36f3,0x0024,0x0000,0x004d,0x0021,0x0ecf,0x2821,0x0bc0,
        0x0001,0x2f4e,0x2801,0x1240,0x3c30,0x4024,0x0000,0x190d,
        0x0000,0x464e,0x2821,0x0f80,0x0027,0x9e0f,0x0020,0xcd4f,
        0x2820,0xc780,0x0001,0x318e,0x0006,0xf017,0x0000,0x0015,
        0xb070,0xbc15,0x0000,0x464e,0x0027,0x9e0f,0x2820,0xcd80,
        0x0000,0x190d,0x3613,0x0024,0x3e10,0xb803,0x3e14,0x3811,
        0x3e11,0x3805,0x3e00,0x3801,0x0007,0xc390,0x0006,0xa011,
        0x3010,0x0444,0x3050,0x4405,0x6458,0x0302,0xff94,0x4081,
        0x0003,0xffc5,0x48b6,0x0024,0xff82,0x0024,0x42b2,0x0042,
        0xb458,0x0003,0x4cd6,0x9801,0xf248,0x1bc0,0xb58a,0x0024,
        0x6de6,0x1804,0x0006,0x0010,0x3810,0x9bc5,0x3800,0xc024,
        0x36f4,0x1811,0x36f0,0x9803,0x283e,0x2d80,0x0fff,0xffc3,
        0x2801,0x47c0,0x0000,0x0024,0x3413,0x0024,0x2801,0x3bc5,
        0xf400,0x4517,0x2801,0x3fc0,0x6894,0x13cc,0x37b0,0x184c,
        0x6090,0x1d51,0x0000,0x0910,0x3f00,0x060c,0x3100,0x4024,
        0x6016,0xb812,0x000c,0x8012,0x2801,0x3e51,0xb884,0x0024,
        0x6894,0x3002,0x0000,0x028d,0x003a,0x5e0f,0x0001,0x4fce,
        0x2939,0xb0c0,0x3e10,0x93cc,0x4084,0x9bd2,0x4282,0x0024,
        0x0000,0x0040,0x2801,0x41c5,0x4292,0x130c,0x3443,0x0024,
        0x2801,0x4305,0x000c,0x8390,0x2a01,0x4680,0x3444,0x0024,
        0x3073,0x0024,0xc090,0x014c,0x2801,0x4680,0x3800,0x0024,
        0x000c,0x4113,0xb880,0x2380,0x3304,0x4024,0x3800,0x05cc,
        0xcc92,0x05cc,0x3910,0x0024,0x3910,0x4024,0x000c,0x8110,
        0x3910,0x0024,0x39f0,0x4024,0x3810,0x0024,0x38d0,0x4024,
        0x3810,0x0024,0x38f0,0x4024,0x34c3,0x0024,0x3444,0x0024,
        0x3073,0x0024,0x3063,0x0024,0x3000,0x0024,0x4080,0x0024,
        0x0000,0x0024,0x2839,0x53d5,0x4284,0x0024,0x3613,0x0024,
        0x2801,0x49c5,0x6898,0xb804,0x0000,0x0084,0x293b,0x1cc0,
        0x3613,0x0024,0x000c,0x8117,0x3711,0x0024,0x37d1,0x4024,
        0x4e8a,0x0024,0x0000,0x0015,0x2801,0x4c85,0xce9a,0x0024,
        0x3f11,0x0024,0x3f01,0x4024,0x000c,0x8197,0x408a,0x9bc4,
        0x3f15,0x4024,0x2801,0x4ec5,0x4284,0x3c15,0x6590,0x0024,
        0x0000,0x0024,0x2839,0x53d5,0x4284,0x0024,0x0000,0x0024,
        0x2801,0x3a98,0x458a,0x0024,0x2a39,0x53c0,0x003e,0x2d4f,
        0x283a,0x5ed5,0x0001,0x334e,0x000c,0x4653,0x0000,0x0246,
        0xffac,0x0c01,0x48be,0x0024,0x4162,0x4546,0x6642,0x4055,
        0x3501,0x8024,0x0000,0x0087,0x667c,0x4057,0x000c,0x41d5,
        0x283a,0x62d5,0x3501,0x8024,0x667c,0x1c47,0x3701,0x8024,
        0x283a,0x62d5,0xc67c,0x0024,0x0000,0x0024,0x283a,0x62c5,
        0x0000,0x0024,0x2a3a,0x5ec0,0x3009,0x3851,0x3e14,0xf812,
        0x3e12,0xb817,0x3e11,0x8024,0x0006,0x0293,0x3301,0x8024,
        0x468c,0x3804,0x0006,0xa057,0x2801,0x5bc4,0x0006,0x0011,
        0x469c,0x0024,0x3be1,0x8024,0x2801,0x5bd5,0x0006,0xc392,
        0x3311,0x0024,0x33f1,0x2844,0x3009,0x2bc4,0x0030,0x04d2,
        0x3311,0x0024,0x3a11,0x0024,0x3201,0x8024,0x003f,0xfc04,
        0xb64c,0x0fc4,0xc648,0x0024,0x3a01,0x0024,0x3111,0x1fd3,
        0x6498,0x07c6,0x868c,0x2444,0x0023,0xffd2,0x3901,0x8e06,
        0x0030,0x0551,0x3911,0x8e06,0x3961,0x9c44,0xf400,0x44c6,
        0xd46c,0x1bc4,0x36f1,0xbc13,0x2801,0x6555,0x36f2,0x9817,
        0x002b,0xffd2,0x3383,0x188c,0x3e01,0x8c06,0x0006,0xa097,
        0x3009,0x1c12,0x3213,0x0024,0x468c,0xbc12,0x002b,0xffd2,
        0xf400,0x4197,0x2801,0x6244,0x3713,0x0024,0x2801,0x6285,
        0x37e3,0x0024,0x3009,0x2c17,0x3383,0x0024,0x3009,0x0c06,
        0x468c,0x4197,0x0006,0xa052,0x2801,0x6484,0x3713,0x2813,
        0x2801,0x64c5,0x37e3,0x0024,0x3009,0x2c17,0x36f1,0x8024,
        0x36f2,0x9817,0x36f4,0xd812,0x2100,0x0000,0x3904,0x5bd1,
        0x2a01,0x558e,0x3e11,0x7804,0x0030,0x0257,0x3701,0x0024,
        0x0013,0x4d05,0xd45b,0xe0e1,0x0007,0xc795,0x2801,0x6cd5,
        0x0fff,0xff45,0x3511,0x184c,0x4488,0xb808,0x0006,0x8a97,
        0x2801,0x6c85,0x3009,0x1c40,0x3511,0x1fc1,0x0000,0x0020,
        0xac52,0x1405,0x6ce2,0x0024,0x0000,0x0024,0x2801,0x6c81,
        0x68c2,0x0024,0x291a,0x8a40,0x3e10,0x0024,0x2921,0xca80,
        0x3e00,0x4024,0x36f3,0x0024,0x3009,0x1bc8,0x36f0,0x1801,
        0x3601,0x5804,0x3e13,0x780f,0x3e13,0xb808,0x0008,0x9b0f,
        0x0001,0x6f8e,0x2908,0x9300,0x0000,0x004d,0x36f3,0x9808,
        0x2000,0x0000,0x36f3,0x580f,0x0007,0x81d7,0x3711,0x8024,
        0x3711,0xc024,0x3700,0x0024,0x0000,0x2001,0xb012,0x0024,
        0x0034,0x0000,0x2801,0x7245,0x0000,0x01c1,0x0014,0xc000,
        0x0000,0x01c1,0x4fce,0x0024,0xffea,0x0024,0x48b6,0x0024,
        0x4384,0x4097,0xb886,0x45c6,0xfede,0x0024,0x4db6,0x0024,
        0x466c,0x0024,0x0006,0xc610,0x8dd6,0x8007,0x0000,0x00c6,
        0xff6e,0x0024,0x48b2,0x0024,0x0034,0x2406,0xffee,0x0024,
        0x2914,0xaa80,0x40b2,0x0024,0xf1c6,0x0024,0xf1d6,0x0024,
        0x0000,0x0201,0x8d86,0x0024,0x61de,0x0024,0x0006,0xc612,
        0x2801,0x78c1,0x0006,0xc713,0x4c86,0x0024,0x2912,0x1180,
        0x0006,0xc351,0x0006,0x0210,0x2912,0x0d00,0x3810,0x984c,
        0xf200,0x2043,0x2808,0xa000,0x3800,0x0024,
        0x0007,0x0001, /*copy 1*/
        0x802e,
        0x0006,0x0002, /*copy 2*/
        0x2801,0x6640,
        0x0007,0x0001, /*copy 1*/
        0x8030,
        0x0006,0x0002, /*copy 2*/
        0x2800,0x1b40,
        0x0007,0x0001, /*copy 1*/
        0x8028,
        0x0006,0x0002, /*copy 2*/
        0x2a00,0x42ce,
        0x0007,0x0001, /*copy 1*/
        0x8032,
        0x0006,0x0002, /*copy 2*/
        0x2800,0x5f40,
        0x0007,0x0001, /*copy 1*/
        0x3580,
        0x0006, 0x8038, 0x0000, /*Rle(56)*/
        0x0007,0x0001, /*copy 1*/
        0xfab3,
        0x0006,0x01a4, /*copy 420*/
        0x0001,0x0001,0x0001,0x0001,0x0000,0xffff,0xfffe,0xfffb,
        0xfff9,0xfff5,0xfff2,0xffed,0xffe8,0xffe3,0xffde,0xffd8,
        0xffd3,0xffce,0xffca,0xffc7,0xffc4,0xffc4,0xffc5,0xffc7,
        0xffcc,0xffd3,0xffdc,0xffe6,0xfff3,0x0001,0x0010,0x001f,
        0x002f,0x003f,0x004e,0x005b,0x0066,0x006f,0x0074,0x0075,
        0x0072,0x006b,0x005f,0x004f,0x003c,0x0024,0x0009,0xffed,
        0xffcf,0xffb0,0xff93,0xff77,0xff5f,0xff4c,0xff3d,0xff35,
        0xff34,0xff3b,0xff4a,0xff60,0xff7e,0xffa2,0xffcd,0xfffc,
        0x002e,0x0061,0x0094,0x00c4,0x00f0,0x0114,0x0131,0x0144,
        0x014b,0x0146,0x0134,0x0116,0x00eb,0x00b5,0x0075,0x002c,
        0xffde,0xff8e,0xff3d,0xfeef,0xfea8,0xfe6a,0xfe39,0xfe16,
        0xfe05,0xfe06,0xfe1b,0xfe43,0xfe7f,0xfecd,0xff2a,0xff95,
        0x0009,0x0082,0x00fd,0x0173,0x01e1,0x0242,0x0292,0x02cc,
        0x02ec,0x02f2,0x02da,0x02a5,0x0253,0x01e7,0x0162,0x00c9,
        0x0021,0xff70,0xfebc,0xfe0c,0xfd68,0xfcd5,0xfc5b,0xfc00,
        0xfbc9,0xfbb8,0xfbd2,0xfc16,0xfc85,0xfd1b,0xfdd6,0xfeae,
        0xff9e,0x009c,0x01a0,0x02a1,0x0392,0x046c,0x0523,0x05b0,
        0x060a,0x062c,0x0613,0x05bb,0x0526,0x0456,0x0351,0x021f,
        0x00c9,0xff5a,0xfde1,0xfc6a,0xfb05,0xf9c0,0xf8aa,0xf7d0,
        0xf73d,0xf6fa,0xf70f,0xf77e,0xf848,0xf96b,0xfadf,0xfc9a,
        0xfe8f,0x00ad,0x02e3,0x051a,0x073f,0x0939,0x0af4,0x0c5a,
        0x0d59,0x0de1,0x0de5,0x0d5c,0x0c44,0x0a9e,0x0870,0x05c7,
        0x02b4,0xff4e,0xfbaf,0xf7f8,0xf449,0xf0c7,0xed98,0xeae0,
        0xe8c4,0xe765,0xe6e3,0xe756,0xe8d2,0xeb67,0xef19,0xf3e9,
        0xf9cd,0x00b5,0x088a,0x112b,0x1a72,0x2435,0x2e42,0x3866,
        0x426b,0x4c1b,0x553e,0x5da2,0x6516,0x6b6f,0x7087,0x7441,
        0x7686,0x774a,0x7686,0x7441,0x7087,0x6b6f,0x6516,0x5da2,
        0x553e,0x4c1b,0x426b,0x3866,0x2e42,0x2435,0x1a72,0x112b,
        0x088a,0x00b5,0xf9cd,0xf3e9,0xef19,0xeb67,0xe8d2,0xe756,
        0xe6e3,0xe765,0xe8c4,0xeae0,0xed98,0xf0c7,0xf449,0xf7f8,
        0xfbaf,0xff4e,0x02b4,0x05c7,0x0870,0x0a9e,0x0c44,0x0d5c,
        0x0de5,0x0de1,0x0d59,0x0c5a,0x0af4,0x0939,0x073f,0x051a,
        0x02e3,0x00ad,0xfe8f,0xfc9a,0xfadf,0xf96b,0xf848,0xf77e,
        0xf70f,0xf6fa,0xf73d,0xf7d0,0xf8aa,0xf9c0,0xfb05,0xfc6a,
        0xfde1,0xff5a,0x00c9,0x021f,0x0351,0x0456,0x0526,0x05bb,
        0x0613,0x062c,0x060a,0x05b0,0x0523,0x046c,0x0392,0x02a1,
        0x01a0,0x009c,0xff9e,0xfeae,0xfdd6,0xfd1b,0xfc85,0xfc16,
        0xfbd2,0xfbb8,0xfbc9,0xfc00,0xfc5b,0xfcd5,0xfd68,0xfe0c,
        0xfebc,0xff70,0x0021,0x00c9,0x0162,0x01e7,0x0253,0x02a5,
        0x02da,0x02f2,0x02ec,0x02cc,0x0292,0x0242,0x01e1,0x0173,
        0x00fd,0x0082,0x0009,0xff95,0xff2a,0xfecd,0xfe7f,0xfe43,
        0xfe1b,0xfe06,0xfe05,0xfe16,0xfe39,0xfe6a,0xfea8,0xfeef,
        0xff3d,0xff8e,0xffde,0x002c,0x0075,0x00b5,0x00eb,0x0116,
        0x0134,0x0146,0x014b,0x0144,0x0131,0x0114,0x00f0,0x00c4,
        0x0094,0x0061,0x002e,0xfffc,0xffcd,0xffa2,0xff7e,0xff60,
        0xff4a,0xff3b,0xff34,0xff35,0xff3d,0xff4c,0xff5f,0xff77,
        0xff93,0xffb0,0xffcf,0xffed,0x0009,0x0024,0x003c,0x004f,
        0x005f,0x006b,0x0072,0x0075,0x0074,0x006f,0x0066,0x005b,
        0x004e,0x003f,0x002f,0x001f,0x0010,0x0001,0xfff3,0xffe6,
        0xffdc,0xffd3,0xffcc,0xffc7,0xffc5,0xffc4,0xffc4,0xffc7,
        0xffca,0xffce,0xffd3,0xffd8,0xffde,0xffe3,0xffe8,0xffed,
        0xfff2,0xfff5,0xfff9,0xfffb,0xfffe,0xffff,0x0000,0x0001,
        0x0001,0x0001,0x0001,0x0000,
        0x0007,0x0001, /*copy 1*/
        0x180b,
        0x0006,0x000b, /*copy 11*/
        0x000f,0x0010,0x001c,0xfab3,0x3580,0x8037,0xa037,0x0001,
        0x0000,0x3580,0x01a4,
        0x000a,0x0001, /*copy 1*/
        0x0300,
    };

    sci_write(spi, VS1053_REG_CLOCKF, 0xC000);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    while(!audio_ready_for_data(spi));

    sci_write(spi, VS1053_REG_BASS, 0);
    audio_soft_reset(spi);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    while(!audio_ready_for_data(spi));

    sci_write(spi, VS1053_SCI_AIADDR, 0);
    // not sure if need to add interuppt stuff?
    sci_write(spi, VS1053_REG_WRAMADDR, VS1053_INT_ENABLE);
    sci_write(spi, VS1053_REG_WRAM, 0x02);
    audio_load_plg(spi, sizeof(vs1053b_decoding_patch) / sizeof(vs1053b_decoding_patch[0]), vs1053b_decoding_patch);
    ESP_LOGI(TAG, "%d", sizeof(vs1053b_decoding_patch) / sizeof(vs1053b_decoding_patch[0]))
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
    while (!audio_ready_for_data(spi));
}

rb_t rb_init(uint32_t capacity)
{
    rb_t rb;

    rb.write = 0;
    rb.read = 0;
    rb.capacity = capacity;
    rb.ring_buffer = (uint8_t *) malloc(sizeof(uint8_t) * capacity);
    assert(rb.ring_buffer != NULL);

    return rb;
}

void rb_free(rb_t *rb)
{
    free(rb->ring_buffer);
    rb->capacity = 0;
    rb->ring_buffer = NULL;
}

uint32_t rb_mask(rb_t *rb, uint32_t val)
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
    // ESP_LOGI(TAG, "%d (%d)", rb->read, rb_mask(rb, rb->read));
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
