/* VS1053 header */
#ifndef _VS1053_H_
#define _VS1053_H_

#include "driver/spi_master.h"

#define VS1053_1_RESET 22
#define VS1053_1_DREQ 32
#define VS1053_2_RESET 21
#define VS1053_2_DREQ 35

#define VS1053_SCI_READ 0x03
#define VS1053_SCI_WRITE 0x02

#define VS1053_REG_MODE  0x00
#define VS1053_REG_STATUS 0x01
#define VS1053_REG_BASS 0x02
#define VS1053_REG_CLOCKF 0x03
#define VS1053_REG_DECODETIME 0x04
#define VS1053_REG_AUDATA 0x05
#define VS1053_REG_WRAM 0x06
#define VS1053_REG_WRAMADDR 0x07
#define VS1053_REG_HDAT0 0x08
#define VS1053_REG_HDAT1 0x09
#define VS1053_REG_VOLUME 0x0B

#define VS1053_GPIO_DDR 0xC017
#define VS1053_GPIO_IDATA 0xC018
#define VS1053_GPIO_ODATA 0xC019

#define VS1053_INT_ENABLE  0xC01A

#define VS1053_MODE_SM_DIFF 0x0001
#define VS1053_MODE_SM_LAYER12 0x0002
#define VS1053_MODE_SM_RESET 0x0004
#define VS1053_MODE_SM_CANCEL 0x0008
#define VS1053_MODE_SM_EARSPKLO 0x0010
#define VS1053_MODE_SM_TESTS 0x0020
#define VS1053_MODE_SM_STREAM 0x0040
#define VS1053_MODE_SM_SDINEW 0x0800
#define VS1053_MODE_SM_ADPCM 0x1000
#define VS1053_MODE_SM_LINE1 0x4000
#define VS1053_MODE_SM_CLKRANGE 0x8000


#define VS1053_SCI_AIADDR 0x0A
#define VS1053_SCI_AICTRL0 0x0C
#define VS1053_SCI_AICTRL1 0x0D
#define VS1053_SCI_AICTRL2 0x0E
#define VS1053_SCI_AICTRL3 0x0F

typedef struct {
	spi_device_handle_t control;
	spi_device_handle_t data;
	uint32_t reset;
	uint32_t dreq;
} audio_bus_t;

typedef struct {
	audio_bus_t spi1;
	audio_bus_t spi2;
} audio_spi_t;

typedef struct {
	uint32_t write;
	uint32_t read;
	uint32_t capacity;
	uint8_t *ring_buffer;
} rb_t;

/* helper functions */
audio_spi_t audio_spi_init();
void sci_write(audio_bus_t spi, uint8_t addr, uint16_t data);
uint16_t sci_read(audio_bus_t spi, uint8_t addr);
void sdi_write(audio_bus_t spi, uint16_t num_bytes, uint8_t *data);
void audio_soft_reset(audio_bus_t spi);
void audio_reset(audio_bus_t spi);
bool audio_ready_for_data(audio_bus_t spi);
bool audio_prepare_ogg(audio_bus_t spi);
uint16_t audio_recorded_words_waiting(audio_bus_t spi);
uint16_t audio_recorded_read_word(audio_bus_t spi);
void audio_stop_record(audio_bus_t spi);
void audio_start_record(audio_bus_t spi, bool mic);
void audio_start_playback(audio_bus_t spi);

rb_t rb_init(uint32_t capacity);
uint8_t rb_mask(rb_t *rb, uint8_t val);
void rb_push(rb_t *rb, uint8_t push);
uint8_t rb_shift(rb_t *rb);
bool rb_empty(rb_t *rb);
bool rb_full(rb_t *rb);
uint32_t rb_size(rb_t *rb);
void rb_free(rb_t *rb);

#endif /* _VS1053_H_ */