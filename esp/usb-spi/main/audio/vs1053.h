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

#define VS1053_MODE_SM_RESET 0x0004
#define VS1053_MODE_SM_SDINEW 0x0800

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

/* helper functions */
audio_spi_t audio_spi_init();
void sci_write(audio_bus_t spi, uint8_t addr, uint16_t data);
uint16_t sci_read(audio_bus_t spi, uint8_t addr);
void sdi_write(audio_bus_t spi, uint16_t num_bytes, uint8_t *data);
void audio_soft_reset(audio_bus_t spi);
void audio_reset(audio_bus_t spi);
#endif /* _VS1053_H_ */