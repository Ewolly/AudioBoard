#include <SPI.h>

#define VS1053_SCI_READ 0x03
#define VS1053_SCI_WRITE 0x02

#define VS1053_CONTROL_SPI_SETTING  SPISettings(250000,  MSBFIRST, SPI_MODE0)
#define VS1053_DATA_SPI_SETTING     SPISettings(8000000, MSBFIRST, SPI_MODE0)

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

#define SCK 13
#define MISO 12
#define MOSI 11
#define SS 10
#define DS 9
#define RESET 8
#define DREQ 7

uint16_t sciRead(uint8_t addr);
void sciWrite(uint8_t addr, uint16_t data);

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128); 

  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  pinMode(DS, OUTPUT);
  digitalWrite(DS, HIGH);
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);

  reset();
  delay(100);
  Serial.print("mode = 0x"); Serial.println(sciRead(VS1053_REG_MODE), HEX);
  delay(100);
  Serial.print("stat = 0x"); Serial.println(sciRead(VS1053_REG_STATUS), HEX);
  delay(100);
  Serial.print("clkf = 0x"); Serial.println(sciRead(VS1053_REG_CLOCKF), HEX);
  delay(100);
  Serial.print("vol. = 0x"); Serial.println(sciRead(VS1053_REG_VOLUME), HEX);
}

void loop() {
  // put your main code here, to run repeatedly:
}

uint8_t spiRead(void)
{
    return SPI.transfer(0x00);
}

void spiWrite(uint8_t c)
{
  uint8_t x __attribute__ ((aligned (32))) = c;
  spiWrite(&x,1);
}

void spiWrite(uint8_t *c, uint16_t num)
{
  while (num--){
    SPI.transfer(c[0]);
    c++;
  }
}

uint16_t sciRead(uint8_t addr)
{
  uint16_t data;

  SPI.beginTransaction(VS1053_CONTROL_SPI_SETTING);
  digitalWrite(SS, LOW);
  spiWrite(VS1053_SCI_READ);
  spiWrite(addr);
  delayMicroseconds(10);
  data = spiRead();
  data <<= 8;
  data |= spiRead();
  SPI.endTransaction();
  while(!digitalRead(DREQ));
  digitalWrite(SS, HIGH);
  
  Serial.println(data, HEX);
  return data;
}

void sciWrite (uint8_t addr, uint16_t data)
{
  SPI.beginTransaction(VS1053_CONTROL_SPI_SETTING);
  digitalWrite(SS, LOW);
  spiWrite(VS1053_SCI_WRITE);
  spiWrite(addr);
  spiWrite(data >> 8);
  spiWrite(data & 0xFF);
  SPI.endTransaction();
  while(!digitalRead(DREQ));
  digitalWrite(SS, HIGH);
}

void setVolume(uint8_t left, uint8_t right){
  uint16_t v;
  v = left;
  v <<= 8;
  v |= right;

  noInterrupts();
  sciWrite(VS1053_REG_VOLUME, v);
  interrupts();
}

void softReset(void){
  sciWrite(VS1053_REG_MODE, VS1053_MODE_SM_SDINEW | VS1053_MODE_SM_RESET);
  delay(100);
}

void reset(){
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET,HIGH);
  digitalWrite(SS, HIGH);
  digitalWrite(DS, HIGH);
  delay(100);
  softReset();
  delay(100);

  sciWrite(VS1053_REG_CLOCKF, 0x6000);

  setVolume(40,40);
}


