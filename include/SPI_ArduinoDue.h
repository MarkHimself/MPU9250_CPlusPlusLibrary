#ifndef SPI_ARDUINODUE_H
#define SPI_ARDUINODUE_H

#include "Arduino.h"

// setup spi and pins
void setup_SPI0_Master(uint8_t spi_mode);
bool SPI0_Clock_Rate_khz(uint16_t f);
void setup_PIOA25_as_SPI0_MISO();
void setup_PIOA26_as_SPI0_MOSI();
void setup_PIOA27_as_SPI0_SCK();
void setup_PIOA28_as_cs_digital();

// write/read through spi
uint8_t write_SPI0_blocking(uint8_t val);
void spi_cs_imu_due(bool high);

#endif