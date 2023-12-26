/*
 * bmp280.h
 *
 *  Created on: Dec 25, 2023
 *      Author: Marcin Kosela (KoSik)
 *		e-mail: kosik84@gmail.com
 *		
 *	   version: 1.0
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

struct BMP280 {
	SPI_HandleTypeDef *hspi;
	uint8_t status;
	uint8_t id;
	uint64_t temperature;
	uint64_t pressure;
	uint8_t config;
	uint8_t ctrlMeas;
	GPIO_TypeDef *CsPort;
	uint16_t CsPin;
	unsigned short dig_T1; //0x88 / 0x89
	signed short dig_T2; //0x8A / 0x8B
	signed short dig_T3; //0x8C / 0x8D
	unsigned short dig_P1; //0x8E / 0x8F
	signed short dig_P2; //0x90 / 0x91
	signed short dig_P3; //0x92 / 0x93
	signed short dig_P4; //0x94 / 0x95
	signed short dig_P5; //0x96 / 0x97
	signed short dig_P6;  //0x98 / 0x99
	signed short dig_P7; //0x9A / 0x9B
	signed short dig_P8;  //0x9C / 0x9D
	signed short dig_P9;  //0x9E / 0x9F
};

#define BMP280_ID_NUMBER 0x58

#define BMP280_BIT_READ 0x80

#define BMP280_TEMP_XLSB 0xFC
#define BMP280_TEMP_LSB 0xFB
#define BMP280_TEMP_MSB 0xFA
#define BMP280_PRESS_XLSB 0xF9
#define BMP280_PRESS_LSB 0xF8
#define BMP280_PRESS_MSB 0xF7
#define BMP280_CONFIG 0xF5
#define BMP280_CTRL_MEAS 0xf4
#define BMP280_STATUS 0xF3
#define BMP280_RESET 0xE0
#define BMP280_ID 0xD0
#define BMP280_CALIBRATION 0x88

#define BMP280_OVRSAMP_OFF 0b000
#define BMP280_OVRSAMP_1 0b001
#define BMP280_OVRSAMP_2 0b010
#define BMP280_OVRSAMP_4 0b011
#define BMP280_OVRSAMP_8 0b100
#define BMP280_OVRSAMP_16 0b101

#define BMP280_SLEEP 0b00
#define BMP280_FORCED 0b01
#define BMP280_NORMAL 0b11

#define BMP280_TSB_0_5  0b000
#define BMP280_TSB_62_5 0b001
#define BMP280_TSB_125  0b010
#define BMP280_TSB_250  0b011
#define BMP280_TSB_500  0b100
#define BMP280_TSB_1000 0b101
#define BMP280_TSB_2000 0b110
#define BMP280_TSB_4000 0b111

#define BMP280_FILTER_OFF 0b000
#define BMP280_FILTER_2   0b001
#define BMP280_FILTER_4   0b010
#define BMP280_FILTER_8   0b011
#define BMP280_FILTER_16  0b100

#define BMP280_3W_SPI_OFF 0b0
#define BMP280_3W_SPI_ON  0b1

void bmp280_setConfig(uint8_t tSb, uint8_t filter, uint8_t spi3w);
void bmp280_setCtrlMeas(uint8_t osrsT, uint8_t osrsP, uint8_t mode);
HAL_StatusTypeDef bmp280_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin);
HAL_StatusTypeDef bmp280_reset(void);
void bmp280_startMeasuring(void);
uint8_t bmp280_readRegister(uint8_t register);
HAL_StatusTypeDef bmp280_writeRegister(uint8_t rreg, uint8_t data);
HAL_StatusTypeDef bmp280_readSensor(void);
HAL_StatusTypeDef bmp280_readCalibration(void);

void bmp280_CsPinDisable(void);
void bmp280_CsPinEnable(void);

uint32_t getUs(void);
void delayUs(uint16_t micros);


#endif /* INC_BMP280_H_ */
