/*
 * bmp280.c
 *
 *  Created on: Dec 25, 2023
 *      Author: Marcin Kosela (KoSik)
 *		e-mail: kosik84@gmail.com
 *		
 *	   version: 1.0
 */

#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "main.h"
#include "bmp280.h"

struct BMP280 bmp280;
uint8_t spiBusyFlag = 0;

#define bmp280_20bit_reg(b1, b2, b3) ( \
	((int32_t)(b1) << 12) \
	| ((int32_t)(b2) << 4) \
	| ((int32_t)(b3) >> 4) \
)

/*
 * Function create config setting:
 * tSb - Controls inactive duration tstandby in normal mode
 * filter - Controls the time constant of the IIR filter
 * spi3w - Enables 3-wire SPI interface when set to ‘1’
 * */
void bmp280_setConfig(uint8_t tSb, uint8_t filter, uint8_t spi3w){
	bmp280.config = (tSb & 0b00000111) << 5 | (filter & 0b00000111) << 2 | (spi3w & 0b00000001);
}

/*
 * Function create ctrl_meas setting:
 * osrsT - Controls oversampling of temperature data
 * osrsP - Controls oversampling of pressure data
 * mode - Controls the power mode of the device
 * */
void bmp280_setCtrlMeas(uint8_t osrsT, uint8_t osrsP, uint8_t mode){
	bmp280.ctrlMeas = (osrsT & 0b00000111) << 5 | (osrsP & 0b00000111) << 2 | (mode & 0b00000011);
}

HAL_StatusTypeDef bmp280_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin){
	bmp280.hspi = hspi;
	bmp280.CsPort = port;
	bmp280.CsPin = pin;
	bmp280_CsPinDisable();
	bmp280.id =  bmp280_readRegister(BMP280_ID);

	bmp280_readCalibration();
	bmp280_writeRegister(BMP280_CONFIG, bmp280.config);
	bmp280_writeRegister(BMP280_CTRL_MEAS, bmp280.ctrlMeas);
	if(bmp280.id == BMP280_ID_NUMBER){
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef bmp280_reset(void){
	HAL_StatusTypeDef res;
	res = bmp280_writeRegister(RESET, 0xB6);
	HAL_Delay(10);
	return res;
}

void bmp280_startMeasuring(void){
	bmp280_writeRegister(BMP280_CTRL_MEAS, bmp280.ctrlMeas);
}

uint8_t bmp280_readRegister(uint8_t rreg){
	HAL_StatusTypeDef res = HAL_OK;
	uint8_t txData[2];
	uint8_t rxData[2];
	for(uint8_t i=0; i<sizeof(txData); i++){
		txData[i] = 0x00;
	}

	bmp280_CsPinEnable();
	txData[0] = rreg | BMP280_BIT_READ;
	res = HAL_SPI_TransmitReceive_DMA(bmp280.hspi, txData, rxData, 2);
	while(spiBusyFlag){
		delayUs(1);
	}
	if(res == HAL_OK){
		return rxData[1];
	} else {
		return 0x00;
	}
}

HAL_StatusTypeDef bmp280_writeRegister(uint8_t rreg, uint8_t data){
	HAL_StatusTypeDef res = HAL_OK;
	uint8_t txData[2];
	uint8_t rxData[2];
	for(uint8_t i=0; i<sizeof(txData); i++){
		txData[i] = 0x00;
	}
	bmp280_CsPinEnable();
	txData[0] = rreg;
	txData[0] &= ~(1 << 7);
	txData[1] = data;
	res = HAL_SPI_TransmitReceive_DMA(bmp280.hspi, txData, rxData, 2);
	while(spiBusyFlag){
		delayUs(1);
	}
	return res;
}

HAL_StatusTypeDef bmp280_readSensor(void){
	HAL_StatusTypeDef res = HAL_OK;
	uint8_t txData[7];
	uint8_t rxData[7];
	int32_t temp_raw, pres_raw, var1, var2, t_fine;
	for(uint8_t i=0; i<sizeof(txData); i++){
			txData[i] = 0x00;
		}

	bmp280_CsPinEnable();
	txData[0] = BMP280_PRESS_MSB | BMP280_BIT_READ;
	res = HAL_SPI_TransmitReceive_DMA(bmp280.hspi, txData, rxData, 7);
	while(spiBusyFlag){
		delayUs(1);
	}
	pres_raw = bmp280_20bit_reg(rxData[1], rxData[2], rxData[3]);
	temp_raw = bmp280_20bit_reg(rxData[4], rxData[5], rxData[6]);

	// TEMPERATURE
	var1 = ((((temp_raw >> 3) - ((int32_t)bmp280.dig_T1 << 1)))
		* ((int32_t)bmp280.dig_T2)) >> 11;
	var2 = (((((temp_raw >> 4) - ((int32_t)bmp280.dig_T1))
		* ((temp_raw >> 4) - ((int32_t)bmp280.dig_T1))) >> 12)
		* ((int32_t)bmp280.dig_T3)) >> 14;
	t_fine = var1 + var2;
	bmp280.temperature = (t_fine * 5 + 128) >> 8;

	// PRESSURE
	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)bmp280.dig_P6);
	var2 = var2 + ((var1 * ((int32_t)bmp280.dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)bmp280.dig_P4) << 16);
	var1 = (((bmp280.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)bmp280.dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)bmp280.dig_P1)) >> 15);

	if (var1 == 0) {
		bmp280.pressure = 0;
	} else {
		bmp280.pressure = (((uint32_t)(((int32_t)1048576)-pres_raw) - (var2 >> 12))) * 3125;
		if (bmp280.pressure < 0x80000000) {
			bmp280.pressure = (bmp280.pressure << 1) / ((uint32_t)var1);
		} else {
			bmp280.pressure = (bmp280.pressure / (uint32_t)var1) * 2;
		}
		var1 = (((int32_t)bmp280.dig_P9) * ((int32_t)(((bmp280.pressure>>3) * (bmp280.pressure >> 3)) >> 13))) >> 12;
		var2 = (((int32_t)(bmp280.pressure >> 2)) * ((int32_t)bmp280.dig_P8)) >> 13;
		bmp280.pressure = (uint32_t)((int32_t)bmp280.pressure + ((var1 + var2 + bmp280.dig_P7) >> 4));
	}
	return res;
}

uint64_t bmp280_getTeperature(void){
	return bmp280.temperature;
}

uint64_t bmp280_getPressure(void){
	return bmp280.pressure;
}

HAL_StatusTypeDef bmp280_readCalibration(void){
	HAL_StatusTypeDef res = HAL_OK;
	uint8_t txData[26];
	uint8_t rxData[26];
	for(uint8_t i=0; i<sizeof(txData); i++){
		txData[i] = 0x00;
	}

	spiBusyFlag = 1;
	bmp280_CsPinEnable();
	txData[0] = BMP280_CALIBRATION | BMP280_BIT_READ;
	res = HAL_SPI_TransmitReceive_DMA(bmp280.hspi, txData, rxData, 26);
	while(spiBusyFlag){
		delayUs(1);
	}
	bmp280.dig_T1 = rxData[2] << 8 | rxData[1]; //0x88 / 0x89
	bmp280.dig_T2 = rxData[4] << 8 | rxData[3]; //0x8A / 0x8B
	bmp280.dig_T3 = rxData[6] << 8 | rxData[5]; //0x8C / 0x8D
	bmp280.dig_P1 = rxData[8] << 8 | rxData[7]; //0x8E / 0x8F
	bmp280.dig_P2 = rxData[10] << 8 | rxData[9]; //0x90 / 0x91
	bmp280.dig_P3 = rxData[12] << 8 | rxData[11]; //0x92 / 0x93
	bmp280.dig_P4 = rxData[14] << 8 | rxData[13]; //0x94 / 0x95
	bmp280.dig_P5 = rxData[16] << 8 | rxData[15]; //0x96 / 0x97
	bmp280.dig_P6 = rxData[18] << 8 | rxData[17];  //0x98 / 0x99
	bmp280.dig_P7 = rxData[20] << 8 | rxData[19]; //0x9A / 0x9B
	bmp280.dig_P8 = rxData[22] << 8 | rxData[21];  //0x9C / 0x9D
	bmp280.dig_P9 = rxData[24] << 8 | rxData[23];  //0x9E / 0x9F
	return res;
}

/*
 * SPI CALLBACK - CS PIN DISABLE WHEN TRANMIT COMPLETE
 * */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(bmp280.hspi == hspi){
		bmp280_CsPinDisable();
		spiBusyFlag = 0;
	}
}

void bmp280_CsPinDisable(void){
	HAL_GPIO_WritePin(bmp280.CsPort, bmp280.CsPin, 1);
}

void bmp280_CsPinEnable(void){
	// LOW is enable
	HAL_GPIO_WritePin(bmp280.CsPort, bmp280.CsPin, 0);
	spiBusyFlag = 1;
}

uint32_t getUs(void){
	uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
	register uint32_t ms, cycle_cnt;
	do {
		ms = HAL_GetTick();
		cycle_cnt = SysTick->VAL;
	} while (ms != HAL_GetTick());
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void delayUs(uint16_t micros){
	uint32_t start = getUs();
	while (getUs() - start < (uint32_t) micros) {
		asm("nop");
	}
}


