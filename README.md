<h1 align="center">BMP280 Library for STM32</h1>

- SPI connection with DMA
- HAL library uses

<h2>How to use:</h2>

<h3>INIT:</h3>

    bmp280_setCtrlMeas(BMP280_OVRSAMP_8, BMP280_OVRSAMP_8, BMP280_FORCED);
    bmp280_setConfig(BMP280_TSB_1000, BMP280_FILTER_8, BMP280_3W_SPI_OFF);

    bmp280_init(&hspi1, BMP280_CS_GPIO_Port, BMP280_CS_Pin);

<h3>READ:</h3>

    bmp280_startMeasuring();
    HAL_Delay(1000);
    bmp280_readSensor();

## :memo: License ##
This project is licensed under the MIT License. For more details, please refer to the [LICENSE](LICENSE.md) file.

<br/>
<p align="center">Made by <a href="https://github.com/kosik-prog/" target="_blank">KoSik</a><p/>
<br/>
<a href="#top">Back to top</a>