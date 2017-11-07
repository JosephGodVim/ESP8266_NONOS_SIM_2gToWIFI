/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "ets_sys.h"
#include "osapi.h"
#include "mem.h"

#include "user_interface.h"
#include "driver/uart.h"
#include "driver/spi_interface.h"
#include "gpio.h"
/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void spi_slave_isr_sta(void *para)
{
    uint32 regvalue;
    uint32 statusW, statusR, counter;
    if (READ_PERI_REG(0x3ff00020)&BIT4) {
        //following 3 lines is to clear isr signal
        CLEAR_PERI_REG_MASK(SPI_SLAVE(SpiNum_SPI), 0x3ff);
    } else if (READ_PERI_REG(0x3ff00020)&BIT7) { //bit7 is for hspi isr,
        regvalue = READ_PERI_REG(SPI_SLAVE(SpiNum_HSPI));
        os_printf("spi_slave_isr_sta SPI_SLAVE[0x%08x]\n\r", regvalue);
        SPIIntClear(SpiNum_HSPI);
        SET_PERI_REG_MASK(SPI_SLAVE(SpiNum_HSPI), SPI_SYNC_RESET);
        SPIIntClear(SpiNum_HSPI);

        SPIIntEnable(SpiNum_HSPI, SpiIntSrc_WrStaDone
                 | SpiIntSrc_RdStaDone
                 | SpiIntSrc_WrBufDone
                 | SpiIntSrc_RdBufDone);

        if (regvalue & SPI_SLV_WR_BUF_DONE) {
            // User can get data from the W0~W7
            os_printf("spi_slave_isr_sta : SPI_SLV_WR_BUF_DONE\n\r");
        } else if (regvalue & SPI_SLV_RD_BUF_DONE) {
            // TO DO
            os_printf("spi_slave_isr_sta : SPI_SLV_RD_BUF_DONE\n\r");
        }
        if (regvalue & SPI_SLV_RD_STA_DONE) {
            statusR = READ_PERI_REG(SPI_RD_STATUS(SpiNum_HSPI));
            statusW = READ_PERI_REG(SPI_WR_STATUS(SpiNum_HSPI));
            os_printf("spi_slave_isr_sta : SPI_SLV_RD_STA_DONE[R=0x%08x,W=0x%08x]\n\r", statusR, statusW);
        }

        if (regvalue & SPI_SLV_WR_STA_DONE) {
            statusR = READ_PERI_REG(SPI_RD_STATUS(SpiNum_HSPI));
            statusW = READ_PERI_REG(SPI_WR_STATUS(SpiNum_HSPI));
            os_printf("spi_slave_isr_sta : SPI_SLV_WR_STA_DONE[R=0x%08x,W=0x%08x]\n\r", statusR, statusW);
        }
        if ((regvalue & SPI_TRANS_DONE) && ((regvalue & 0xf) == 0)) {
            os_printf("spi_slave_isr_sta : SPI_TRANS_DONE\n\r");

        }
        SHOWSPIREG(SpiNum_HSPI);
    }
}

void ICACHE_FLASH_ATTR
spi_master_config()
{
	SpiAttr	cSpiAttr;
	cSpiAttr.bitOrder = SpiBitOrder_MSBFirst;
	cSpiAttr.mode = SpiMode_Slave;
	cSpiAttr.speed = 0;
	cSpiAttr.subMode = SpiSubMode_0;

	WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2);

	os_printf("\r\n ============= spi init slave =============\r\n");
	SPIInit(SpiNum_HSPI, &cSpiAttr);

	SpiIntInfo cSpiIntInfo;
	cSpiIntInfo.src = (SpiIntSrc_TransDone
	        |SpiIntSrc_WrStaDone
			|SpiIntSrc_RdStaDone
			|SpiIntSrc_WrBufDone
			|SpiIntSrc_RdBufDone);

	cSpiIntInfo.isrFunc = spi_slave_isr_sta;
	SPISlaveRecvData(SpiNum_HSPI);
	uint32_t sndData[8] = { 0 };
	sndData[0] = 0x35343332;
	sndData[1] = 0x39383736;
    sndData[2] = 0x3d3c3b3a;
	sndData[3] = 0x11103f3e;
	sndData[4] = 0x15141312;
	sndData[5] = 0x19181716;
	sndData[6] = 0x1d1c1b1a;
	sndData[7] = 0x21201f1e;

    SPISlaveSendData(SpiNum_HSPI, sndData, 8);
    WRITE_PERI_REG(SPI_RD_STATUS(SpiNum_HSPI), 0x8A);
    WRITE_PERI_REG(SPI_WR_STATUS(SpiNum_HSPI), 0x83);
}
void ICACHE_FLASH_ATTR
user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_printf("\r\n =======================================================\r\n");
	os_printf("\t ESP8266 %s application \n\r", __func__);
	os_printf("\t\t SDK version:%s    \n\r", system_get_sdk_version());
	os_printf("\t\t Complie time:%s  \n\r", __TIME__);
	os_printf("\r\n =======================================================\r\n");
	spi_master_config();
}


