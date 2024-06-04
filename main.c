#include "MKL46Z4.h"
#include <stdbool.h>
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "pin_mux.h"
#include "board.h"
#include "lcd.h"
#include <math.h>

#define MAG3110_I2C_ADDR 0x0E

void init_i2c(void);
void send_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t value);
void read_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);

volatile bool isReadyToGetData;
void PORTC_PORTD_IRQHandler(void);

int16_t xmax = INT16_MIN;
int16_t xmin = INT16_MAX;
int16_t ymax = INT16_MIN;
int16_t ymin = INT16_MAX;
float xOffset = 0.0;
float yOffset = 0.0;
float xScale = 1.0;
float yScale = 1.0;

int main(void)
{
	init_i2c();
	send_i2c(MAG3110_I2C_ADDR, 0x10, 0x01);
	send_i2c(MAG3110_I2C_ADDR, 0x11, 0x80);

	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD->PCR[1] = PORT_PCR_MUX(1);
	PTD->PDDR &= ~(1 << 1);
	
	LCD_Init();

	PRINTF("Init MAG3110 Complete\n\r");

	uint8_t rxBuff[6];
	while (1)
	{
		if((PTD->PDIR & (1<<1)) == 0) continue;
		
		read_i2c(MAG3110_I2C_ADDR, 1, rxBuff, 6);
		int16_t x = ((int16_t)((rxBuff[0] * 256U) | rxBuff[1]));
		int16_t y = ((int16_t)((rxBuff[2] * 256U) | rxBuff[3]));
		int16_t z = ((int16_t)((rxBuff[4] * 256U) | rxBuff[5]));

		PRINTF("status_reg = 0x%x , x = %5d , y = %5d , z = %5d\r\n", PTD->PDIR & (1<<1), x, y, z);
		
		if (x > xmax) xmax = x;
    if (x < xmin) xmin = x;
		if (y > ymax) ymax = y;
    if (y < ymin) ymin = y;

    // Calculate offset and scale
    xOffset = (xmax + xmin) / 2.0;
    yOffset = (ymax + ymin) / 2.0;
    xScale = 2.0 / (xmax - xmin);
    yScale = 2.0 / (ymax - ymin);

    // Apply calibration
    float xCalibrated = (x - xOffset) * xScale;
    float yCalibrated = (y - yOffset) * yScale;
		
		LCD_DisplayDemical(atan2(yCalibrated,xCalibrated) * 57.296);
		
		delay(3000000);
	}
}

void init_i2c()
{
	i2c_master_config_t masterConfig;
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	BOARD_I2C_ConfigurePins();

	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 100000U;
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(I2C0_CLK_SRC));
}

void send_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
	i2c_master_transfer_t masterXfer;

	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = (uint32_t)reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &value;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferBlocking(I2C0, &masterXfer);
}

void read_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
	i2c_master_transfer_t masterXfer;

	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = (uint32_t)reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = rxBuff;
	masterXfer.dataSize = rxSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferBlocking(I2C0, &masterXfer);
}
