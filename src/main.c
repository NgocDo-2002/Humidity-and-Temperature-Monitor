#include <stddef.h>
#include "stm32l1xx.h"
#include "nucleo152start.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define DHT22_PIN 6
#define STATE_HIGH 1
#define STATE_LOW 0
#define SLAVE_ADDRESS 0x06
#define TEMPERATURE_REQUEST 0x01
#define HUMIDITY_REQUEST 0x02

//Time functions
void delay_ms(int);
void delay_40us(int);

//UART functions
void USART2_Init(void);
void USART2_write(char);
void uartPrintLiteralStr(char*);
void uartPrint(char*, unsigned int);
void USART2_IRQHandler(void);
char USART2_read();

// DHT22
unsigned int readSensor(int request);
unsigned int getDHT22Pin();
void sendStartToDHT22();
bool receiveDHT22PreparationResponse();
void receiveDHT22Data(unsigned int*, unsigned int*);

//Crc check
bool crcCheck(char*);
unsigned short int CRC16(char *nData, unsigned short int wLength);

char mFlag = 0;

int main(void) {
	__disable_irq();
	USART2_Init();

	SetSysClock();
	SystemCoreClockUpdate();

	USART2->CR1 |= 0x0020;
	NVIC_EnableIRQ(USART2_IRQn);
	__enable_irq();

	RCC->AHBENR |= 1;
	GPIOA->MODER &= ~0x00000C00;
	GPIOA->MODER |= 0x400;
	char frame[8];

	while (1) {
		if (mFlag == 1) {
			mFlag = 0;

			frame[0] = SLAVE_ADDRESS;
			for (int i = 1; i < 8; i++) {
				frame[i] = USART2_read();
			}
			uartPrintLiteralStr("Modbus Request:\n");
			for (int i = 0; i < 8; i++) {
				char bit[10];
				if (frame[i] > 0x0F) {
					sprintf(bit, "%X ", frame[i]);
				} else {
					sprintf(bit, "0%X ", frame[i]);
				}
				uartPrintLiteralStr(bit);
			}
			uartPrintLiteralStr("\n\n");

			bool isCrcValid = crcCheck(frame);
			if (isCrcValid) {
				int requestType = -1;
				if (frame[3] == TEMPERATURE_REQUEST) {
					requestType = TEMPERATURE_REQUEST;
				} else if (frame[3] == HUMIDITY_REQUEST) {
					requestType = HUMIDITY_REQUEST;
				}
				unsigned int result = readSensor(requestType);

				char modbusResponse[7];
				modbusResponse[0] = SLAVE_ADDRESS;
				modbusResponse[1] = 0x04;
				modbusResponse[2] = 0x02;
				modbusResponse[3] = (result & 0xFF00) >> 8;
				modbusResponse[4] = result & 0x00FF;
				unsigned short crc = CRC16(modbusResponse, 5);
				modbusResponse[5] = crc & 0x00FF;
				modbusResponse[6] = (crc & 0xFF00) >> 8;
				uartPrintLiteralStr("Modbus Response:\n");
				for (int i = 0; i < 7; i++) {
					char bit[10];
					if (modbusResponse[i] > 0x0F) {
						sprintf(bit, "%X ", modbusResponse[i]);
					} else {
						sprintf(bit, "0%X ", modbusResponse[i]);
					}
					uartPrintLiteralStr(bit);
				}
				uartPrintLiteralStr("\n\n");

				char sensorResult[100];
				if (requestType == TEMPERATURE_REQUEST) {
					sprintf(sensorResult, "Temperature: %d\n", result);
				} else if (requestType == HUMIDITY_REQUEST) {
					sprintf(sensorResult, "Humidity: %d\n", result);
				}
				uartPrintLiteralStr(sensorResult);
			} else {
				uartPrintLiteralStr("Modbus CRC invalid\n");
			}
			uartPrintLiteralStr("==========\n\n");
			delay_ms(1000);
			USART2->CR1 |= 0x0020;
		} else if (mFlag == 2) {
			USART2->CR1 &= ~0x00000004;
			delay_ms(10);
			USART2->CR1 |= 0x00000004;
			USART2->CR1 |= 0x0020;
			mFlag = 0;
			uartPrintLiteralStr("Wrong slave address\n");
			uartPrintLiteralStr("==========\n\n");
		}
	}

	return 0;
}

bool crcCheck(char* frame) {
	// first 6 bits are data, the last 2 bits are CRC
	unsigned short crc = CRC16(frame, 6);
	unsigned short crcFirstBit = crc & 0x00FF;
	unsigned short crcSecondBit = (crc & 0xFF00) >> 8;
	return (char) crcFirstBit == frame[6] && (char) crcSecondBit == frame[7];
}

unsigned short int CRC16(char *nData, unsigned short int wLength) {
	static const unsigned short int wCRCTable[] = { 0X0000, 0XC0C1, 0XC181,
			0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601, 0X06C0, 0X0780,
			0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0, 0X0D80,
			0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81,
			0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801, 0X18C0, 0X1980,
			0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81,
			0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581,
			0X1540, 0XD701, 0X17C0, 0X1680, 0XD641, 0XD201, 0X12C0, 0X1380,
			0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040, 0XF001, 0X30C0, 0X3180,
			0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240, 0X3600, 0XF6C1, 0XF781,
			0X3740, 0XF501, 0X35C0, 0X3480, 0XF441, 0X3C00, 0XFCC1, 0XFD81,
			0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41, 0XFA01, 0X3AC0, 0X3B80,
			0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981,
			0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80,
			0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580,
			0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640, 0X2200, 0XE2C1, 0XE381,
			0X2340, 0XE101, 0X21C0, 0X2080, 0XE041, 0XA001, 0X60C0, 0X6180,
			0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240, 0X6600, 0XA6C1, 0XA781,
			0X6740, 0XA501, 0X65C0, 0X6480, 0XA441, 0X6C00, 0XACC1, 0XAD81,
			0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01, 0X6AC0, 0X6B80,
			0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1, 0XB981,
			0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80,
			0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40, 0XB401, 0X74C0, 0X7580,
			0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381,
			0X7340, 0XB101, 0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181,
			0X5140, 0X9301, 0X53C0, 0X5280, 0X9241, 0X9601, 0X56C0, 0X5780,
			0X9741, 0X5500, 0X95C1, 0X9481, 0X5440, 0X9C01, 0X5CC0, 0X5D80,
			0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40, 0X5A00, 0X9AC1, 0X9B81,
			0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841, 0X8801, 0X48C0, 0X4980,
			0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40, 0X4E00, 0X8EC1, 0X8F81,
			0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1, 0X8581,
			0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380,
			0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

	unsigned char nTemp;
	unsigned short int wCRCWord = 0xFFFF;

	while (wLength--) {
		nTemp = *nData++ ^ wCRCWord;
		wCRCWord >>= 8;
		wCRCWord ^= wCRCTable[nTemp];
	}
	return wCRCWord;

}

char USART2_read() {
	char data = 0;
	//wait while RX buffer is data is ready to be read
	while (!(USART2->SR & 0x0020)) {
	} 	//Bit 5 RXNE: Read data register not empty
	data = USART2->DR;			//p739
	return data;
}

unsigned int getDHT22Pin() {
	unsigned short mask = 1 << 6;
	return (GPIOA->IDR & mask) >> 6;
}

void delay_ms(int delay) {
	int i = 0;
	for (; delay > 0; delay--)
		for (i = 0; i < 2460; i++)
			; //measured with oscilloscope
}

void delay_40us(int delay) {
	int i = 0;
	for (; delay > 0; delay--)
		for (i = 0; i < 90; i++)
			; //measured with oscilloscope
}

void USART2_Init(void) {
	RCC->APB1ENR |= 0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR |= 0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0] = 0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->AFR[0] |= 0x00007000;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER |= 0x00000020; //MODER2=PA2(TX) to mode 10=alternate function mode. p184
	GPIOA->MODER |= 0x00000080; //MODER2=PA3(RX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000D05;	//9600 BAUD and crystal 32MHz. p710, D05
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART2_write(char data) {
	//wait while TX buffer is empty
	while (!(USART2->SR & 0x0040)) {
	} 	//6. p736-737
	USART2->DR = (data);		//p739
}

void uartPrintLiteralStr(char* s) {
	for (int i = 0; i < strlen(s); i++) {
		USART2_write(s[i]);
		if (s[i] == '\n')
			USART2_write('\r');

	}
}

void uartPrint(char* s, unsigned int length) {
	for (int i = 0; i < length; i++) {
		USART2_write(s[i]);
		if (s[i] == '\n')
			USART2_write('\r');

	}
}

void USART2_IRQHandler(void) {
	char received_slave_address = 0;

	if (USART2->SR & 0x0020) 		//if data available in DR register. p737
			{
		received_slave_address = USART2->DR;
	}
	if (received_slave_address == SLAVE_ADDRESS) //if we have right address
	{
		mFlag = 1;
		GPIOA->ODR |= 0x20;				//0010 0000 or bit 5. p186
	} else {
		mFlag = 2;
		GPIOA->ODR &= ~0x20;				//0010 0000 ~and bit 5. p186

	}
	USART2->CR1 &= ~0x0020;			//disable RX interrupt

}

unsigned int readSensor(int requestType) {
	int retryTimes = 9;
	int list[retryTimes];
	for (int i = 0; i < retryTimes; i++) {
		unsigned int h, t;
		sendStartToDHT22();
		receiveDHT22PreparationResponse();
		receiveDHT22Data(&h, &t);
		list[i] = (requestType == TEMPERATURE_REQUEST) ? t : h;
		delay_ms(450);
	}

	for (int i = 0; i < retryTimes - 1; i++) {
		for (int j = i + 1; j < retryTimes; j++) {
			if (list[j] > list[i]) {
				int temp = list[j];
				list[j] = list[i];
				list[i] = temp;
			}
		}
	}
	int median;
	if (retryTimes % 2 == 0) {
		median = (list[retryTimes / 2 - 1] + list[retryTimes / 2]) / 2;
	} else {
		median = list[retryTimes / 2];
	}
	return median;
}

void sendStartToDHT22() {
	GPIOA->MODER |= 1 << 12; //PA6 goes output

	GPIOA->ODR |= 1 << 6; //high 10ms
	delay_ms(10);

	GPIOA->ODR &= ~(1 << 6); //low 2ms
	delay_ms(2);

	GPIOA->ODR |= 1 << 6; //high 40us
	delay_40us(1);

	GPIOA->MODER &= ~(0b11 << 12);
}

bool receiveDHT22PreparationResponse() {
	while (getDHT22Pin() == STATE_HIGH) {
		//do something
	}
	delay_40us(2);
	while (getDHT22Pin() == STATE_HIGH) {
	}
	return true;
}

void receiveDHT22Data(unsigned int* h, unsigned int* t) {
	unsigned int data[40];
	for (int i = 0; i < 40; i++) {
		while (getDHT22Pin() == STATE_LOW) {
		}
		delay_40us(1);
		data[i] = getDHT22Pin();
		while (getDHT22Pin() == STATE_HIGH) {
		}
	}

	unsigned int humidityData = 0, temperatureData = 0;
	for (int i = 0; i < 16; i++) {
		humidityData = (humidityData << 1) | data[i];
	}

	for (int i = 16; i < 32; i++) {
		temperatureData = (temperatureData << 1) | data[i];
	}

	bool isCheckSumOk = true;
	int carry = 0;
	for (int i = 39; i >= 32; i--) {
		int sum = data[i - 32] + data[i - 24] + data[i - 16] + data[i - 8]
				+ carry;
		carry = sum / 2;
		sum = sum % 2;
		if (sum != data[i]) {
			isCheckSumOk = false;
			break;
		}
	}
	if (!isCheckSumOk) {
		humidityData = 0;
		temperatureData = 0;
	}

	*h = humidityData;
	*t = temperatureData;
}
