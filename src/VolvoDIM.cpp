/*
  VolvoDIM.cpp - Library for powering and controlling a P2 Volvo DIM.
  Created by Andrew J. Gabler, August 17, 2021.
*/
#include "VolvoDIM.h"
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#endif

mcp2515_can CAN(0);
int _relayPin = 0;
VolvoDIM::VolvoDIM(int SPI_CS_PIN, int relayPin)
{
	mcp2515_can temp_CAN(SPI_CS_PIN);
	CAN = temp_CAN;
	_relayPin = relayPin;
}
bool enableSerialErrMsg = false;
int genCnt = 0;
int cnt = 0;
constexpr int listLen = 14;
int carConCnt = 0;
int configCnt = 0;
int blinkerInterval = 0;
bool leftBlinker = false, rightBlinker = false, solidState = false;
unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long address;
constexpr int arrSpeed = 0, arrRpm = 1, arrCoolant = 2, arrTime = 3, arrBrakes = 4, arrBlinker = 5, arrAntiSkid = 6, arrAirbag = 7, arr4c = 8, arrConfig = 9, arrGear = 10, arrTrailer = 11, arrDmWindow = 12, arrDmMessage = 13;
constexpr unsigned long addrLi[listLen] = {0x217FFC, 0x2803008, 0x3C01428, 0x381526C, 0x3600008, 0xA10408, 0x2006428, 0x1A0600A, 0x2616CFC, 0x1017FFC, 0x3200408, 0x3E0004A, 0x02A0240E, 0x1800008};
/*
   addrLi[0] = Speed/KeepAlive
   addrLi[1] = RPM/Backlights
   addrLi[2] = Coolant/OutdoorTemp
   addrLi[3] = Time/GasTank
   addrLi[4] = Brakes Keep alive?
   addrLi[5] = Blinker
   addrLi[6] = Anti-Skid
   addrLi[7] = Aibag Light
   addrLi[8] = 4C Error
   addrLi[9] = Car Config
   addrLi[10] = Gear Pos
   addrLi[11] = Trailer Attached
   addrLi[12] = Dim Message Window
   addrLi[13] = Dim Message Content
*/

unsigned char defaultData[listLen][8] = {
	{0x01, 0x4B, 0x00, 0xD8, 0xF0, 0x58, 0x00, 0x00}, // 0, Speed/KeepAlive , 0x217FFC
	{0xFF, 0xE1, 0xFF, 0xF0, 0xFF, 0xCF, 0x00, 0x00}, // 1, RPM/Backlights , 0x2803008
	{0xC0, 0x80, 0x51, 0x89, 0x0E, 0x00, 0x00, 0x00}, // 2, Coolant/OutdoorTemp , 0x3C01428
	{0x00, 0x01, 0x05, 0xBC, 0x05, 0xA0, 0x40, 0x40}, // 3, Time/GasTank , 0x381526C
	{0x00, 0x00, 0xB0, 0x60, 0x30, 0x00, 0x00, 0x00}, // 4, Brake system Keep alive , 0x3600008
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 5, Blinker , 0xA10408
	{0x01, 0xE3, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00}, // 6, Anti-Skid , 0x2006428
	{0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x49, 0x00}, // 7, Aibag Light , 0x1A0600A
	{0x0B, 0x42, 0x00, 0x00, 0xFD, 0x1F, 0x00, 0xFF}, // 8, 4C keep alive / prevent error , 0x2616CFC
	{0x01, 0x0F, 0xF7, 0xFA, 0x00, 0x00, 0x00, 0xC0}, // 9, Car Config default , 0x1017FFC
	{0x11, 0xDE, 0x53, 0x00, 0x24, 0x00, 0x10, 0x00}, // 10 Gear Postion , 0x3200408
	{0x00, 0x00, 0x00, 0x00, 0x1D, 0xE0, 0x00, 0x00}, // 11 Trailer Attatched , 0x3E0004A
	{0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35}, // 12 Dim Message Window , 0x02A0240E
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  // 13 Dim Message Content , 0x1800008
};

// Sets the data for what the car is equiped with.
// Taken from a 2007 M66 SR with Climate package, rear parking sensors, no nav.
unsigned char carConfigData[16][8] = {
	{0x10, 0x10, 0x01, 0x03, 0x02, 0x01, 0x00, 0x01},
	{0x11, 0x18, 0x05, 0x05, 0x02, 0x03, 0x01, 0x05},
	{0x12, 0x03, 0x02, 0x02, 0x01, 0x02, 0x05, 0x01},
	{0x13, 0x01, 0x01, 0x01, 0x02, 0x02, 0x01, 0x04},
	{0x14, 0x04, 0x01, 0x02, 0x01, 0x02, 0x03, 0x11},
	{0x15, 0x15, 0x03, 0x02, 0x02, 0x01, 0x01, 0x01},
	{0x16, 0x02, 0x62, 0x02, 0x01, 0x02, 0x01, 0x02},
	{0x17, 0x01, 0x02, 0x02, 0x01, 0x03, 0x03, 0x01},
	{0x18, 0x01, 0x05, 0x03, 0x03, 0x12, 0x04, 0x04},
	{0x19, 0x11, 0x02, 0x10, 0x01, 0x05, 0x01, 0x02},
	{0x1A, 0x01, 0x02, 0x01, 0x04, 0x01, 0x01, 0x03},
	{0x1B, 0x01, 0x02, 0x10, 0x01, 0x01, 0x01, 0x01},
	{0x1C, 0x01, 0x04, 0x01, 0x03, 0x01, 0x06, 0x01},
	{0x1D, 0x04, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01},
	{0x1E, 0x01, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01},
	{0x1F, 0x01, 0x02, 0x05, 0x02, 0x02, 0x22, 0x01}};

void VolvoDIM::sendMsgWrapper(unsigned long wId, unsigned char *wBuf)
{
	CAN.sendMsgBuf(wId, 1, 8, wBuf);
}
void VolvoDIM::initSRS()
{
	unsigned char temp[8] = {0xC0, 0x0, 0x0, 0x0, 0x0, 0xBC, 0xDB, 0x80};
	unsigned long tempAddr = addrLi[arrAirbag];
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	temp[0] = (char)0x0;
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	temp[0] = (char)0xC0;
	temp[6] = (char)0xC9;
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	temp[0] = (char)0x80;
	sendMsgWrapper(tempAddr, temp);
}
void VolvoDIM::genSRS(long address, byte stmp[])
{
	int randomNum = random(0, 794);
	if (randomNum >= 0 && randomNum < 180)
	{
		stmp[0] = (char)0x40;
	}
	else if (randomNum >= 180 && randomNum < 370)
	{
		stmp[0] = (char)0xC0;
	}
	else if (randomNum >= 370 && randomNum < 575)
	{
		stmp[0] = (char)0x0;
	}
	else if (randomNum >= 575 && randomNum < 794)
	{
		stmp[0] = (char)0x80;
	}
	sendMsgWrapper(address, stmp);
	delay(15);
}
void VolvoDIM::init4C()
{
	unsigned char temp[8] = {0x09, 042, 0x0, 0x0, 0x0, 0x50, 0x00, 0x00};
	unsigned long tempAddr = addrLi[arr4c];
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	temp[0] = (char)0x0B;
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	temp[0] = (char)0x0B;
	sendMsgWrapper(tempAddr, temp);
}
void VolvoDIM::genCC(long address, byte stmp[])
{
	int randomNum = random(0, 7);
	if (randomNum > 3)
	{
		stmp[6] = (char)0xFF;
		stmp[7] = (char)0xF3;
	}
	sendMsgWrapper(address, stmp);
	delay(15);
}
void VolvoDIM::genTemp(long address, byte stmp[])
{
	int randomNum = random(0, 133);
	if (randomNum < 16)
	{
		stmp[0] = (char)0x00;
	}
	else if (randomNum >= 16 && randomNum < 41)
	{
		stmp[0] = (char)0x40;
	}
	else if (randomNum >= 41 && randomNum < 76)
	{
		stmp[0] = (char)0xC0;
	}
	else if (randomNum >= 76 && randomNum < 133)
	{
		stmp[0] = (char)0x80;
	}
	if (randomNum < 76)
	{
		stmp[1] = (char)0x80;
	}
	else
	{
		stmp[1] = (char)0x00;
	}
	randomNum = random(0, 105);
	if (randomNum < 7)
	{
		stmp[2] = (char)0x11;
	}
	else if (randomNum >= 7 && randomNum < 15)
	{
		stmp[2] = (char)0x71;
	}
	else if (randomNum >= 15 && randomNum < 25)
	{
		stmp[2] = (char)0x61;
	}
	else if (randomNum >= 25 && randomNum < 60)
	{
		stmp[2] = (char)0x51;
	}
	else if (randomNum >= 60 && randomNum < 105)
	{
		stmp[2] = (char)0x41;
	}
	sendMsgWrapper(address, stmp);
	delay(15);
}
void VolvoDIM::genBlinking(long address, byte stmp[], bool isBlinking, int interval, int blinkSpeed)
{
	int blinkRatio = 7;
	if (blinkSpeed == 0)
	{
		blinkRatio = 5;
	}
	else if (blinkSpeed == 2)
	{
		blinkRatio = 15;
	}
	else if (blinkSpeed == 3)
	{
		blinkRatio = 50;
	}
	if (isBlinking && (interval % blinkRatio == 0) && !solidState)
	{
		int prevState = 0;
		if (leftBlinker && rightBlinker)
		{
			if (defaultData[arrBlinker][7] == 0x0E)
			{
				defaultData[arrBlinker][7] = (char)0x08;
			}
			else
			{
				defaultData[arrBlinker][7] = (char)0x0E;
			}
			stmp[7] = defaultData[arrBlinker][7];
			sendMsgWrapper(address, stmp);
			delay(15);
		}
		else if (!solidState)
		{
			if (defaultData[arrBlinker][7] == 0x0A || defaultData[arrBlinker][7] == 0x0C)
			{
				prevState = 1;
			}
			// blinks left blinker
			if (leftBlinker)
			{
				if (prevState == 1)
				{
					defaultData[arrBlinker][7] = (char)0x08;
				}
				else
				{
					defaultData[arrBlinker][7] = (char)0x0A;
				}
				stmp[7] = defaultData[arrBlinker][7];
				sendMsgWrapper(address, stmp);
				delay(15);
			}
			// blinks right blinker
			if (rightBlinker)
			{
				if (prevState == 1)
				{
					defaultData[arrBlinker][7] = (char)0x08;
				}
				else
				{
					defaultData[arrBlinker][7] = (char)0x0C;
				}
				stmp[7] = defaultData[arrBlinker][7];
				sendMsgWrapper(address, stmp);
				delay(15);
			}
		}
	}
	else if (solidState)
	{
		if (leftBlinker)
		{
			defaultData[arrBlinker][7] = (char)0x0A;
			stmp[7] = defaultData[arrBlinker][7];
			sendMsgWrapper(address, stmp);
			delay(15);
		}
		else
		{
			defaultData[arrBlinker][7] = (char)0x00;
			stmp[7] = defaultData[arrBlinker][7];
			sendMsgWrapper(address, stmp);
		}
		if (rightBlinker)
		{
			defaultData[arrBlinker][7] = (char)0x0C;
			stmp[7] = defaultData[arrBlinker][7];
			sendMsgWrapper(address, stmp);
			delay(15);
		}
		else
		{
			defaultData[arrBlinker][7] = (char)0x00;
			stmp[7] = defaultData[arrBlinker][7];
			sendMsgWrapper(address, stmp);
		}
	}
}
void VolvoDIM::powerOn()
{
	pinMode(_relayPin, OUTPUT);
	digitalWrite(_relayPin, HIGH);
}
void VolvoDIM::powerOff()
{
	pinMode(_relayPin, OUTPUT);
	digitalWrite(_relayPin, LOW);
}
void VolvoDIM::init()
{
	while (CAN_OK != CAN.begin(CAN_125KBPS, MCP_16MHz))
	{
		delay(100);
	}
	if (_relayPin > 0)
	{
		powerOn();
	}
	initSRS();
	init4C();
}
void VolvoDIM::simulate()
{
	address = addrLi[cnt];

	// Copy default data
	memcpy(stmp, defaultData[cnt], sizeof(stmp));

	// Process based on address
	switch (address)
	{
	case addrLi[arrAirbag]:
		genSRS(address, stmp);
		break;
	case addrLi[arrCoolant]:
		genTemp(address, stmp);
		break;
	case addrLi[arrConfig]:
		if (carConCnt % 12 == 1)
		{
			memcpy(stmp, carConfigData[configCnt], sizeof(stmp));
			configCnt++;
		}
		else
		{
			genCC(address, stmp);
		}
		break;
	case addrLi[arrBlinker]:
		genBlinking(address, stmp, true, blinkerInterval, 1);
		break;
	case addrLi[arrDmWindow]:
		// doesn't need to be sent every cycle, send every 4
		if (cnt % 4 == 0)
		{
			sendMsgWrapper(address, stmp);
		}
	case addrLi[arrDmMessage]:
		// Do nothing this is all handeled in the function call
	default:
		sendMsgWrapper(address, stmp);
		// delay(15); // send data per 15ms
	}

	// Increment counters
	cnt++;
	blinkerInterval++;

	// Reset counters if necessary
	if (cnt == listLen)
	{
		cnt = 0;
		carConCnt = (configCnt >= 16) ? 0 : carConCnt + 1;
	}

	if (blinkerInterval >= 50)
	{
		blinkerInterval = 0;
	}
}

void VolvoDIM::setTime(int inputTime)
{
	if (inputTime >= 0 && inputTime <= 1440)
	{
		int b4;
		for (int i = 0; i < 6; i++)
		{
			if (inputTime >= (i * 256) && inputTime < ((i + 1) * 256))
			{
				b4 = i;
			}
		}
		int b5 = inputTime - (b4 * 256);
		defaultData[arrTime][4] = b4;
		defaultData[arrTime][5] = b5;
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Not a valid time");
		}
	}
}
int VolvoDIM::clockToDecimal(int hour, int minute, int AM)
{
	if ((hour >= 0 && hour <= 12) && (minute >= 0 && minute < 60) && (AM == 1 || AM == 0))
	{
		if (AM)
		{
			if (hour == 12)
			{
				return (minute);
			}
			else
			{
				return ((hour * 60) + minute);
			}
		}
		return ((hour * 60) + minute) + 720;
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Not a valid time");
		}
		return 0;
	}
}
double VolvoDIM::celsToFahr(double temp)
{
	return ((temp * (9 / 5)) + 32);
}
void VolvoDIM::setOutdoorTemp(int oTemp)
{
	if (!(oTemp < -49 || oTemp > 176))
	{
		if (oTemp > -50 && oTemp <= 32)
		{
			defaultData[arrCoolant][4] = (char)0x0D;
			defaultData[arrCoolant][5] = ceil((oTemp + 83) * 2.21);
		}
		else if (oTemp > 32 && oTemp <= 146)
		{
			defaultData[arrCoolant][4] = (char)0x0E;
			defaultData[arrCoolant][5] = ceil((oTemp - 33) * 2.25);
		}
		else if (oTemp > 146 && oTemp < 177)
		{
			defaultData[arrCoolant][4] = (char)0x0F;
			defaultData[arrCoolant][5] = ceil((oTemp - 147) * 2.20);
		}
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Temp out of range");
		}
	}
}
void VolvoDIM::setCoolantTemp(int range)
{
	if (range >= 0 && range <= 55)
	{
		defaultData[arrCoolant][3] = range + 88;
	}
	else if (range > 55 && range <= 100)
	{
		// upper limit scales very slowly.
		// This calculation makes it fit the 0-100 scale much better.
		defaultData[arrCoolant][3] = ceil((range - 55) * .333) + 173;
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Value out of range");
		}
	}
}
void VolvoDIM::setSpeed(int carSpeed)
{
	if (carSpeed >= 0 && carSpeed <= 160)
	{
		if (carSpeed >= 0 && carSpeed <= 40)
		{
			defaultData[arrSpeed][5] = (char)0x58;
			defaultData[arrSpeed][6] = round(carSpeed * 6.375);
		}
		else if (carSpeed > 40 && carSpeed <= 80)
		{
			defaultData[arrSpeed][5] = (char)0x59;
			defaultData[arrSpeed][6] = round(carSpeed * 6.375);
		}
		else if (carSpeed > 80 && carSpeed <= 120)
		{
			defaultData[arrSpeed][5] = (char)0x5A;
			defaultData[arrSpeed][6] = round(carSpeed * 6.375);
		}
		else if (carSpeed > 120 && carSpeed <= 160)
		{
			defaultData[arrSpeed][5] = (char)0x5B;
			defaultData[arrSpeed][6] = round(carSpeed * 6.375);
		}
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Speed out of range");
		}
	}
}
void VolvoDIM::setGasLevel(int level)
{
	if (level >= 0 && level <= 100)
	{
		defaultData[arrTime][6] = round(level * .64);
		defaultData[arrTime][7] = round(level * .64);
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Gas level out of range");
		}
	}
}
void VolvoDIM::setRpm(int rpm)
{
	if (rpm > 501 && rpm <= 8000)
	{
		defaultData[arrRpm][6] = floor((rpm * .031875) / 32) * 32 + floor((rpm * .031875) / 8);
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("RPM out of range");
		}
	}
}
void VolvoDIM::setOverheadBrightness(int value)
{
	if (value >= 0 && value <= 256)
	{
		if (value == 256)
		{
			value = 255;
		}
		defaultData[arrRpm][2] = value;
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Value out of range");
		}
	}
}
void VolvoDIM::setLcdBrightness(int value)
{
	if (value >= 0 && value <= 256)
	{
		if (value > 250)
		{
			value = 0x3F;
		}
		else
		{
			value = round(value / 32);
		}
		defaultData[arrRpm][4] = value;
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Value out of range");
		}
	}
}
void VolvoDIM::setTotalBrightness(int value)
{
	if (value >= 0 && value <= 256)
	{
		if (value == 256)
		{
			value = 255;
		}
		setLcdBrightness(value);
		setOverheadBrightness(value);
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Value out of range");
		}
	}
}
void VolvoDIM::setLeftBlinker(int state)
{
	bool setState = false;
	if (state == 1)
	{
		setState = true;
		defaultData[arrBlinker][7] = (char)0x0A;
	}
	else
	{
		defaultData[arrBlinker][7] = (char)0x0;
	}

	leftBlinker = setState;
}
void VolvoDIM::setRightBlinker(int state)
{
	bool setState = false;
	if (state == 1)
	{
		setState = true;
		defaultData[arrBlinker][7] = (char)0x0C;
	}
	else
	{
		defaultData[arrBlinker][7] = (char)0x0;
	}
	rightBlinker = setState;
}
void VolvoDIM::setLeftBlinkerSolid(int state)
{
	bool val = false;
	if (state == 1)
	{
		val = true;
	}

	solidState = val;
	leftBlinker = val;
}
void VolvoDIM::setRightBlinkerSolid(int state)
{
	bool val = false;
	if (state == 1)
	{
		val = true;
	}
	solidState = val;
	rightBlinker = val;
}

void VolvoDIM::setGearPosText(const char *gear)
{
	if (gear == 'low' || gear == 'Low' || gear == 'l' || gear == 'L')
	{
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x96;
		// Gear L (low?)
	}
	else if (gear == 'park' || gear == 'Park' || gear == 'p' || gear == 'P')
	{
		defaultData[arrGear][4] = (char)0x24;
		defaultData[arrGear][6] = (char)0x10;
		// Park
	}
	else if (gear == 'reverse' || gear == 'Reverse' || gear == 'r' || gear == 'R')
	{
		defaultData[arrGear][4] = (char)0xE4;
		defaultData[arrGear][6] = (char)0x20;
		// Reverse
	}
	else if (gear == 'Neutral' || gear == 'neutral' || gear == 'N' || gear == 'n')
	{
		defaultData[arrGear][4] = (char)0x24;
		defaultData[arrGear][6] = (char)0x30;
		// Neutral
	}
	else if (gear == '1' || gear == 'one' || gear == 'One')
	{
		defaultData[arrGear][4] = (char)0x34;
		defaultData[arrGear][6] = (char)0x40;
		// 1st Gear
	}
	else if (gear == '2' || gear == 'two' || gear == 'Two')
	{
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x70;
		// 2nd Gear
	}
	else if (gear == '3' || gear == 'three' || gear == 'Three')
	{
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x60;
		// 3rd Gear
	}
	else if (gear == '4' || gear == 'four' || gear == 'Four')
	{
		defaultData[arrGear][4] = (char)0x44;
		defaultData[arrGear][6] = (char)0x50;
		// 4th Gear
	}
	else if (gear == '5' || gear == 'five' || gear == 'Five')
	{
		defaultData[arrGear][4] = (char)0xBE;
		defaultData[arrGear][6] = (char)0x00;
		// 5th Gear
	}
	else if (gear == '6' || gear == 'six' || gear == 'Six')
	{
		defaultData[arrGear][4] = (char)0xD5;
		defaultData[arrGear][6] = (char)0x00;
		// 6th Gear
	}
	else
	{
		defaultData[arrGear][4] = (char)0x10;
		defaultData[arrGear][6] = (char)0x40;
		// Nothing Displayed
	}
}
void VolvoDIM::setGearPosInt(int gear)
{
	// Don't know what to do with the automatic gears for an integer assignment so we'll use negatives
	switch (gear)
	{
	case -4:
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x96;
		break; // Gear L (low?)
	case -3:
		defaultData[arrGear][4] = (char)0x24;
		defaultData[arrGear][6] = (char)0x10;
		break; // Park
	case -2:
		defaultData[arrGear][4] = (char)0xE4;
		defaultData[arrGear][6] = (char)0x20;
		break; // reverse
	case -1:
		defaultData[arrGear][4] = (char)0x10;
		defaultData[arrGear][6] = (char)0x40;
		break; // Nothing Displayed
	case 0:
		defaultData[arrGear][4] = (char)0x24;
		defaultData[arrGear][6] = (char)0x30;
		break; // Neutral
	case 1:
		defaultData[arrGear][4] = (char)0x34;
		defaultData[arrGear][6] = (char)0x40;
		break; // 1st Gear
	case 2:
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x70;
		break; // 2nd Gear
	case 3:
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x60;
		break; // 3rd Gear
	case 4:
		defaultData[arrGear][4] = (char)0x44;
		defaultData[arrGear][6] = (char)0x50;
		break; // 4th Gear
	case 5:
		defaultData[arrGear][4] = (char)0xBE;
		defaultData[arrGear][6] = (char)0x00;
		break; // 5th Gear
	case 6:
		defaultData[arrGear][4] = (char)0xD5;
		defaultData[arrGear][6] = (char)0x00;
		break; // 6th Gear
	}
}
void VolvoDIM::enableTrailer(int enabled)
{
	if (enabled == 1)
	{
		defaultData[arrTrailer][4] = (char)0x11;
		defaultData[arrTrailer][5] = (char)0xE4;
	}
	else
	{
		defaultData[arrTrailer][4] = (char)0x1D;
		defaultData[arrTrailer][5] = (char)0xE0;
	}
}

void VolvoDIM::setError(int error)
{
	if (error == 1)
	{
		// Engine system service required orange light
		defaultData[arrGear][3] = (char)0x10;
	}
	else if (error == 2)
	{
		// reduced brake performance orange light
		defaultData[arrGear][3] = (char)0x30;
	}
	else if (error == 3)
	{
		// Fuel filler cap open-loose
		defaultData[arrGear][3] = (char)0x40;
	}
	else if (error == 4)
	{
		// Engine System Servcie urgent red
		defaultData[arrGear][3] = (char)0x80;
	}
	else if (error == 5)
	{
		// Engine System Servcie Urgent red light
		defaultData[arrGear][3] = (char)0x90;
	}
	else if (error == 6)
	{
		// reduced brake performance red light
		defaultData[arrGear][3] = (char)0xAA;
	}
	else if (error == 7)
	{
		// reduced engine performance red light
		defaultData[arrGear][3] = (char)0xBB;
	}
	else if (error == 8)
	{
		// slow down or shift up orange
		defaultData[arrGear][3] = (char)0x01;
	}
	else if (error == 9)
	{
		// reduced engine performance orange light
		defaultData[arrGear][3] = (char)0x0E;
	}
	else if (error == 9)
	{
		// slow down or shift up red
		defaultData[arrGear][3] = (char)0xDD;
	}
}

void VolvoDIM::engineServiceRequiredOrange(int on)
{
	if (on == 1)
	{
		defaultData[arrGear][3] = (char)0x10;
	}
	else
	{
		defaultData[arrGear][3] = (char)0x00;
	}
}
void VolvoDIM::reducedBrakePerformanceOrange(int on)
{
	if (on == 1)
	{
		defaultData[arrGear][3] = (char)0x30;
	}
	else
	{
		defaultData[arrGear][3] = (char)0x00;
	}
}
void VolvoDIM::fuelFillerCapLoose(int on)
{
	if (on == 1)
	{
		defaultData[arrGear][3] = (char)0x40;
	}
	else
	{
		defaultData[arrGear][3] = (char)0x00;
	}
}
void VolvoDIM::engineSystemServiceUrgentRed(int on)
{
	if (on == 1)
	{
		defaultData[arrGear][3] = (char)0x90;
	}
	else
	{
		defaultData[arrGear][3] = (char)0x00;
	}
}
void VolvoDIM::brakePerformanceReducedRed(int on)
{
	if (on == 1)
	{
		defaultData[arrGear][3] = (char)0xAA;
	}
	else
	{
		defaultData[arrGear][3] = (char)0x00;
	}
}
void VolvoDIM::reducedEnginePerformanceRed(int on)
{
	if (on == 1)
	{
		defaultData[arrGear][3] = (char)0xBB;
	}
	else
	{
		defaultData[arrGear][3] = (char)0x00;
	}
}
void VolvoDIM::slowDownOrShiftUpOrange(int on)
{
	if (on == 1)
	{
		defaultData[arrGear][3] = (char)0x01;
	}
	else
	{
		defaultData[arrGear][3] = (char)0x00;
	}
}
void VolvoDIM::reducedEnginePerformanceOrange(int on)
{
	if (on == 1)
	{
		defaultData[arrGear][3] = (char)0x0E;
	}
	else
	{
		defaultData[arrGear][3] = (char)0x00;
	}
}

void VolvoDIM::setCustomText(const char* text) {
    const int messageLen = strlen(text);
    const int totalMessageLength = 32;
    const int chunkSize = 7;

    memcpy(stmp, defaultData[arrDmWindow], sizeof(stmp));
    defaultData[arrDmWindow][7] = 0x31;
    sendMsgWrapper(addrLi[arrDmWindow], stmp);
    delay(40);
    clearCustomText();

	//force the message into fixed length array
    char message[totalMessageLength] = {0};
    strncpy(message, text, messageLen);

    // Send the first part of the message
    stmp[0] = 0xA7;
    stmp[1] = 0x00;
    memcpy(&stmp[2], message, chunkSize - 1);
    sendMsgWrapper(addrLi[arrDmMessage], stmp);
    delay(40);

    // Send the remaining parts of the message
    for (int i = chunkSize - 1, messageSendIndex = 0x21; i < totalMessageLength; i += chunkSize, ++messageSendIndex) {
        stmp[0] = messageSendIndex;
        memcpy(&stmp[1], &message[i], chunkSize);
        sendMsgWrapper(addrLi[arrDmMessage], stmp);
        delay(40);
    }

    // End of sending
    stmp[0] = 0x65;
    memset(&stmp[1], ' ', chunkSize);
    sendMsgWrapper(addrLi[arrDmMessage], stmp);
}


void VolvoDIM::clearCustomText()
{
	unsigned char clearValues[] = {0xE1, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	sendMsgWrapper(addrLi[arrDmWindow], clearValues);
}

void VolvoDIM::gaugeReset()
{
	powerOn();
	delay(7000);
	powerOff();
}
void VolvoDIM::sweepGauges()
{
	setRpm(8000);
	setSpeed(160);
	delay(500);
	setRpm(0);
	setSpeed(0);
}
void VolvoDIM::enableSerialErrorMessages()
{
	enableSerialErrMsg = true;
}
void VolvoDIM::disableSerialErrorMessages()
{
	enableSerialErrMsg = false;
}
