/*
  VolvoDIM.h - Library for powering and controlling a P2 Volvo DIM.
  Created by Andrew J. Gabler, August 17, 2021.
*/
#ifndef VolvoDIM_h
#define VolvoDIM_h

#include "mcp2515_can.h"
#include <mcp_can.h>
#include <SPI.h>
#include <math.h>
#include <time.h>
class VolvoDIM
{
    public:
        VolvoDIM(int SPI_CS_PIN, int relayPin=0);
        void setTime(int inputTime);
        int clockToDecimal(int hour, int minute, int AM); 
        double celsToFahr(double temp);
        void setOutdoorTemp(int oTemp);
        void setCoolantTemp(int range);
        void setSpeed(int carSpeed);
        void setGasLevel(int level);
        void setRpm(int rpm);
        void setOverheadBrightness(int value);
        void setLcdBrightness(int value);
        void setTotalBrightness(int value);
        void setLeftBlinker(bool state);
        void setRightBlinker(bool state);
        void init();
        void simulate();
        void powerOff();
        void powerOn();
        void gaugeReset();
        void sweepGauges();
        void enableSerialErrorMessages();
        void disableSerialErrorMessages();

    private:
        void sendMsgWrapper(unsigned long wId, int wExt, int wLen ,unsigned char* wBuf);
        void initSRS();
        void genSRS(long address, byte stmp[]);
        void init4C();
        void genCC(long address, byte stmp[]);
        void genTemp(long address, byte stmp[]);
        void genBlinking(long address, byte stmp[], bool isBlinking, int interval, int blinkSpeed);
};
#endif