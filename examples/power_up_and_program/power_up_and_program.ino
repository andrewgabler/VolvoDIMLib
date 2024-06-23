#include <VolvoDIM.h>

VolvoDIM VolvoDIM(9,1); //SPI pin for your can bus shield
//3 is used for the Arduino MKR Wifi 1010 Can Shield
//9 is used for the Arduino Uno Can Bus Shield V2.0
//10 is used for the Arduino Uno Can Bus Shield V1.2
void setup() {
  VolvoDIM.init(); //This will power your dim with the default values
  int timeValue = VolvoDIM.clockToDecimal(3,45,1); //This converts a 12 hour time into a number suitable for setting the clock 
  //The format for the function above is hour, minute, am = 0, pm = 1
  VolvoDIM.setTime(timeValue); //This sets the time on the dim
  VolvoDIM.setOutdoorTemp(63); //This accepts a temperature in fahrenheit -49 - 176 and sets it
  VolvoDIM.setCoolantTemp(67); //This sets the coolant gauge 0 - 100 
  VolvoDIM.setSpeed(105); //This sets the spedometer in mph 0-160
  VolvoDIM.setGasLevel(58); //This sets the gas guage 0 - 100
  VolvoDIM.setRpm(5234); //This sets the tachometer 0 - 8000
  VolvoDIM.setOverheadBrightness(255); //This sets the overhead light brightness 0 - 256
  VolvoDIM.setLcdBrightness(255); //This sets the lcd backlight brightness 0 - 256
  VolvoDIM.setTotalBrightness(255); //This sets both of the above brightness's 0 - 256
  VolvoDIM.setGearPosText('2'); //This sets the gear to park
  VolvoDIM.setLeftBlinker(0); //This enables the left blinker
  VolvoDIM.setRightBlinkerSolid(0); //This enables the right blinker
  VolvoDIM.enableTrailer(true); //This enables the blinker function that tells you your trailer is signaling in your car.
  VolvoDIM.reducedEnginePerformanceOrange(false);
  VolvoDIM.reducedBrakePerformanceOrange(false);
  VolvoDIM.setCustomText("123456789 123456123456789 123456");
}

void loop() {
  VolvoDIM.simulate(); // This needs to be placed in the loop for the dim to power up
}
