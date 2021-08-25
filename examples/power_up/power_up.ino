#include <VolvoDIM.h>

VolvoDIM VolvoDIM(3); //SPI pin for your can bus shield
//3 is used for the Arduino MKR Wifi 1010 Can Shield
//9 is used for the Arduino Uno Can Bus Shield V2.0
//10 is used for the Arduino Uno Can Bus Shield V1.2
void setup() {
  VolvoDIM.init(); //This will power your dim with the default values
}

void loop() {
  VolvoDIM.simulate(); // This needs to be placed in the loop for the dim to power up
}
