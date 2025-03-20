//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "Basicmicro.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
Basicmicro ctrl(&serial,10000);

#define address 0x80

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(57600);
  ctrl.begin(115200);
}

void loop() {
  char version[32];

  if(ctrl.ReadVersion(address,version)){
    Serial.println(version);  
  }

  delay(100);
}

