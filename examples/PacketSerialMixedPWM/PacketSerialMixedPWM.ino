//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "Basicmicro.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
Basicmicro ctrl(&serial,10000);

#define address 0x80

void setup() {
  //Communciate with roboclaw at 38400bps
  ctrl.begin(38400);
  ctrl.DutyM1M2(address, 0, 0);
}

void loop() {
  ctrl.DutyM1M2(address, 16384, 16384);
  delay(2000);
  ctrl.DutyM1M2(address, -16384, -16384);
  delay(2000);
  ctrl.DutyM1M2(address, 16384, -16384);
  delay(2000);
  ctrl.DutyM1M2(address, -16384, 16384);
  delay(2000);
}
