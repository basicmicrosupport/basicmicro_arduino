//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "Basicmicro.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
Basicmicro ctrl(&serial,10000);

#define address 0x80

void setup() {
  //Open roboclaw serial ports
  ctrl.begin(38400);
}

void loop() {
  ctrl.DutyM1(address,16384); //start Motor1 forward at half speed
  ctrl.DutyM2(address,-16384); //start Motor2 backward at half speed
  delay(2000);

  ctrl.DutyM1(address,-16384);
  ctrl.DutyM2(address,16384);
  delay(2000);
}
