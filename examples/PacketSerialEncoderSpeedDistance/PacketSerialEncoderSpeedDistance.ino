//See BareMinimum example for a list of library functions

//Set PID Coefficients using Motion Studio and remember to Write Settings to the controller

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
  ctrl.begin(38400);
}

void displayspeed(void)
{
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  
  int32_t enc1 = ctrl.ReadEncM1(address, &status1, &valid1);
  int32_t enc2 = ctrl.ReadEncM2(address, &status2, &valid2);
  int32_t speed1 = ctrl.ReadSpeedM1(address, &status3, &valid3);
  int32_t speed2 = ctrl.ReadSpeedM2(address, &status4, &valid4);

  Serial.print("Encoder1:");
  if(valid1){
    Serial.print(enc1,DEC);
    Serial.print(" ");
    Serial.print(status1,HEX);
    Serial.print(" ");
  }
  else{
	Serial.print("failed ");
  }
  Serial.print("Encoder2:");
  if(valid2){
    Serial.print(enc2,DEC);
    Serial.print(" ");
    Serial.print(status2,HEX);
    Serial.print(" ");
  }
  else{
	Serial.print("failed ");
  }
  Serial.print("Speed1:");
  if(valid3){
    Serial.print(speed1,DEC);
    Serial.print(" ");
  }
  else{
	Serial.print("failed ");
  }
  Serial.print("Speed2:");
  if(valid4){
    Serial.print(speed2,DEC);
    Serial.print(" ");
  }
  else{
	Serial.print("failed ");
  }
  Serial.println();
}

void loop() {
  uint8_t depth1,depth2;
  
  ctrl.SpeedDistanceM1(address,12000,48000,1);
  ctrl.SpeedDistanceM2(address,-12000,48000,1);
  do{
    displayspeed();
    ctrl.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80 && depth2!=0x80);	//Loop until distance command has completed
  
  delay(1000);

  ctrl.SpeedDistanceM1(address,-12000,48000,1);
  ctrl.SpeedDistanceM2(address,12000,48000,1);
  do{
    displayspeed();
    ctrl.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80 && depth2!=0x80);	//Loop until distance command has completed
  
  delay(1000);  //When no second command is given the motors will automatically slow down to 0 which takes 1 second

  ctrl.SpeedDistanceM1(address,12000,48000,1);
  ctrl.SpeedDistanceM2(address,-12000,48000,1);
  ctrl.SpeedDistanceM1(address,-12000,48000,0);
  ctrl.SpeedDistanceM2(address,12000,48000,0);
  ctrl.SpeedDistanceM1(address,0,48000,0);
  ctrl.SpeedDistanceM2(address,0,48000,0);
  do{
    displayspeed();
    ctrl.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80 && depth2!=0x80);	//Loop until distance command has completed
  
  delay(1000);
}
