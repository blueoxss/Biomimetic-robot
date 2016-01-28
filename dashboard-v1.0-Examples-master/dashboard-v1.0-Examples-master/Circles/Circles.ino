/*

This sample code will have Dash turn in circles. Wheee!

Author: Leanne Teeter (lrteeter@gmail.com)

Dash Examples: Fade Lights by Leanne Teeter is licensed under a Creative Commons
Attribution-ShareAlike 4.0 International License.

*/

//Libraries
#include <EEPROM.h>
#include <DashBot.h>

DashBot myrobot; //Gyro, motors, controller, LEDs, eyes

  float L_motor = 0; //Value for Left Motor
  float R_motor = 0; //Value for Right Motor
  
void setup() {
  myrobot.dashRadioSetup();

}

void loop(){
  
  //Spin to the Right
  while (L_motor < 50){ 
    L_motor = L_motor + 5; //Gently rev up the left motor from 0 - 50
    myrobot.motorDriveL(L_motor);
    myrobot.motorDriveR(R_motor);
    delay(10);
  }
 
  delay(1000); //Increase the delay time to make Dash spin longer or vice versa
 
  while (L_motor > 0){
    L_motor = L_motor - 5; //Gently slow the left motor down from 50 - 0
    myrobot.motorDriveL(L_motor);
    myrobot.motorDriveR(R_motor);
    delay(10);
  }
  
  delay(1000); //Increase or decrease the delay to change the length of pause in between spins
  
  //Spin to the Left
  while (R_motor < 50){
    R_motor = R_motor + 5; //Gently rev up the right motor from 0 - 50
    myrobot.motorDriveL(L_motor);
    myrobot.motorDriveR(R_motor);
    delay(10);
  }
 
  delay(1000); //Another delay indicates how long he will spin to the left
 
  while (R_motor > 0){
    R_motor = R_motor - 5; //Gently slow the left motor down from 50 - 0
    myrobot.motorDriveL(L_motor);
    myrobot.motorDriveR(R_motor);
    delay(10);
  }
  
  delay(1000); //Another pause in between spins
}
