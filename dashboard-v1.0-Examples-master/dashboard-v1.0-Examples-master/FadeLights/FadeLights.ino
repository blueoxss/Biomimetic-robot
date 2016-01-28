/*

This sample code will have Dash's eyes change color gradually and will cycle through 
the LED red, yellow, and green lights.

Author: Leanne Teeter (lrteeter@gmail.com)

Dash Examples: Fade Lights by Leanne Teeter is licensed under a Creative Commons 
Attribution-ShareAlike 4.0 International License.

*/


//libraries
#include <EEPROM.h>
#include <DashBot.h>

DashBot myrobot; //gyro, motors, controller, LEDs, eyes

  //Dash's eye color is determined with RGB values (Red, Green, and Blue), which range from 0 - 255
  //We're going to start with these inital RGB values:
  int red = 255; /
  int green = 0; 
  int blue = 0; 
  
  byte ColorPhase = 1; //Dash's eyes will change color in 3 phases - Red to Green, Green to Blue, and Blue back to Red
  
void setup() {
  myrobot.dashRadioSetup();

}

void loop(){
  
  if (ColorPhase == 1){ //Red to Green Eye Color Phase
    digitalWrite(LED_RED, HIGH); //Red LED will switch ON
    digitalWrite(LED_GREEN, LOW); //Green LED will switch OFF
    while (red > 0){
      red = red - 5; //Intensity of Red eye color gradually decreases
      green = green + 5; //Intensity of Green eye color gradually increases
      myrobot.setEyeColor(red, green, blue); //Sends these RGB color values to Dash to display
      delay(50);
    }
    ColorPhase = 2;
  }
  if (ColorPhase == 2){ //Green to Blue Eye Color Phase
    digitalWrite(LED_RED, LOW); //Red LED will switch OFF
    digitalWrite(LED_YELLOW, HIGH); //Yellow LED will switch ON
    while (green > 0){
      green = green - 5; //Intensity of Green eye color gradually decreases
      blue = blue + 5; //Intensity of Blue eye color gradually increases
      myrobot.setEyeColor(red, green, blue);
      delay(50);
    }
    ColorPhase = 3;
  }
  if (ColorPhase == 3){ //Blue to Red Eye Color Phase
    digitalWrite(LED_YELLOW, LOW); //Yellow LED will switch OFF
    digitalWrite(LED_GREEN, HIGH); //Green LED will switch ON
    while (blue > 0){
      blue = blue - 5; //Intensity of Blue eye color gradually decreases
      red = red + 5; //Intensity of Red eye color gradually increases
      myrobot.setEyeColor(red, green, blue);
      delay(50);
    }
    ColorPhase = 1;
  }

 
 
}
