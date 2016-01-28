/*
basic Dash firmware. 

In this sample program, Dash will use his ambient light sensor to turn on his headlights (his eyes) if it gets dark.

Author: Leanne Teeter (lrteeter@gmail.com)

Dash Examples: Fade Lights by Leanne Teeter is licensed under a Creative Commons
Attribution-ShareAlike 4.0 International License.

*/

//libraries
#include <EEPROM.h>
#include <DashBot.h>

DashBot myrobot; //gyro, motors, controller, LEDs, eyes

  int currentAmbientLight; 
  
  //Dash's eye color is determined with RGB values (Red, Green, and Blue), which range from 0 - 255.
  int red;
  int green;
  int blue;

void setup() {
  myrobot.dashRadioSetup();

}

void loop(){

  currentAmbientLight = myrobot.readAmbientLight(); //Detects the ambient light and stores its value to the variable currentAmbientLight.
  
  //Use Serial.println to display the value of currentAmbientLight in the serial monitor.
  //This knowledge is useful for determining the average ambient light your Dash is detecting. My ambient light values are over 100 when the lights are on and below 100 when the lights are off, but your values might be different.
  Serial.println(currentAmbientLight); 
  
  if (currentAmbientLight < 100){ 
       red = 255; //Intensity of Red eye color goes to maximum
       green = 255; //Intensity of Green eye color goes to maximum
       blue = 255; //Intensity of Blue eye color goes to maximum
       myrobot.setEyeColor(red, green, blue); //Sends these RGB color values to Dash to display
  }
  else {
       red = 0; //Intensity of Red eye color goes to minimum
       green = 0; //Intensity of Green eye color goes to minimum
       blue = 0; //Intensity of Blue eye color goes to minimum
       myrobot.setEyeColor(red, green, blue); //Sends these RGB color values to Dash to display
  }
       
}

