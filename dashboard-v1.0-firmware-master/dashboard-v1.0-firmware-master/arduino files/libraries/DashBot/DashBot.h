/* 
  DashBot.h - Library for using Dash.
  Created by Nick Kohut, Dwight Springthorpe II, and Kevin Peterson, June 10, 2014.
  Preoperty of Dash Robotics, Inc.
  Released into the public domain.
  */
  
  #ifndef DashBot_h
  #define DashBot_h

  //libraries
  #include "EEPROM.h"
  #include "Arduino.h"
  
  // motor pins
  #define MOTOR_RIGHT_BACKWARD 10 //mtrA1, 9 for EP2, 10 for DashBoard
  #define MOTOR_RIGHT_FORWARD 9 //mtrA2, 10 for EP2, 9 for DashBoard
  #define MOTOR_LEFT_BACKWARD 5 //mtrB1, 5 for EP2 and DashBoard
  #define MOTOR_LEFT_FORWARD 13 //mtrB2, 13 for EP2 and DashBoard

  //gyro pin
  #define GYRO_OUT A1

  // LED pins
  #define LED_RED 15
  #define LED_YELLOW 16
  #define LED_GREEN 14

  #define EYE_RED 11 //6 for EP2, 11 for EP3
  #define EYE_GREEN 3 //11 for EP2, 3 for EP3
  #define EYE_BLUE 6 // 3 for EP2, 6 for EP3

  // Sensor pins
  #define IR_ENABLE_LEFT 4
  #define IR_ENABLE_RIGHT 12
  #define PROXIMITY_LEFT  A4
  #define PROXIMITY_RIGHT  A3
  #define AMBIENT_LIGHT  A2

  
  // radio and radio commands
  #define RADIO_BAUD_RATE 57600
  #define USB_BAUD_RATE 9600
  
  #define RADIO_PACKET_LENGTH 14
  #define MAX_NAME_LENGTH 10
  
  #define ALL_STOP '0'
  #define SET_NAME '1'
  #define JOYSTICK_DRIVE '2'
  #define GYRO_DRIVE '3'
  #define SET_EYE_COLOR '4'
  #define SEND_INFO_PACKET '5'
  #define SET_INFO_PACKET '6'
  #define EXECUTE_AUTO_MODE '7'

  #define NAME_PACKET '1'
  #define INFO_PACKET '2'

  // drive modes
  #define STOP_MODE 0
  #define JOYSTICK_MODE 2
  #define GYRO_MODE 3

  // autonomous modes
  #define DASH_TEST '0'
  #define DASH_CIRCLE '1'
  #define DASH_FIG_8 '2'
  #define DASH_DANCE '3'
  #define DASH_BUMP '4'


  class DashBot
  {
    public:
      DashBot(void);
      
      // Gyro functions
      void gyroSetup();
      float readGyroDeg();
      
      // Motor functions
      void motorDriveR(float motor_pwm);
      void motorDriveL(float motor_pwn);
      
      void allStop(void);
      void directDrive(byte right_bckwd, byte right_fwd, byte left_bckwd, byte left_fwd);

      // Controller functions
      float dashRun(float ref_pwm, float ref_yaw);

      // LED functions
      void startupBlink();
      void debugBlinkOn();
      void debugBlinkOff();
      void setEyeColor(int eyeRedVal, int eyeGreenVal, int eyeBlueVal);

      // Visible Light functions
      int readAmbientLight(void);

      // IR functions
      void enableIRsensors(boolean left, boolean right);
      int readRightIRsensor(void);
      int readLeftIRsensor(void);
      void setupIRsensors(void);
      boolean detectCollisionLeft(int thresh);
      boolean detectCollisionRight(int thresh);

      // Radio functions
      void clearRadioPacket(void);
      boolean readRadioPacket(void);
      void executeRadioCommand(void);
      void executeAutoMode(void);
      void sendInfoPacket(void);
      void sendNamePacket(void);
      void stabilizedDrive(void);
      void setName(void);
      void setInfoPacketMode(void);

      // System functions
      void setupSystemFunctions(void);
      void readNameFromEEPROM(void);
      void writeNameToEEPROM(void);
      void dashPacketHandler(void);
      void dashRadioSetup(void);

      // Autonomous functions
      void dashCircle(void);
      void dashFig8(void);
      void dashDance(void);
      void dashBump(void);
      void dashTest(void);

      //the following should be written to program memory:
      char robotName[MAX_NAME_LENGTH]; //max name length is 9 chars (length is 10 b/c compiler needs to add a null char to the end 
      byte robotColor;
      byte codeVersion;
      byte robotType;
      

      // Gyro Variables
      int gyro_init; //gyro reading at rest (reset in setup)

      // IR variables
      int baseline_IR_left;
      int baseline_IR_right;

    private:
      // Gyro variables
      // <none>
      
      // Controller Variables
      float err_integral;
      int auto_flag;

      byte infoPacketTransmissionMode;
      byte mode;
      byte motor_right_backward_value; //mtrA1
      byte motor_right_forward_value; // mtrA2
      byte motor_left_backward_value; // mtr B1
      byte motor_left_forward_value; // mtr B2

      byte receivedRadioPacket[RADIO_PACKET_LENGTH];
      float power;
      float heading;
      unsigned long infoPacketTime;
      unsigned long lastPacketTime;

      void motorDrive(float motor_pwm, char side);
  };

  #endif