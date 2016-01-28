/* 
  DashBot.cpp - Library for using Dash.
  Created by Nick Kohut, Dwight Springthorpe II, Paul Birkmeyer, and Kevin Peterson, August 14, 2014.
  Property of Dash Robotics, Inc.
  Released into the public domain.
  */

#include "Arduino.h"
#include "DashBot.h"

DashBot::DashBot(void)
{  
  // Gyro variables
  gyro_init = 512;//gyro reading at rest (reset in setup)

  // Controller variables
  err_integral = 0;

  // IR variales
  baseline_IR_left = 800;
  baseline_IR_right = 800;

  // Initialize LED pins
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // Initialize IR pins
  pinMode(IR_ENABLE_LEFT, OUTPUT);
  pinMode(IR_ENABLE_RIGHT, OUTPUT);
  digitalWrite(IR_ENABLE_LEFT, HIGH);
  digitalWrite(IR_ENABLE_RIGHT, HIGH);
}

////////////////////////////////////////////////////////////////////////////////
// Motor, Gyro, Control functions
////////////////////////////////////////////////////////////////////////////////

// reads the gyroscope, returns degrees/sec
float DashBot::readGyroDeg(void)
{
  float gyro_current;
  float gyro_init_fl;
  float gyro_deg;

  static const float GYRO_MAX_RANGE = 2000.0; // the LY3200ALH gyro reads plus or minus 2000 deg/sec

  gyro_current = float(analogRead(GYRO_OUT)); 
  gyro_init_fl = float (gyro_init); 
  gyro_deg = (gyro_current - gyro_init_fl)*(GYRO_MAX_RANGE/gyro_init_fl);
  return gyro_deg;
}

// "zeroes" the gyro by taking an average measurement over 50 ms at startup
void DashBot::gyroSetup(void)
{
  float val = 0;
  int i;
  
  //read the gyro 5 times in 50 ms
  for (i = 0; i < 5; i++)
  {
    val += analogRead(GYRO_OUT);
    delay(10);
  }

  gyro_init = val/5; 
}

// drive the right motor, input should be between -100 and 100
void DashBot::motorDriveR(float motor_pwm)
{
  motorDrive(motor_pwm, 'R'); 
}

// drive the left motor, input should be between -100 and 100
void DashBot::motorDriveL(float motor_pwm)
{
  motorDrive(motor_pwm, 'L');
}

// takes in a number between -100 and 100, converts to bytes, drives motors. only called by motorDriveL and motorDriveR
void DashBot::motorDrive (float motor_pwm, char side) {
  float pwm_scaled;
  byte PWM_byte;

  if (motor_pwm > 100.0) {
    motor_pwm = 100.0;
  }  
  if (motor_pwm < -100.0) {
    motor_pwm = -100.0;
  }
  
  pwm_scaled = motor_pwm*255.0/100.0; // scale motor_pwm to a byte

  if (side == 'R') {

    if (pwm_scaled > 0.0) {
      
      PWM_byte = byte(pwm_scaled);
      analogWrite(MOTOR_RIGHT_FORWARD, PWM_byte);
      analogWrite(MOTOR_RIGHT_BACKWARD, 0);
      motor_right_backward_value = 0;
      motor_right_forward_value = PWM_byte;
    }

   else { 
     
      PWM_byte = byte(abs(pwm_scaled));
      analogWrite(MOTOR_RIGHT_BACKWARD, PWM_byte);
      analogWrite(MOTOR_RIGHT_FORWARD, 0);
      motor_right_backward_value = PWM_byte;
      motor_right_forward_value = 0;
  }
}

if (side == 'L') {
    if (pwm_scaled > 0.0) {
    
    PWM_byte = byte(pwm_scaled);
    analogWrite(MOTOR_LEFT_FORWARD, PWM_byte);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    motor_left_backward_value = 0;
    motor_left_forward_value = PWM_byte;
  }
 else { 
         
    PWM_byte = byte(abs(pwm_scaled));
    analogWrite(MOTOR_LEFT_BACKWARD, PWM_byte);
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    motor_left_backward_value = PWM_byte;
    motor_left_forward_value = 0;
}

}
}   


/*
float DashBot::dashRun (float ref_pwm, float ref_yaw)
{
  static const float p_gain = 0.3;
  static const float i_gain = 0.02;
  static const int antiwindup = 7500;

  float L_motor;
  float R_motor;
  float err;
  float mix;

  //PI controller with custom anti-windup
  err = ref_yaw - readGyroDeg();
  err_integral = err_integral + err;
  if (abs(err_integral) > antiwindup)
  {
    err_integral = 0;
  }
  mix = p_gain*err+i_gain*err_integral;
  L_motor = ref_pwm - mix;
  R_motor = ref_pwm + mix;
  motorDriveL(L_motor);
  motorDriveR(R_motor);
  return ref_yaw;
}
*/

/*
// gyro feedback steering for Dash. implements PI control law with "anti-windup" that resets integral error to zero
float DashBot::dashRun (float ref_pwm, float ref_yaw) {
    float x;
    float y;
  static const float p_gain = .1; 
  static const float i_gain = .0005; 
  static const float antiwindup = 250/.0005; // 250/i_gain
    
  float L_motor;
  float R_motor;
  float err;
  float mix;

  ref_yaw = 1.5*ref_yaw*abs(ref_pwm)/100; // Turn slowly if moving slowly
  
  // Can do this next part more intelligently...
  L_motor = ref_pwm;
  R_motor = ref_pwm;
  

  
  //PI controller with anti-windup
  err = ref_yaw - readGyroDeg();
  err_integral = err_integral + err;
  if (err_integral > antiwindup) {
    err_integral = antiwindup;
  }
  else if (err_integral < -antiwindup){
      err_integral = -antiwindup;
  }
  mix = p_gain*err+i_gain*err_integral;
  mix = mix*abs(ref_pwm)/100;   // Cap mixing based on speed
  L_motor = L_motor - mix;
  R_motor = R_motor + mix;
  
  // Send commands
  motorDriveL(L_motor);
  motorDriveR(R_motor);
  
  return ref_yaw;
}  

*/

float DashBot::dashRun (float ref_pwm, float ref_yaw) {
  // yaw ranges from -400 to 400 (degrees per sec) from iPad
  // pwm ranges from -100 to 100 (duty cycle) from iPad
  // gyro feedback sttering for Dash. implements PI control law with anti-windup

  static const float p_gain = .1; 
  static const float i_gain = .0005; 
  static const float antiwindup = 20/.0005; // 250/i_gain
  static const int max_yaw = 400;   // maximum from iPad
    
  float L_motor;
  float R_motor;
  float err;
  float mix;
  
  // Baseline PWM values
  L_motor = ref_pwm;
  R_motor = ref_pwm;
  
  // Scale PWM based on dirction (forward and backward) and target yaw rate
  if (ref_pwm > 0) {
      if (ref_yaw < 0) {   // If right turn
          L_motor = ref_pwm;   // Left motor positive
          R_motor = ref_pwm + (2 * ref_pwm) * (ref_yaw/max_yaw);    // Right motor goes from 100 to -100
      }
      else if (ref_yaw > 0) {   // If left turn
          L_motor = ref_pwm - (2 * ref_pwm) * (ref_yaw/max_yaw);   // Left motor goes from 100 to -100 (2 = (100 - (-100)) / max_pwm)
          R_motor = ref_pwm;    // Right motor positive
      }
  }
  else {
      if (ref_yaw > 0) {   // If right turn
            L_motor = ref_pwm;   // Left motor positive
            R_motor = ref_pwm + (2 * abs(ref_pwm)) * (ref_yaw/max_yaw);    // Right motor goes from 100 to -100 (2 = (100 - (-100)) / max_pwm)
        }
        else if (ref_yaw < 0) {   // If left turn
            L_motor = ref_pwm - (2 * abs(ref_pwm)) * (ref_yaw/max_yaw);   // Left motor goes from 100 to -100
            R_motor = ref_pwm;    // Right motor positive
        }
  }
      

  // Translate yaw rate to a turning radius and amplify by 50%
  ref_yaw = 1.5*ref_yaw*abs(ref_pwm)/100; // Turn slowly if moving slowly
  
  
  //PI controller with anti-windup
  err = ref_yaw - readGyroDeg();
  err_integral = err_integral + err;
  if (err_integral > antiwindup) {
    err_integral = antiwindup;
  }
  else if (err_integral < -antiwindup){
      err_integral = -antiwindup;
  }
  mix = p_gain*err+i_gain*err_integral;
  mix = mix*abs(ref_pwm)/100;   // Cap mixing based on speed
  L_motor = L_motor - mix;
  R_motor = R_motor + mix;
  
  // Send commands
  motorDriveL(L_motor);
  motorDriveR(R_motor);
  
  return ref_yaw;
} 



// stop Dash's motors
void DashBot::allStop(void)
{
  motorDriveL(0);
  motorDriveR(0);
  err_integral = 0;
}

// drive Dash's motors directly. used in app for joystick mode
void DashBot::directDrive(byte right_bckwd, byte right_fwd, byte left_bckwd, byte left_fwd)
{
  analogWrite(MOTOR_RIGHT_BACKWARD, right_bckwd); // right backward 
  analogWrite(MOTOR_RIGHT_FORWARD, right_fwd); // right forward
  analogWrite(MOTOR_LEFT_BACKWARD, left_bckwd); // left backward
  analogWrite(MOTOR_LEFT_FORWARD, left_fwd); // left forward

  motor_right_backward_value = right_bckwd;
  motor_right_forward_value = right_fwd;
  motor_left_backward_value = left_bckwd;
  motor_left_forward_value = left_fwd;
}

////////////////////////////////////////////////////////////////////////////////
// Light functions
////////////////////////////////////////////////////////////////////////////////

// light up green LED to indicate robot is ready
void DashBot::startupBlink(void)
{
  digitalWrite(LED_GREEN, HIGH);
}

// red LED high, used for debugging purposes
void DashBot::debugBlinkOn(void)
{
  digitalWrite(LED_RED, HIGH);
}

// red LED low, used for debugging purposes
void DashBot::debugBlinkOff(void)
{
  digitalWrite(LED_RED, LOW);
}

// set the color of Dash's eyes in 3 bytes (Red, Green, Blue)
void DashBot::setEyeColor(int eyeRedVal, int eyeGreenVal, int eyeBlueVal)
{   
  analogWrite(EYE_RED, eyeRedVal);
  analogWrite(EYE_GREEN, eyeGreenVal);
  analogWrite(EYE_BLUE, eyeBlueVal);
}

////////////////////////////////////////////////////////////////////////////////
// Sensor functions
////////////////////////////////////////////////////////////////////////////////

int DashBot::readAmbientLight(void)
{
  return analogRead(AMBIENT_LIGHT);
}

int DashBot::readRightIRsensor(void)
{
  return analogRead(PROXIMITY_RIGHT);
}

int DashBot::readLeftIRsensor(void)
{
  return analogRead(PROXIMITY_LEFT);
}

// used to set a sensor baseline at startup
void DashBot::setupIRsensors(void)
{
  baseline_IR_left = readLeftIRsensor();
  baseline_IR_right = readRightIRsensor();
}

// detects a collision on the left side of the robot by comparing current IR reading to the baseline
// takes a threshold variable as an input to set sensitivity
boolean DashBot::detectCollisionLeft(int thresh)
{
  if (readLeftIRsensor() < baseline_IR_left-thresh)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// detects a collision on the right side of the robot by comparing current IR reading to the baseline
// takes a threshold variable as an input to set sensitivity
boolean DashBot::detectCollisionRight(int thresh)
{
  if(readRightIRsensor() < baseline_IR_right-thresh){
    return true;
  }
  else
  {
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Radio functions
////////////////////////////////////////////////////////////////////////////////

void DashBot::clearRadioPacket(void) 
{
  for(byte i = 0; i< RADIO_PACKET_LENGTH; i++)
    receivedRadioPacket[i] = 0;
}

boolean DashBot::readRadioPacket(void)
{
  //if there's a full packet, read it and update the global packet variable. 
  //if there's not enoufgh data yet, do nothing and return a false
  if(Serial1.available() >= RADIO_PACKET_LENGTH)
  {
    //digitalWrite(ledGreen, HIGH);
    for (byte i = 0; i < RADIO_PACKET_LENGTH; i++)
    {
      receivedRadioPacket[i] = Serial1.read();
    }
    
    lastPacketTime = millis();
    
    return true;
  }
  else
  {
    return false;
  }
}

void DashBot::executeRadioCommand(void)
{
  switch (receivedRadioPacket[0]) {
    case ALL_STOP:
      mode = STOP_MODE;
      allStop();
      clearRadioPacket();
      auto_flag = 0;
      break;
    case SET_NAME:
      setName();
      break;
    case JOYSTICK_DRIVE:
      // joystick drive
      mode = JOYSTICK_MODE;
      directDrive(receivedRadioPacket[1], receivedRadioPacket[2], receivedRadioPacket[3], receivedRadioPacket[4]);
      clearRadioPacket();
      break;
    case GYRO_DRIVE:
      // gyro-assisted driving
      stabilizedDrive();
      break;
    case SET_EYE_COLOR:
      setEyeColor(receivedRadioPacket[1], receivedRadioPacket[2], receivedRadioPacket[3]);
      clearRadioPacket();
      break;
    case SEND_INFO_PACKET:
      // send name and color to iOS device
      sendInfoPacket();
      break;
    case SET_INFO_PACKET:
      // set whether a name/color or sensor packet is being sent
      setInfoPacketMode();
      break;
    case EXECUTE_AUTO_MODE:
      executeAutoMode();
      clearRadioPacket();
      break;
    default:
      debugBlinkOn();
      clearRadioPacket();
  }
}

// executes one of Dash's autonomous behaviors
void DashBot::executeAutoMode(void){
  auto_flag = 1; // indicates an autonomous mode is on, if an all_stop is called, resets to zero to end auto behavior
  switch (receivedRadioPacket[1]) {
    case DASH_TEST:
      dashTest();
      Serial1.write('3');
    case DASH_CIRCLE:
      dashCircle();
      Serial1.write('3');
      break;
    case DASH_FIG_8:
      dashFig8();
      Serial1.write('3');
      break;
    case DASH_DANCE:
      dashDance();
      Serial1.write('3');
      break;
    case DASH_BUMP:
      dashBump();
      Serial1.write('3');
      break;
    default:
      setEyeColor(0,0,0);
  }
}

//sends sensor data to mobile device
void DashBot::sendInfoPacket(void){
  infoPacketTime = millis();
  
  Serial1.write(INFO_PACKET); //2 means this is an info packet
  Serial1.write(mode);
  
  // send
  int currentYaw = readGyroDeg();
  if (currentYaw < 0)
    currentYaw = -currentYaw; 
  Serial1.write(highByte(currentYaw));
  Serial1.write(lowByte(currentYaw));
  
  int currentAmbientLight = readAmbientLight();
  Serial1.write(highByte(currentAmbientLight));
  Serial1.write(lowByte(currentAmbientLight));
  
  int currentProxLeft = readLeftIRsensor();
  Serial1.write(highByte(currentProxLeft));
  Serial1.write(lowByte(currentProxLeft));
  
  int currentProxRight = readRightIRsensor();
  Serial1.write(highByte(currentProxRight));
  Serial1.write(lowByte(currentProxRight));
  
  Serial1.write(motor_right_backward_value);
  Serial1.write(motor_right_forward_value);
  Serial1.write(motor_left_backward_value);
  Serial1.write(motor_left_forward_value);
}

// send robot name and color to iOS device
void DashBot::sendNamePacket(void){
  Serial1.write(NAME_PACKET); //1 means this is a name packet
  Serial1.write(robotType);
  Serial1.write(robotColor);
  Serial1.write(codeVersion);
  for (byte i = 0; i < MAX_NAME_LENGTH; i++){
    Serial1.write(robotName[i]);
  }
  
}

// drives Dash via joystick with gyro-stabilized control
void DashBot::stabilizedDrive(void)
{
  //enable stabilized drive mode:
  mode = GYRO_MODE;
  
  //note: I could send floats instead. need to find a good way to take 4-bytes and turn it into a float.
  power = float(int(word(receivedRadioPacket[1],receivedRadioPacket[2]))); //convert the 2-byte value into signed value and cast as float. 
  heading = float(int(word(receivedRadioPacket[3],receivedRadioPacket[4])));
  
  clearRadioPacket();
}

// sets Dash's name on the robot
void DashBot::setName(void)
{
  //name packets should update variables store in program memory
  robotType = receivedRadioPacket[1];
  robotColor = receivedRadioPacket[2];
  codeVersion = receivedRadioPacket[3]; //i don't think I really want the ability to update this value but I'll leave this in for now
  
  robotName[0] = receivedRadioPacket[4];
  robotName[1] = receivedRadioPacket[5];
  robotName[2] = receivedRadioPacket[6];
  robotName[3] = receivedRadioPacket[7];
  robotName[4] = receivedRadioPacket[8];
  robotName[5] = receivedRadioPacket[9];
  robotName[6] = receivedRadioPacket[10];
  robotName[7] = receivedRadioPacket[11];
  robotName[8] = receivedRadioPacket[12];
  robotName[9] = 0x00; //receivedRadioPacket[13];  //this should always be a null character
  
  clearRadioPacket();
  
  writeNameToEEPROM();
}

//toggles a mode variable which will enable automatic transmission of info packets
void DashBot::setInfoPacketMode(void){
 
  switch (receivedRadioPacket[1]) {
    case 0:
      infoPacketTransmissionMode = 0; // send info packet (sensor readings)
      break;
    case 1:
      infoPacketTransmissionMode = 1; // send name packet
      break;
  }
}
  
////////////////////////////////////////////////////////////////////////////////
// System functions
////////////////////////////////////////////////////////////////////////////////

// creates serial channels for radio and USB
void DashBot::setupSystemFunctions(void){
  Serial1.begin(RADIO_BAUD_RATE); //radio channel
  Serial.begin(USB_BAUD_RATE); //USB channel  
  //read name from eeprom and update global name variable:
  readNameFromEEPROM();
}

//reads EEPROM to get name/info and loads results into global variables
void DashBot::readNameFromEEPROM(void){

  robotType = EEPROM.read(0);
  robotColor = EEPROM.read(1);
  for (byte i = 0; i < MAX_NAME_LENGTH; i++){
    robotName[i] = EEPROM.read(2 + i);
  }
}

//writes Dash's name to memory
void DashBot::writeNameToEEPROM(void){
  //puts robot name/info into eeprom
  EEPROM.write(0, robotType);
  EEPROM.write(1, robotColor);
  for (byte i = 0; i < MAX_NAME_LENGTH; i++){
    EEPROM.write(2+i, robotName[i]);
  }
}

// If there is a radio packet, read and execute it. send info packets if appropriate
void DashBot::dashPacketHandler(void){
  if(readRadioPacket()) { 
    executeRadioCommand();
  }
  debugBlinkOff();
  
  //if in gyro-assisted mode, update controller and allow it to change the motor settings
  if(mode == GYRO_MODE) {
    dashRun(power, -heading);
  }
  
  //if in automatic info packet transmission mode is enabled, send an info packet
  if(millis() - infoPacketTime > 100)
  {
    infoPacketTime = millis();
    if (infoPacketTransmissionMode == 1)
      sendInfoPacket();
    else 
      sendNamePacket();
  }
  
  //if there's not been a packet for the last 1/2 second, call an all-stop
  if(millis() > lastPacketTime + 500)
  {
    allStop();
  }
}

// setup Dash for iOS app
void DashBot::dashRadioSetup(void){
  startupBlink(); // green LED high
  setupSystemFunctions(); //sets up serial radio and USB
  setupIRsensors(); // baseline for IR sensors
  gyroSetup(); // baseline for gyro
}


////////////////////////////////////////////////////////////////////////////////
// Autonomous functions
////////////////////////////////////////////////////////////////////////////////

// make Dash run in a circle!
void DashBot::dashCircle(void){

  
  unsigned long init_time = millis();
  unsigned long current_time = millis();
  
  // auto_flag must be 1, if not, an all stop has been called and the auto mode should exit
  while (current_time - init_time < 5000 && auto_flag == 1) {
    setEyeColor(0, 100, 0);
    dashRun(60, 150);
    current_time = millis();
    while (millis()-current_time < 50) {} // Give yourself time to run!
    setEyeColor(0, 0, 100);
    dashPacketHandler(); // listen for other commands
}
  
  setEyeColor(0,0,0);
}

// make Dash run in a figure 8!
void DashBot::dashFig8(void){
  unsigned long init_time = millis();
  unsigned long current_time = millis();

  // Turn left
  // auto_flag must be 1, if not, an all stop has been called and the auto mode should exit
  while (current_time- init_time < 2000 && auto_flag == 1) {
    dashRun(60,150);
    current_time = millis();
    while (millis()-current_time < 50) {} // Give yourself time to run!
    setEyeColor(0, 100, 0);
    dashPacketHandler(); // listen for other commands
  }


  // Turn right 
  while (current_time- init_time < 4000 && auto_flag == 1) {
    dashRun(60,-150);
    current_time = millis();
    while (millis()-current_time < 50) {} // Give yourself time to run!
    setEyeColor(0, 0, 100);
    dashPacketHandler();
  }

  setEyeColor(0,0,0);

}


// make Dash dance!
void DashBot::dashDance(void){
  
  unsigned long init_time = millis();

  // auto_flag must be 1, if not, an all stop has been called and the auto mode should exit
  while (millis() - init_time < 2000 && auto_flag == 1) {
  motorDriveL(-80);
  motorDriveR(80);
  setEyeColor(0, 0, 100);
  dashPacketHandler(); // listen for other commands
  
  }
  
  while (millis() - init_time < 4000 && auto_flag == 1) {
  motorDriveL(-80);
  motorDriveR(80);
  setEyeColor(0, 100, 0);
  dashPacketHandler();
  
  }

  while (millis() - init_time < 6000 && auto_flag == 1) {
  motorDriveL(80);
  motorDriveR(-80);
  setEyeColor(100, 0, 0);
  dashPacketHandler();
  
  }

  while (millis() - init_time < 8000 && auto_flag == 1) {
  motorDriveL(80);
  motorDriveR(-80);
  setEyeColor(100, 100, 100);
  dashPacketHandler();
  
  }

  allStop();
  setEyeColor(0,0,0);
}

// Dash will run forward until a collision is detected with the IR sensors, at which point Dash stops, backs up, turns, and again starts running
void DashBot::dashBump(void) {
  
  unsigned long init_time = millis();
  unsigned long current_time = millis();

  // auto_flag must be 1, if not, an all stop has been called and the auto mode should exit
  while (millis() - init_time < 20000 && auto_flag == 1){
    // run around
    dashRun(40,0);
    current_time = millis();
    while (millis()-current_time < 50) {} // Give yourself time to run!
    setEyeColor(100, 0, 0);
    dashPacketHandler(); // listen for other commands
    
    //if collision, back up, turn around, try again
    if (detectCollisionLeft(25) || detectCollisionRight(25)) {
      unsigned long bump_time = millis();


      // Stop the robot
      while (millis() - bump_time < 500 && auto_flag == 1) {
      setEyeColor(0, 100, 0);
      allStop();
      //dashPacketHandler();

      }
      
      // Back up
      while (millis() - bump_time < 1500 && auto_flag == 1) {
      dashRun(-40,0);
      //dashPacketHandler();
 
      }

      // Turn around
      while (millis() - bump_time < 2500 && auto_flag == 1) {
      dashRun(40,150);
      //dashPacketHandler();
  
      }
      
    }
  }
  allStop();
  setEyeColor(0,0,0);
  
}


// make Dash dance!
void DashBot::dashTest(void){
  
  digitalWrite(LED_RED, HIGH);
  delay(100);
  digitalWrite(LED_YELLOW, HIGH);
  delay(100);
  digitalWrite(LED_GREEN, HIGH);
  delay(100);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, LOW);
  
  
  unsigned long init_time = millis();

  // auto_flag must be 1, if not, an all stop has been called and the auto mode should exit
  while (millis() - init_time < 500 && auto_flag == 1) {
  motorDriveL(80);
  motorDriveR(0);
  dashPacketHandler(); // listen for other commands
  }
  
  init_time = millis();

  while (millis() - init_time < 500 && auto_flag == 1) {
  motorDriveL(0);
  motorDriveR(80);
  dashPacketHandler();
  }

  init_time = millis();

  while (millis() - init_time < 500 && auto_flag == 1) {
  motorDriveL(-80);
  motorDriveR(0);
  dashPacketHandler();
  }

  init_time = millis();

  while (millis() - init_time < 500 && auto_flag == 1) {
  motorDriveL(0);
  motorDriveR(-80);
  dashPacketHandler();
  }

  init_time = millis();

  while (millis() - init_time < 500 && auto_flag == 1) {
  motorDriveL(0);
  motorDriveR(0);
  setEyeColor(100,0,0);
  dashPacketHandler();
  }

  init_time = millis();

  while (millis() - init_time < 500 && auto_flag == 1) {
  motorDriveL(0);
  motorDriveR(0);
  setEyeColor(0,100,0);
  dashPacketHandler();
  }

  init_time = millis();

  while (millis() - init_time < 500 && auto_flag == 1) {
  motorDriveL(0);
  motorDriveR(0);
  setEyeColor(0,0,100);
  dashPacketHandler();
  }

  init_time = millis();

  while (millis() - init_time < 10000 && auto_flag == 1) {
  motorDriveL(0);
  motorDriveR(0);
  setEyeColor(50,50,50);
  dashPacketHandler();
  }

  allStop();
  setEyeColor(0,0,0);
}
