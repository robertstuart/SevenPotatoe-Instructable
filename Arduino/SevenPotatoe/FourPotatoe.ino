//#include "Common.h"
//#include "pwm01.h"
//#include <DueTimer.h>
//#include <Wire.h>
//#include <LSM6.h>
//int polls = 0;
//
//
//// ------------------------- Tuning variables ------------------------
////  Any changes in things like gear ratios, motors, wheel size, weight distribution, 
////  or changes in performance characteristics, stability, battery voltage etc. will
////  require changes in one or more of the following variables.
//
////----------- PitchBalance()
//float cosFactor = 10.0;
//float cosTc = 0.2;
//float pitchGain = 4.0;        // 4.0
//float speedErrorGain = 0.04 ;   // 0.4
//float speedTc = 0.05;
//
////------------ yawAction()
//float pRollFactor = 500.0;
//void setT(float t) { pRollFactor = t; }
//float pVectorFactor = 120.0;
//void setU(float u) { pVectorFactor = u; }
//float dRollFactor = 40.0;
//void setV(float v) { dRollFactor = v; }
//float rwAccelTc = 0.0;
//void setW(float w) { rwAccelTc = w; }
//float yawPFactor = 400.0;
//void setX(float x) { yawPFactor = x; }
//float yawDFactor = 500.0;
//void setY(float y) { yawDFactor = y; }
//
////------------- rwHeadAngle()
//float rwHeadAngle = 0.0;
//// void setT(float t) rwHeadAngle = t:
//
////-------------- aPwmSpeed()
//float rwSpeed = 0.0;
////void setT(float t) { rwSpeed = t; }
//float dwSpeed = 0.0;
////void setU(float u) { rwSpeed = u; }
//
////---------------- setAccelData()
//float pitchOffset = 0.0;
//void setZ(float z) { pitchOffset = z; }
//
////float motorFpsToInput = 0.163;        // Ration of change in speedR/speedL to fps.
////float cosFactor = -0.9;               // May need to be changed when there is a change of weight distribution.
////float cosTc = 0.98;                   // Shouldn't need to be changed.
////float angleFactor = 10.0;             // Speed error to pitch angle of FivePotatoe
////float fpsFactor = 0.2;                // Angle error to speed adjustment.
////float pitchError = 7.0;               // Adjust to have no drift forward or backward.
//
//
//// fpRun values
////const float TP6_T_VAL = 500.0;     // pRoll for rollBalance()
////const float TP6_U_VAL = 120.0;     // pVector for rollBalance()
////const float TP6_V_VAL = 40.0;    // dRoll for rollBalance()
////const float TP6_W_VAL = 0.0;      // tc for yawAction()
////const float TP6_X_VAL = 400.0;    // yawP for yawAction()
////const float TP6_Y_VAL = 500.0;     // yawD for yawAction()
////const float TP6_Z_VAL = 1.1;      // pitch Accel offset
//
//// old fpRun values
////const float TP6_T1_VAL = 6.0;
////const float TP6_U1_VAL = 10.0;
////const float TP6_V1_VAL = 4.0;
////const float TP6_W1_VAL = 0.40; 
////const float TP6_X1_VAL = 0.05;
//
//
//// aRwAngle() values
//const float RW_T_VAL = 0.0;     // angle
//const float RW_W_VAL = 0.0;     // tc for yawAction()
//const float RW_X_VAL = 400.0;   // yawP for yawAction()
//const float RW_Y_VAL = 500.0;    // yawP for yawAction()
//
//
//const boolean IS_ENABLE_DW = true;
//const boolean IS_ENABLE_RW = true;
//const float TICKS_PER_FOOT = 1231.4; // 2.3075' circ., 1.5 ratio, 2048 ticks  
//const float FEET_PER_TICK = 1.0 / TICKS_PER_FOOT;
//const float TICKS_PER_REV = 1024;
//const int PWM_FREQ = 20000;
//const float COS_TC = 0.2;
//const float SPEED_MULTIPLIER = 5.0; // Multiplier for controllerX to get speed
//const float PITCH_DRIFT = -34.37;
//const float ROLL_DRIFT =   -8.2;
//const float YAW_DRIFT =     3.4;
//const float GYRO_SENS = 0.0696;     // Multiplier to get degree. subtract 1.8662% * 8 for 2000d/sec
//const float GYRO_WEIGHT = 0.98;     // Weight for gyro compared to accelerometer
////const float GYRO_WEIGHT = 0.995;     // Weight for gyro compared to accelerometer
////const float MOTOR_GAIN_DW = 10.0;  // Actobotics motors
//const float MOTOR_GAIN_DW = 5.0;
//const float MOTOR_GAIN_RW = 5.0;
////const float JIG_MAX_FPS = 1.0;
////const float JIG_FPS_INC = 0.01;   // Speed increase each 1/200 sec during jig
//const int JIG_TICK_RANGE = 55;
//const float MAX_RW_RPS = 12.0;
//
///*
// * 1/293 = .00341
// */
//const float DW_FPS_FACTOR = 1000000.0 / TICKS_PER_FOOT;
//const float RW_RPS_FACTOR = 1000000.0 / TICKS_PER_REV;
////const double ENC_FACTOR = 3458.0f;  // Change pulse width to fps speed
//const int DW_FPS_FACTOR_M = DW_FPS_FACTOR * 1000.0;  // Change pulse width to milli-fps speed
//const int RW_RPS_FACTOR_M = RW_RPS_FACTOR * 1000.0;
//
//static const int ZERO_PW = 32766;  // Center of pulse width range. FPS = 0;
////static const int DEAD_ZONE = 1200; // Zone around ZERO_PW that gives zero FPS, Actobotics
//static const int DEAD_ZONE = 600; // Zone around ZERO_PW that gives zero FPS, 775
////static const float PW_VS_FPS = 2800.0; // change in PW gives change of 1.0 FPS, Actobotics motor
//static const float PW_VS_FPS = 1100.0; // change in PW gives change of 1.0 FPS, 775 12V motor
//static const float PW_VS_RPS = 638.0; // change in PW gives change of 1.0 RPS, 775 12V motor
//
//// State of the reaction wheel (rwState)
//static const int RW_STOP       = 0;
//static const int RW_SPIN_UP    = 1;
//static const int RW_STABILIZE  = 2;
//static const int RW_CALIBRATE  = 3;
//static const int RW_RUNNING    = 4;
//static const int RW_SPIN_DOWN  = 5;
//
//#define BLUE_SER Serial3
//
//// Pin definitions
//const int BATTERY   = 0;
//
//const int PWM_DW   = 6;
//const int DIR_DW   = 7;
//const int PWM_RW   = 8;
//const int DIR_RW   = 9;
//
//const int BD_LED     = 13; // led on arduino board
//const int BU_LED     = 28;
//const int RE_LED     = 29;
//const int RE_SWITCH  = 30;
//const int BU_SWITCH  = 31;
//const int X_SWITCH   = 32;
//const int Y_SWITCH   = 33;
//const int Z_SWITCH   = 34;
//const int SPEAKER    = 35;
//const int ACCEL_INT1  = 37;
//const int GYRO_INT2  = 36;
//
//const int RADIO_RX   = 43;  // Right joystick X, range restricted when right switch is 
//const int CH2_RADIO  = 45;  //* Right joystick Y, range restricted when right switch is 
//const int CH3_RADIO  = 47;  //* Left joystick Y, left switch turns off
//const int CH4_RADIO  = 49;  //* Left joystick X
//const int RADIO_VRA  = 51;  // knob on right
//const int RADIO_VRB  = 53;  // knob on left
//
//const int ENC_A_RW = 42;
//const int ENC_B_RW = 44;
//const int ENC_Z_RW = 46;
//const int ENC_Z_DW = 48;
//const int ENC_B_DW = 50;
//const int ENC_A_DW = 52;
//
//
//const float END_FLOAT_ARRAY = 4242.42;
//const float END_FLOAT_ARRAY_M = 4241.42;
//
//
//
//// Due has 96 kbytes sram
//// Arrays to save data to be dumped in blocks.
//#define DATA_ARRAY_SIZE 2500
//long  aArray[ DATA_ARRAY_SIZE];
//short bArray[ DATA_ARRAY_SIZE];
//short cArray[ DATA_ARRAY_SIZE];
//short dArray[ DATA_ARRAY_SIZE];
//short eArray[ DATA_ARRAY_SIZE];
//short fArray[ DATA_ARRAY_SIZE];
//short gArray[ DATA_ARRAY_SIZE];
//unsigned int dataArrayPtr = 0;
//
//
//// Flash sequences
//const byte END_MARKER = 42;
//byte BLINK_OFF[] = {0,END_MARKER};               // Off
//byte BLINK_SF[] = {1,0,0,0,0,0,0,0,END_MARKER};  // Slow flash
//byte BLINK_FF[] = {1,0,END_MARKER};              // Fast flash
//byte BLINK_SB[] = {1,1,1,1,0,0,0,0,END_MARKER};  // Slow blink
//byte BLINK_ON[] = {1,END_MARKER};                // On
//
//// Beep sequences
//int BEEP_UP [] = {1200, 100, 1500, 100, 0};
//int BEEP_WARBLE[] = {2400, 300, 2600, 300, 
//                  2400, 300, 2600, 300, 
//                  2400, 300, 2600, 300, 
//                  2400, 300, 2600, 300, 0};
//int BEEP_DOWN[] = {1000, 200, 666, 200, 0};
//
//float PULSE_END = 42.42;
// 
//int mode = MODE_4P;
////int mode = MODE_RW_ANGLE;
//int debugInt = 0;
//float debugFloat = 0.0;
//
///******************** Algorithm.ino ************************************/
//// xBalance()
//float fpRotation = 0.0;
//float fpCos = 0.0;
//float fpLpfCos = 0.0;
//float fpLpfCosAccel = 0.0;
//float fpLpfCosOld = 0.0;
//float fpControllerSpeed = 0.0; 
//float fpSpeedError = 0.0;
//float fpTargetAngle = 0.0;
//float fpAngleError = 0.0;
//float fpCorrection = 0.0;
//float fpLpfCorrection = 0.0;
//float fpLpfCorrectionOld = 0.0;
//float fpFps = 0.0;
//float targetHeading = 0.0;
//float coHeading = 0.0;
//
//// logXX()
//char message[200];
//
//float roll_correction = 3.2;
//float yaw_p = 10.0;
//float yaw_d = 10.0;
//
//float targetRwRps = 0.0;
//float targetRwAngle = 0.0;
//
///************************ Imu.ino *************************************/
//LSM6 lsm6;
//// setGyroData()
//float gyroPitchRaw = 0.0;
//float gyroPitchRate = 0.0;
//float gyroPitchDelta = 0.0;
//float gPitch = 0.0;
//float gaPitch = 0.0;
//float gyroRollRaw = 0.0;
//float gyroRollRate = 0.0;
//float gyroRollDelta = 0.0;
//float gRoll = 0.0;
//float gaRoll = 0.0;
//float gyroYawRaw = 0.0;
//float gyroYawRate = 0.0;
//float gyroYawDelta = 0.0;
//float gYaw = 0.0;
//float gyroCumHeading = 0.0;
//float gyroHeading = 0.0;
//// setAccelData()
//float aPitch = 0.0;
//float aRoll = 0.0;
//unsigned int gyroLoop = 0;
//unsigned int accelLoop = 0;
//
///************************ Motor.ino ************************************/
//volatile int mFpsDw = 0;
//volatile int mCorMRpsRw = 0;
//volatile int mRpsRw = 0;
//volatile int interruptErrorsDw = 0;
//volatile int interruptErrorsRw = 0;
//volatile unsigned int tickTimeDw = 0;
//volatile unsigned int tickTimeRw = 0;
//volatile int tickPeriodDw = 0;
//volatile int tickPeriodRw = 0;
//volatile int tickPositionDw = 0;
//volatile int tickPositionRw = 0;
//volatile int tickCycleArray[24];
//volatile int tickCyclePtr;
//float fpsDw = 0.0;
//
//float rpsRw = 0.0;
//float targetSpeedDw = 0.0;
//float targetMFpsDw = 0;
//float targetMRpsRw = 0;
//int targetPwDw = 0;
//int targetPwRw = 0;
////unsigned int pwDw = 0;
////unsigned int pwRw = 0;
//
///************************ Msg.ino **************************************/
//// readPc()
//unsigned int tPc = 0;
//// doMsg()
//float hcX = 0.0;
//float pcX = 0.0;
//float hcY = 0.0;
//float pcY = 0.0;
//float pcYSlider = 0.0;
//float controllerX = 0.0;
//float controllerY = 0.0;
////float tVal = 0.0;
////float uVal = 0.0;
////float vVal = 0.0;
////float wVal = 0.0;
////float xVal = 0.0;
////float yVal = 0.0;
////float zVal = 0.0;
//
///************************ Tasks.ino ************************************/
////controllerConnected()
//boolean isHcActive = false;
//boolean isPcActive = false;
//boolean isController = false;
////battery()
//float battVolt = 0.0;
//
///*********************** Global variables ************************************/
//boolean isRunReady = false;
//boolean isRunning = false;
//boolean isJigger = false;
//boolean isUpright = true;
//boolean isDumpingData = false;
//boolean isBattAlarmDisabled = false;
//boolean isJig = false;
//boolean xToggle = false;
//boolean yToggle = false;
//volatile int rwState = RW_STOP;
//
//int ch2pw = 0;
//int ch3pw = 0;
//int ch4pw = 0;
//volatile unsigned int ch2riseTime = 0;
//volatile unsigned int ch3riseTime = 0;
//volatile unsigned int ch4riseTime = 0;
//boolean ch3sw = false;
//
//unsigned int timeMilliseconds = 0;
//unsigned int timeMicroseconds = 0;
//
//void setup() {
//
//  pinMode(BU_LED, OUTPUT);
//  pinMode(RE_LED, OUTPUT);
//  pinMode(BD_LED, OUTPUT);
//  pinMode(SPEAKER, OUTPUT);
//  pinMode(RE_SWITCH, INPUT_PULLUP);
//  pinMode(BU_SWITCH, INPUT_PULLUP);
//  pinMode(X_SWITCH, INPUT_PULLUP);
//  pinMode(Y_SWITCH, INPUT_PULLUP);
//  pinMode(Z_SWITCH, INPUT_PULLUP);
//  
//  pinMode(ENC_A_DW, INPUT);
//  pinMode(ENC_B_DW, INPUT);
//  pinMode(ENC_A_RW, INPUT);
//  pinMode(ENC_B_RW, INPUT);
//  
//  BLUE_SER.begin(115200);   // Bluetooth 
//  Serial.begin(115200);     // for debugging output
//
//  imuInit();
//  motorInit();
//  rcRadioInit();
//  beep(BEEP_UP);
//}
//
//void loop() {
//  switch (mode) {
//  case MODE_PWM_SPEED:
//    aPwmSpeed();
//    break;
////  case MODE_T_SPEED:
////    aTickSpeed();
////    break;
//  case MODE_4P:
//    fpRun();
//    break;
//  case MODE_RW_ANGLE:
//    aRwAngle();
//    break;
////  case MODE_RW_DRIVE:
////    aRwDrive();
////    break;
//  default:
//    commonTasks; 
//    break;
//  } // end switch(mode)
//
//}
//
//
//
///**************************************************************************.
// * aPwmSpeed()
// **************************************************************************/
////static int pwTick;
//
//void aPwmSpeed() {
//  unsigned long pwmTrigger = 0;
//  unsigned long tt;
//
//  
//  delay(100);  // For switches()
//  rwSpeed = ZERO_PW;
//  dwSpeed = ZERO_PW;
//  isRunning = true;
//  setBlink(RE_LED, BLINK_FF);
//  setBlink(BD_LED, BLINK_FF);
//  yToggle = false;
//  
//  while (mode == MODE_PWM_SPEED) {
//    commonTasks();
//    if (isNewGyro()) {
//      setGyroData();
//      sendLog();
//    }
//    if (timeMicroseconds > pwmTrigger) {
//      pwmTrigger = timeMicroseconds + 100000; // 10/sec
//      
//      pwm_write_duty(DIR_DW, ((unsigned int) (dwSpeed)));
//      pwm_write_duty(DIR_RW, ((unsigned int) (rwSpeed)));
//      readSpeed(true);  
//      readSpeed(false);  
//      sendStatusBluePc();
////      if (isDumpingTicks) dumpTicks();
//    } // end timed loop 
//  } // while
//} // aPwmSpeed()
//
//
//
///**************************************************************************.
// * aTickSpeed()
// **************************************************************************/
////void aTickSpeed() {
////  static unsigned int loop = 0;
////
////  delay(100);  // For switches()
////  tVal = 0.0;
////  uVal = 0.0;
////  setBlink(RE_LED, BLINK_FF);
////  setBlink(BD_LED, BLINK_FF);
////  
////  while (mode == MODE_T_SPEED) {
////    commonTasks();
////polls++;
////    if (isNewGyro()) {
////      setGyroData();
////      fpControllerSpeed = ((float) tVal) / 1000.0;
////      setDwTargetSpeed(fpControllerSpeed);
////      targetRwRps = ((float) uVal) / 1000.0;
////      setRwTargetSpeed(targetRwRps);
////      readSpeed(true);  
////      readSpeed(false);  
////      checkMotor(true);
////      checkMotor(false);
////      sendLog();
////polls = 0;
////    }
////  } // while
////} // aTickSpeed()
//
//
//
///**************************************************************************.
// * aRwAngle()
// **************************************************************************/
//const int PW_TICK_LIMIT = 24*4;
//
//void aRwAngle() {
//  static float sum = 0.0;
//  float headAngle;
//  
//  delay(100);  // For switches()
//  
//  yToggle = false;
//  
//  while (mode == MODE_RW_ANGLE) {
//    commonTasks();
//    if (isNewGyro()) {
//      setGyroData();
//      readSpeed(false); 
//      
//      if (rwHeadAngle > 179) headAngle = pcYSlider * 45.0;
//      else headAngle = rwHeadAngle;
//      yawAction(headAngle - gyroHeading);
//      
//      checkMotor(false);
//      sendLog();  // needed for dump
//      if (!isRunning) {
//        zeroYaw();
//      }
//
////      if ((gyroLoop % 200) == 0) {
////        sprintf(message, "fpControllerSpeed: %5.2f \t rpsRw: %5.2f \t rwState: %d", fpControllerSpeed, rpsRw, rwState);
////        sendBMsg(SEND_MESSAGE, message);
////      }
//    }
//  } // while
//} // aRwAngle()
