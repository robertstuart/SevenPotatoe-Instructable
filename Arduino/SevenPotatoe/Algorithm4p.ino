//
//void fpRun() {
//  setBlink(RE_LED, BLINK_FF);
//  setBlink(BU_LED, BLINK_FF);
//  setBlink(BD_LED, BLINK_SB);
//  delay(100);  // For switches()
//  while (mode == MODE_4P) { // main loop
//    commonTasks();
//    timeMicroseconds = micros();
//    timeMilliseconds = timeMicroseconds / 1000;
//
//    // Timed loop
//    if (isNewGyro()) {
//      setGyroData();
//      readRcRadio();
//      pitchBalance(); 
////      rollBalance();
//      sendLog();
//      checkMotor(true);
//      checkMotor(false);     
//    }
//    if (isNewAccel()) {
//      setAccelData();
//    }
//  }
//}
//
//
//
///***********************************************************************.
// *  pitchBalance()
// *      Balance on the pitch axis.  Go forward or backward to counteract
// *      falling forward or backward.
// *      Set the motor speed (fpFps).
// ***********************************************************************/
//void pitchBalance() {
//  readSpeed(true);
//  // compute the Center of Oscillation Speed (COS)
//  fpRotation = cosFactor * (-gyroPitchDelta); // 10.0 for 4potatoe
//  fpCos = fpsDw + fpRotation; // subtract rotation
//  fpLpfCos = (fpLpfCosOld * (1.0 - COS_TC))  + (fpCos  * cosTc); // smooth it out a little (0.2)
//  fpLpfCosAccel = fpLpfCos - fpLpfCosOld;
//  fpLpfCosOld = fpLpfCos;
//
//  fpControllerSpeed = controllerY * SPEED_MULTIPLIER;
////  fpControllerSpeed = 4.0;
//
////  fpControllerSpeed = jigZero(fpControllerSpeed); 
//
//  // find the speed error
//  fpSpeedError =  fpLpfCos - fpControllerSpeed;
//
//  // compute a weighted angle to eventually correct the speed error
//  fpTargetAngle = fpSpeedError * pitchGain; //************ Speed error to angle *******************
//
//  // Compute maximum angles for the current wheel speed and enforce limits.
//  float fwdA = fpsDw - 16.0;
//  float bkwdA = fpsDw + 16.0;
//  if (fpTargetAngle < fwdA)  fpTargetAngle = fwdA;
//  if (fpTargetAngle > bkwdA) fpTargetAngle = bkwdA;
//
//  // Compute angle error and weight factor
//  fpAngleError = fpTargetAngle - gaPitch;  //
//  fpCorrection = fpAngleError * speedErrorGain; //******************* Angle error to speed *******************
//  fpLpfCorrection = (fpLpfCorrectionOld * (1.0f - speedTc))  + (fpCorrection * speedTc);
//  fpLpfCorrectionOld = fpLpfCorrection;
//
//  // Add the angle error to the base speed to get the target speed.
//  fpFps = fpLpfCorrection + fpCos;
//
//  setDwTargetSpeed(fpFps);
////  if (true) {
//    addLog(
//      (long) (timeMicroseconds),
//      (short) (fpLpfCos * 100.0),
//      (short) (fpSpeedError * 100.0),
//      (short) (gaPitch * 100.0),
//      (short) (fpTargetAngle + 100.0),
//      (short) (fpLpfCorrection * 100.0),
//      (short) (fpFps * 100.0)
//    );
////  }
//} // end fpAlgorithm()
//
//
//
///***********************************************************************.
//    jigZero()  Jig back and fort then fpControllerSpeed is low.
// ***********************************************************************/
//const float JIG_SPEED = 4.0;     // Target speed during jig
//const float JIG_DISTANCE = 200.0;  // distance from center (JigTickPosition
//const float GO_TO_INCREMENT = 0.03;
//
//float jigZero(float controllerSpeed) {
//  static int jigHomeTickPosition = 0;
//  static float jigFps = 0.0;
//  static boolean isJigFwd = true;
//  int path = 0;
//  float newSpeed = 0;
//  if (abs(controllerSpeed) < 0.2) {
//    if (!isJig) {  // just entered stopped condition?
//      isJig = true;
//      isJigFwd = true;
//      jigHomeTickPosition = tickPositionDw + (TICKS_PER_FOOT * 3.0);  // Jig in front of start
//    }
//    if (isJigFwd) {
//      if (tickPositionDw > (jigHomeTickPosition + (TICKS_PER_FOOT * JIG_DISTANCE))) {
//        isJigFwd = false;
//      }
//      path = 1;
//      newSpeed = jigFps = goToFps(jigFps, JIG_SPEED, GO_TO_INCREMENT);
//    } else {
//      if (tickPositionDw < (jigHomeTickPosition - (TICKS_PER_FOOT * JIG_DISTANCE))) {
//        isJigFwd = true;
//      }
//      path = 2;
//      newSpeed = jigFps = goToFps(jigFps, -JIG_SPEED, GO_TO_INCREMENT);
//    }
//  } else {
//    isJig = false;
//    newSpeed = controllerSpeed;
//  }
//
////    sprintf(message, "jigTickPosition: %6d \t tickPositionDw: %6d \t", jigTickPosition, tickPositionDw);
////  sendBMsg(SEND_MESSAGE, message);
//
////
////  addLog(
////    (long) (isStopped),
////    (short) (isJigFwd),
////    (short) (tickPositionDw),
////    (short) (jigTickPosition),
////    (short) (fpControllerSpeed * 100.0),
////    (short) (fpsDw * 100.0),
////    (short) (path)
////  );
//
//  return newSpeed;
//}
//
//
//
///***********************************************************************.
// *   rollBalance()
// ***********************************************************************/
//const float ACCEL_TO_FPS = -0.2;
//
//void rollBalance() {
//  float correction = 0.0;
//  static boolean isContZeroLast = false;
//  static double yFps = 0.0;
//  static double xFps = 0.0;
//  static float oldGRoll = 0;
//  float pRoll, pVector, dRoll;
//
//  float rollRate = gRoll - oldGRoll;
//  oldGRoll = gRoll;
//  coHeading = gyroCumHeading + (rollRate * 50.0);
//
//  boolean isContZero = (abs(controllerX) <= 0.05) ? true : false;
//  if (isContZero && !isContZeroLast) { // Switched states & goes to zero
//    targetHeading = gyroCumHeading;  // Point it at current direction
//  } 
//  if (isContZero) {
//    pVector = (coHeading - targetHeading) * (pVectorFactor * 0.005); 
//  } else {
//    pVector = 0.0; // No pVector when steering by joystick.
//  }
//  isContZeroLast = isContZero;
//
//
//  pRoll = (gaRoll - (controllerX * 3.0)) * (pRollFactor * 0.05);
//  dRoll = gyroRollRate * (dRollFactor * 0.005);            
//  float pd = pRoll + pVector + dRoll;
//  
//  if (!isRunning) {
//    zeroYaw();
//    targetHeading = 0.0;
//    tickPositionDw = 0;
////    yFps = xFps = 0.0D;
//  }
//
//  if (abs(fpsDw) > 0.5) {
//    correction = pd / fpsDw;
//  } else {
//    // Set course when FP reaches speed.
//   targetHeading = gyroCumHeading;  // Point it at current direction
//   correction = 0.0;
//  }
//  yawAction(correction);
//  
////  if (isRunning) {
////    addLog(
////      (long) (targetHeading * 100.0),
////      (short) (coHeading * 100.0),
////      (short) (pVector * 100.0),
////      (short) (pRoll * 100.0),
////      (short) (pd * 100.0),
////      (short) (correction * 100.0),
////      (short) (0)
////    );
////  }
//  
////  static unsigned int loop = 0;
////  if ((++loop % 104) == 0) {
////    sprintf(message, "targetHeading: %5.1f \t gyroCumHeading: %5.1f \t coHeading: %5.1f", targetHeading, gyroCumHeading, coHeading);
////    sendBMsg(SEND_MESSAGE, message);
////  }
//}
//
//
///**************************************************************************.
// *  yawAction()    Called 200/sec.  Adjusts the Reaction Wheel Speed so that
// *                 FourPotatoe orients toward target
// **************************************************************************/
//void yawAction(float headingCorrection) {
//  static float oldRwAccel;
//
//  readSpeed(false);
//  
//  float yawP = headingCorrection * (yawPFactor * 0.00005);
//  float yawD = gyroYawRaw *(yawDFactor * 0.0000005);
//  float newRwAccel = yawP - yawD;
//
//  // LP filter it
//  float dec = (newRwAccel - oldRwAccel) * (rwAccelTc/1000.0);
//  oldRwAccel = newRwAccel - dec;
//
//  if (isRunning) targetRwRps -= oldRwAccel;
//
//  targetRwRps = constrain(targetRwRps, -MAX_RW_RPS, MAX_RW_RPS);
//  if (isUpright) {
//    setRwTargetSpeed(targetRwRps);
//  } else {
//    if (rwState == RW_RUNNING) rwState = RW_SPIN_DOWN;
//  }
//  
//   
////  if ((gyroLoop % 104) == 0) {
////    sprintf(message, "wVal: %5.2f \t gaRoll: %5.1f", wVal, gaRoll);
////    sendBMsg(SEND_MESSAGE, message);
////  }
//
////  if (isRunning) {
////    addLog(
////      (long) (headingCorrection * 100.0),
////      (short) (newRwAccel * 100.0),
////      (short) (oldRwAccel * 100.0),
////      (short) (dec * 100.0),
////      (short) (0),
////      (short) (0),
////      (short) (gyroHeading * 100.0)
////    );
////  }
//}
//
//
//
//
///***********************************************************************.
//    sendLog() Called 208 times/sec.
// ***********************************************************************/
//void sendLog() {
//  static unsigned int logLoop = 0;
//  logLoop++;
//
//  if (isDumpingData) {
//    if ((logLoop % 2) == 0)  dumpData();
//  }
//  //  if ((logLoop % 208) == 5) log1PerSec();
//  if ((logLoop % 51) == 5) log4PerSec();
//  //  if ((logLoop % 10) == 5) log20PerSec();  // 20/sec
//  //  if ((logLoop % 10) == 7) routeLog(); // 20/sec
//  //  log200PerSec();
//}
//
//
//void log1PerSec() {
//  sprintf(message, "battVolt: %3.1f \t gaPitch: %5.1f", battVolt, gaPitch);
//  sendBMsg(SEND_MESSAGE, message);
//}
//
//
//void log4PerSec() {
////  Serial.print(battVolt); Serial.print("\t");
////  Serial.print(aPitch); Serial.print("\t");
////  Serial.print(aRoll); Serial.print("\t");
////  Serial.print(gYaw); Serial.print("\t");  
////  Serial.println();
////  sprintf(message, "tickPositionDw: %6d \t", tickPositionDw);
////  sendBMsg(SEND_MESSAGE, message);
//  sprintf(message, "dw: %2d   isUpright: %2d   isRunning %2d", digitalRead(PWM_DW), isUpright, isRunning);
//  sendBMsg(SEND_MESSAGE, message);
//}
//
//
//void log20PerSec() {
//  if (isRunning) {
//    addLog(
//      (long) (0),
//      (short) (fpsDw * 100.0),
//      (short) (rpsRw * 100.0),
//      (short) (gaPitch * 100.0),
//      (short) (gaRoll * 100.0),
//      (short) (gyroCumHeading * 10.0),
//      (short) (targetHeading * 10.0)
//    );
//  }
////  Serial.print(battVolt); Serial.print("\t");
////  Serial.print(fpsDw); Serial.print("\t");
////  Serial.print(fpsRw); Serial.print("\t");
////  Serial.println();
//}
//
//
//void log200PerSec() {
//  static int loop = 0;
//  static float sum = 0.0;
//  sum += aPitch;
//  if (loop++ >= 208) {
//    Serial.print(battVolt); Serial.print("\t");
//    Serial.print(sum / 208.0); Serial.print("\t");
//    Serial.println();
//    sprintf(message, "aPitch %5.2f", (sum / 208.0));
//    sendBMsg(SEND_MESSAGE, message);
//    loop = 0;
//    sum = 0.0;
//  }
//}
//
//
//
///***********************************************************************.
// *  goToFps() change the fpControllerSpeed toward the target
// ***********************************************************************/
//float goToFps(float currentFps, float targetFps, float goToIncrement) {
//  float resultFps;
//  if (targetFps >= currentFps) {
//    resultFps = currentFps + goToIncrement;
//    if (resultFps > targetFps) resultFps = targetFps;
//  } else {
//    resultFps = currentFps - goToIncrement;
//    if (resultFps < targetFps) resultFps = targetFps;
//  }
//  return resultFps;
//}
//
//
