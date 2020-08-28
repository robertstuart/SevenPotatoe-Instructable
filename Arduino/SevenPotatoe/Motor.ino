/*****************************************************************************-
 *                        Motor.ino
 *****************************************************************************/
volatile unsigned long timesReact[5];
volatile bool dirsReact[5];
volatile int ptrReact = 0;
volatile int tickCountReact = 0;
volatile unsigned long timesDrive[5];
volatile bool dirsDrive[5];
volatile int ptrDrive = 0;
volatile int tickCountDrive = 0;

/*****************************************************************************-
 *  motorInit()
 *****************************************************************************/
void motorInit() {
  pinMode(DIR_DRIVE_PIN, OUTPUT);
  pinMode(DIR_REACT_PIN, OUTPUT);
  pinMode(ENC_A_DRIVE_PIN, INPUT);
  pinMode(ENC_A_REACT_PIN, INPUT);
  pinMode(ENC_B_DRIVE_PIN, INPUT);
  pinMode(ENC_B_REACT_PIN, INPUT);

  analogWriteFrequency(PWM_DRIVE_PIN, 20000);
  analogWriteFrequency(PWM_REACT_PIN, 20000);

  digitalWrite(DIR_DRIVE_PIN, LOW);
  digitalWrite(DIR_REACT_PIN, LOW);
  analogWrite(PWM_DRIVE_PIN, 0);
  analogWrite(PWM_REACT_PIN, 0);

  attachInterrupt(digitalPinToInterrupt(ENC_A_DRIVE_PIN), encoderIsrDriveA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_DRIVE_PIN), encoderIsrDriveB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_REACT_PIN), encoderIsrReactA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_REACT_PIN), encoderIsrReactB, CHANGE);
}



/*****************************************************************************-
 * encoderIsr???()
 *****************************************************************************/
void encoderIsrDriveA() { encoderIsrDrive(true); }
void encoderIsrDriveB() { encoderIsrDrive(false); }
void encoderIsrDrive(bool isA) {
  static bool oldA = true;
  static bool oldB = true;
  static bool isLastA = true;
  static unsigned int lastTickTime = 0ul;
 
  boolean encA = (digitalReadFast(ENC_A_DRIVE_PIN) == HIGH) ? true : false;
  boolean encB = (digitalReadFast(ENC_B_DRIVE_PIN) == HIGH) ? true : false;
  unsigned long tickTime = micros();
  bool isFlipA = (encA != oldA);
  bool isFlipB = (encB != oldB);
  oldA = encA;
  oldB = encB;
//  addTickLog(tickTimeDrive, isMotFwd, isA, encA, encB);
  if ((isA && !isFlipA) || (!isA && !isFlipB)) {
    interruptErrorsDriveA++; // Bogus unterrupt'
    return;
  }
  if (isLastA == isA) {
    interruptErrorsDriveB++;  // Bogus unterrupt or direction change.
    return;
  }
  if ((tickTime - lastTickTime) < 55) {
    interruptErrorsDriveC++;  // Shouldn't happen
  }
  isLastA = isA;
  lastTickTime = tickTime;

  bool isFwd;
  if      (isA  && (encA == encB)) isFwd = true;
  else if (!isA && (encA != encB)) isFwd = true;
  else                             isFwd = false;
  byte stateByte = (isFwd) ? STATE_DIR : 0;
  stateByte     |= (isA)   ? STATE_ISA : 0;
  stateByte     |= (encA)  ? STATE_A : 0;
  stateByte     |= (encB)  ? STATE_B : 0;
  
  tickPtrDrive++;
  tickPtrDrive %= N_TICK_BUFF;
  tickTimesDrive[tickPtrDrive] = tickTime;
  tickStatesDrive[tickPtrDrive] = stateByte;
  ticksDrive++;
} // encoderIsrDrive()


/*****************************************************************************-
   encoderIsrReact()
 *****************************************************************************/
void encoderIsrReactA() { encoderIsrReact(true); }
void encoderIsrReactB() { encoderIsrReact(false); }
void encoderIsrReact(bool isA) {
  static bool oldA = true;
  static bool oldB = true;
  static bool isLastA = true;
  static unsigned int lastTickTime = 0ul;
 
  boolean encA = (digitalReadFast(ENC_A_REACT_PIN) == HIGH) ? true : false;
  boolean encB = (digitalReadFast(ENC_B_REACT_PIN) == HIGH) ? true : false;
  unsigned long tickTime = micros();
  bool isFlipA = encA == oldA;
  bool isFlipB = encB == oldB;
  oldA = encA;
  oldB = encB;
  
  if ((isA && isFlipA) || (!isA &&isFlipB)) {
    interruptErrorsReactA++; // Bogus unterrupt'
    return;
  }
  if (isLastA == isA) {
    interruptErrorsReactB++;  // Bogus unterrupt or direction change.
    return;
  }
  if ((tickTime - lastTickTime) < 55) {
    interruptErrorsReactC++;  // Shouldn't happen
  }
  isLastA = isA;
  lastTickTime = tickTime;

  bool isFwd;
  if      (isA  && (encA == encB)) isFwd = false;
  else if (!isA && (encA != encB)) isFwd = false;
  else                             isFwd = true;
  byte stateByte = (isFwd) ? STATE_DIR : 0;
  stateByte     |= (isA)   ? STATE_ISA : 0;
  stateByte     |= (encA)  ? STATE_A : 0;
  stateByte     |= (encB)  ? STATE_B : 0;
  
  tickPtrReact++;
  tickPtrReact %= N_TICK_BUFF;
  tickTimesReact[tickPtrReact] = tickTime;
  tickStatesReact[tickPtrReact] = stateByte;
  ticksReact++;
} // end encoderIsrReact();



/*****************************************************************************-
 * readSpeed????()  Called every loop from CheckMotor()
 *****************************************************************************/
void readSpeedDrive() {
  static float predictedKphDrive = 0.0;
  static int oldTickPtr = 0;
  
  int tickSum = 0;
  int tickCount = 0;
  bool isEndFwd = false;

  // Calculate predicted kph
  float tgtKph =((float) motorPwDrive) / KPH_TO_PW;
  float pdiff = predictedKphDrive - tgtKph;
  predictedKphDrive -= pdiff * K6;

  bool isStartFwd = ((tickStatesDrive[oldTickPtr] & STATE_DIR) != 0) ^ ENCODER_PHASE;
  tickCount = ((tickPtrDrive + N_TICK_BUFF) - oldTickPtr) % N_TICK_BUFF;
  if (tickCount == 0) { // No interrupts?
    unsigned long t = micros();
    int period = t - tickTimesDrive[tickPtrDrive];
    float newKph = USEC_TO_KPH / ((float) period);
    if (abs(newKph) < abs(wKphDrive)) {
      wKphDrive = newKph;
      if (!isStartFwd) wKphDrive *= -1.0;
    }
  } else {
    if ((MOTOR_EVENTS < 30) && (tickCount < 3)) {
      oldTickPtr = tickPtrDrive - 4;
      if (oldTickPtr < 0) oldTickPtr += N_TICK_BUFF;
      tickCount = 4;
    }
    for (int i = 0; i < tickCount; i++) {
      tickSum += tickTimesDrive[(oldTickPtr + 1) % N_TICK_BUFF] - tickTimesDrive[oldTickPtr];
      oldTickPtr++;
      oldTickPtr %= N_TICK_BUFF;
      isEndFwd = ((tickStatesDrive[oldTickPtr] & STATE_DIR) != 0) ^ ENCODER_PHASE;
      tickPositionDrive +=  (isEndFwd) ? 1 : -1;
    }
    int period = tickSum / tickCount;
    wKphDrive = USEC_TO_KPH / ((float) period);
    if (isStartFwd != isEndFwd) wKphDrive = 0.0;
    if (!isEndFwd) wKphDrive *= -1.0;
  }
if (abs(wKphDrive) > 0.1) Serial.printf("%6.2f  %6.2f %6.2f\n", wKphDrive, predictedKphDrive, pdiff);
if (isRunning) add8Log(wKphDrive, predictedKphDrive, imu.maPitch, motorPwDrive, 
                       (float) interruptErrorsDriveA,
                       (float) interruptErrorsDriveB,
                       (float) interruptErrorsDriveC,
                       0.0);

}

void readSpeedReact() {
  static float predictedKphReact = 0.0;
  static int oldTickPtr = 0;
  
  int tickSum = 0;
  int tickCount = 0;
  bool isEndFwd = false;

  // Calculate predicted kph
  float tgtKph =((float) motorPwReact) / KPH_TO_PW;
  float pdiff = predictedKphReact - tgtKph;
  predictedKphReact -= pdiff * K6;
  
  bool isStartFwd = ((tickStatesReact[oldTickPtr] & STATE_DIR) != 0) ^ ENCODER_PHASE;
  tickCount = ((tickPtrReact + N_TICK_BUFF) - oldTickPtr) % N_TICK_BUFF;

  if (tickCount == 0) { // No interrupts?
    unsigned long t = micros();
    int period = t - tickTimesReact[tickPtrReact];
    float newKph = USEC_TO_KPH / ((float) period);
    if (abs(newKph) < abs(wKphReact)) {
      wKphReact = newKph;
      if (!isStartFwd) wKphReact *= -1.0;
    }
  } else {
    if ((MOTOR_EVENTS < 30) && (tickCount < 3)) {
      oldTickPtr = tickPtrReact - 4;
      if (oldTickPtr < 0) oldTickPtr += N_TICK_BUFF;
      tickCount = 4;
    }
    for (int i = 0; i < tickCount; i++) {
      tickSum += tickTimesReact[(oldTickPtr + 1) % N_TICK_BUFF] - tickTimesReact[oldTickPtr];
      oldTickPtr++;
      oldTickPtr %= N_TICK_BUFF;
      isEndFwd = ((tickStatesReact[oldTickPtr] & STATE_DIR) != 0) ^ ENCODER_PHASE;
      tickPositionReact +=  (isEndFwd) ? 1 : -1;
    }
    int period = tickSum / tickCount;
    wKphReact = USEC_TO_KPH / ((float) period);
    if (isStartFwd != isEndFwd) wKphReact = 0.0;
    if (!isEndFwd) wKphReact *= -1.0;
  }
//if (abs(wKphReact) > 0.1) Serial.printf("%6.2f  %6.2f %6.2f\n", wKphReact, predictedKphReact, pdiff);
}



/*****************************************************************************-
 * runMotors()
 *****************************************************************************/
void runMotors() {
  runMotorDrive();
  runMotorReact();
  wKph = (wKphReact + wKphDrive) / 2.0;
  tickPosition = tickPositionDrive + tickPositionReact;
  tickMeters = tickPosition / (TICKS_PER_METER * 2.0);
}



/*****************************************************************************-
 * runMotor????()  Called every loop
 *****************************************************************************/
const float WS_TC = 0.6;  // 0 to 1;
void runMotorDrive() {
  static float wsErrorLpf = 0.0;
  float g = K0_RESULT;  // motor gain computed locally
  readSpeedDrive();
  
  float wsError = (float) (targetWKphDrive - wKphDrive);
  wsErrorLpf = (WS_TC * wsError) + ((1.0 - WS_TC) * wsErrorLpf);
  if (abs(targetWKphDrive) < 1.0) {  // reduce gain below 1 kph
    g = (abs(targetWKphDrive) * g);
  }
  motorTargetKphDrive = targetWKphDrive + (wsErrorLpf * g);  // Target speed to correct error
  int pw = (int) (motorTargetKphDrive * KPH_TO_PW);            // Pw for the target.
  setMotorDrive(pw);
}

void runMotorReact() {
  static float wsErrorLpf = 0.0;
  float g = K0_RESULT;  // motor gain computed locally
  readSpeedReact();

  float wsError = (float) (targetWKphReact - wKphReact);
  wsErrorLpf = (WS_TC * wsError) + ((1.0 - WS_TC) * wsErrorLpf);
  if (abs(targetWKphReact) < 1.0) {  // reduce gain below 1 kph
    g = (abs(targetWKphReact) * g);
  }
  motorTargetKphReact = targetWKphReact + (wsErrorLpf * g);  // Target speed to correct error
  int pw = (int) (motorTargetKphReact * KPH_TO_PW);            // Pw for the target.
  setMotorReact(pw);
}



/*****************************************************************************-
 * setMotor????() Set pw and diriction. pw between -254 & +255
 ******************************************************************************/
void setMotorDrive(int pw) {
  pw = constrain(pw, -254, 255);
  motorPwDrive = pw; 
  int dir = (pw >= 0) ? LOW : HIGH;
  pw = abs(pw);
  if (isBatteryCritical) pw = 0;
  if (!isRunning) pw = 0;
  digitalWrite(DIR_DRIVE_PIN, dir);
}

void setMotorReact(int pw) {
  pw = constrain(pw, -254, 255);
  motorPwReact = pw;
  int dir = (pw >= 0) ? HIGH : LOW;
  pw = abs(pw);
  if (isBatteryCritical) pw = 0;
  if (!isRunning) pw = 0;
  digitalWrite(DIR_REACT_PIN, dir);
  analogWrite(PWM_REACT_PIN, pw);
}
