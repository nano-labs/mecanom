#include <movingAvg.h>
#include <PID_v1.h>

// =====================================================================
//  MECANUM DRIVE — 4 motors, 4 angular position sensors, Futaba RC in
// =====================================================================
//
//  Reorganized from the original single-file version:
//    - The four near-identical MoveMotorX() functions are now one
//      moveMotor(index, speed) driven by pin arrays.
//    - The four near-identical RC channel read/deadband blocks in
//      readFutaba() are now one readChannel() helper.
//    - Added a PULSE_TIMEOUT so pulseIn() can never block for its
//      default 1 second if a signal is lost.
//    - Added a divide-by-zero guard in readSpeed() (dt == 0 case).
//    - readSpeed() now computes a true circular-wraparound delta
//      instead of guessing direction from the commanded target.
//    - Added a POSITION_NOISE_DEADBAND to reject sensor/pulseIn jitter
//      from being differentiated into a false speed reading.
//    - Added SINGLE_MOTOR_DEBUG / DEBUG_MOTOR: when enabled, only that
//      one motor's pins and position sensor are ever touched, so the
//      other three can be safely left unpowered/disconnected on the
//      bench. This replaces the old hardcoded `motor = 0;` override
//      in readSpeed(), which has been removed — DEBUG_MOTOR is now
//      the single place that selects which motor is under test.
// =====================================================================


// ------------------------- Motor pin assignments -------------------------
// Index: 0 = back right, 1 = back left, 2 = front left, 3 = front right
const int motorPinA[4]    = {6, 3, 12, 9};   // IN1 (0,2) / IN3 (1,3)
const int motorPinB[4]    = {5, 4, 11, 10};  // IN2 (0,2) / IN4 (1,3)
const int motorEnable[4]  = {7, 2, 13, 8};
const int motorPosPins[4] = {32, 34, 36, 50}; // angular position sensors

// Left-side motors (1,2) are mounted mirrored relative to right-side
// motors (0,3), so a physically-forward rotation reads as an increasing
// raw angle on one side and a decreasing raw angle on the other.
const int motorDirSign[4] = {1, -1, -1, 1};

// ------------------------- RC receiver pins -------------------------
// ch1: left horizontal stick
// ch2: left vertical stick
// ch3: right vertical stick   (repurposed below as a ramp/differential dial)
// ch4: right horizontal stick
// ch5: 2-position switch
const int ch1_pin = 30;
const int ch2_pin = 28;
const int ch3_pin = 26;
const int ch4_pin = 24;
const int ch5_pin = 22;

// ------------------------- RC calibration (per channel) -------------------------
int ch1_min = 1282, ch1_max = 1673;
int ch2_min = 1274, ch2_max = 1695;
int ch3_min = 1041, ch3_max = 1700;
int ch4_min = 1278, ch4_max = 1705;
int ch5_min = 987,  ch5_max = 1960;

// ------------------------- Control constants -------------------------
const int dead_center = 50;
const int max_limit = 250;
const int min_limit = -250;
const unsigned long PULSE_TIMEOUT = 50000UL; // 25ms; prevents pulseIn() stalling up to 1s on signal loss

bool calibrating = false;
unsigned long calibration_start = 0;

// ------------------------- Single-motor debug mode -------------------------
// When true, only DEBUG_MOTOR is driven and its sensor is read; the other
// three motors' pins are never touched at all (no moveMotor(), no pulseIn()
// on their position sensors). Safe to use with only one wheel wired up on
// the bench and the rest fully unpowered/disconnected.
const bool SINGLE_MOTOR_DEBUG = true;
const int DEBUG_MOTOR = 0; // index of the motor currently on the bench

// ------------------------- Motor state -------------------------
double motorSpeeds[4]       = {0.0, 0.0, 0.0, 0.0};
int motorInputs[4]          = {0, 0, 0, 0};
int motorTargetSpeeds[4]    = {0, 0, 0, 0};
unsigned long motorPos[4]   = {0, 0, 0, 0};
unsigned long motorLastReads[4] = {0, 0, 0, 0};

movingAvg m1Avg(3);
movingAvg m2Avg(3);
movingAvg m3Avg(3);
movingAvg m4Avg(3);

// ------------------------- RC channel values -------------------------
int ch1 = 0, ch2 = 0, ch3 = 0, ch4 = 0, ch5 = 0;

// ------------------------- Drive/ramp state -------------------------
int direction = 1;
double differential = 0.0;
int difference = 0;


// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(motorPinA[i], OUTPUT);
    pinMode(motorPinB[i], OUTPUT);
    pinMode(motorEnable[i], OUTPUT);
  }

  motorPos[0] = map(pulseIn(motorPosPins[0], HIGH, PULSE_TIMEOUT), 0, 850, 0, 360);
  motorLastReads[0] = millis();

  pinMode(ch1_pin, INPUT);
  pinMode(ch2_pin, INPUT);
  pinMode(ch3_pin, INPUT);
  pinMode(ch4_pin, INPUT);
  pinMode(ch5_pin, INPUT);

  Serial.begin(57600); // Pour a bowl of Serial
  delay(3000);

  m1Avg.begin();
  m2Avg.begin();
  m3Avg.begin();
  m4Avg.begin();
}


// =====================================================================
//  MAIN LOOP
// =====================================================================
void loop() {
  move();
  int motor = SINGLE_MOTOR_DEBUG ? DEBUG_MOTOR : 0;
  Serial.println("Motor:" + String(motor) +
                  ",Speed:" + String(motorSpeeds[motor]) +
                  ",Output:" + String(motorInputs[motor]) +
                  ",Target:" + String(motorTargetSpeeds[motor]) +
                  ",Delta:" + String(differential));
}


// =====================================================================
//  RC INPUT
// =====================================================================

// Applies the shared "dead_center" deadband/rebase logic used by ch1-ch4.
int applyDeadband(int value, int deadband) {
  if (value <= deadband && value >= -deadband) {
    return 0;
  } else if (value > deadband) {
    return value - deadband;
  } else {
    return value + deadband;
  }
}

// Reads one RC channel, maps it into [outLow, outHigh] (order matters —
// some channels are intentionally inverted), clamps it, and optionally
// applies the deadband/rebase used for the four stick axes.
int readChannel(int pin, int rawMin, int rawMax, int outLow, int outHigh, bool deadband) {
  int value = map(pulseIn(pin, HIGH, PULSE_TIMEOUT), rawMin, rawMax, outLow, outHigh);
  value = constrain(value, min(outLow, outHigh), max(outLow, outHigh));
  if (deadband) {
    value = applyDeadband(value, dead_center);
  }
  return value;
}

void readFutaba() {
  // ch1: left horizontal stick
  // ch2: left vertical stick
  // ch3: right vertical stick
  // ch4: right horizontal stick
  // ch5: 2-position switch

  ch1 = readChannel(ch1_pin, ch1_min, ch1_max, min_limit, max_limit, true);
  ch2 = readChannel(ch2_pin, ch2_min, ch2_max, max_limit, min_limit, true); // inverted
  ch3 = readChannel(ch3_pin, ch3_min, ch3_max, max_limit, min_limit, true); // inverted

  // ch3 repurposed here as a ramp/differential dial, not a drive axis
  differential = map((float)ch3, (float)max_limit, (float)min_limit, 0.0, 100.0) / 200.0;

  ch4 = readChannel(ch4_pin, ch4_min, ch4_max, min_limit, max_limit, true);
  ch5 = readChannel(ch5_pin, ch5_min, ch5_max, min_limit, max_limit, false); // switch, no deadband

  if (ch5 > 0) {
    ch1 = ch1 * 0.5;
    ch2 = ch2 * 0.5;
    ch3 = ch3 * 0.5;
    ch4 = ch4 * 0.5;
  }

  // Serial.println(String(ch1) +", "+ String(ch2) +", "+ String(ch3) +", "+ String(ch4) +", "+ String(ch5));
}

// Streams live averaged channel values over Serial for calibrating
// ch1_min/max .. ch5_min/max. Not currently called from setup()/loop();
// invoke manually (e.g. from a debug build) when recalibrating sticks.
void checkCalibration() {
  movingAvg ch1_avg(20);
  movingAvg ch2_avg(20);
  movingAvg ch3_avg(20);
  movingAvg ch4_avg(20);
  movingAvg ch5_avg(20);
  ch1_avg.begin();
  ch2_avg.begin();
  ch3_avg.begin();
  ch4_avg.begin();
  ch5_avg.begin();

  while (true) {
    ch1_avg.reading(pulseIn(ch1_pin, HIGH, PULSE_TIMEOUT));
    ch2_avg.reading(pulseIn(ch2_pin, HIGH, PULSE_TIMEOUT));
    ch3_avg.reading(pulseIn(ch3_pin, HIGH, PULSE_TIMEOUT));
    ch4_avg.reading(pulseIn(ch4_pin, HIGH, PULSE_TIMEOUT));
    ch5_avg.reading(pulseIn(ch5_pin, HIGH, PULSE_TIMEOUT));
    Serial.println(String(ch1_avg.getAvg()) + ", " + String(ch2_avg.getAvg()) + ", " +
                    String(ch3_avg.getAvg()) + ", " + String(ch4_avg.getAvg()) + ", " +
                    String(ch5_avg.getAvg()));
  }
}


// =====================================================================
//  LOW-LEVEL MOTOR CONTROL
// =====================================================================

// Drives motor `idx` (0=back right, 1=back left, 2=front left, 3=front right)
// at `speed` in [-255, 255].
void moveMotor(int idx, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(motorPinA[idx], HIGH);
    digitalWrite(motorPinB[idx], LOW);
    analogWrite(motorEnable[idx], speed);
  } else {
    digitalWrite(motorPinA[idx], LOW);
    digitalWrite(motorPinB[idx], HIGH);
    analogWrite(motorEnable[idx], -speed);
  }
}


// =====================================================================
//  SPEED FEEDBACK (angular position sensors)
// =====================================================================

// Returns the shortest signed angular distance from `from` to `to` on a
// circular scale of `range` units (e.g. 3600 = one full revolution).
// Result is in (-range/2, range/2].
int circularDelta(long from, long to, long range) {
  long d = to - from;
  long half = range / 2;
  if (d > half) d -= range;
  else if (d <= -half) d += range;
  return (int)d;
}

// Minimum |delta| (in mapped 0-3600 units) worth trusting as real motion.
// Empirically, pulseIn()'s ~4us timing resolution plus normal sensor noise
// produces position jitter of up to ~30 units while the wheel is fully
// stationary; anything at or below this is treated as no movement rather
// than differentiated into a false speed reading.
const int POSITION_NOISE_DEADBAND = 40;

double readSpeed(int motor) {
  unsigned long new_pos = constrain(
      map(pulseIn(motorPosPins[motor], HIGH, PULSE_TIMEOUT), 0, 900, 0, 3600),
      0, 3600);
  unsigned long new_read = millis();

  // True wraparound delta — no dependency on the commanded target, so this
  // reflects what the wheel is actually doing even while stopping, stalled,
  // or being pushed opposite to the commanded direction.
  int delta = circularDelta((long)motorPos[motor], (long)new_pos, 3600);
  delta *= motorDirSign[motor];

  int speed;
  if (abs(delta) <= POSITION_NOISE_DEADBAND) {
    // Below the sensor's noise floor — treat as stationary rather than
    // computing a spurious instantaneous speed from jitter.
    speed = 0;
  } else {
    unsigned long dt = new_read - motorLastReads[motor];
    if (dt == 0) dt = 1; // guard against divide-by-zero if called twice in the same millisecond

    // Same empirical calibration as before (raw units/ms scaled so that
    // ~140 raw-units-per-10ms reads as full speed, i.e. 255).
    long magnitude = (abs((long)delta) * 10 * 255L) / (dt * 140L);
    magnitude = constrain(magnitude, 0, 255);
    speed = (delta < 0) ? -(int)magnitude : (int)magnitude;
  }

  if (motor == 0) {
    m1Avg.reading(speed);
    speed = m1Avg.getAvg();
  } else if (motor == 1) {
    m2Avg.reading(speed);
    speed = m2Avg.getAvg();
  } else if (motor == 2) {
    m3Avg.reading(speed);
    speed = m3Avg.getAvg();
  } else if (motor == 3) {
    m4Avg.reading(speed);
    speed = m4Avg.getAvg();
  }
  motorSpeeds[motor] = speed;

  motorPos[motor] = new_pos;
  motorLastReads[motor] = new_read;
  // Serial.println(String(motorSpeeds[motor]) + ";" + String(motorTargetSpeeds[motor]));
  return motorSpeeds[motor];
}


// =====================================================================
//  DRIVE MIXING / HIGH-LEVEL MOTION
// =====================================================================

void setSpeed(int motor, int speed) {
  readSpeed(motor);
  motorTargetSpeeds[motor] = speed;

  if (speed > 0 && motorInputs[motor] < 0) {
    speed = 0;
  } else if (speed < 0 && motorInputs[motor] > 0) {
    speed = 0;
  }

  if (speed == 0) {
    motorInputs[motor] = 0;
    motorSpeeds[motor] = 0;
  } else if (abs(motorTargetSpeeds[motor] - motorSpeeds[motor]) > 2) {
    difference = motorTargetSpeeds[motor] - motorSpeeds[motor];
    difference = round(float(difference) * differential);
    if (motorTargetSpeeds[motor] > 0 && difference < 2 && difference > 0) {
      difference = 2;
    } else if (motorTargetSpeeds[motor] < 0 && difference > -2 && difference < 0) {
      difference = -2;
    }
    motorInputs[motor] = motorInputs[motor] + difference;
  }

  motorInputs[motor] = constrain(motorInputs[motor], -255, 255);
  moveMotor(motor, motorInputs[motor]);
}

// Drives only DEBUG_MOTOR, straight from the left vertical stick (ch2) —
// the same axis that drives "forward" in normal mixing, so stick feel
// while bench-testing is representative of real driving. No other motor's
// pins or sensors are touched.
void debugSingleMotor() {
  setSpeed(DEBUG_MOTOR, ch2);
}

void move() {
  readFutaba();
  // ch1: left horizontal stick
  // ch2: left vertical stick
  // ch3: right vertical stick
  // ch4: right horizontal stick
  // ch5: 2-position switch

  if (SINGLE_MOTOR_DEBUG) {
    debugSingleMotor();
    return;
  }

  if (ch2 != 0) {
    // left vertical stick
    int m1 = ch2;
    int m2 = ch2;
    int m3 = ch2;
    int m4 = ch2;

    if (ch2 > 0) {
      direction = -1;
    } else {
      direction = 1;
    }

    if (ch1 != 0) {
      // forward + some side movement
      m1 = m1 + ch1;
      m2 = m2 + (ch1 * -1);
      m3 = m3 + ch1;
      m4 = m4 + (ch1 * -1);
    }

    if (ch4 != 0) {
      // forward + some rotational movement
      m1 = m1 + (ch4 * -0.5 * direction);
      m2 = m2 + (ch4 * 0.5 * direction);
      m3 = m3 + (ch4 * 0.5 * direction);
      m4 = m4 + (ch4 * -0.5 * direction);
    }

    if (direction == 1) {
      m1 = min(m1, -1);
      m2 = min(m2, -1);
      m3 = min(m3, -1);
      m4 = min(m4, -1);
    } else {
      m1 = max(m1, 1);
      m2 = max(m2, 1);
      m3 = max(m3, 1);
      m4 = max(m4, 1);
    }

    setSpeed(0, m1);
    setSpeed(1, m2);
    setSpeed(2, m3);
    setSpeed(3, m4);

  } else if (ch4 != 0) {
    // right horizontal stick — rotation, plus optional side movement
    int m1 = ch4 * 0.5;
    int m2 = ch4 * -0.5;
    int m3 = ch4 * -0.5;
    int m4 = ch4 * 0.5;

    if (ch1 != 0) {
      m1 = m1 + ch1;
      m2 = m2 + (ch1 * -1);
      m3 = m3 + ch1;
      m4 = m4 + (ch1 * -1);
    }

    setSpeed(0, m1);
    setSpeed(1, m2);
    setSpeed(2, m3);
    setSpeed(3, m4);

  } else if (ch1 != 0) {
    // pure side (strafe) movement
    int m1 = ch1;
    int m2 = ch1 * -1;
    int m3 = ch1;
    int m4 = ch1 * -1;

    setSpeed(0, m1);
    setSpeed(1, m2);
    setSpeed(2, m3);
    setSpeed(3, m4);

  } else {
    setSpeed(0, 0);
    setSpeed(1, 0);
    setSpeed(2, 0);
    setSpeed(3, 0);
  }
}
