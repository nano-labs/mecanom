#include <movingAvg.h>

// Wheel speeds returned by mecanumDrive(), in the same [min_limit,max_limit]
// domain as the RC channels — ready to pass straight into setSpeed().
// Defined here, before anything else in the file (including the comment
// block below), because Arduino auto-generates a forward declaration for
// mecanumDrive() and inserts it near the very top of the translation unit.
// If this struct were defined further down (as it originally was, right
// before mecanumDrive() itself), that auto-generated prototype would
// reference WheelSpeeds before the compiler had seen what it is —
// "'WheelSpeeds' does not name a type". Defining it first avoids that
// regardless of exactly where Arduino decides to insert prototypes.
struct WheelSpeeds {
  double frontLeft;
  double frontRight;
  double rearLeft;
  double rearRight;
};

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
//    - move()'s old ad-hoc ch1/ch2/ch4 mixing branches are replaced by
//      mecanumDrive(x, y, yaw, reservePercent), a verified port of the
//      Python/JS mecanum_drive() developed and tested in the companion
//      simulator. ch5 now selects the reserve percentage (BOOST/SAFE)
//      instead of its old job of halving the raw stick inputs.
//    - Added an RC failsafe: readChannel()/readFutaba() now detect a
//      pulseIn() timeout (signal loss) per channel, and move() forces a
//      hard stop instead of trusting whatever a timeout's zero reading
//      happens to map to.
//    - Replaced setSpeed()'s manual ramp (difference * differential,
//      driven by ch3) with a real PI controller against the sensor-
//      measured speed. Gains (PID_KP/PID_KI) are untuned placeholders —
//      start low and tune by hand on real hardware. ch3 is no longer
//      used for anything.
//    - setup() previously only initialized motorPos[0]/motorLastReads[0]
//      (regardless of DEBUG_MOTOR) using a *different* raw-to-position
//      scale (0-850->0-360) than readSpeed()'s (0-900->0-3600) — both
//      bugs fixed by checkSensorAtStartup(), which now seeds whichever
//      motor(s) are actually expected to be connected, on the same
//      scale readSpeed() uses, and warns over serial if a sensor
//      doesn't respond at all.
// =====================================================================


// ------------------------- Motor pin assignments -------------------------
// Index: 0 = back right, 1 = back left, 2 = front left, 3 = front right
const int motorPinA[4]    = {6, 3, 12, 9};   // IN1 (0,2) / IN3 (1,3)
const int motorPinB[4]    = {5, 4, 11, 10};  // IN2 (0,2) / IN4 (1,3)
const int motorEnable[4]  = {7, 2, 13, 8};
const int motorPosPins[4] = {32, 34, 36, 50}; // angular position sensors

// ------------------------- RC receiver pins -------------------------
// ch1: left horizontal stick
// ch2: left vertical stick
// ch3: right vertical stick   (currently unused by drive mixing)
// ch4: right horizontal stick
// ch5: 2-position switch
const int ch1_pin = 30;
// const int ch2_pin = 28;
const int ch2_pin = 26;
// const int ch3_pin = 26;
const int ch3_pin = 28;
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
const unsigned long PULSE_TIMEOUT = 50000UL; // 50ms; prevents pulseIn() stalling up to 1s on signal loss

// Worst case for the mecanum mixer below: |x|+|y|+|yaw| = 300 when all three
// axes are simultaneously maxed, requiring a coefficient of 3. Reserving this
// much guarantees zero clipping/distortion for any combined command, at the
// cost of capping single-axis full-throttle to 1/3 of max wheel speed.
// Selected by ch5 (2-position switch): SAFE position reserves this amount,
// BOOST position reserves nothing.
const double RESERVE_SAFE_PERCENT = 200.0 / 3.0; // ~66.67%

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

// Set once in setup() after Serial.begin(); print() only writes when this
// is true, so calls are silently skipped if the serial connection never
// came up (e.g. no host attached) instead of blocking/wasting cycles.
bool serialReady = false;

// ------------------------- RC failsafe -------------------------
// Updated every readFutaba() call: true only if every channel's pulseIn()
// got a real pulse this cycle. move() forces all motors to a hard stop
// when this is false, rather than trusting whatever a pulseIn() timeout's
// zero reading happens to map to.
bool rcSignalValid = true;

// ------------------------- Speed PI controller -------------------------
// NOTE: these gains are untuned placeholders — they've never run on the
// real hardware. Start low and increase gradually while watching for
// oscillation/overshoot, the standard way to tune a PI loop by hand.
const double PID_KP = 0.5;  // proportional gain
const double PID_KI = 0.8;  // integral gain, per second
const double PID_INTEGRAL_MAX = 255.0 / PID_KI; // anti-windup: caps the integral term's own contribution to a full-scale PWM swing

double motorIntegral[4] = {0.0, 0.0, 0.0, 0.0};
unsigned long motorLastDt[4] = {1, 1, 1, 1}; // ms; interval readSpeed() measured on its last call, per motor


// =====================================================================
//  SETUP
// =====================================================================
// Reads one motor's position sensor once at startup and seeds motorPos/
// motorLastReads from it, warning if the sensor doesn't respond at all.
// Uses the same 0-900 raw -> 0-3600 mapping as readSpeed(), so the very
// first reading is on the same scale as every subsequent one.
void checkSensorAtStartup(int idx) {
  unsigned long raw = pulseIn(motorPosPins[idx], HIGH, PULSE_TIMEOUT);
  if (raw == 0) {
    print("WARNING: motor " + String(idx) + " position sensor not responding at startup.");
  } else {
    motorPos[idx] = constrain(map(raw, 0, 900, 0, 3600), 0, 3600);
  }
  motorLastReads[idx] = millis();
}

void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(motorPinA[i], OUTPUT);
    pinMode(motorPinB[i], OUTPUT);
    pinMode(motorEnable[i], OUTPUT);
  }

  pinMode(ch1_pin, INPUT);
  pinMode(ch2_pin, INPUT);
  pinMode(ch3_pin, INPUT);
  pinMode(ch4_pin, INPUT);
  pinMode(ch5_pin, INPUT);

  Serial.begin(57600); // Pour a bowl of Serial
  delay(3000);
  serialReady = (bool)Serial; // true once the connection is actually up

  // Only self-check the sensor(s) actually expected to be wired up right
  // now — checking all four during SINGLE_MOTOR_DEBUG would just spam
  // false "not responding" warnings for the intentionally-disconnected ones.
  if (SINGLE_MOTOR_DEBUG) {
    checkSensorAtStartup(DEBUG_MOTOR);
  } else {
    for (int i = 0; i < 4; i++) {
      checkSensorAtStartup(i);
    }
  }

  m1Avg.begin();
  m2Avg.begin();
  m3Avg.begin();
  m4Avg.begin();
}


// =====================================================================
//  SERIAL OUTPUT
// =====================================================================

// Only writes to Serial if the connection was confirmed up in setup();
// use this instead of calling Serial.println() directly anywhere else.
void print(const String &msg) {
  if (serialReady) {
    Serial.println(msg);
  }
}


// =====================================================================
//  MAIN LOOP
// =====================================================================
void loop() {
  move();
  int motor = SINGLE_MOTOR_DEBUG ? DEBUG_MOTOR : 0;
  print("Motor:" + String(motor) +
        ",Speed:" + String(motorSpeeds[motor]) +
        ",Output:" + String(motorInputs[motor]) +
        ",Target:" + String(motorTargetSpeeds[motor]) +
        ",RC:" + String(rcSignalValid));
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
// applies the deadband/rebase used for the four stick axes. If validOut is
// given, it's set to false when pulseIn() timed out (raw reading of 0),
// which readFutaba() uses to detect signal loss.
int readChannel(int pin, int rawMin, int rawMax, int outLow, int outHigh, bool deadband, bool *validOut = nullptr) {
  unsigned long raw = pulseIn(pin, HIGH, PULSE_TIMEOUT);
  if (validOut) *validOut = (raw != 0);
  int value = map(raw, rawMin, rawMax, outLow, outHigh);
  value = constrain(value, min(outLow, outHigh), max(outLow, outHigh));
  if (deadband) {
    value = applyDeadband(value, dead_center);
  }
  return value;
}

void readFutaba() {
  // ch1: left horizontal stick
  // ch2: left vertical stick
  // ch3: right vertical stick   (currently unused by drive mixing — see note below)
  // ch4: right horizontal stick
  // ch5: 2-position switch

  bool ch1Valid, ch2Valid, ch3Valid, ch4Valid, ch5Valid;

  ch1 = readChannel(ch1_pin, ch1_min, ch1_max, min_limit, max_limit, true, &ch1Valid);
  ch2 = readChannel(ch2_pin, ch2_min, ch2_max, max_limit, min_limit, true, &ch2Valid); // inverted
  ch3 = readChannel(ch3_pin, ch3_min, ch3_max, max_limit, min_limit, true, &ch3Valid); // inverted

  // ch3 previously drove a manual ramp-rate ("differential") dial; that's
  // superseded now that setSpeed() uses a real PI controller instead of a
  // manual ramp. ch3 is still read (and still counts toward the failsafe
  // check below, since receiver channels typically drop out together) but
  // isn't used for anything yet — available for e.g. live gain tuning later.

  ch4 = readChannel(ch4_pin, ch4_min, ch4_max, min_limit, max_limit, true, &ch4Valid);
  ch5 = readChannel(ch5_pin, ch5_min, ch5_max, min_limit, max_limit, false, &ch5Valid); // switch, no deadband

  bool signalValidNow = ch1Valid && ch2Valid && ch3Valid && ch4Valid && ch5Valid;

  if (signalValidNow && !rcSignalValid) {
    print("RC signal restored.");
  } else if (!signalValidNow && rcSignalValid) {
    print("RC signal lost — stopping.");
  }
  rcSignalValid = signalValidNow;

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
    print(String(ch1_avg.getAvg()) + ", " + String(ch2_avg.getAvg()) + ", " +
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

  unsigned long dt = new_read - motorLastReads[motor];
  if (dt == 0) dt = 1; // guard against divide-by-zero if called twice in the same millisecond
  motorLastDt[motor] = dt; // available to setSpeed()'s PI controller

  // True wraparound delta — no dependency on the commanded target, so this
  // reflects what the wheel is actually doing even while stopping, stalled,
  // or being pushed opposite to the commanded direction. Assumes motor
  // wiring/polarity is set up so that a positive command moves every motor
  // "forward" (i.e. increases raw position) — adjust wiring, not software,
  // if a motor reads backwards.
  int delta = circularDelta((long)motorPos[motor], (long)new_pos, 3600);

  int speed;
  if (abs(delta) <= POSITION_NOISE_DEADBAND) {
    // Below the sensor's noise floor — treat as stationary rather than
    // computing a spurious instantaneous speed from jitter.
    speed = 0;
  } else {
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

  // Enforce a full stop before reversing direction, rather than snapping
  // straight from one polarity's PWM output to the opposite one.
  if (speed > 0 && motorInputs[motor] < 0) {
    speed = 0;
  } else if (speed < 0 && motorInputs[motor] > 0) {
    speed = 0;
  }

  if (speed == 0) {
    motorInputs[motor] = 0;
    motorIntegral[motor] = 0.0; // avoid windup while stopped
  } else {
    double dtSeconds = motorLastDt[motor] / 1000.0;
    double error = (double)speed - motorSpeeds[motor];

    motorIntegral[motor] += error * dtSeconds;
    motorIntegral[motor] = constrain(motorIntegral[motor], -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);

    double output = PID_KP * error + PID_KI * motorIntegral[motor];
    motorInputs[motor] = (int)constrain(output, -255.0, 255.0);
  }

  moveMotor(motor, motorInputs[motor]);
}

// Direct port of the verified Python mecanum_drive(x, y, yaw, reserve_percent),
// adapted to operate directly in the RC channels' native [min_limit,max_limit]
// range instead of [-100,100], so callers can pass ch1/ch2/ch4 straight through
// with no rescaling. reservePercent is still a plain 0-100 percentage of that
// range. reservePercent=0 reproduces the un-reserved behavior exactly.
WheelSpeeds mecanumDrive(double x, double y, double yaw, double reservePercent) {
  double scale = (100.0 - reservePercent) / 100.0;
  x *= scale;
  y *= scale;
  yaw *= scale;

  double frontRight = x + y + yaw;
  double frontLeft  = x - y - yaw;
  double backRight  = x - y + yaw;
  double backLeft   = x + y - yaw;

  double proportionalCoefficient =
      max(max(abs(frontRight), abs(frontLeft)), max(abs(backRight), abs(backLeft))) / (double)max_limit;

  if (proportionalCoefficient > 1.0) {
    frontRight /= proportionalCoefficient;
    frontLeft  /= proportionalCoefficient;
    backRight  /= proportionalCoefficient;
    backLeft   /= proportionalCoefficient;
  }

  WheelSpeeds w;
  w.frontLeft  = frontLeft;
  w.frontRight = frontRight;
  w.rearLeft   = backLeft;
  w.rearRight  = backRight;
  return w;
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
  // ch1: left horizontal stick   -> y   (assumed positive = right strafe;
  //                                       flip sign below if this reads
  //                                       backwards on the bench)
  // ch2: left vertical stick     -> x   (forward; already sign-corrected
  //                                       in readFutaba's inverted mapping)
  // ch3: right vertical stick    -> currently unused by mixing
  // ch4: right horizontal stick  -> yaw (assumed positive = clockwise;
  //                                       flip sign below if reversed)
  // ch5: 2-position switch       -> reserve select (BOOST / SAFE)

  if (SINGLE_MOTOR_DEBUG) {
    if (!rcSignalValid) {
      setSpeed(DEBUG_MOTOR, 0); // failsafe, scoped to the one motor actually wired up
    } else {
      debugSingleMotor();
    }
    return;
  }

  if (!rcSignalValid) {
    // Signal lost — force a hard stop rather than trusting whatever a
    // pulseIn() timeout's zero reading happens to map to.
    setSpeed(0, 0);
    setSpeed(1, 0);
    setSpeed(2, 0);
    setSpeed(3, 0);
    return;
  }

  double reservePercent = (ch5 > 0) ? RESERVE_SAFE_PERCENT : 0.0;

  // ch1/ch2/ch4 are already in mecanumDrive()'s native [min_limit,max_limit]
  // domain, so they're passed straight through with no rescaling.
  WheelSpeeds w = mecanumDrive(ch2, ch1, ch4, reservePercent);

  setSpeed(0, (int)w.rearRight);  // back right
  setSpeed(1, (int)w.rearLeft);   // back left
  setSpeed(2, (int)w.frontLeft);  // front left
  setSpeed(3, (int)w.frontRight); // front right
}
