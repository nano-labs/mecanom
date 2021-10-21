
#include <movingAvg.h>
#include <PID_v1.h>

const int Motor1_IN1=6;
const int Motor1_IN2=5;
const int Motor1_Enable=7;

const int Motor2_IN3=3;
const int Motor2_IN4=4;
const int Motor2_Enable=2;

const int Motor3_IN1=12;
const int Motor3_IN2=11;
const int Motor3_Enable=13;

const int Motor4_IN3=9;
const int Motor4_IN4=10;
const int Motor4_Enable=8;

//movingAvg motorSpeeds[4] = {movingAvg(3), movingAvg(3), movingAvg(3), movingAvg(3)};
double motorSpeeds[4] = {0.0, 0.0, 0.0, 0.0};
int motorInputs[4] = {0, 0, 0, 0};
int motorTargetSpeeds[4] = {0, 0, 0, 0};
unsigned long motorPos[4] = {0, 0, 0, 0};
int motorPosPins[4] = {32, 34, 36, 50};
unsigned long motorLastReads[4] = {0, 0, 0, 0};

movingAvg m1Avg(3);
movingAvg m2Avg(3);
movingAvg m3Avg(3);
movingAvg m4Avg(3);

int ch1 = 0;
int ch2 = 0;
int ch3 = 0;
int ch4 = 0;
int ch5 = 0;

bool calibrating = false;
unsigned long calibration_start = 0;
const int dead_center = 50;
int ch1_min = 1035;
int ch1_max = 1880;

int ch2_min = 1045;
int ch2_max = 1920;

int ch3_min = 1055;
int ch3_max = 1895;

int ch4_min = 1100;
int ch4_max = 1880;

int ch5_min = 987;
int ch5_max = 1960;

const int max_limit = 250;
const int min_limit = -250;
int direction = 1;

double differential = 0.0;
int difference = 0;

// ch1: left horizontal stick
// ch2: left vertical stick
// ch3: right vertical stick
// ch4: right horizontal stick
// ch5: 2 position switch
const int ch1_pin = 30;
const int ch2_pin = 28;
const int ch3_pin = 26;
const int ch4_pin = 24;
const int ch5_pin = 22;


void setup() {

  pinMode(Motor1_IN1, OUTPUT);
  pinMode(Motor1_IN2, OUTPUT);
  pinMode(Motor1_Enable, OUTPUT);
  motorPos[0] = map(pulseIn(motorPosPins[0], HIGH), 0, 850, 0, 360);
  motorLastReads[0] = millis();
  
  pinMode(Motor2_IN4, OUTPUT);
  pinMode(Motor2_IN3, OUTPUT);
  pinMode(Motor2_Enable, OUTPUT);

  pinMode(Motor3_IN1, OUTPUT);
  pinMode(Motor3_IN2, OUTPUT);
  pinMode(Motor3_Enable, OUTPUT);
  
  pinMode(Motor4_IN4, OUTPUT);
  pinMode(Motor4_IN3, OUTPUT);
  pinMode(Motor4_Enable, OUTPUT);

  pinMode(ch1_pin, INPUT);
  pinMode(ch2_pin, INPUT);
  pinMode(ch3_pin, INPUT);
  pinMode(ch4_pin, INPUT);
  pinMode(ch5_pin, INPUT);

  Serial.begin(57600); // Pour a bowl of Serial
  delay(3000);
//  calibrate();
//  motorSpeeds[0] = (3);
//  motorSpeeds[1] = (3);
//  motorSpeeds[2] = (3);
//  motorSpeeds[3] = (3);
//  motorSpeeds[0].begin();
//  motorSpeeds[1].begin();
//  motorSpeeds[2].begin();
//  motorSpeeds[3].begin();
  m1Avg.begin();
  m2Avg.begin();
  m3Avg.begin();
  m4Avg.begin();
}

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
    ch1_avg.reading(pulseIn(ch1_pin, HIGH));
    ch2_avg.reading(pulseIn(ch2_pin, HIGH));
    ch3_avg.reading(pulseIn(ch3_pin, HIGH));
    ch4_avg.reading(pulseIn(ch4_pin, HIGH));
    ch5_avg.reading(pulseIn(ch5_pin, HIGH));
    Serial.println(String(ch1_avg.getAvg()) + ", " + String(ch2_avg.getAvg()) + ", " + String(ch3_avg.getAvg()) + ", " + String(ch4_avg.getAvg()) + ", " + String(ch5_avg.getAvg()));
  }
}

void readFutaba() {
  // ch1: left horizontal stick
  // ch2: left vertical stick
  // ch3: right vertical stick
  // ch4: right horizontal stick
  // ch5: 2 position switch
  
  ch1 = map(pulseIn(ch1_pin, HIGH), ch1_min, ch1_max, min_limit, max_limit);
  if (ch1 <= dead_center && ch1 >= (dead_center * -1)) {
    ch1 = 0;
  } else if (ch1 > dead_center) {
    ch1 = ch1 - dead_center;
  } else {
    ch1 = ch1 + dead_center;
  }
  ch2 = map(pulseIn(ch2_pin, HIGH), ch2_min, ch2_max, max_limit, min_limit);
  if (ch2 <= dead_center && ch2 >= (dead_center * -1)) {
    ch2 = 0;
  } else if (ch2 > dead_center) {
    ch2 = ch2 - dead_center;
  } else {
    ch2 = ch2 + dead_center;
  }
  ch3 = map(pulseIn(ch3_pin, HIGH), ch3_min, ch3_max, max_limit, min_limit);
  if (ch3 <= dead_center && ch3 >= (dead_center * -1)) {
    ch3 = 0;
  } else if (ch3 > dead_center) {
    ch3 = ch3 - dead_center;
  } else {
    ch3 = ch3 + dead_center;
  }
  // used for differential here
  differential = map((float)ch3, (float)max_limit, (float)min_limit, 0.0, 100.0) / 200.0;
  //
  ch4 = map(pulseIn(ch4_pin, HIGH), ch4_min, ch4_max, min_limit, max_limit);
  if (ch4 <= dead_center && ch4 >= (dead_center * -1)) {
    ch4 = 0;
  } else if (ch4 > dead_center) {
    ch4 = ch4 - dead_center;
  } else {
    ch4 = ch4 + dead_center;
  }
  ch5 = map(pulseIn(ch5_pin, HIGH), ch5_min, ch5_max, min_limit, max_limit);

  ch1 = max(min(ch1, max_limit), min_limit);
  ch2 = max(min(ch2, max_limit), min_limit);
  ch3 = max(min(ch3, max_limit), min_limit);
  ch4 = max(min(ch4, max_limit), min_limit);
  ch5 = max(min(ch5, max_limit), min_limit);
  if (ch5 > 0) {
    ch1 = ch1 * 0.5;
    ch2 = ch2 * 0.5;
    ch3 = ch3 * 0.5;
    ch4 = ch4 * 0.5;
  }

//  Serial.println(String(ch1) +", "+ String(ch2) +", "+ String(ch3) +", "+ String(ch4) +", "+ String(ch5));
//  Serial.println(String(ch5));
//  delay(500);
}

// back right
void MoveMotor1(int speed) {
  speed = max(min(speed, 255), -255);

//  if (speed == 0) {
//     digitalWrite(Motor1_IN1, LOW);
//     digitalWrite(Motor1_IN2, LOW);
  if (speed >= 0) {
     digitalWrite(Motor1_IN1,HIGH);
     digitalWrite(Motor1_IN2,LOW);
     analogWrite(Motor1_Enable,speed);
  } else {
     digitalWrite(Motor1_IN1,LOW);
     digitalWrite(Motor1_IN2,HIGH);
     analogWrite(Motor1_Enable,speed * -1);  
  }
}

// back left
void MoveMotor2(int speed) {
  speed = max(min(speed, 255), -255);

//  if (speed == 0) {
//     digitalWrite(Motor2_IN3, LOW);
//     digitalWrite(Motor2_IN4, LOW);
  if (speed >= 0) {
     analogWrite(Motor2_Enable,speed);
     digitalWrite(Motor2_IN3,HIGH);
     digitalWrite(Motor2_IN4,LOW);
  } else {
     analogWrite(Motor2_Enable,speed * -1);  
     digitalWrite(Motor2_IN3,LOW);
     digitalWrite(Motor2_IN4,HIGH);
  }
}

// front left
void MoveMotor3(int speed) {
  speed = max(min(speed, 255), -255);

//  if (speed == 0) {
//     digitalWrite(Motor3_IN1, LOW);
//     digitalWrite(Motor3_IN2, LOW);
  if (speed >= 0) {
     digitalWrite(Motor3_IN1,HIGH);
     digitalWrite(Motor3_IN2,LOW);
     analogWrite(Motor3_Enable,speed);
  } else {
     digitalWrite(Motor3_IN1,LOW);
     digitalWrite(Motor3_IN2,HIGH);
     analogWrite(Motor3_Enable,speed * -1);  
  }
}

// front right
void MoveMotor4(int speed) {
  speed = max(min(speed, 255), -255);

//  if (speed == 0) {
//     digitalWrite(Motor4_IN3, LOW);
//     digitalWrite(Motor4_IN4, LOW);
//  } else 
  if (speed >= 0) {
     digitalWrite(Motor4_IN3,HIGH);
     digitalWrite(Motor4_IN4,LOW);
     analogWrite(Motor4_Enable,speed);
  } else {
     digitalWrite(Motor4_IN3,LOW);
     digitalWrite(Motor4_IN4,HIGH);
     analogWrite(Motor4_Enable,(speed)* -1);  
  }
}
void move() {
  readFutaba();
  // ch1: left horizontal stick
  // ch2: left vertical stick
  // ch3: right vertical stick
  // ch4: right horizontal stick
  // ch5: 2 position switch

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
      // forward + some side movements
      m1 = m1 + ch1;
      m2 = m2 + (ch1 * -1);
      m3 = m3 + ch1;
      m4 = m4 + (ch1 * -1);
    };
    if (ch4 != 0) {
      // forward + some rotational movement
      m1 = m1 + (ch4 * -0.5 * direction);
      m2 = m2 + (ch4 * 0.5 * direction);
      m3 = m3 + (ch4 * 0.5 * direction);
      m4 = m4 + (ch4 * -0.5 * direction);
    };
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
//    Serial.println(String(m1) +", "+ String(m2) +", "+ String(m3) +", "+ String(m4) +", "+ String(direction));

  } else if (ch4 != 0) {
    int m1 = ch4 * 0.5;
    int m2 = ch4 * -0.5;
    int m3 = ch4 * -0.5;
    int m4 = ch4 * 0.5;
    // right horizontal stick
    if (ch1 != 0) {
      // rotational + side movement
      m1 = m1 + ch1;
      m2 = m2 + (ch1 * -1);
      m3 = m3 + ch1;
      m4 = m4 + (ch1 * -1);
    }
    setSpeed(0, m1);
    setSpeed(1, m2);
    setSpeed(2, m3);
    setSpeed(3, m4);
//    Serial.println(String(m1) +", "+ String(m2) +", "+ String(m3) +", "+ String(m4) +", "+ String(direction));
  } else if (ch1 != 0) {
    int m1 = ch1;
    int m2 = ch1 * -1;
    int m3 = ch1;
    int m4 = ch1 * -1;
    // pure side movement
    setSpeed(0, m1);
    setSpeed(1, m2);
    setSpeed(2, m3);
    setSpeed(3, m4);
//    Serial.println(String(m1) +", "+ String(m2) +", "+ String(m3) +", "+ String(m4) +", "+ String(direction));
  } else {
    setSpeed(0, 0);
    setSpeed(1, 0);
    setSpeed(2, 0);
    setSpeed(3, 0);
  }

}
double readSpeed(int motor) {
    unsigned long new_pos = min(max(map(pulseIn(motorPosPins[motor], HIGH), 0, 900, 0, 3600), 0), 3600);
    unsigned long new_read = millis();
    int delta = 0;
    int target = motorTargetSpeeds[motor];
    if (motor == 0 || motor == 3) {
      if (target > 0) {
        if (new_pos >= motorPos[motor]) {
          delta = new_pos - motorPos[motor];
        } else {
          delta = new_pos + (3600 - motorPos[motor]);
        }
      } else if (target < 0) {
        if (new_pos < motorPos[motor]) {
          delta = (motorPos[motor] - new_pos) * -1;
        } else {
          delta = (motorPos[motor] + (3600 - new_pos)) * -1;
        }
      }
    } else {
      if (target < 0) {
        if (new_pos >= motorPos[motor]) {
          delta = (new_pos - motorPos[motor]) * -1;
        } else {
          delta = (new_pos + (3600 - motorPos[motor])) * -1;
        }
      } else if (target > 0) {
        if (new_pos < motorPos[motor]) {
          delta = motorPos[motor] - new_pos;
        } else {
          delta = motorPos[motor] + (3600 - new_pos);
        }
      }
    }
    if (delta != 0 && delta < 2000 && delta > -2000) {
      int speed = map((abs(delta) * 10) / (new_read - motorLastReads[motor]), -140, 140, -255, 255);
      if (delta < 0) {
        speed = speed * -1;  
      }
      speed = min(max(speed, -255), 255);
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
      };
      motorSpeeds[motor] = speed;
    }
    motorPos[motor] = new_pos;
    motorLastReads[motor] = new_read;
//    Serial.println(String(motorSpeeds[motor]) + ";" + String(target));
    return motorSpeeds[motor];
}

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
  motorInputs[motor] = min(max(motorInputs[motor], -255), 255);

  if (motor == 0) {
    MoveMotor1(motorInputs[motor]);
  } else if (motor == 1) {
    MoveMotor2(motorInputs[motor]);
  } else if (motor == 2) {
    MoveMotor3(motorInputs[motor]);
  } else if (motor == 3) {
    MoveMotor4(motorInputs[motor]);
  }
}

void loop() {
  move();
  int motor = 1;
  Serial.println(String(motorSpeeds[motor]) + "," + String(motorInputs[motor]) + "," + String(motorTargetSpeeds[motor]) + "," + String(differential));
}
