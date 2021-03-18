/*!
* @file QuadMotorDriverShield.ino
* @brief QuadMotorDriverShield.ino  Motor control program
*
* Every 2 seconds to control motor positive inversion
*
* @author linfeng(490289303@qq.com)
* @version  V1.0
* @date  2016-4-5
*/

#include <movingAvg.h>

const int Motor1_IN1=5;
const int Motor1_IN2=4;
const int Motor1_Enable=6;

const int Motor2_IN3=8;
const int Motor2_IN4=7;
const int Motor2_Enable=9;

const int Motor3_IN1=11;
const int Motor3_IN2=10;
const int Motor3_Enable=2;

const int Motor4_IN3=13;
const int Motor4_IN4=12;
const int Motor4_Enable=3;

int ch1 = 0;
int ch2 = 0;
int ch3 = 0;
int ch4 = 0;
int ch5 = 0;

bool calibrating = false;
unsigned long calibration_start = 0;
const int dead_center = 50;
int ch1_min = 1283;
int ch1_center = 1492;
int ch1_max = 1720;

int ch2_min = 1061;
int ch2_center = 0;
int ch2_max = 1694;

int ch3_min = 1263;
int ch3_center = 1507;
int ch3_max = 1716;

int ch4_min = 1290;
int ch4_center = 1468;
int ch4_max = 1682;

int ch5_min = 987;
int ch5_max = 1977;

const int max_limit = 180;
const int min_limit = -180;
int direction = 1;


// ch1: left horizontal stick
// ch2: left vertical stick
// ch3: right vertical stick
// ch4: right horizontal stick
// ch5: 2 position switch
const int ch1_pin = 26;
const int ch2_pin = 24;
const int ch3_pin = 28;
const int ch4_pin = 30;
const int ch5_pin = 22;

const int calibration_led = 32;
int calibration_led_state = LOW;
const int calibration_switch = 34;
unsigned long time_keeper = 0;



void setup() {

  pinMode(Motor1_IN1, OUTPUT);
  pinMode(Motor1_IN2, OUTPUT);
  pinMode(Motor1_Enable, OUTPUT);
  
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
  pinMode(calibration_switch, INPUT_PULLUP);
  pinMode(calibration_led, OUTPUT);

  Serial.begin(9600); // Pour a bowl of Serial
//  calibrate();
  digitalWrite(calibration_led, LOW);

}

void checkCalibration() {
  if (digitalRead(calibration_switch) == LOW) {
    digitalWrite(calibration_led, HIGH);
    if (calibrating == false) {
      // push down the switch
      calibration_start = millis();
      calibrating = true;
    } else {
      if (millis() - calibration_start >= 3000) {
        // 3 seconds
        // Actual calibration
        digitalWrite(calibration_led, LOW);
        calibration_start = millis();
        time_keeper = millis();
        movingAvg ch1_avg(10);
        movingAvg ch2_avg(10);
        movingAvg ch3_avg(10);
        movingAvg ch4_avg(10);
        movingAvg ch5_avg(10);
        ch1_avg.begin();
        ch2_avg.begin();
        ch3_avg.begin();
        ch4_avg.begin();
        ch5_avg.begin();
        while(millis() - calibration_start <= 3000) {
          // calibrate center
          ch1_avg.reading(pulseIn(ch1_pin, HIGH));
          ch2_avg.reading(pulseIn(ch2_pin, HIGH));
          ch3_avg.reading(pulseIn(ch3_pin, HIGH));
          ch4_avg.reading(pulseIn(ch4_pin, HIGH));
          ch1_center = ch1_avg.getAvg();
          ch2_center = ch2_avg.getAvg();
          ch3_center = ch3_avg.getAvg();
          ch4_center = ch4_avg.getAvg();
        }
        ch1_min = 3000;
        ch1_max = 0;
        ch2_min = 3000;
        ch2_max = 0;
        ch3_min = 3000;
        ch3_max = 0;
        ch4_min = 3000;
        ch4_max = 0;
        ch5_min = 3000;
        ch5_max = 0;
        ch1_avg.reset();
        ch2_avg.reset();
        ch3_avg.reset();
        ch4_avg.reset();
        ch5_avg.reset();
        time_keeper = millis();
        while (digitalRead(calibration_switch) == HIGH) {
          // calibrate extremes
          if (millis() - time_keeper >= 200) {
            if (calibration_led_state == LOW) {
              calibration_led_state = HIGH;
            } else {
              calibration_led_state = LOW;
            }
            digitalWrite(calibration_led, calibration_led_state);
            time_keeper = millis();
          }
          // calibrate extremes
          ch1_avg.reading(pulseIn(ch1_pin, HIGH));
          ch1_min = min(ch1_avg.getAvg(), ch1_min);
          ch1_max = max(ch1_avg.getAvg(), ch1_max);
          ch2_avg.reading(pulseIn(ch2_pin, HIGH));
          ch2_min = min(ch2_avg.getAvg(), ch2_min);
          ch2_max = max(ch2_avg.getAvg(), ch2_max);
          ch3_avg.reading(pulseIn(ch3_pin, HIGH));
          ch3_min = min(ch3_avg.getAvg(), ch3_min);
          ch3_max = max(ch3_avg.getAvg(), ch3_max);
          ch4_avg.reading(pulseIn(ch4_pin, HIGH));
          ch4_min = min(ch4_avg.getAvg(), ch4_min);
          ch4_max = max(ch4_avg.getAvg(), ch4_max);

          ch5_avg.reading(pulseIn(ch5_pin, HIGH));
          ch5_min = min(ch5_avg.getAvg(), ch5_min);
          ch5_max = max(ch5_avg.getAvg(), ch5_max);
          
//          Serial.println(String(pulseIn(ch5_pin, HIGH)) + ", " + String(ch5_avg.getAvg()) + ", " + String(ch5_min) + ", " + String(ch5_max));
//          Serial.println(String(ch1_min) +", "+ String(ch1_max) +", "+ String(ch2_min) +", "+ String(ch2_max) +", "+ String(ch3_min) +", "+ String(ch3_max) +", "+ String(ch4_min) +", "+ String(ch4_max)+", "+ String(ch5_min) +", "+ String(ch5_max) );
        }
        digitalWrite(calibration_led, LOW);
        calibrating = false;
      }
    }
  } else {
    calibrating = false;
    digitalWrite(calibration_led, LOW);
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
  ch2 = map(pulseIn(ch2_pin, HIGH), ch2_min, ch2_max, min_limit, max_limit);
  if (ch2 <= dead_center && ch2 >= (dead_center * -1)) {
    ch2 = 0;
  } else if (ch2 > dead_center) {
    ch2 = ch2 - dead_center;
  } else {
    ch2 = ch2 + dead_center;
  }
  ch3 = map(pulseIn(ch3_pin, HIGH), ch3_min, ch3_max, min_limit, max_limit);
  if (ch3 <= dead_center && ch3 >= (dead_center * -1)) {
    ch3 = 0;
  } else if (ch3 > dead_center) {
    ch3 = ch3 - dead_center;
  } else {
    ch3 = ch3 + dead_center;
  }
  ch4 = map(pulseIn(ch4_pin, HIGH), ch4_min, ch4_max, max_limit, min_limit);
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
  if (speed == 0) {
     digitalWrite(Motor1_IN1, LOW);
     digitalWrite(Motor1_IN2, LOW);
  } else if (speed > 1) {
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
  if (speed == 0) {
     digitalWrite(Motor2_IN3, LOW);
     digitalWrite(Motor2_IN4, LOW);
  } else if (speed > 1) {
     digitalWrite(Motor2_IN3,HIGH);
     digitalWrite(Motor2_IN4,LOW);
     analogWrite(Motor2_Enable,speed);
  } else {
     digitalWrite(Motor2_IN3,LOW);
     digitalWrite(Motor2_IN4,HIGH);
     analogWrite(Motor2_Enable,speed * -1);  
  }
}

// front left
void MoveMotor3(int speed) {
  speed = max(min(speed, 255), -255);
  if (speed == 0) {
     digitalWrite(Motor3_IN1, LOW);
     digitalWrite(Motor3_IN2, LOW);
  } else if (speed > 1) {
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
  if (speed == 0) {
     digitalWrite(Motor4_IN3, LOW);
     digitalWrite(Motor4_IN4, LOW);
  } else if (speed > 1) {
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
      m1 = m1 + (ch1 * -1);
      m2 = m2 + ch1;
      m3 = m3 + (ch1 * -1);
      m4 = m4 + ch1;
    };
    if (ch4 != 0) {
      // forward + some rotational movement
      m1 = m1 + (ch4 * 0.5 * direction);
      m2 = m2 + (ch4 * -0.5 * direction);
      m3 = m3 + (ch4 * -0.5 * direction);
      m4 = m4 + (ch4 * 0.5 * direction);
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
    MoveMotor1(m1);
    MoveMotor2(m2);
    MoveMotor3(m3);
    MoveMotor4(m4);
    Serial.println(String(m1) +", "+ String(m2) +", "+ String(m3) +", "+ String(m4) +", "+ String(direction));

  } else if (ch4 != 0) {
    int m1 = ch4 * 0.5;
    int m2 = ch4 * -0.5;
    int m3 = ch4 * -0.5;
    int m4 = ch4 * 0.5;
    // right horizontal stick
    if (ch1 != 0) {
      // rotational + side movement
      m1 = m1 + (ch1 * -1);
      m2 = m2 + ch1;
      m3 = m3 + (ch1 * -1);
      m4 = m4 + ch1;
    }
    MoveMotor1(m1);
    MoveMotor2(m2);
    MoveMotor3(m3);
    MoveMotor4(m4);
    Serial.println(String(m1) +", "+ String(m2) +", "+ String(m3) +", "+ String(m4) +", "+ String(direction));
  } else if (ch1 != 0) {
    int m1 = ch1 * -1;
    int m2 = ch1;
    int m3 = ch1 * -1;
    int m4 = ch1;
    // pure side movement
    MoveMotor1(m1); 
    MoveMotor2(m2); 
    MoveMotor3(m3); 
    MoveMotor4(m4); 
    Serial.println(String(m1) +", "+ String(m2) +", "+ String(m3) +", "+ String(m4) +", "+ String(direction));
  } else {
    MoveMotor1(0);
    MoveMotor2(0);
    MoveMotor3(0);
    MoveMotor4(0);
  }

}
void loop() {
//  checkCalibration();
//readFutaba();
  move();
}
