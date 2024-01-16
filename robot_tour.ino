/* WIRING:
Digital Pin 2: Left encoder A
Digital Pin 4: Left encoder B
Digital Pin 3: Right encoder A
Digital Pin 5: Right encoder B
Digital Pin 3: Right encoder
Motor shield M2: Left motor
Motor shield M3: Right motor
Digital Pin 7: Button
Digital Pin 8: LED
Digital Pin 8: Reset

SDA/SCL: IMU
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_BNO055.h>

// Constants
const float TICKS_PER_CM = 11; // TICKS
const float TRACK_WIDTH = 7; // CM
const float SPEED = 140;
const float P = 75;
const float I = 0;
const float D = 20;

const float DEADBAND = 60; // Deadband of motors
const float MAXANG = 140; // Max w value

float START_HEADING = 0; // RADIANS

// Localization
float heading = 0; // RADIANS
float x = 0; // CM
float y = 0; // CM

// Path
typedef struct Point {
  float x;
  float y;
} Point;
Point path[] = {
  {0, 0},
  {75, 0},
  {75, 75},
  {0, 75},
  {0, 0},
};

/* Main code */
void setup() {
  Serial.begin(9600);
  delay(100);

  setupMotors();
  setupLocalization();
  waitForStart();
}

bool done = false;
void loop() {
  if (digitalRead(9) == LOW) {
    stopMotors();
    digitalWrite(8, LOW);
    done = true;
  }

  if (!done) {
    loopLocalization();
    loopPid();
  }

  delay(20);
}
/* Main code */

/* Localization code */
volatile int lTicks = 0;
volatile int rTicks = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
void setupLocalization() {
  attachInterrupt(0, incrementL, RISING);
  attachInterrupt(1, incrementR, RISING);
  bno.begin();
  bno.setExtCrystalUse(true);
}

void incrementL() {
  bool dir = digitalRead(4);
  if (dir) {
    lTicks++;
  } else {
    lTicks--;
  }
}

void incrementR() {
  bool dir = digitalRead(5);
  if (dir) {
    rTicks--;
  } else {
    rTicks++;
  }
}

unsigned long lastTime;
void resetLocalization() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float offset = 2*PI - euler.x() * DEG_TO_RAD;
  START_HEADING -= offset;
  lTicks = 0;
  rTicks = 0;
  lastTime = millis();
}

float prevHeading = START_HEADING;
float dT;
void loopLocalization() {
  float l = ((float)lTicks)/(TICKS_PER_CM);
  float r = ((float)rTicks)/(TICKS_PER_CM);

  unsigned long time = millis();
  dT = (float)(time - lastTime)/1000;
  lastTime = time;

  // Heading
  /*float dHeading = (r - l)/TRACK_WIDTH; // Encoder-based orientation
  heading += dHeading;*/

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // IMU-based orientation
  heading = 2*PI - euler.x() * DEG_TO_RAD + START_HEADING;
  float dHeading = heading - prevHeading;
  prevHeading = heading;
  if (dHeading == 0) {
    dHeading = (millis() % 2 == 0 ? 0.00001 : -0.00001);
  }

  // X and Y change
  float z = ((2*l)/dHeading + TRACK_WIDTH)*sin(dHeading/2);
  Serial.print("l:");
  Serial.print(l);
  Serial.print(",dHeading:");
  Serial.println(dHeading);
  float dx = z*cos(heading);
  float dy = z*sin(heading);

  x += dx;
  y += dy;

  // Reset ticks because this operates on delta
  lTicks = 0;
  rTicks = 0;
}
/* Localization code */

/* Motor control code */
Adafruit_DCMotor* leftMotor;
Adafruit_DCMotor* rightMotor;
void setupMotors() {
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  leftMotor = AFMS.getMotor(3);
  rightMotor = AFMS.getMotor(4);
  AFMS.begin();
}

// Powers from 0 to 255
void powerMotors(int lPower, int rPower) {
  leftMotor->setSpeed(abs(lPower));
  rightMotor->setSpeed(abs(rPower));
  if (lPower < 0) {
    leftMotor->run(BACKWARD);
  } else {
    leftMotor->run(FORWARD);
  }
  if (rPower < 0) {
    rightMotor->run(BACKWARD);
  } else {
    rightMotor->run(FORWARD);
  }
}

void stopMotors() {
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}
/* Motor control code */

/* Path following */
int pathlen() {
  return sizeof(path)/sizeof(Point);
}
// Heading from a to b
float calcH(Point a, Point b) {
  float ret = atan2(b.y - a.y, b.x - a.x);
  if (ret < 0) {
    ret += 2*PI;
  }
  return ret;
}

int pathPoint = 0;
float iV = 0;
float pErr = 0; // Prev err
void loopPid() {
  Point goal = path[pathPoint];
  if (abs(goal.x - x) < 5 && abs(goal.y - y) < 5) { // Check if next point
    if (pathPoint < pathlen()-1) {
      pathPoint++;
      iV = 0; // Stop integral windup
      stopMotors();
      delay(100);
    } else {
      done = true;
      digitalWrite(8, LOW);
      stopMotors();
      return;
    }
  }

  float err = atan2((double)(goal.y - y), (double)(goal.x - x)) - heading;
  if (err > PI) {
    err -= 2*PI;
  } else if (err < -1 * PI) {
    err += 2*PI;
  }
  iV += err * dT;
  float dV = (err - pErr)/dT;
  pErr = err;
  float w = err * P + iV * I + dV * D;
  if (abs(w) > MAXANG) { // Limit max speed
    if (w < 0) {
      w = -MAXANG;
    } else {
      w = MAXANG;
    }
  }

  /*Serial.print("heading:");
  Serial.print(heading);
  Serial.print(",atan:");
  Serial.print(atan2((double)(goal.y - y), (double)(goal.x - x)));
  Serial.print(",ey:");
  Serial.print(goal.y - y);
  Serial.print(",ex:");
  Serial.print(goal.x - x);
  Serial.print(",w:");
  Serial.print(w);
  Serial.print(",err:");
  Serial.println(err);*/

  if (abs(err) > PI/20) { // Turn in place until <9deg of error
    powerMotors(w + DEADBAND, -1 * w - DEADBAND); // 50s help it get out of deadband
  } else {
    powerMotors(SPEED + w, SPEED - w);
  }
}
/* Path following */

/* Button input code */
void waitForStart() {
  pinMode(7, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  bool pressed = false;
  while (true) {
    int buttonDown = digitalRead(7);
    if (buttonDown == LOW) {
      digitalWrite(8, LOW);
      pressed = true;
    }
    if (buttonDown == HIGH && pressed) { // Start when released
      break;
    }
    delay(50);
  }
  resetLocalization();
  digitalWrite(8, HIGH);
}
/* Button input code */