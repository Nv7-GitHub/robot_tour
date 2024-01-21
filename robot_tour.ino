/* WIRING:
Digital Pin 2: Left encoder A
Digital Pin 4: Left encoder B
Digital Pin 3: Right encoder A
Digital Pin 5: Right encoder B
Digital Pin 3: Right encoder
Motor shield M3: Left motor
Motor shield M4: Right motor
Digital Pin 7: Button
Digital Pin 8: LED
Digital Pin 9: Reset

SDA/SCL: IMU
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_BNO055.h>

// Constants
const float TICKS_PER_CM = 11.6; // TICKS
const float TRACK_WIDTH = 16; // CM
const float SPEED = 60;
const float P = 70;
const float I = 0;
const float D = 20;

const float MAXANG = 40; // Max w value
const float DEADBAND = 20;

// Path
const float START_HEADING = PI; // RADIANS
typedef struct Point {
  float x;
  float y;
} Point;
Point path[] = {
  {100, -75},
  {25, -75},
  {25, -25},
  {75, -25},
  // GO THROUGH GATE 1
  {75, 75},
  {25, 75},
  {25, 25},
  {-25, 25},
  {-25, -25},
  {-75, -25},
  {-75, -75},
  // GO THROUGH GATE 2
  {-74, -25},
  {-25, -25},
  {-25, 25},
  {-75, 25},
  // GO THROUGH GATE 3
  {-75, 75},
  {-25, 75}
};

// Localization
float heading = 0; // RADIANS
float x = path[0].x; // CM
float y = path[0].y; // CM

/* Main code */
void setup() {
  pinMode(7, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(8, OUTPUT);

  Serial.begin(9600);

  setupMotors();
  setupLocalization();
  waitForStart();
}

void loop() {
  if (digitalRead(9) == LOW) {
    stopMotors();
    digitalWrite(8, LOW);
    waitForStart();
    return;
  }

  loopLocalization();
  loopPid();

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
float startHeading;
float prevHeading = heading;
void resetLocalization() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float offset = 2*PI - euler.x() * DEG_TO_RAD;
  startHeading = START_HEADING - offset;
  prevHeading = START_HEADING;
  lTicks = 0;
  rTicks = 0;
  lastTime = millis();
}

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
  heading = 2*PI - euler.x() * DEG_TO_RAD + startHeading;
  float dHeading = heading - prevHeading;
  prevHeading = heading;
  if (dHeading == 0) {
    dHeading = (millis() % 2 == 0 ? 0.00001 : -0.00001);
  }

  // X and Y change
  float z = ((2*l)/dHeading + TRACK_WIDTH)*sin(dHeading/2);
  /*Serial.print("l:");
  Serial.print(l);
  Serial.print(",dHeading:");
  Serial.println(dHeading);*/
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
  if (abs(lPower) > 255) {
    if (lPower > 0) {
      lPower = 255;
    } else {
      lPower = -255;
    }
  }
  if (abs(rPower) > 255) {
    if (rPower > 0) {
      rPower = 255;
    } else {
      rPower = -255;
    }
  }

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
bool turninplace = false;
void loopPid() {
  Point goal = path[pathPoint];
  int maxerr = pathPoint < pathlen()-1 ? TRACK_WIDTH : 4; // Allow room to turn for early points
  if (abs(goal.x - x) < maxerr && abs(goal.y - y) < maxerr) { // Check if next point
    if (pathPoint < pathlen()-1) {
      pathPoint++;
      iV = 0; // Stop integral windup
      return;
    } else {
      waitForStart();
      return;

      digitalWrite(8, LOW);
      powerMotors(0, 0);
      delay(300);
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

  // Turn in place
  if (abs(err) > PI/1.5) { // Around 135 deg
    turninplace = true;
  } else if (abs(err) < PI/20) {
    turninplace = false; // Stop turning when <9deg of error if turning place
  }

  // PID
  iV += err * dT;
  float dV = (err - pErr)/dT;
  pErr = err;
  float w = err * P + iV * I + dV * (turninplace ? 0 : D);

  if (abs(w) > MAXANG) { // Limit max speed
    if (w < 0) {
      w = -MAXANG;
    } else {
      w = MAXANG;
    }
  }

  Serial.print("heading:");
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
  Serial.println(err);

  if (turninplace) { // Turn in place until <90deg of error
    powerMotors(-w - (w > 0 ? DEADBAND : -DEADBAND), w + (w > 0 ? DEADBAND : -DEADBAND));
  } else {
    powerMotors(SPEED - w, SPEED + w);
  }
}
/* Path following */

/* Button input code */
void waitForStart() {
  digitalWrite(8, LOW);
  delay(100);

  x = path[0].x;
  y = path[0].y;
  heading = START_HEADING;

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