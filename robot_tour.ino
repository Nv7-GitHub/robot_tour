/* WIRING:
Digital Pin 2: Left encoder A (White)
Digital Pin 4: Left encoder B (Yellow)
Digital Pin 3: Right encoder A (White)
Digital Pin 5: Right encoder B (Yellow)
Motor shield M3: Left motor (Red -> M3, Green -> Gnd)
Motor shield M4: Right motor (Green -> M4, Red -> Gnd)
Digital Pin 7 (Black): Button
Digital Pin 8 (Red): Reset
Digital Pin 9 (Black): LED

SDA (Yellow)/SCL (Green): IMU
+5V, GND, VIN jumper, VIN: Arduino power

Encoder blue: 5V
Encoder black: GND

Ultrasonic Echo: 10
Ultrasonic Trig: 11
Connect Ultrasonic VCC & GND
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_BNO055.h>
#include <HCSR04.h>


// Constants
const float TICKS_PER_CM = 12.6; // TICKS
const float TRACK_WIDTH = 16; // CM
const float SPEED = 45;
const float P = 35;
const float I = 0;
const float D = 15;
const float VEL_P = 0.1;

const float MAXANG = 25; // Max w value
const float DEADBAND = 30;

// Path
const float START_HEADING = PI/2; // RADIANS
const float TIME = 60; // SECONDS
const float WALLPOS = 0; // X or Y-coordinate of wall
const float WALLYAXIS = true; // if true, the wall goes up and down on the y axis, robot approaches along x axis
const float WALLEN = true;
typedef struct Point {
  float x; // CM
  float y; // CM
} Point;
Point path[] = {
  {75, -100},
  {75, -75},
  {25, -75},
  {25, -25},
  {-75, -25},
  {-75, -75},
  {-25, -75}, // GATE 1
  {-75, -75},
  {-75, -25},
  {25, -25},
  {25, -75},
  {75, -75}, // BACK TO START
  {75, 75},
  {25, 75}, // GATE 2
  {25, 25},
  {-25, 25},
  {-25, 75}, // PASS THROUGH END
  {-75, 75},
  {-75, 25}, // GATE 3
  {-75, 75},
  {-25, 75}

};
float ptdist(Point a, Point b) {
  float ex = a.x - b.x;
  float ey = a.y - b.y;
  return sqrt(ex*ex + ey*ey);
}

// Localization
float heading = 0; // RADIANS
float x = path[0].x; // CM
float y = path[0].y; // CM
float vel = 0;

/* Main code */
void setup() {
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, OUTPUT);

  Serial.begin(9600);

  setupMotors();
  setupLocalization();
  setupPid();
  waitForStart();
}

void loop() {
  if (digitalRead(8) == LOW) {
    stopMotors();
    digitalWrite(9, LOW);
    waitForStart();
    return;
  }

  loopLocalization();
  loopPid();

  delay(20);
}
/* Main code */

/* Localization code */
UltraSonicDistanceSensor hc(11, 10); // Use measureDistanceCm()

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
  x = path[0].x;
  y = path[0].y;
  heading = START_HEADING;
  vel = 0;

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

  unsigned long currTime = millis();
  dT = (float)(currTime - lastTime)/1000;
  lastTime = currTime;
  vel = ((abs(l) + abs(r))/2)/dT;

  // Heading
  /*float dHeading = (r - l)/TRACK_WIDTH; // Encoder-based orientation
  heading += dHeading;*/

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // IMU-based orientation
  heading = fmod(2*PI - euler.x() * DEG_TO_RAD + startHeading, 2*PI);
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
float velOffset = 0;
float targetVel = 0;

void setupPid() {
  // Calculate distleft
  float dist = 0;
  for (int i = 1; i < pathlen(); i++) {
    dist += ptdist(path[i-1], path[i]);
  }
  targetVel = (dist - (pathlen()-1)*TRACK_WIDTH/3) / TIME; // Subtract some amount for each turn
  Serial.print("TARGET VEL: ");
  Serial.println(targetVel);
}

void resetPid() {
  pathPoint = 0;
  iV = 0;
  pErr = 0;
  turninplace = false;
  velOffset = 0;
}

void loopPid() {
  Point goal = path[pathPoint];

  // Use distance sensor if last point
  float targetHeading = START_HEADING;
  if (pathPoint > 0) {
    targetHeading = atan2((double)(path[pathPoint].y - path[pathPoint-1].y), (double)(path[pathPoint].x - path[pathPoint-1].x));
  }
  float targetErr = targetHeading - heading;
  while (targetErr > PI) {
    targetErr -= 2*PI;
  }
  while (targetErr < (-1 * PI)) {
    targetErr += 2*PI;
  }
  if (pathPoint == pathlen()-1 && WALLEN && abs(targetErr) < PI/12) { // Error <15deg
    float rawDist = hc.measureDistanceCm();
    float dist = cos(targetErr) * rawDist; // Account for heading error
    if (dist > 0) { // Check value
      digitalWrite(9, 0);
      if (WALLYAXIS) {
        if (path[pathPoint].x > path[pathPoint-1].x) {
          x = WALLPOS - dist;
          Serial.println(x);
          Serial.println(dist);
        } else {
          x = WALLPOS + dist;
        }
      } else {
        if (path[pathPoint].y > path[pathPoint-1].y) {
          y = WALLPOS - dist;
        } else {
          y = WALLPOS + dist;
        }
      }
    }
  } else {
    digitalWrite(9, 1);
  }

  // Calculate errors
  float ex = goal.x - x;
  float ey = goal.y - y;
  float err = atan2((double)(ey), (double)(ex)) - heading;
  while (err > PI) {
    err -= 2*PI;
  }
  while (err < (-1 * PI)) {
    err += 2*PI;
  }

  // Check if ready for next point
  int maxerr = pathPoint < pathlen()-1 ? TRACK_WIDTH : 1; // Allow room to turn for early points
  if (abs(ex) < maxerr && abs(ey) < maxerr) { // Check if next point
    if (pathPoint < pathlen()-1) {
      pathPoint++;
      iV = 0; // Stop integral windup
      return;
    } else {
      Serial.println(ex);
      digitalWrite(9, LOW);
      powerMotors(0, 0);
      delay(300);
      stopMotors();
      waitForStart();
      return;
    }
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

  // Time control
  float terr = targetVel - vel;
  velOffset += terr * VEL_P;

  if (abs(w) > (MAXANG + velOffset)) { // Limit max speed
    if (w < 0) {
      w = -(MAXANG + velOffset);
    } else {
      w = (MAXANG + velOffset);
    }
  }

  Serial.print("heading:");
  Serial.print(heading);
  Serial.print(",atan:");
  Serial.print(atan2((double)(goal.y - y), (double)(goal.x - x)));
  Serial.print(",ey:");
  Serial.print(ey);
  Serial.print(",ex:");
  Serial.print(ex);
  Serial.print(",y:");
  Serial.print(y);
  Serial.print(",x:");
  Serial.print(x);
  Serial.print(",vel:");
  Serial.print(vel);
  Serial.print(",w:");
  Serial.print(w);
  Serial.print(",voffset:");
  Serial.print(velOffset);
  Serial.print(",err:");
  Serial.println(err);

  if (turninplace) { // Turn in place until <90deg of error
    powerMotors(-w - (w > 0 ? DEADBAND : -DEADBAND), w + (w > 0 ? DEADBAND : -DEADBAND));
  } else {
    powerMotors(SPEED + velOffset - w, SPEED + velOffset + w);
  }
}
/* Path following */

/* Button input code */
void waitForStart() {
  digitalWrite(9, LOW);
  delay(100);

  bool pressed = false;
  int pwr = 64;
  while (true) {
    analogWrite(9, pwr);
    pwr += 4; 
    if (pwr >= 128) {
      pwr = 0;
    }

    int buttonDown = digitalRead(7);
    if (buttonDown == LOW) {
      digitalWrite(9, LOW);
      pressed = true;
    }
    if (buttonDown == HIGH && pressed) { // Start when released
      break;
    }
    delay(50);
  }
  resetLocalization();
  resetPid();
  digitalWrite(9, HIGH);
}
/* Button input code */