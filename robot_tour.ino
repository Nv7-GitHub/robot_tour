/* WIRING:
Digital Pin 2: Left encoder
Digital Pin 3: Right encoder
Motor shield M2: Left motor
Motor shield M3: Right motor
Digital Pin 7: Button
Digital Pin 8: LED
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_BNO055.h>

// Constants
const float TICKS_PER_CM = 3.5; // TICKS
const float TRACK_WIDTH = 13;
const float VEL = 20; // Forwards speed proportional
const float AVEL = 100; // Angular speed proportional
const float Kp = 0.5;
const float Kd = 0.1;
const float TOTAL_TIME = 25; // SECOND
float START_HEADING = 0; // RADIANS
const float END_HEADING = PI/2; // RADIANS

// Localization
float heading = 0; // RADIANS
float x = 0; // CM
float y = 0; // CM
float time = 0; // SECONDS

// Path
typedef struct Point {
  float x;
  float y;
  float time;
} Point;
Point path[] = {
  {0, 0, 0},
  {50, 0},
  {50, 50},
  {0, 50},
  {0, 0},
  {50, 0},
  {50, 50},
  {0, 50},
  {0, 0},
};

/* Main code */
void setup() {
  Serial.begin(9600);

  setupMotors();
  setupLocalization();
  setupRamsete();
  waitForStart();
}

bool done = false;
void loop() {
  if (!done) {
    loopLocalization();
    loopRamsete();
  }
  
  /*Serial.print("x:");
  Serial.print(x);
  Serial.print(",y:");
  Serial.print(y);
  Serial.print(",heading:");
  Serial.println(heading * RAD_TO_DEG);*/

  delay(100);
}
/* Main code */

/* Path following */
float ptdist(Point pt1, Point pt2) {
  return sqrt((pt1.y - pt2.y)*(pt1.y - pt2.y) + (pt1.x - pt2.x)*(pt1.x - pt2.x));
}
int pathlen() {
  return sizeof(path)/sizeof(Point);
}
void setupRamsete() {
  // Plan time out for all the points
  float dist = 0;
  for (int i = 0; i < pathlen()-1; i++) {
    dist += ptdist(path[i], path[i+1]);
  }
  float prog = 0;
  for (int i = 1; i < pathlen(); i++) {
    prog += ptdist(path[i-1], path[i]);
    path[i].time = (prog/dist)*TOTAL_TIME;
  }
}

int pathPoint = 0;
void loopRamsete() {
  // Get expected point
  if (pathPoint < pathlen()-1 && path[pathPoint+1].time < time) {
    pathPoint++;
  }
  Point goalPoint;
  float targetHeading;
  if (pathPoint >= pathlen()-1) {
    goalPoint = path[pathlen()-1];
    targetHeading = END_HEADING;
  } else {
    float dt = path[pathPoint+1].time - path[pathPoint].time;
    float pathprog = (time - path[pathPoint].time)/dt; // 0-1 proportion of path done
    float x = path[pathPoint].x + (path[pathPoint+1].x - path[pathPoint].x)*pathprog;
    float y = path[pathPoint].y + (path[pathPoint+1].y - path[pathPoint].y)*pathprog;
    targetHeading = atan2(path[pathPoint+1].y - path[pathPoint].y, path[pathPoint+1].x - path[pathPoint].x);
    goalPoint = {x, y, 0};
  }
  /*Serial.print("x: ");
  Serial.print(goalPoint.x);
  Serial.print(", y: ");
  Serial.print(goalPoint.y);
  Serial.print(", heading: ");
  Serial.println(targetHeading*RAD_TO_DEG);*/

  // Calculate error
  float rawerrx = goalPoint.x - x;
  float rawerry = goalPoint.y - y;
  float errx = rawerrx * cos(heading) + rawerry * sin(heading);
  float erry = rawerrx * -1 * sin(heading) + rawerry * cos(heading);
  float errh = targetHeading - heading;
  if (errh < -PI) {
    errh += 2*PI;
  }
  if (errh > PI) {
    errh -= 2*PI;
  }
  if (errh == 0) {
    errh = 0.0001; // Fix divide by 0 errors
  }
  /*if (pathPoint >= pathlen()-1) {
    LINEARVEL = Kp*errx;
    ANGVEL = Kp*errh;
  }*/

  // Calculate velocities
  float LINEARVEL = VEL * errx;
  float ANGVEL = AVEL * errh;

  // Calculate ramsete (https://wiki.purduesigbots.com/software/control-algorithms/ramsete)
  float k = 2*Kd*sqrt(ANGVEL*ANGVEL + Kp*LINEARVEL*LINEARVEL);
  float v = LINEARVEL*cos(errh) + k*errx;
  float w = ANGVEL + k*errh + (Kp*LINEARVEL*sin(errh)*erry)/errh;

  Serial.print("ex: ");
  Serial.print(errx);
  Serial.print(", ey: ");
  Serial.print(erry);
  Serial.print(", eheading: ");
  Serial.print(errh*RAD_TO_DEG);
  Serial.print(", v: ");
  Serial.print(v);
  Serial.print(", w: ");
  Serial.println(w);

  // Command power
  powerMotors(v - w, v + w);
}
/* Path following */

/* Localization code */
int lTicks = 0;
int rTicks = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
void setupLocalization() {
  attachInterrupt(0, incrementL, RISING);
  attachInterrupt(1, incrementR, RISING);
  bno.begin();
  bno.setExtCrystalUse(true);
}

bool lForwards = true;
void incrementL() {
  if (lForwards) {
    lTicks++;
  } else {
    lTicks--;
  }
}

bool rForwards = true;
void incrementR() {
  if (rForwards) {
    rTicks++;
  } else {
    rTicks--;
  }
}

int startTime = 0;
void resetLocalization() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float offset = 2*PI - euler.x() * DEG_TO_RAD;
  START_HEADING -= offset;
  lTicks = 0;
  rTicks = 0;
  startTime = millis();
}

float prevHeading = START_HEADING;
void loopLocalization() {
  float l = ((float)lTicks)/(TICKS_PER_CM);
  float r = ((float)rTicks)/(TICKS_PER_CM);
  time = ((float)(millis() - startTime))/1000.0;

  // Heading
  /*float dHeading = (r - l)/TRACK_WIDTH; // Encoder-based orientation
  heading += dHeading;*/

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // IMU-based orientation
  heading = 2*PI - euler.x() * DEG_TO_RAD + START_HEADING;
  float dHeading = heading - prevHeading;
  prevHeading = heading;
  if (dHeading == 0) {
    dHeading = 0.0000001;
  }

  // X and Y change
  float z = ((2*l)/dHeading + TRACK_WIDTH)*sin(dHeading/2);
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
  leftMotor = AFMS.getMotor(4);
  rightMotor = AFMS.getMotor(3);
  AFMS.begin();
}

// Powers from 0 to 255
void powerMotors(int lPower, int rPower) {
  leftMotor->setSpeed(abs(lPower));
  rightMotor->setSpeed(abs(rPower));
  if (lPower < 0) {
    leftMotor->run(BACKWARD);
    lForwards = false;
  } else {
    leftMotor->run(FORWARD);
    lForwards = true;
  }
  if (rPower < 0) {
    rightMotor->run(BACKWARD);
    rForwards = false;
  } else {
    rightMotor->run(FORWARD);
    rForwards = true;
  }
}

void stopMotors() {
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}
/* Motor control code */

/* Button input code */
void waitForStart() {
  pinMode(7, INPUT_PULLUP);
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
    delay(200);
  }
  resetLocalization();
  digitalWrite(8, HIGH);
}
/* Button input code */