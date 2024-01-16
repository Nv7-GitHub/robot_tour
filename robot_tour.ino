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
const float MAX_AVEL = 1/PI; // Max rotation speed SECOND/RADIANS
const float Kp = 0.01;
const float Kd = 0.01;
const float Hp = 0.0001; // Error proportional for non-moving component
const float ENDp = 0.1; // Error proportional for when done
const float TOTAL_TIME = 27; // SECOND
float START_HEADING = 0; // RADIANS

// Localization
float heading = 0; // RADIANS
float x = 0; // CM
float y = 0; // CM
float time = 0; // SECONDS

// Path
typedef struct Point {
  float x;
  float y;
} Point;
Point path[] = {
  {0, 0},
  {1000, 0},
  /*{300, 50},
  {0, 50},
  {0, 0},*/
};

/* Main code */
void setup() {
  Serial.begin(9600);
  delay(100);

  setupMotors();
  setupLocalization();
  setupRamsete();
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
    loopRamsete();
  }

  delay(20);
}
/* Main code */

/* Path following */
const int PATH_LEN = 2*sizeof(path)/sizeof(Point) - 1;
typedef struct RamsetePoint {
  float x;
  float y;
  float heading;
  float time;
  float linvel;
  float angvel;
} RamsetePoint;
RamsetePoint ramsetePath[PATH_LEN];

void printPoint(struct RamsetePoint pt) {
  Serial.print("x: ");
  Serial.print(pt.x);
  Serial.print(", y: ");
  Serial.print(pt.y);
  Serial.print(", heading: ");
  Serial.print(pt.heading * RAD_TO_DEG);
  Serial.print(", time: ");
  Serial.print(pt.time);
  Serial.print(", linvel: ");
  Serial.print(pt.linvel);
  Serial.print(", angvel: ");
  Serial.println(pt.angvel);
}

float ptdist(Point pt1, Point pt2) {
  return sqrt((pt1.y - pt2.y)*(pt1.y - pt2.y) + (pt1.x - pt2.x)*(pt1.x - pt2.x));
}
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

void setupRamsete() {
  // Plan time out for all the points
  float dist = 0;
  float ang = 0;
  float targetHeading = START_HEADING;
  for (int i = 0; i < pathlen()-1; i++) {
    dist += ptdist(path[i], path[i+1]);
    float newTargetHeading = calcH(path[i], path[i+1]);
    ang += abs(newTargetHeading - targetHeading);
    targetHeading = newTargetHeading;
  }
  
  // Calculate speeds
  float angTime = ang*MAX_AVEL;
  float vel = (TOTAL_TIME - angTime) / dist; // seconds/cm

  // Calculate path
  ramsetePath[0] = {path[0].x, path[0].y, calcH(path[0], path[1]), 0, 1.0/MAX_AVEL};
  int ramseteI = 1;
  float time = 0;
  for (int i = 1; i < pathlen(); i++) {
    float heading = calcH(path[i-1], path[i]);
    time += abs(heading - ramsetePath[ramseteI-1].heading) * MAX_AVEL;
    ramsetePath[ramseteI] = {path[i-1].x, path[i-1].y, heading, time, 1.0/vel, 0};
    ramseteI++;
    time += ptdist(path[i], path[i-1]) * vel;

    float sgn = 1;
    if (heading - ramsetePath[ramseteI-1].heading < 0) {
      sgn = -1;
    }
    ramsetePath[ramseteI] = {path[i].x, path[i].y, heading, time, 0, 1.0/MAX_AVEL * sgn};
    ramseteI++;
  }

  // Print path
  Serial.println("GENERATED PATH:");
  for (int i = 0; i < PATH_LEN; i++) {
    printPoint(ramsetePath[i]);
  }
}

int pathPoint = 0;
void loopRamsete() {
  // Get expected point
  while (pathPoint < PATH_LEN-1 && ramsetePath[pathPoint+1].time < time) {
    pathPoint++;
  }
  RamsetePoint goalPoint;
  if (pathPoint >= PATH_LEN-1) {
    goalPoint = ramsetePath[PATH_LEN-1];
    if (ptdist({x, y}, {goalPoint.x, goalPoint.y}) < 5) {
      done = true;
      digitalWrite(8, LOW);
      stopMotors();
      return;
    }
  } else {
    float dt = ramsetePath[pathPoint+1].time - ramsetePath[pathPoint].time;
    float pathprog = (time - ramsetePath[pathPoint].time)/dt; // 0-1 proportion of path done
    float x = ramsetePath[pathPoint].x + (ramsetePath[pathPoint+1].x - ramsetePath[pathPoint].x)*pathprog;
    float y = ramsetePath[pathPoint].y + (ramsetePath[pathPoint+1].y - ramsetePath[pathPoint].y)*pathprog;
    float h = ramsetePath[pathPoint].heading + (ramsetePath[pathPoint+1].heading - ramsetePath[pathPoint].heading)*pathprog;
    goalPoint = {x, y, h, ramsetePath[pathPoint].time, ramsetePath[pathPoint].linvel, ramsetePath[pathPoint].angvel};
  }
  /*Serial.print("x: ");
  Serial.print(goalPoint.x);
  Serial.print(", y: ");
  Serial.print(goalPoint.y);
  Serial.print(", heading: ");
  Serial.println(goalPoint.heading*RAD_TO_DEG);*/

  // Calculate error
  float rawerrx = goalPoint.x - x;
  float rawerry = goalPoint.y - y;
  float errx = rawerrx * cos(heading) + rawerry * sin(heading);
  float erry = rawerrx * -1 * sin(heading) + rawerry * cos(heading);
  float errh = goalPoint.heading - heading;
  
  if (errh < -PI) {
    errh += 2*PI;
  }
  if (errh > PI) {
    errh -= 2*PI;
  }
  if (errh == 0) {
    errh = 0.0001; // Fix divide by 0 errors
  }
  if (goalPoint.linvel == 0) {
    goalPoint.linvel = Hp*errx;
  }
  if (goalPoint.angvel == 0) {
    goalPoint.angvel = Hp*errh;
  }
  if (pathPoint >= PATH_LEN-1) {
    goalPoint.linvel = ENDp*errx;
    goalPoint.angvel = ENDp*errh;
  }

  // Calculate ramsete (https://wiki.purduesigbots.com/software/control-algorithms/ramsete)
  float k = 2*Kd*sqrt(goalPoint.angvel*goalPoint.angvel + Kp*goalPoint.linvel*goalPoint.linvel);
  float v = goalPoint.linvel*cos(errh) + k*errx;
  float w = goalPoint.angvel + k*(errh*RAD_TO_DEG) + (Kp*goalPoint.linvel*sin(errh)*erry)/errh;

  // Convert to motor powers
  float motorv = linvelToPower(v);
  float motorw = angvelToPower(w);

  Serial.print("ex:");
  Serial.print(errx);
  Serial.print(",ey:");
  Serial.print(erry);
  Serial.print(",eheading:");
  Serial.print(errh*RAD_TO_DEG);
  Serial.print(",linvel:");
  Serial.print(goalPoint.linvel);
  Serial.print(",angvel:");
  Serial.print(goalPoint.angvel);
  Serial.print(",motorv:");
  Serial.print(motorv);
  Serial.print(",motorw:");
  Serial.print(motorw);
  Serial.print(",v:");
  Serial.print(v);
  Serial.print(",w:");
  Serial.println(w);

  /*Serial.print("x:");
  Serial.print(x);
  Serial.print(",y:");
  Serial.print(y);
  Serial.print(",heading:");
  Serial.println(heading * RAD_TO_DEG);*/

  // Command power
  // Linear power from velocity

  powerMotors(motorv + motorw, motorv - motorw);
}
/* Path following */

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
  leftMotor = AFMS.getMotor(3);
  rightMotor = AFMS.getMotor(4);
  AFMS.begin();
}

// Linvel in cm/s
float linvelToPower(float linvel) {
  float val = abs(linvel)/1.7;
  if (val < 0) {
    val = 0;
  }
  return linvel < 0 ? val*-1 : val;
}
// Angvel in rad/s
float angvelToPower(float angvel) {
  float val = TRACK_WIDTH*abs(angvel)/1.7;
  if (val < 0) {
    val = 0;
  }
  return angvel < 0 ? val*-1 : val;
}

// Powers from 0 to 255
void powerMotors(int lPower, int rPower) {
  leftMotor->setSpeed(abs(lPower)+51);
  rightMotor->setSpeed(abs(rPower)+51);
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