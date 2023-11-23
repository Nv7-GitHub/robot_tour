/* WIRING:
Digital Pin 2: Left encoder
Digital Pin 3: Right encoder
Motor shield M1: Left motor
Motor shield M2: Right motor
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Constants
const int TICKS_PER_CM = 100; // TICKS
const float TRACK_WIDTH = 2; // CM
const float LOOKAHEAD = 2; // CM
const float VEL = 127;

// Localization
float heading; // RADIANS
float x; // CM
float y; // CM

/* Main code */
void setup() {
  Serial.begin(9600);
  setupLocalization();
  setupMotors();
}

void loop() {
  loopLocalization();
  //loopPursuit();
  
  Serial.print("x: ");
  Serial.println(x);
  Serial.print("y: ");
  Serial.println(y);
  Serial.print("heading: ");
  Serial.println(heading * RAD_TO_DEG);

  powerMotors(255, 255);
}
/* Main code */

/* Pure pursuit */
// https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit#putting-everything-together
typedef struct Point {
  float x;
  float y;
} Point;

const Point path[] = {
  {0, 10},
  {10, 10},
};

float ptdist(struct Point pt1, struct Point pt2) {
  return sqrt((pt2.x - pt1.x)*(pt2.x - pt1.x) + (pt2.y - pt1.x)*(pt2.y - pt1.y));
}

float sgn(float num) {
  if (num >= 0) {
    return 1;
  }
  return 0;
}

int lastFound = 0;
float loopPursuit() {
  bool foundIntersect = false;
  int start = lastFound;
  Point goalPt;
  for (int i = start; i < sizeof(path)/sizeof(Point)-1; i++) {
    // Intersect circle
    float x1 = path[i].x - x;
    float y1 = path[i].y - y;
    float x2 = path[i+1].x - x;
    float y2 = path[i+1].y - y;
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dr = sqrt(dx*dx + dy*dy);
    float D = x1*y2 - x2*y1;
    float discriminant = (LOOKAHEAD*LOOKAHEAD) * (dr*dr) - D*D;
    
    if (discriminant >= 0) {
      float sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / dr*dr;
      float sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / dr*dr;
      float sol_y1 = (-1 * D * dx + abs(dy) * sqrt(discriminant)) / dr*dr;
      float sol_y2 = (-1 * D * dx - abs(dy) * sqrt(discriminant)) / dr*dr;
      Point sol_pt1 = {sol_x1, sol_y1};
      Point sol_pt2 = {sol_x2, sol_y2};
      // End of circle intersection

      // Test if solutions are in right range
      float minX = min(path[i].x, path[i+1].x);
      float minY = min(path[i].y, path[i+1].y);
      float maxX = max(path[i].x, path[i+1].x);
      float maxY = max(path[i].y, path[i+1].y);
      bool pt1Found = ((minX <= sol_pt1.x <= maxX) && (minY <= sol_pt1.y <= maxY));
      bool pt2Found = ((minX <= sol_pt2.x <= maxX) && (minY <= sol_pt2.y <= maxY));
      if (pt1Found || pt2Found) {
        foundIntersect = true;
        
        // If both, get best
        if (pt1Found && pt2Found) {
          if (ptdist(sol_pt1, path[i+1]) < ptdist(sol_pt2, path[i+1])) {
            goalPt = sol_pt1;
          } else {
            goalPt = sol_pt2;
          }
        } else if (pt1Found) {
          goalPt = sol_pt1;
        } else if (pt2Found) {
          goalPt = sol_pt2;
        }

        // Only exit if solution is closer to next than current
        if (ptdist(goalPt, path[i+1]) < ptdist({x, y}, path[i+1])) {
          lastFound = i;
          break;
        } else {
          // In case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
          lastFound = i+1;
        }
      } else {
        foundIntersect = false;
        goalPt = path[lastFound]; // Potentially deviated from path
      }
    }
  }

  // Calculate angle to goal point
  float ang = atan2(goalPt.y - y, goalPt.x - x);
  if (ang < 0) {
    ang += 2*PI;
  }

  // Find minimum angle
  float turnErr = ang - heading;
  if (turnErr > PI || turnErr < -1*PI) {
    turnErr = -1*sgn(turnErr) * (2*PI - abs(turnErr));
  }

  // Calculate power for l and r
  float turnVel = (TRACK_WIDTH * sin(turnErr))/LOOKAHEAD * VEL;
  powerMotors(VEL - turnVel, VEL + turnVel);
}

/* Pure pursuit */

/* Localization code */
int lTicks = 0;
int rTicks = 0;
void setupLocalization() {
  attachInterrupt(0, incrementL, RISING);
  attachInterrupt(1, incrementR, RISING);
}

void incrementL() {
  lTicks++;
}

void incrementR() {
  rTicks++;
}

void loopLocalization() {
  // Heading
  float l = ((float)lTicks)/((float)TICKS_PER_CM);
  float r = ((float)rTicks)/((float)TICKS_PER_CM);
  float dHeading = (r - l)/TRACK_WIDTH;
  heading += dHeading;

  // X and Y change
  float z = ((2*l)/dHeading + TRACK_WIDTH)*sin(dHeading/2);
  float dx = z*cos((PI-dHeading)/2);
  float dy = z*sin((PI-dHeading)/2);

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
  leftMotor = AFMS.getMotor(1);
  rightMotor = AFMS.getMotor(2);
  AFMS.begin();
}

// Powers from -255 to 255
void powerMotors(int lPower, int rPower) {
  leftMotor->setSpeed(abs(lPower));
  rightMotor->setSpeed(abs(rPower));
  if (lPower >= 0) {
    leftMotor->run(FORWARD);
  } else {
    leftMotor->run(BACKWARD);
  }
  if (rPower >= 0) {
    rightMotor->run(FORWARD);
  } else {
    rightMotor->run(BACKWARD);
  }
}

void stopMotors() {
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}
/* Motor control code */