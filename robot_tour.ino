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
const float TRACK_WIDTH = 13; // CM
const float LOOKAHEAD = 20; // CM
const float VEL = 127; // SPEED (out of 255)
float START_HEADING = PI/2;

// Localization
float heading = 0; // RADIANS
float x = -25; // CM
float y = -100; // CM

// Path
typedef struct Point {
  float x;
  float y;
} Point;
const Point path[] = {
  {-25, -100},
  {-25, -75},
  {-75, -75},
  {-75, -25},
  {-25, -25},
  {-25, 15},
  {25, 60},
  {40, 0},
  {60, -25},
};

/* Main code */
void setup() {
  setupMotors();
  setupLocalization();
  Serial.begin(9600);
  waitForStart();
}

bool done = false;
void loop() {
  if (!done) {
    loopLocalization();
    loopPursuit();
  }
  
  /*Serial.print("x:");
  Serial.print(x);
  Serial.print(",y:");
  Serial.print(y);
  Serial.print(",heading:");
  Serial.println(heading * RAD_TO_DEG);*/

  delay(20);
}
/* Main code */

/* Pure pursuit */
// https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit#putting-everything-together
float ptdist(struct Point pt1, struct Point pt2) {
  return sqrt((pt2.x - pt1.x)*(pt2.x - pt1.x) + (pt2.y - pt1.y)*(pt2.y - pt1.y));
}

float sgn(float num) {
  if (num >= 0) {
    return 1;
  }
  return 0;
}

const float EPSILON = 0.01;

Point goalPt;
int lastFound = 0;
float loopPursuit() {
  bool foundIntersect = false;
  int start = lastFound;
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
      float sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / (dr*dr);
      float sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / (dr*dr);
      float sol_y1 = (-1 * D * dx + abs(dy) * sqrt(discriminant)) / (dr*dr);
      float sol_y2 = (-1 * D * dx - abs(dy) * sqrt(discriminant)) / (dr*dr);
      Point sol_pt1 = {sol_x1 + x, sol_y1 + y};
      Point sol_pt2 = {sol_x2 + x, sol_y2 + y};

      // End of circle intersection

      // Test if solutions are in right range
      float minX = min(path[i].x, path[i+1].x) - EPSILON;
      float minY = min(path[i].y, path[i+1].y) - EPSILON;
      float maxX = max(path[i].x, path[i+1].x) + EPSILON;
      float maxY = max(path[i].y, path[i+1].y) + EPSILON;
      bool pt1Found = ((minX <= sol_pt1.x && sol_pt1.x <= maxX) && (minY <= sol_pt1.y && sol_pt1.y <= maxY));
      bool pt2Found = ((minX <= sol_pt2.x && sol_pt2.x <= maxX) && (minY <= sol_pt2.y && sol_pt2.y <= maxY));
      if (pt1Found || pt2Found) {
        foundIntersect = true;
        
        Point foundPt;
        // If both, get best
        if (pt1Found && pt2Found) {
          if (ptdist(sol_pt1, path[i+1]) < ptdist(sol_pt2, path[i+1])) {
            foundPt = sol_pt1;
          } else {
            foundPt = sol_pt2;
          }
        } else if (pt1Found) {
          foundPt = sol_pt1;
        } else if (pt2Found) {
          foundPt = sol_pt2;
        }

        // Only exit if solution is closer to next than current
        if (ptdist(foundPt, path[i+1]) < ptdist({x, y}, path[i+1])) {
          goalPt = foundPt;
          /*Serial.println("GOAL");
          Serial.println(ptdist(goalPt, path[i+1]));
          Serial.println(ptdist({x, y}, path[i+1]));
          Serial.println(goalPt.x);
          Serial.println(goalPt.y);
          Serial.println("GOAL");*/
          lastFound = i;
          break;
        } else {
          // In case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
          foundIntersect = false;
          /*Serial.println("LOST"); // Potentially deviated from path, but stay with the last found goal point
          Serial.println(goalPt.x);
          Serial.println(goalPt.y);
          Serial.println("LOST");*/
          //goalPt = path[lastFound]; // Potentially deviated from path
        }
      } else {
        foundIntersect = false;
        goalPt = path[lastFound]; // Potentially deviated from path
        /*Serial.println("GOALMISS");
        Serial.println(goalPt.x);
        Serial.println(goalPt.y);
        Serial.println("GOALMISS");*/
      }
    } else {
      //Serial.println("NO CIRCLE FOUND");
    }
  }

  /*Serial.println("TURNVEL");
  Serial.println(lastFound);*/
  // Calculate angle to goal point
  float ang = atan2(goalPt.y - y, goalPt.x - x);
  if (ang < 0) {
    ang += 2*PI;
  }
  //Serial.println(ang * RAD_TO_DEG);
  
  // Find minimum angle
  float turnErr = ang - heading;
  //Serial.println(turnErr * RAD_TO_DEG);
  if (turnErr > PI) {
    turnErr -= 2*PI;
  } else if (turnErr < -PI) {
    turnErr += 2*PI;
  }
  if (turnErr > PI/2) {
    turnErr = PI/2;
  } else if (turnErr < -PI/2) {
    turnErr = -PI/2;
  }
  //Serial.println(turnErr * RAD_TO_DEG);

  // Calculate power for l and r
  float turnVel = (TRACK_WIDTH * sin(turnErr))/LOOKAHEAD * VEL;
  if (turnVel > VEL) {
    turnVel = VEL;
  }
  if (turnVel < -VEL) {
    turnVel = -VEL;
  }
  if (lastFound >= sizeof(path)/sizeof(Point)-2) {
    float dist = ptdist(path[lastFound+1], {x, y})/LOOKAHEAD;
    if (dist > 1) {
      dist = 1;
    }
    /*Serial.println("ENDING");
    Serial.println(ptdist(path[lastFound+1], {x, y}));
    Serial.println(dist);
    Serial.println(turnVel);*/
    if (dist < 0.25) {
      stopMotors(); // Close to the end, done
      done = true;
      digitalWrite(8, LOW);
      //Serial.println("DONE");
    } else {
     //Serial.println("APPROACHING");
      powerMotors((VEL - turnVel) * dist, (VEL + turnVel) * dist); // Slow down as approaching the end
    }
  } else {
    /*Serial.println("NORMAL");
    Serial.println(turnVel);*/
    powerMotors(VEL - turnVel, VEL + turnVel);
  }
}

/* Pure pursuit */

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

void incrementL() {
  lTicks++;
}

void incrementR() {
  rTicks++;
}

void resetLocalization() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float offset = 2*PI - euler.x() * DEG_TO_RAD;
  START_HEADING -= offset;
  lTicks = 0;
  rTicks = 0;
}

float prevHeading = START_HEADING;
void loopLocalization() {
  float l = ((float)lTicks)/(TICKS_PER_CM);
  float r = ((float)rTicks)/(TICKS_PER_CM);

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
  if (lPower < 0) {
    lPower = 0;
  }
  if (rPower < 0) {
    rPower = 0;
  }
  if (lPower > 255) {
    lPower = 255;
  }
  if (rPower > 255) {
    rPower = 255;
  }
  leftMotor->setSpeed(lPower);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(rPower);
  rightMotor->run(FORWARD);
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