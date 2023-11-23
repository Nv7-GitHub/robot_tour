/* WIRING:
Digital Pin 2: Left encoder
Digital Pin 3: Right encoder
Motor shield M1: Left motor
Motor shield M2: Right motor

UNITS:
Everything is in centimeters
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>


const int TICKS_PER_CM = 100;
const float TRACK_WIDTH = 2;

/* Pure pursuit */
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

float heading; // RADIANS
float x; // CM
float y; // CM

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

/* Main code */
void setup() {
  Serial.begin(9600);
  setupLocalization();
  setupMotors();
}

void loop() {
  loopLocalization();
  Serial.print("x: ");
  Serial.println(x);
  Serial.print("y: ");
  Serial.println(y);
  Serial.print("heading: ");
  Serial.println(heading * RAD_TO_DEG);

  powerMotors(255, 255);
}
/* Main code */