#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <protothreads.h>

#define LEFT_CAL 120 // calibration value of left motors default 127
#define RIGHT_CAL 160 // calibration value of right motors default 127
float DRIVE_SCALE = 0.65;
float MAX_DRIVE_CORRECT = 0.15;
float SPIN_SCALE = 0.8;

#define MIN_SAFE_DIST 16

#define SERVO_CAL 3 // added to servo angle input to account for misalaignment

#define SWEEP_STEP_NUM 13 // number of directions to look when sweeping
#define FORWARD_INDEX 6 // the forward distance index of distances
#define SWEEP_STEP_SIZE 180 / (SWEEP_STEP_NUM-1) // sweep increment angle 

#define FORWARD_PING_ANGLE 90
#define RIGHT_PING_ANGLE 0
#define LEFT_PING_ANGLE 180

#define GYRO_TURN_ERROR 2

#define LEFT_MOTOR_DIGITAL_PIN 2
#define LEFT_MOTOR_ANALOG_PIN 9
#define RIGHT_MOTOR_DIGITAL_PIN 4
#define RIGHT_MOTOR_ANALOG_PIN 5
#define PING_PIN 10
#define PING_SERVO_PIN 3

pt ptGyroAngle;
pt ptPingAngle;
pt ptNavigate;
pt ptDrive;
pt ptSweep;
Adafruit_MPU6050 mpu;

long currentX = 0, currentY = 0;

bool missionAccomplished = false, firstNavLoop = true;

float currentGyroAngle = 0;
float goalDriveAngle = 0;
float goalAngleDiff = 0;

int pingAngle = FORWARD_PING_ANGLE;

long prevGyroTime = 0, prevDist = 0;

bool commandTurn = false, commandForward = false, commandBackward = false, commandRightWheelOnly;
bool commandDriveDist = false;
float goalDriveDist = 0, initDriveDist = 0;
bool commandSweep = false;
int sweepIndex = 0;

long distances[SWEEP_STEP_NUM]; // saved distances of a sweep. 0 = right distance, SWEEP_STEP_NUM-1 = left distance
long rightDist = 0;
int farthestIndex = FORWARD_INDEX;

void setup() {
  // init threads
  PT_INIT(&ptGyroAngle);
  PT_INIT(&ptPingAngle);
  PT_INIT(&ptNavigate);
  PT_INIT(&ptDrive);
  PT_INIT(&ptSweep);
  // pin modes
  pinMode(LEFT_MOTOR_DIGITAL_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIGITAL_PIN, OUTPUT);
  pinMode(PING_SERVO_PIN, OUTPUT);
  // setup gyro
  Serial.begin(9600);
  while(!Serial) delay(10);
  Serial.println("SETUP BEGIN");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // finish
  Serial.println("SETUP FINISH");
  prevGyroTime = millis();
}

void loop() {
  PT_SCHEDULE(gyroAngleThread(&ptGyroAngle));
  PT_SCHEDULE(pingServoThread(&ptPingAngle));
  //PT_SCHEDULE(driveTurnTestThread(&ptNavigate));
  PT_SCHEDULE(navigateThread(&ptNavigate));
  PT_SCHEDULE(driveThread(&ptDrive));
  PT_SCHEDULE(sweepThread(&ptSweep));
}

int navigateThread(struct pt* pt) {
  PT_BEGIN(pt);
  for(;;) {
    if (missionAccomplished) {
      PT_YIELD(pt);
      break;
    }
    // if there is not enough room to turn drive, drive right wheels only until turned ~40 degrees
    if (firstNavLoop) {
      lookRight();
      PT_SLEEP(pt, 600);
      rightDist = ping(2);
      PT_SLEEP(pt, 100);
      if (rightDist < 20) {
        //lookForward();
        PT_SLEEP(pt, 200);
        //goalDriveAngle = 45;
        commandRightWheelOnly = true;
        PT_SLEEP(pt, 1200);
        commandRightWheelOnly = false;
        PT_SLEEP(pt, 100);
      }
      firstNavLoop = false;
    }
    // sweep
    startSweep();
    PT_SLEEP(pt, 2000);
    endSweep();
    //printDistances();
    // check win condition
    if (checkWinCondition()) {
      missionAccomplished = true;
      Serial.println("PASS WIN CONDITION");
      PT_YIELD(pt);
      break;
    } else {
      Serial.println("FAIL WIN CONDITION");
    }
    // find and set goal angle to farthest distance
    int newAngle = getFarthestDistanceAngle();
    Serial.print("NEW ANGLE: ");
    Serial.println(newAngle);
    startDriveTurn(newAngle);
    PT_SLEEP(pt, 1200);
    endDriveTurn();
    PT_SLEEP(pt, 100);
    // drive towards farthest point 15 cm twice
    lookForward();
    PT_SLEEP(pt, 200);
    commandForward = true;
    PT_SLEEP(pt, 500);
    commandForward = false;
    PT_SLEEP(pt, 100);
  }
  PT_END(pt);
}

bool somethingTooClose() {
  for (int i = FORWARD_INDEX-3; i <= FORWARD_INDEX+3; ++i) {
    if (distances[i] != 0 && distances[i] < 20) {
      return true;
    }
  }
  return false;
}

int getFarthestDistanceAngle() {
  if (isWallAhead()) return 90;
  if (hasPotentialObstacleLeftSide()) return 0;
  farthestIndex = FORWARD_INDEX;
  long maxDist = 0;
  for (int i = 0; i < SWEEP_STEP_NUM; ++i) {
    if (ignoreSweepIndex(i)) continue;
    if (distances[i] > maxDist) {
      farthestIndex = i;
      maxDist = distances[i];
    }
  }
  return getAngleFromSweepIndex(farthestIndex);
}

bool isWallAhead() {
  // check for wall
  bool wall = false;
  for (int i = 0; i < SWEEP_STEP_NUM; ++i) {
    if (ignoreSweepIndex(i)) continue;
    int angle = getAngleFromSweepIndex(i);
    if (angle > 20) continue;
    if (distances[i] < 15) wall = true;
  }
  if (!wall) return false;
  // check if left wall is far enough
  for (int i = 0; i < SWEEP_STEP_NUM; ++i) {
    if (ignoreSweepIndex(i)) continue;
    int angle = getAngleFromSweepIndex(i);
    if (angle < 35) continue;
    if (distances[i] > 20) return true;
  }
  return false;
}

bool hasPotentialObstacleLeftSide() {
  for (int i = 0; i < SWEEP_STEP_NUM; ++i) {
    if (ignoreSweepIndex(i)) continue;
    int angle = getAngleFromSweepIndex(i);
    if (angle < 25) continue;
    if (distances[i] < 25) return true;
  }
  return false;
}

bool checkSweepIndexBounds(int index) {
  return index >= 0 && index < SWEEP_STEP_NUM;
}
 
bool ignoreSweepIndex(int index) {
  if (index == FORWARD_INDEX) return false;
  int angle = getAngleFromSweepIndex(index);
  if (angle > 105 || angle < -15) {
    //Serial.println(" IGNORED");
    return true;
  }
  //Serial.println(" CHECKING");
  return false;
}

int getAngleFromSweepIndex(int index) {
  return currentGyroAngle - (FORWARD_INDEX - index) * SWEEP_STEP_SIZE;
}

bool checkWinCondition() {
  for (int i = 0; i < SWEEP_STEP_NUM; ++i) {
    if (!ignoreSweepIndex(i) && distances[i] > 20) {
      return false;
    }
  }
  return true;
}

void printDistances() {
  Serial.print("Distances = ");
  for (int i = 0; i < SWEEP_STEP_NUM; ++i) {
    Serial.print(distances[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

int sweepThread(struct pt* pt) {
  PT_BEGIN(pt);
  for (;;) {
    if (commandSweep) {
      if (!ignoreSweepIndex(sweepIndex)) {
        int angle = sweepIndex * SWEEP_STEP_SIZE;
        pingAngle = angle;
        PT_SLEEP(pt, 160);
        //Serial.print("Angle: ");
        //Serial.println(angle);
        distances[sweepIndex] = ping(3);
      }
      ++sweepIndex;
      if (sweepIndex >= SWEEP_STEP_NUM) {
        commandSweep = false;
      }
    } else {
      PT_YIELD(pt);
    }
  }
  PT_END(pt);
}

void startSweep() {
  commandSweep = true;
  sweepIndex = 0;
}

void endSweep() {
  commandSweep = false;
}

int driveTurnTestThread(struct pt* pt) {
  PT_BEGIN(pt);
  for(;;) {
    //lookForward();
    // drive forward 20 cm
    startDriveDist(20);
    PT_SLEEP(pt, 2000);
    endDriveDist();
    PT_SLEEP(pt, 500);
    // drive backward 20 cm
    startDriveDist(-20);
    PT_SLEEP(pt, 2000);
    endDriveDist();
    PT_SLEEP(pt, 500);
    // turn left 30 degrees
    startDriveTurn(30);
    PT_SLEEP(pt, 2000);
    endDriveTurn();
    PT_SLEEP(pt, 500);
    // drive forward 20 cm
    startDriveDist(20);
    PT_SLEEP(pt, 2000);
    endDriveDist();
    PT_SLEEP(pt, 500);
    // drive backward 20 cm
    startDriveDist(-20);
    PT_SLEEP(pt, 2000);
    endDriveDist();
    PT_SLEEP(pt, 500);
    // turn left 60 degrees
    startDriveTurn(90);
    PT_SLEEP(pt, 2000);
    endDriveTurn();
    PT_SLEEP(pt, 500);
    // drive forward 20 cm
    startDriveDist(20);
    PT_SLEEP(pt, 2000);
    endDriveDist();
    PT_SLEEP(pt, 500);
    // drive backward 20 cm
    startDriveDist(-20);
    PT_SLEEP(pt, 2000);
    endDriveDist();
    PT_SLEEP(pt, 500);
    // turn right 90 degrees
    startDriveTurn(0);
    PT_SLEEP(pt, 2000);
    endDriveTurn();
    PT_SLEEP(pt, 500);
    //PT_YIELD(pt);
  }
  PT_END(pt);
}

void startDriveDist(long dist) {
  commandDriveDist = true;
  initDriveDist = ping();
  goalDriveDist = dist;
}

void endDriveDist() {
  commandDriveDist = false;
}

void startDriveTurn(int degrees) {
  commandTurn = true;
  goalDriveAngle = degrees;
}

void endDriveTurn() {
  commandTurn = false;
}

int driveThread(struct pt* pt) {
  PT_BEGIN(pt);
  for(;;) {
    if (commandTurn) driveTurn();
    else if (commandForward) driveStraightForward();
    else if (commandBackward) driveStraightBackward();
    else if (commandDriveDist) driveDistance();
    else if (commandRightWheelOnly) rightWheelOnly();
    else stop();
    PT_SLEEP(pt, 50);
  }
  PT_END(pt);
}

void rightWheelOnly() {
  /*long currentDist = ping1();
  if (currentDist < MIN_SAFE_DIST) {
    stop();
    return;
  }*/
  if (currentGyroAngle > 40) {
    stop();
    return;
  }
  drive(0, DRIVE_SCALE, HIGH, HIGH);
}

void driveDistance() {
  long goalDist = initDriveDist - goalDriveDist;
  long currentDist = ping(2);
  updatePosition(currentDist);
  if (goalDriveDist > 0 && currentDist < MIN_SAFE_DIST) stop();
  else if (goalDist < currentDist) driveStraightForward();
  else if (goalDist > currentDist) driveStraightBackward();
  else stop();
}

void updatePosition(long currentDist) {
  long distTraveled = currentDist - prevDist;
  currentY += distTraveled * cos(goalDriveAngle*DEG_TO_RAD);
  currentX += distTraveled * sin(goalDriveAngle*DEG_TO_RAD);
  prevDist = currentDist;
}

void driveTurn() {
  if (abs(goalAngleDiff) > GYRO_TURN_ERROR) {
    if (goalAngleDiff > 0) spinLeft();
    else spinRight();
    Serial.print("TURN DIFF: ");
    Serial.println(goalAngleDiff);
  } else {
    stop();
    Serial.print("REACHED GOAL ANGLE: ");
    Serial.println(currentGyroAngle);
  } 
}

void driveStraightForward() {
  long currentDist = ping1();
  if (currentDist < MIN_SAFE_DIST) {
    stop();
    return;
  }
  float correction = goalAngleDiff * 0.01;
  correction = min(MAX_DRIVE_CORRECT, correction);
  drive(DRIVE_SCALE - correction, DRIVE_SCALE + correction, HIGH, HIGH);
}

void driveStraightBackward() {
  float correction = goalAngleDiff * 0.01;
  correction = min(MAX_DRIVE_CORRECT, correction);
  drive(DRIVE_SCALE + correction, DRIVE_SCALE - correction, LOW, LOW);
}

int gyroAngleThread(struct pt* pt) {
  PT_BEGIN(pt);
  for(;;) {
    PT_SLEEP(pt, 80);
    float turnRate = getGyroTurnRate();
    long time = millis();
    currentGyroAngle += turnRate * (time - prevGyroTime) * 0.001;
    goalAngleDiff = goalDriveAngle - currentGyroAngle;
    prevGyroTime = time;
  }
  PT_END(pt);
}

float getGyroTurnRate() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float error = 0.02;
  float zRad = g.gyro.heading + error;
  if (zRad > -0.01 && zRad < 0.01) return 0;
  return zRad * RAD_TO_DEG;
}

int pingServoThread(struct pt* pt) {
  PT_BEGIN(pt);
  for(;;) {
    int pulsewidth = (pingAngle+SERVO_CAL) * 10 + 620;
    digitalWrite(PING_SERVO_PIN, HIGH);
    delayMicroseconds(pulsewidth);
    digitalWrite(PING_SERVO_PIN, LOW);
    PT_SLEEP(pt, (20 - pulsewidth*0.001));
  }
  PT_END(pt);
}

void lookForward() {
  pingAngle = FORWARD_PING_ANGLE;
}

void lookRight() {
  pingAngle = RIGHT_PING_ANGLE;
}

long ping() {
  return ping(5);
}

long ping(int checks) {
  long min = ping1();
  for (int i = 1; i < 5; ++i) {
    long trial = ping1();
    if (trial < min) {
      min = trial;
    }
  }
  return min;
}

long ping1() {
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW); 
  delayMicroseconds(2); 
  digitalWrite(PING_PIN, HIGH); 
  delayMicroseconds(5); 
  digitalWrite(PING_PIN, LOW); 
  pinMode(PING_PIN, INPUT); 
  long duration = pulseIn(PING_PIN, HIGH); 
  long cm = microsecondsToCentimeters(duration);
  delay(10);
  return cm;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void stop() {
  forward(0);
}

void forward() {
  forward(DRIVE_SCALE);
}

void backward() {
  backward(DRIVE_SCALE);
}

void forward(float scale) {
  drive(scale, scale, HIGH, HIGH);
}

void backward(float scale) {
  drive(scale, scale, LOW, LOW);
}

void spinLeft() {
  drive(SPIN_SCALE, SPIN_SCALE, LOW, HIGH);
}

void spinRight() {
  drive(SPIN_SCALE, SPIN_SCALE, HIGH, LOW);
}

void drive(float leftScale, float rightScale, int leftType, int rightType) {
  digitalWrite(LEFT_MOTOR_DIGITAL_PIN, leftType);
  analogWrite(LEFT_MOTOR_ANALOG_PIN, leftScale*LEFT_CAL);
  digitalWrite(RIGHT_MOTOR_DIGITAL_PIN, rightType);
  analogWrite(RIGHT_MOTOR_ANALOG_PIN, rightScale*RIGHT_CAL);
}