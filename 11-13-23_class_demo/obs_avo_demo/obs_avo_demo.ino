#define LEFT_CAL 120 // calibration value of left motors default 127
#define RIGHT_CAL 140 // calibration value of right motors default 127

#define SERVO_CAL 3

#define SPIN_STEP_LEFT_TIME 215 // TIME TO SPIN LEFT 30 DEGREES
#define SPIN_STEP_RIGHT_TIME 215 // TIME TO RIGHT LEFT 30 DEGREES

#define leftMotorsDigitalPin 2
#define leftMotorsAnalogPin 9
#define rightMotorsDigitalPin 4
#define rightMotorsAnalogPin 5
#define PING_PIN 10
#define PING_SERVO_PIN 3

#define SWEEP_STEP_NUM 7
#define FORWARD_INDEX 3
#define SWEEP_STEP_SIZE 30

#define MIN_SAFE_DIST 4

long distances[SWEEP_STEP_NUM];
long forwardDistance;
int farthestIndex; 
int prevServoAngle;

void setup() {
  pinMode(leftMotorsDigitalPin, OUTPUT);
  pinMode(rightMotorsDigitalPin, OUTPUT);
  pinMode(PING_SERVO_PIN, OUTPUT);
  setPingAngle(90);
  Serial.begin(9600);
  Serial.println("SETUP");
}

void loop() {
  //wheelTest();
  //pingServoTest();
  //calibrateForward();
  //calibrateSpin();
  //pingTest();
  //pingSweepTest();
  navigate();
}

void calibrateForward() {
  forward();
  delay(1250);
  stop(2000);
  backward();
  delay(1250);
  stop(2000);
}

void calibrateSpin() {
  spinStepLeft();
  stop(2000);
  spinStepLeft();
  stop(2000);
  spinStepLeft();
  stop(2000);
  spinStepRight();
  stop(2000);
  spinStepRight();
  stop(2000);
  spinStepRight();
  stop(2000);
}

void pingServoTest() {
  setPingAngle(90);
  setPingAngle(0);
  setPingAngle(30);
  setPingAngle(60);
  setPingAngle(90);
  setPingAngle(120);
  setPingAngle(150);
  setPingAngle(180);
  setPingAngle(0);
  setPingAngle(180);
}

void wheelTest() {
  drive(1, 1, HIGH, HIGH);
  delay(1000);
  drive(1, 0, HIGH, HIGH);
  delay(1000);
  drive(0, 1, HIGH, HIGH);
  delay(1000);
}

void navigate() {
  updateAllDistances();
  turnToFarthestIndex();
  driveToFar(20);
}

void driveToFar(long maxTravelDist) {
  updateForwardDistance();
  while (forwardDistance > 0) {
    forward(1);
    delay(100);
    long currentDist = ping();
    if (currentDist <= MIN_SAFE_DIST) break;
    long diff = forwardDistance - currentDist;
    if (diff >= maxTravelDist) break;
  }
  stop();
}

void turnToFarthestIndex() {
  printDistances();
  findFarthestIndex();
  Serial.print("Farthest index = ");
  Serial.println(farthestIndex);
  int turns = FORWARD_INDEX - farthestIndex;
  Serial.print("turns = ");
  Serial.println(turns);
  if (turns < 0) for (int i = 0; i > turns; --i) spinStepLeft();
  else if (turns > 0) for (int i = 0; i < turns; ++i) spinStepRight();
}

void findFarthestIndex() {
  long maxDist = 0;
  for (int i = 0; i < SWEEP_STEP_NUM; ++i) {
    if (distances[i] > maxDist) {
      farthestIndex = i;
      maxDist = distances[i];
    }
  }
}

void printDistances() {
  Serial.print("Distances = ");
  for (int i = 0; i < SWEEP_STEP_NUM; ++i) {
    Serial.print(distances[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

void updateAllDistances() {
  Serial.println("Updating All Distances");
  for (int i = 0; i < SWEEP_STEP_NUM; ++i) {
    int angle = i*SWEEP_STEP_SIZE;
    setPingAngle(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    distances[i] = ping();
  }
  setPingAngle(90);
}

void updateForwardDistance() {
  setPingAngle(90);
  forwardDistance = ping();
}

void setPingAngle(int angle) {
  angle += SERVO_CAL;
  int turnTime = abs(angle-prevServoAngle) / 3;
  for (int i = 0; i < turnTime; ++i) sendPingAnglePulse(angle);
  delay(10);
  prevServoAngle = angle;
}

void sendPingAnglePulse(int angle) {
  int pulsewidth = angle * 10 + 620;
  digitalWrite(PING_SERVO_PIN, HIGH);
  delayMicroseconds(pulsewidth);
  digitalWrite(PING_SERVO_PIN, LOW);
  delay((20 - pulsewidth / 1000));
}

void pingTest() {
  ping();
  delay(500);
}

void pingSweepTest() {
  updateAllDistances();
  delay(2000);
}

long ping() {
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW); 
  delayMicroseconds(2); 
  digitalWrite(PING_PIN, HIGH); 
  delayMicroseconds(5); 
  digitalWrite(PING_PIN, LOW); 
  pinMode(PING_PIN, INPUT); 
  long duration = pulseIn(PING_PIN, HIGH); 
  //Serial.print("Ping Duration: ");
  //Serial.println(duration);
  long cm = microsecondsToCentimeters(duration);
  //Serial.print("Ping cm: ");
  //Serial.println(cm);
  return cm;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void spinStepLeft() {
  spinLeft();
  delay(SPIN_STEP_LEFT_TIME);
  stop(50);
}

void spinStepRight() {
  spinRight();
  delay(SPIN_STEP_RIGHT_TIME);
  stop(50);
}

void stop() {
  stop(0);
}

void stop(int time) {
  forward(0);
  delay(time);
}

void forward() {
  forward(1);
}

void backward() {
  backward(1);
}

void forward(float scale) {
  drive(scale, scale, HIGH, HIGH);
}

void backward(float scale) {
  drive(scale, scale, LOW, LOW);
}

void spinLeft() {
  drive(1, 1, LOW, HIGH);
}

void spinRight() {
  drive(1, 1, HIGH, LOW);
}

void drive(float leftScale, float rightScale, int leftType, int rightType) {
  digitalWrite(leftMotorsDigitalPin, leftType);
  analogWrite(leftMotorsAnalogPin, leftScale*LEFT_CAL);
  digitalWrite(rightMotorsDigitalPin, rightType);
  analogWrite(rightMotorsAnalogPin, rightScale*RIGHT_CAL);
}
