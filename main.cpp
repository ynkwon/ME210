#include <Arduino.h>
/*

TO DO (top level):
-start speed at max, after 1 second drop it low
- change directions on time in case the switches fail


TO DO (extras for competition):

-test DC motors' directions and braking. a direction pin
may need to move to high to get braking to work. 


-new shoot methods that call shoot, but set the shootType variable before. 
-code to call the different firing routines based on the shootType setting. 

spray:
	-call shoot with a few additional degrees. 
	-start stepping while activating another rotation
	-

TEST LOG

-tracing front wall perfectly with 1.08 leftRatio
-bouncing off the front and left corners.
  -slow it down only when going forward and left?
  -flushLeft code from before
  -delay a half second before cutting motors

RESOLVED
-left side catches the switch, pulling towards its right - drive right a bit first
-the left wheel sometimes doesn't turn when driving back - resolved, poor solder
- transitions out of the shoot state before done shooting - transition to another state, and finish after done shooting


*/

/* LIST OF INPUT AND OUTPUT PINS 
NOTE: left and right motors, and front and back motors
are paired. They are driven with the same signals. 
*/
int ENABLE_LEFT = 3;  //PWM enabled
int ENABLE_RIGHT = 4; //PWM enabled
int IN1_LR = 2;
int IN2_LR = 1;

int ENABLE_FRONT = 9; //PWM enabled
int ENABLE_BACK = 10; //PWM enabled
int IN1_FB = 11;
int IN2_FB = 12;

//Bumpers
int BUMP_FRONT = 15; //light green
int BUMP_BACK = 17; //dark green no tape
int BUMP_LEFT = 14; //red guy
int BUMP_RIGHT = 16; //yellow taped

//Stepper motor for aiming
int STEP = 5; //red
int DIR = 7; //green
int STEP_ENABLE = 6; //white

//Flywheel pins (need PWM enable, directions pulled)
int FLYWHEEL = 23; //PWM enabled
int FLYWHEEL_BRAKE = 22;
int SOLENOID_ENB = 21;

//IR sensors
int SENSOR1 = 18;
int SENSOR2 = 19;
int SENSOR3 = 20;

//REMAINING PINS: 0, 8, 21

//-------------------------------------------
//VARIABLES THAT WILL CHANGE
int last_available = 0;
int count = 0; //for printing
int times_fired = 0;
int steps_to_go = 0;
IntervalTimer stepTimer;
int stepState = LOW;
bool solenoidIsOn = false;
IntervalTimer rightTimer;
int currAngle = 0;
bool shooting = false;
IntervalTimer driveTimer;
int leftTime = 20 * 1000000; //10 seconds
int righttime = 20 * 1000000;
int forwardTime = 15 * 1000000;
int backwardTime = 15 * 1000000;
bool driving_forward = false;
bool driving_left = false;
IntervalTimer gameTimer;
IntervalTimer accelDriveTimer;
int accelTime = 500000;

//PARAMETERS THAT WE'LL NEED TO TUNE
//Ratios when we want to drive slightly into the wall
//Ratios should always be applied to the second wheel
int gameTime = 40 * 1000000; //100 seconds for now
float leftRatio = 1.07; //driving against front wall. SET AT 1.08!
float rightRatio = 1; //ratio of FRONT to BACK motor's speed, driving against RIGHT wall
float frontRatio = 1.05; //ratio of LEFT to RIGHT motor's speed, driving against FRONT wall
float backRatio = 0.95; //ratio of RIGHT to LEFT motor's speed, driving against BACK wall
int normalSpeed = 195; //up to 256
int slowSpeed = 185; //not enough to drive it when theres any friction
int step_interval = 30000; //microseconds per step, tune this
int right_offset = 1000000;
int sprayAngle = 5; //total arc length of the spray

/* LIST OF STATES
*/
typedef enum {
  LEFT, RIGHT, FORWARD, BACKWARD, STOP
} Direction;
Direction drive;

typedef enum {
  ALL, SINGLE, SPRAY
} ShootType;
ShootType shootType;

//When using this, the ratio always is applied to enb1
//And the HIGH is always applied to dir1.
struct motorPins{
  int enb1;
  int enb2;
  int dir1;
  int dir2;
};

typedef enum {
  START, THRONE, RF_CORNER, LF_CORNER, LEFT_OFFSET, ARMORY,
  LOADED, SHOOT_1, SHOOT_2, LEFT_OFFSET_LOADED, PRE_SHOOT,
  DRIVING_RIGHT, DONE, TEST, SHOOTING_LEFT, SHOOTING_RIGHT
} State_pos;
State_pos state;

/* STRATEGY
//make sure that driving_left is false when entering state

*/

//--------------------------------------------
/*FUNCTION DECLARATIONS */
void driveForwardNormal();
void driveBackwardNormal();
void driveLeftNormal();
void driveRightNormal();
void driveForwardAccel();
void driveBackwardAccel();
void driveLeftAccel();
void driveRightAccel();
void stop();
uint8_t testForKey(void);
bool isFlush(int);
bool anyBumper();
void step();
motorPins getPins(Direction);
void driveSlowSpeed(Direction);
void driveNormalSpeed(Direction, float);
void driveFast(Direction, float);
void stopMotors(Direction);
void stopRightMoveBack();
void stopRightMoveForward();
void solenoidOn();
void solenoidOff();
void flywheel(int);
void shoot(int);
void fire();
void brakeFlywheel();
void aimCannon(int);
void handleShooting();

void RFCornerToLFCorner();
void throneToRFCorner();
void leftOffsetToArmory();
void armoryToLoaded();
void leftOffsetLoadedToPreshoot();
void preshootToShoot();
void keepFlushFront();
void keepFlushLeft();
void terminate();
//----------------------------------------

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  state = START;
  pinMode(ENABLE_LEFT, OUTPUT);
  pinMode(ENABLE_RIGHT, OUTPUT);
  pinMode(IN1_LR, OUTPUT);
  pinMode(IN2_LR, OUTPUT);
  pinMode(ENABLE_FRONT, OUTPUT);
  pinMode(ENABLE_BACK, OUTPUT);
  pinMode(IN1_FB, OUTPUT);
  pinMode(IN2_FB, OUTPUT);
  pinMode(BUMP_BACK, INPUT);
  pinMode(BUMP_FRONT, INPUT);
  pinMode(BUMP_LEFT, INPUT);
  pinMode(BUMP_RIGHT, INPUT);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STEP_ENABLE, OUTPUT);
  pinMode(FLYWHEEL, OUTPUT);
  pinMode(SOLENOID_ENB, OUTPUT);
  pinMode(FLYWHEEL_BRAKE, OUTPUT);
}

void loop() {
  count += 1;
  switch (state) {
    case TEST:
      if (isFlush(BUMP_RIGHT)) {
        shooting = false;
        aimCannon(20);
      }
      delay(100);
        /* Code to test the motors front and back in code only
        if (solenoidIsOn) {
          Serial.print("FORWARD");
          digitalWrite(ENABLE_FRONT, HIGH);
          digitalWrite(ENABLE_BACK, HIGH);
          digitalWrite(IN1_FB, HIGH);
          digitalWrite(IN2_FB, LOW);
          solenoidIsOn = false;
        } else {
          Serial.print("BACK");
          digitalWrite(ENABLE_FRONT, HIGH);
          digitalWrite(ENABLE_BACK, HIGH);
          digitalWrite(IN1_FB, LOW);
          digitalWrite(IN2_FB, HIGH);
          solenoidIsOn = true;
          */
      break;
    case START:
      if (anyBumper()) {
        solenoidOn();
        flywheel(100);
        driveForwardAccel();
        state = THRONE;
        gameTimer.begin(terminate, gameTime);
        //driveTimer.begin(throneToRFCorner, forwardTime);
      }
      break;
    case THRONE:
      if (isFlush(BUMP_FRONT)) {
        throneToRFCorner();
      }
      break;
    case RF_CORNER:
      if (isFlush(BUMP_LEFT)) {
        RFCornerToLFCorner();
      }
      break;
    case LF_CORNER:
      //drive right for a bit, entering new state
      driveRightNormal();
      rightTimer.begin(stopRightMoveBack, right_offset);
      Serial.println("lf corner to driving right");
      state = DRIVING_RIGHT;
      break;
    //we're now a bit off the left wall. drive back.
    case LEFT_OFFSET:
      if (isFlush(BUMP_BACK)) {
        leftOffsetToArmory();
      }
      break;
    case ARMORY: //driving left towards the loading thing
      if (isFlush(BUMP_LEFT)) {
        armoryToLoaded();
      }
      break;
    case DRIVING_RIGHT:
      break;
    case LOADED: //finished loading, drive a bit right again. start flywheel.
      driveRightNormal();
      rightTimer.begin(stopRightMoveForward, right_offset);
      Serial.println("loaded to driving right");
      state = DRIVING_RIGHT;
      break;
    case LEFT_OFFSET_LOADED:
      if (isFlush(BUMP_FRONT)) {
        leftOffsetLoadedToPreshoot();
      }
      break;
    case PRE_SHOOT: //a bit off the left corner, driving left
      if (isFlush(BUMP_LEFT)) {
        preshootToShoot();
      }
      break;
    case SHOOT_1: 
      shootType = ALL;
      Serial.println("shoot1 to shooting left");
      state = DONE;
      shoot(0);
      times_fired += 1;
      break;
    case SHOOT_2:
      shootType = ALL;
      Serial.println("shoot2 to shooting left");
      state = SHOOTING_LEFT;
      shoot(0);
      times_fired += 1;
      break;
    case SHOOTING_LEFT:
      keepFlushLeft();
      keepFlushFront();
      break;
    case DONE:
      break;
  }

  if (count % 50000 == 0) {
    Serial.println(state);
  }
}

void throneToRFCorner() {
  Serial.println("throne to RF Corner");
  driveLeftAccel();
  state = RF_CORNER;
}
void RFCornerToLFCorner() {
  Serial.println("rf to lf corner");
  state = LF_CORNER;
}
void leftOffsetToArmory() {
  Serial.println("left offset to armory");
  driveLeftNormal();
  state = ARMORY;
}
void leftOffsetLoadedToPreshoot() {
  Serial.println("left offset loaded to pre-shoot");
  driveLeftNormal();
  state = PRE_SHOOT; //later, can switch to other states based on num shots taken
}
void armoryToLoaded() {
  Serial.println("armory to loaded");
  stop();
  delay(5000);
  state = LOADED;
}
void preshootToShoot() {
  Serial.println("pre-shoot to shoot");
  if (times_fired == 0) {
    state = SHOOT_1;
  } else if (times_fired == 1) {
    state = SHOOT_2;
  } else {
    state = DONE;
  }
  stop();
}


/*
tuple<int, bool> getDrivingVars(Direction dir) {
  switch (dir) {
    case STOP:
      return (0, false);
      break;
    case LEFT:
      retu
  }
}*/

void terminate(){
  state = DONE;
  stop();
  solenoidOn();
  brakeFlywheel();
}

//Call this constantly (in relevant states) to keep 
//flush against the left wall
void keepFlushLeft(){
  if (not driving_left) {
    if (not isFlush(BUMP_LEFT)) {
      driveSlowSpeed(LEFT);
      driving_left = true;
    }
  } else {
    if (isFlush(BUMP_LEFT)) {
      stopMotors(LEFT);
      driving_left = false;
    }
  }
}

void keepFlushFront(){
  if (not driving_forward) {
    if (not isFlush(BUMP_FRONT)) {
      driveSlowSpeed(FORWARD);
      driving_forward = true;
    }
  } else {
    if (isFlush(BUMP_FRONT)) {
      stopMotors(FORWARD);
      driving_forward = false;
    }
  }
}


/*Can either modify this to test for different shooting conditions,
which we set before shooting. Or write copies for different shooting

If high, set to low and vice versa
When we're out of steps, we shoot
*/
void step() {
  if (steps_to_go > 0) { 
    //send a step
    if (stepState == HIGH) {
      digitalWrite(STEP, LOW);
      stepState = LOW;
    } else if (stepState == LOW) {
      digitalWrite(STEP, HIGH);
      stepState = HIGH;
    }
    steps_to_go--;
  } else { 
    stepTimer.end();
    digitalWrite(STEP_ENABLE, LOW);
    digitalWrite(DIR, LOW);
    if (shooting) {
      handleShooting();
    }
  }
}

//reset shooting and angle variables. 
//handle state transitions since we're done shooting. 
//EDIT - switch based on shooting types
//EDIT - add transitions from other shooting states
void handleShooting() {
  fire();
  if (currAngle != 0) {
    aimCannon(0 - currAngle);
    currAngle = 0;
  }
  shooting = false;
  switch(state) {
    case SHOOTING_LEFT:
      Serial.print("shooting left to lf corner");
      state = LF_CORNER;
      break;
    case SHOOTING_RIGHT:
      Serial.print("shooting right to rf corner");
      state = RF_CORNER;
      break;
    default:
      break;
  }
}

//Aims the cannon and shoots when done. 
//The actual shooting occurs in the interrupt when the stepping is done
//After calling, call returnCannon to return the same number of degrees
void shoot(int degrees) {
  currAngle = degrees;
  if (degrees != 0) { 
    shooting = true;
    aimCannon(degrees);
  } else {
    handleShooting();
  }
}

void fire() {
  Serial.println("fire");
  solenoidOff();
  delay(10000); //pause to let shooting take place
  solenoidOn();
}

void fireQuick() {
  solenoidOff();
  delay(100);
  solenoidOn();
}

void aimCannon(int degrees) {
  digitalWrite(STEP_ENABLE, HIGH);
  if (degrees < 0) {
    digitalWrite(DIR, HIGH);
  }
  degrees = abs(degrees);
  steps_to_go = map(degrees, 0, 180, 0, 200); //200 is a half rotation
  stepTimer.begin(step, step_interval);
}

void solenoidOn() {
  digitalWrite(SOLENOID_ENB, HIGH);
}

void solenoidOff() {
  digitalWrite(SOLENOID_ENB, LOW);
}

void flywheel(int pct_speed) {
  analogWrite(FLYWHEEL, 256 * pct_speed / 100);
  digitalWrite(FLYWHEEL_BRAKE, HIGH);
}

void brakeFlywheel() {
  digitalWrite(FLYWHEEL, LOW);
  digitalWrite(FLYWHEEL_BRAKE, LOW);
}

void stopRightMoveBack() {
  rightTimer.end();
  driveBackwardAccel();
  state = LEFT_OFFSET;
  Serial.println("driving right to left offset");
}

void stopRightMoveForward() {
  rightTimer.end();
  driveForwardAccel();
  state = LEFT_OFFSET_LOADED;
  Serial.println("driving right to left offset loaded");
}

void driveForwardAccel() {
  accelDriveTimer.end();
  stopMotors(LEFT);
  driveFast(FORWARD, frontRatio);
  accelDriveTimer.begin(driveForwardNormal, accelTime);
}

void driveForwardNormal() {
  accelDriveTimer.end();
  stopMotors(LEFT);
  driveNormalSpeed(FORWARD, frontRatio);
}

void driveBackwardAccel() {
  accelDriveTimer.end();
  stopMotors(LEFT);
  driveFast(BACKWARD, backRatio);
  accelDriveTimer.begin(driveBackwardNormal, accelTime);
}

void driveBackwardNormal() {
  accelDriveTimer.end();
  Serial.print("drive backwards normal");
  stopMotors(LEFT);
  driveNormalSpeed(BACKWARD, backRatio);
}

void driveLeftAccel() {
  accelDriveTimer.end();
  stopMotors(FORWARD);
  driveFast(LEFT, leftRatio);
  accelDriveTimer.begin(driveLeftNormal, accelTime);
}

void driveLeftNormal() {
  accelDriveTimer.end();
  stopMotors(FORWARD);
  driveNormalSpeed(LEFT, leftRatio);
}

void driveRightAccel() {
  accelDriveTimer.end();
  stopMotors(FORWARD);
  driveFast(RIGHT, rightRatio);
  accelDriveTimer.begin(driveRightNormal, accelTime);
}

void driveRightNormal() {
  accelDriveTimer.end();
  stopMotors(FORWARD);
  driveNormalSpeed(RIGHT, rightRatio);
}

void stop() {
  accelDriveTimer.end();
  stopMotors(FORWARD);
  stopMotors(RIGHT);
}

void driveFast(Direction dir, float ratio) {
  motorPins pins = getPins(dir);
  int speed = max(240 * ratio, 256);
  analogWrite(pins.enb1, speed);
  analogWrite(pins.enb2, 240);
  digitalWrite(pins.dir1, HIGH);
  digitalWrite(pins.dir2, LOW);

}

/*Start a pair of motors at normal speed, second motor at the ratio*/
void driveNormalSpeed(Direction dir, float ratio) {
  motorPins pins = getPins(dir);
  analogWrite(pins.enb1, normalSpeed * ratio);
  analogWrite(pins.enb2, normalSpeed);
  digitalWrite(pins.dir1, HIGH);
  digitalWrite(pins.dir2, LOW);
}

//Writes a pair of motors at slow speed
void driveSlowSpeed(Direction dir) {
  motorPins pins = getPins(dir);
  analogWrite(pins.enb1, slowSpeed);
  analogWrite(pins.enb2, slowSpeed);
  digitalWrite(pins.dir1, HIGH);
  digitalWrite(pins.dir2, LOW);
}

//Writes a pair of motors low
void stopMotors(Direction dir) {
  motorPins pins = getPins(dir);
  digitalWrite(pins.enb1, LOW);
  digitalWrite(pins.enb2, LOW);
  digitalWrite(pins.dir1, LOW);
  digitalWrite(pins.dir2, LOW);
}

/*Determine the pins from the direction
*/
motorPins getPins(Direction dir) {
  motorPins pins;
  switch (dir) {
    case STOP:
      pins.enb1 = 0;
      pins.enb2 = 0;
      pins.dir1 = 0;
      pins.dir2 = 0;
      break;
    case FORWARD:
      pins.enb1 = ENABLE_FRONT;
      pins.enb2 = ENABLE_BACK;
      pins.dir1 = IN1_FB;
      pins.dir2 = IN2_FB;
      break;
    case BACKWARD:
      pins.enb1 = ENABLE_BACK;
      pins.enb2 = ENABLE_FRONT;
      pins.dir1 = IN2_FB;
      pins.dir2 = IN1_FB;
      break;
    case LEFT: 
      pins.enb1 = ENABLE_LEFT;
      pins.enb2 = ENABLE_RIGHT;
      pins.dir1 = IN1_LR;
      pins.dir2 = IN2_LR;
      break;
    case RIGHT: 
      pins.enb1 = ENABLE_RIGHT;
      pins.enb2 = ENABLE_LEFT;
      pins.dir1 = IN2_LR;
      pins.dir2 = IN1_LR;
      break;
  }
  return pins;
}

bool isFlush(int bumper) { 
  return digitalRead(bumper);
}

bool anyBumper() {
  return digitalRead(BUMP_BACK) or digitalRead(BUMP_FRONT) 
      or digitalRead(BUMP_LEFT) or digitalRead(BUMP_RIGHT);
}

uint8_t testForKey(void) {
  uint8_t KeyEventOccurred = Serial.available() - last_available;
  last_available = Serial.available();
  return KeyEventOccurred;
}