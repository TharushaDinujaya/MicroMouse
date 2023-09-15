#include "ESP32SharpIR.h"
#include "Adafruit_VL53L0X.h"
#include "maze.h"
#include "solver.h"


#include <Arduino.h>

#define TEST 0
#define SEARCH_RUN 1
#define RETURN_TO_START 0
#define FAST_RUN 0

#define pwmChannel1 0
#define pwmChannel2 1    // Selects channel 0
#define frequency 30000  // PWM frequency of 1 KHz
#define resolution 8   // 8-bit resolution, 256 possible values

// Motor A
#define pwmA 13
#define in1A 14
#define in2A 12

// Motor B
#define pwmB 4
#define in1B 19
#define in2B 18

// Motor Speed Values - Start at zero
int MotorSpeed1 = 0;
int MotorSpeed2 = 0;

ESP32SharpIR sensor1(ESP32SharpIR::GP2Y0A21YK0F, 26);
ESP32SharpIR sensor2(ESP32SharpIR::GP2Y0A21YK0F, 35);
ESP32SharpIR sensor3(ESP32SharpIR::GP2Y0A21YK0F, 34);

// TOF initialization parameters and variable creations
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x34
int sensor1, sensor2, sensor3, sensor4;


// set the pins to shutdown
#define SHT_LOX1 2 // FRONT
#define SHT_LOX2 4 // RIGHT
#define SHT_LOX3 5 // BACK
#define SHT_LOX4 19 // LEFT

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;


// create two arrays to represent wall map and flood index map with predefine sizes
int wallMap[MAZE_SIZE][MAZE_SIZE];
int floodMap[MAZE_SIZE][MAZE_SIZE];
Orient orient = NORTH; // initial orientation of the mouse

// initial coordination of the mouse
int X = 0;
int Y = 0;

bool run(void);
void left(int speed);
void right(int speed);
void stop();

void forward(int speed);
void reverse(int speed);

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX2
  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }

  // activating LOX4
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  //initing LOX2
  if (!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  // board initializing routines goes here
  sensor1.setFilterRate(0.0f);
  sensor2.setFilterRate(0.0f);
  sensor3.setFilterRate(0.0f);

  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Configuration of channel 0 with the chosen frequency and resolution
  ledcSetup(pwmChannel1, frequency, resolution);
  ledcSetup(pwmChannel2, frequency, resolution);

  // Assigns the PWM channel to pin 23
  ledcAttachPin(pwmA, pwmChannel1);
  ledcAttachPin(pwmB, pwmChannel2);

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);

  Serial.println("Both in reset mode...(pins are low)");


  Serial.println("Starting...");

  // array initialize routines goes here
  initializeMaze(wallMap, floodMap);
  Serial.begin(115200);
}

void test() {
  while (true) {
    int front = measure1.RangeMilliMeter;
    if (front > 80) {
      Serial.printf("Distance (front): %d \n", front);
      forward(150);
    } else {
      int leftD = measure4.RangeMilliMeter;
      int rightD = measure2.RangeMilliMeter;

     if (leftD > 100) {
        Serial.printf("Distance (left): %f \n", leftD);
        left(150);
        delay(ROTATE_TIME);
        stop();
      } else if (rightD < 100) {
        Serial.printf("Distance (right): %f \n", rightD);
        right(150);
        delay(ROTATE_TIME);
        stop();
      } else {
        Serial.printf("Stopped !");
        stop();
      }
    }
  }
}

void loop() {
    Serial.print("loop started");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    
    #if TEST
    test();
    #endif

    #if SEARCH_RUN
    // run the mouse until it reaches the destination
    Serial.println("Running the mouse...");
    bool running = true;
    while (running) {
      Serial.println("Running the mouse step...");
      running = run();
    }
    #endif

    #if RETURN_TO_START
    // TODO: find the way back to the destination
    
    // TODO: follow the calculated path

    #endif

    #if FAST_RUN
    // TODO: fast run implementation
    #endif
    // end of the program

}

void forward(int speed) {
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  ledcWrite(pwmChannel1, speed);  // 1.65 V
  ledcWrite(pwmChannel2, speed);  // 1.65 V
}

void reverse(int speed) {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannel1, speed);  // 1.65 V
  ledcWrite(pwmChannel2, speed);  // 1.65 V
}

void left(int speed) {
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannel1, speed);  // 1.65 V
  ledcWrite(pwmChannel2, speed);  // 1.65 V
}

void right(int speed) {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  ledcWrite(pwmChannel1, speed);  // 1.65 V
  ledcWrite(pwmChannel2, speed);  // 1.65 V
}

void stop() {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
}



void rotateLeft(void) {
  // implement the rotate left mechanism through motor drive and gyroscope
}

void rotateRight(void) {
  // implement the rotate right mechanism through motor drive and gyroscope
}

void rotateMouse(int direction) {
  // rotate the mouse according to given target direction
  switch (direction)
  {
  case LEFT:
    left(150); // TODO: adjust the speed of the motor
    delay(ROTATE_TIME);
    stop();
    break;
  case RIGHT:
    right(150); // TODO: adjust the speed of the motor
    delay(ROTATE_TIME);
    stop();
    break;
  case FORWARD:
    // nothing to do here
    break;
  default:
    break;
  }
}

void moveMouseForward(void) {
  // TODO: apply the logic to move the mouse until it reaches the middle of the next cell
  // get the distance from the front TOF sensor at the beginning
  int frontDistance = measure1.RangeMilliMeter;
  int initialDistance = frontDistance;

  // TODO: start the motor drive to move forward
  forward(150); // TODO: adjust the speed of the motor
  while (frontDistance > initialDistance - CELL_SIZE) {
    // TODO: keep align with the walls
    delay(50);
    frontDistance = measure1.RangeMilliMeter; // distance from the front TOF sensor
  }
  // forward(125);
  // delay(1200);

  stop(); //  stop the motor drive

}

bool run(void) {
  // reset the flood map
  resetFloodMap(floodMap);
  
  // TODO:  first identifying the walls around the mouse
  bool leftWall = measure4.RangeMilliMeter < SIDE_MIN_DISTANCE; // TODO: should change the implementation
  bool rightWall = measure2.RangeMilliMeter < SIDE_MIN_DISTANCE; // TODO: should change the implementation
  bool frontWall = measure1.RangeMilliMeter<  FRONT_MIN_DISTANCE;  // TODO: should change the implementation

  // update the wall map according to sensor inputs
  if (leftWall) setWalls(wallMap, {X, Y}, orient, LEFT);
  if (rightWall) setWalls(wallMap, {X, Y}, orient, RIGHT);
  if (frontWall) setWalls(wallMap, {X, Y}, orient, FORWARD);

  // update the flood map according to wall map
  updateFullFloodArray(floodMap, wallMap);

  // get the next cell to go
  Point next = getNext(floodMap, wallMap, {X, Y}, orient);

  // TODO: move the mouse to the next cell
      // if point is different from current point rotate the mouse until find the correct orientation
      
      // else 
        // rotate the mouse according to given target orientation
        
        // move the mouse forward until it reaches the next cell
  if (next.x == X && next.y == Y) {
    // rotatet the mouse until it find the appropriate oreintation
    rotateMouse(RIGHT); // TODO: change the priority of the rotation to LEFT/RIGHT if changes needed 
    orient = getAbsDirection(orient, RIGHT); // set the new orientation after rotate
    return true;
  } 
    
    
  // rotate the mouse to the given orientation
  // find the direction we should rotate the mouse
  int direction = findDirection({X, Y}, next, orient);
  // TODO: rotate the mouse according to given target direction
  rotateMouse(direction);
  // move the mouse forward until it reaches the next cell
  // TODO: move the mouse forward until it reaches the next cell
  moveMouseForward();
  // delay(1000);
  // update the current position of the mouse and orientation
  X = next.x;
  Y = next.y;
  orient = getAbsDirection(orient, direction);
  // check whether mouse has reached the destination
  if (isFinished(floodMap, next)) {
    return false;
  }
  return true;
}