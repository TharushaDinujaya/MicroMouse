#include "Adafruit_VL53L0X.h"

#define ROTATE_TIME 415
#define THRESHOLD 45

#define pwmChannel1 0
#define pwmChannel2 1    // Selects channel 0
#define resolution 8     // 8-bit resolution, 256 possible values
#define frequency 30000  // PWM frequency of 1 KHz

// Motor A
#define pwmA 26
#define in1A 33
#define in2A 25

// Motor B
#define pwmB 32
#define in1B 27
#define in2B 14

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x34


// set the pins to shutdown
#define SHT_LOX1 18
#define SHT_LOX2 4
#define SHT_LOX3 2
#define SHT_LOX4 19

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

void initializeMotors();
void forward(int speed, int left, int right);  // rotate the motors in which mouse move forward
void reverse(int speed);  // reverse the motors
void right(int speed);    // turn right
void left(int speed);     // turn left
void stop();              // stop the right and left motors

void read_dual_sensors() {

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false);  // pass in 'true' to get debug data printout!

  int frontD, leftD, rightD, backD;
  if (measure1.RangeStatus != 4) {  // if not out of range
    leftD = measure1.RangeMilliMeter;
  } else {
    Serial.print("Out of range");
  }

  // print sensor two reading
  if (measure2.RangeStatus != 4) {
    backD = measure2.RangeMilliMeter;
  } else {
    Serial.print("Out of range");
  }

  if (measure3.RangeStatus != 4) {
    rightD = measure3.RangeMilliMeter;
  } else {
    Serial.print("Out of range");
  }

  if (measure4.RangeStatus != 4) {
    frontD = measure4.RangeMilliMeter;
  } else {
    Serial.print("Out of range");
  }

  Serial.printf("Front: %d, Left: %d, Right: %d, Back: %d\n", frontD, leftD, rightD, backD);

  if (frontD > 80) {
      forward(110, leftD, rightD);
    } else {
      

     if (leftD > 100) {
        stop();
        left(120);
        delay(ROTATE_TIME);
        stop();
      } else if (rightD > 100) {
        stop();
        right(120);
        delay(ROTATE_TIME);
        stop();
      } else {
        stop();
      }
    }
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial) { delay(1); }

  initializeMotors();
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
  setID();
}

void loop() {

  read_dual_sensors();
  delay(10);
}

// motors
void initializeMotors() {
  // set the appropriate pins for motors
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
}

void forward(int speed, int left, int right) {
  
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  if (left > THRESHOLD && right > THRESHOLD) {
    ledcWrite(pwmChannel1, speed);  // 1.65 V
    ledcWrite(pwmChannel2, speed);  // 1.65 V
  } else if (left < THRESHOLD && right > THRESHOLD) {
    ledcWrite(pwmChannel1, speed + 20);  // 1.65 V
    ledcWrite(pwmChannel2, speed - 20);  // 1.65 V
  } else if (right < THRESHOLD && left > THRESHOLD) {
    ledcWrite(pwmChannel1, speed - 20);  // 1.65 V
    ledcWrite(pwmChannel2, speed + 20);  // 1.65 V
  } else {
    ledcWrite(pwmChannel1, speed);  // 1.65 V
    ledcWrite(pwmChannel2, speed);  // 1.65 V
  }
}

void reverse(int speed) {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannel1, speed);  // 1.65 V
  ledcWrite(pwmChannel2, speed);  // 1.65 V
}

void right(int speed) {
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannel1, speed);  // 1.65 V
  ledcWrite(pwmChannel2, speed);  // 1.65 V
}

void left(int speed) {
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