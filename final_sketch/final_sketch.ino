#include "Adafruit_VL53L0X.h"

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
#define LOX5_ADDRESS 0x35

// set the pins to shutdown
#define SHT_LOX1 18
#define SHT_LOX2 4
#define SHT_LOX3 2
#define SHT_LOX4 19
#define SHT_LOX5 5

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1; // original left
VL53L0X_RangingMeasurementData_t measure2; // extra left
VL53L0X_RangingMeasurementData_t measure3; // original right
VL53L0X_RangingMeasurementData_t measure4; // front
VL53L0X_RangingMeasurementData_t measure5; // extra right

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  digitalWrite(SHT_LOX5, LOW);

  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);
  digitalWrite(SHT_LOX5, HIGH);

  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  digitalWrite(SHT_LOX5, LOW);


  Serial.println("setId init");
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

  digitalWrite(SHT_LOX5, HIGH);
  delay(10);
  //initing LOX2
  if (!lox5.begin(LOX5_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }

  Serial.println("SetID end");
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
  pinMode(SHT_LOX5, OUTPUT);


  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  digitalWrite(SHT_LOX5, LOW);


  Serial.println("Both in reset mode...(pins are low)");


  Serial.println("Starting...");
  setID();

}

void loop() {
  
  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false);  // pass in 'true' to get debug data printout!
  lox5.rangingTest(&measure5, false);  // pass in 'true' to get debug data printout!

  int front = measure4.RangeMilliMeter;
  int left = measure1.RangeMilliMeter;
  int right = measure3.RangeMilliMeter;

  Serial.println("%d, %d, %d", front, left, right);
  delay(100);
}
