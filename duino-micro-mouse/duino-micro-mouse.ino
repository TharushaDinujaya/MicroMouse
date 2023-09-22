#include "Adafruit_VL53L0X.h"

#define pwmChannel1 0
#define pwmChannel2 1   // Selects channel 0
#define resolution 8    // 8-bit resolution, 256 possible values
#define frequency 30000 // PWM frequency of 1 KHz

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
#define LOX5_ADDRESS 0x38

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

// struct for represent the Point in the maze (Cell position)
struct Point
{
  int x;
  int y;
};

enum Orient
{
  NORTH = ORIENT_OFFSET + 0,
  EAST = ORIENT_OFFSET + 1,
  SOUTH = ORIENT_OFFSET + 2,
  WEST = ORIENT_OFFSET + 3
};

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID()
{
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
  if (!lox1.begin(LOX1_ADDRESS))
  {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // initing LOX2
  if (!lox2.begin(LOX2_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // initing LOX2
  if (!lox3.begin(LOX3_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }

  // activating LOX4
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  // initing LOX2
  if (!lox4.begin(LOX4_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }

  digitalWrite(SHT_LOX5, HIGH);
  delay(10);
  // initing LOX2
  if (!lox5.begin(LOX5_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }

  Serial.println("SetID end");
} // stop the right and left motors

Orient orient = NORTH; // initial orientation of the mouse
char pathString[1024]; // path string to store the path
int pathLength = 0;    // length of the path

// initial coordination of the mouse
int X = 0;
int Y = 0;

void read_dual_sensors()
{

  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!
  lox5.rangingTest(&measure5, false); // pass in 'true' to get debug data printout!

  int frontD, leftD, rightD, backD;
  if (measure1.RangeStatus != 4)
  { // if not out of range
    leftD = measure1.RangeMilliMeter;
  }
  else
  {
    Serial.print("Out of range");
  }

  // print sensor two reading
  if (measure2.RangeStatus != 4)
  {
    backD = measure2.RangeMilliMeter;
  }
  else
  {
    Serial.print("Out of range");
  }

  if (measure3.RangeStatus != 4)
  {
    rightD = measure3.RangeMilliMeter;
  }
  else
  {
    Serial.print("Out of range");
  }

  if (measure4.RangeStatus != 4)
  {
    frontD = measure4.RangeMilliMeter;
  }
  else
  {
    Serial.print("Out of range");
  }

  // Serial.printf("Front: %d, Left: %d, Right: %d, Back: %d\n", frontD, leftD, rightD, backD);

  int value = random(2);
  if (frontD > 50)
  {
    // forward(110, leftD, rightD, frontD);
    moveMouseForward();
    Point p = getPoint();
    X = p.x;
    Y = p.y;
  }
  else
  {

    int randNum = random(2);

    if (randNum % 2 == 0)
    {
      if (leftD > 100)
      {
        stop();
        left(120);
        delay(ROTATE_TIME);
        stop();
        pathString[pathLength++] = 'l';
        orient = getAbsDirection(orient, LEFT);
      }
      else if (rightD > 100)
      {
        stop();
        right(120);
        delay(ROTATE_TIME);
        stop();
        pathString[pathLength++] = 'r';
        orient = getAbsDirection(orient, RIGHT);
      }
      else
      {
        stop();
        right(120);
        delay(REVERSE_ROTATE_TIME);
        right(120);
        delay(REVERSE_ROTATE_TIME);
        stop();
        pathString[pathLength++] = 'r';
        pathString[pathLength++] = 'r';
        orient = getAbsDirection(orient, RIGHT);
        orient = getAbsDirection(orient, RIGHT);

        // adjust the mouse if it is out of range
        if (!inRange(leftD) || !inRange(rightD))
        {
          digitalWrite(in1A, HIGH);
          digitalWrite(in2A, LOW);
          digitalWrite(in1B, HIGH);
          digitalWrite(in2B, LOW);
          if (leftD < LOWER_THERSHOLD)
          {
            ledcWrite(pwmChannel1, 120);      // 1.65 V
            ledcWrite(pwmChannel2, 120 - 30); // 1.65 V
          }
          else
          {
            ledcWrite(pwmChannel1, 120 - 30); // 1.65 V
            ledcWrite(pwmChannel2, 120);      // 1.65 V
          }
        }
        delay(150);
        stop();
      }
    }
    else
    {
      if (rightD > 100)
      {
        stop();
        left(120);
        delay(ROTATE_TIME);
        stop();
        pathString[pathLength++] = 'r';
        orient = getAbsDirection(orient, LEFT);
      }
      else if (leftD > 100)
      {
        stop();
        right(120);
        delay(ROTATE_TIME);
        stop();
        pathString[pathLength++] = 'r';
        orient = getAbsDirection(orient, RIGHT);
      }
      else
      {
        stop();
        right(120);
        delay(REVERSE_ROTATE_TIME);
        right(120);
        delay(REVERSE_ROTATE_TIME);
        stop();
        pathString[pathLength++] = 'r';
        pathString[pathLength++] = 'r';
        orient = getAbsDirection(orient, RIGHT);
        orient = getAbsDirection(orient, RIGHT);

        // adjust the mouse if it is out of range
        if (!inRange(leftD) || !inRange(rightD))
        {
          digitalWrite(in1A, HIGH);
          digitalWrite(in2A, LOW);
          digitalWrite(in1B, HIGH);
          digitalWrite(in2B, LOW);
          if (leftD < LOWER_THERSHOLD)
          {
            ledcWrite(pwmChannel1, 120);      // 1.65 V
            ledcWrite(pwmChannel2, 120 - 30); // 1.65 V
          }
          else
          {
            ledcWrite(pwmChannel1, 120 - 30); // 1.65 V
            ledcWrite(pwmChannel2, 120);      // 1.65 V
          }
        }
        delay(150);
        stop();
      }
    }
  }
}

void setup()
{
}

void loop()
{
}

int forward(int speed, int left, int right, int front)
{

  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);

  if (front < FORWARD_MIN_DISTANCE)
  {
    //   int front_ = measure4.RangeMilliMeter;
    //   while (front_ > FORWARD_MIN_DISTANCE) {
    //     lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
    //     lox2.rangingTest(&measure2, false);
    //     lox3.rangingTest(&measure3, false);
    //     lox4.rangingTest(&measure4, false);  // pass in 'true' to get debug data printout!
    //     ledcWrite(pwmChannel1, speed);  // 1.65 V
    //     ledcWrite(pwmChannel2, speed);  // 1.65 V
    //     front_ = measure4.RangeMilliMeter;
    //   }
    return -1;
  }
  // ledcWrite(pwmChannel1, speed);  // 1.65 V
  // ledcWrite(pwmChannel2, speed);  // 1.65 V
  if ((inRange(left) && inRange(right)) || (left > CELL_SIZE && right > CELL_SIZE))
  {
    ledcWrite(pwmChannel1, speed); // 1.65 V
    ledcWrite(pwmChannel2, speed); // 1.65 V
  }
  else if (left > CELL_SIZE)
  {
    // int delta_s = get_speed(right);
    if (right < LOWER_THERSHOLD + 5)
    {
      ledcWrite(pwmChannel1, speed - 40); // 1.65 V
      ledcWrite(pwmChannel2, speed);      // 1.65 V
    }
    else
    {
      ledcWrite(pwmChannel1, speed);      // 1.65 V
      ledcWrite(pwmChannel2, speed - 40); // 1.65 V
    }
    delay(20);
    return 35;
  }
  else if (right > CELL_SIZE)
  {
    int delta_s = get_speed(left);
    if (left < LOWER_THERSHOLD + 5)
    {
      ledcWrite(pwmChannel1, speed);      // 1.65 V
      ledcWrite(pwmChannel2, speed - 40); // 1.65 V
    }
    else
    {
      ledcWrite(pwmChannel1, speed - 40); // 1.65 V
      ledcWrite(pwmChannel2, speed);      // 1.65 V
    }
    delay(20);
    return 35;
  }
  else
  {
    if (left < LOWER_THERSHOLD)
    {
      ledcWrite(pwmChannel1, speed);      // 1.65 V
      ledcWrite(pwmChannel2, speed - 40); // 1.65 V
      delay(20);
      return 35;
    }
    else if (right < LOWER_THERSHOLD)
    {
      ledcWrite(pwmChannel1, speed - 40); // 1.65 V
      ledcWrite(pwmChannel2, speed);      // 1.65 V
      delay(20);
      return 35;
    }
    else
    {
      ledcWrite(pwmChannel1, speed); // 1.65 V
      ledcWrite(pwmChannel2, speed); // 1.65 V
    }
  }
  return 0;
}

void reverse(int speed)
{
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannel1, speed); // 1.65 V
  ledcWrite(pwmChannel2, speed); // 1.65 V
}

void right(int speed)
{
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannel1, speed); // 1.65 V
  ledcWrite(pwmChannel2, speed); // 1.65 V
}

void left(int speed)
{
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  ledcWrite(pwmChannel1, speed); // 1.65 V
  ledcWrite(pwmChannel2, speed); // 1.65 V
}

void stop()
{
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
}

void moveMouseForward(void)
{
  // get the distance from the front TOF sensor at the beginning
  int frontDistance = measure4.RangeMilliMeter;
  int initialDistance = frontDistance;

  int startTime = millis();
  while (millis() - startTime < 2000)
  {
    lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
    lox2.rangingTest(&measure2, false);
    lox3.rangingTest(&measure3, false);
    lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!
    lox5.rangingTest(&measure5, false); // pass in 'true' to get debug data printout!

    int leftD = measure1.RangeMilliMeter;
    int rightD = measure3.RangeMilliMeter;
    int frontD = measure4.RangeMilliMeter;
    int delta_t = forward(FORWARD_SPEED, leftD, rightD, frontD); // move the mouse forward under given speed // TODO:
    if (delta_t == -1)
      break;
    startTime += delta_t;
  }
  // start the motor drive to move forward
  //   while (frontDistance > initialDistance - CELL_SIZE) {
  //     delay(50);
  //     frontDistance = measure1.RangeMilliMeter; // distance from the front TOF sensor
  //   }
  // forward(125);
  // delay(2000);
  stop(); //  stop the motor drive
}