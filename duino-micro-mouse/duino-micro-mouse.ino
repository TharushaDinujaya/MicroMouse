#include "Adafruit_VL53L0X.h"
#include "user.h"
#include "motor.h"
#include "TOF.h"
#include "solver.h"

// objects for the vl53l0x
extern Adafruit_VL53L0X lox1;
extern Adafruit_VL53L0X lox2;
extern Adafruit_VL53L0X lox3;
extern Adafruit_VL53L0X lox4;
extern Adafruit_VL53L0X lox5;

extern VL53L0X_RangingMeasurementData_t measure1; // LEFT ORIGINAL TOF
extern VL53L0X_RangingMeasurementData_t measure2; // LEFT EXTRA TOF
extern VL53L0X_RangingMeasurementData_t measure3; // RIGHT ORIGINAL TOF
extern VL53L0X_RangingMeasurementData_t measure4; // FRONT TOF
extern VL53L0X_RangingMeasurementData_t measure5; // RIGHT EXTRA TOF

// maze variables
int X, Y;
Orient orient;
char pathString[BUFFER_MAX_LENGTH];
int pathLength = 0;

void setup()
{
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial)
  {
    delay(1);
  }

  initializeMotors();
  initializeTOF();
  // log("Initializing the mouse...");

  setID();
  // initializeMaze(wallMap, floodMap); // array initialize routines goes here
}

void rotateMouse(int direction)
{
  switch (direction)
  {
  case RIGHT:
    left(ROTATE_SPEED); // rotate the motors at given speed
    delay(ROTATE_TIME); // maintain the rotation until specified time is expired
    stop();
    break;
  case LEFT:
    right(ROTATE_SPEED);
    delay(ROTATE_TIME);
    stop();
    break;
  case FORWARD:
    break;
  default:
    break;
  }
}

void mouseForward()
{
  // get the current time in miilis
  int startTime = millis();
  while (millis() - startTime < FORWARD_TIME)
  {
    // lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
    // lox2.rangingTest(&measure2, false);
    // lox3.rangingTest(&measure3, false);
    // lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!

    // fetch the distance from the sensor
    DistanceMetrix mat = loadDistnaces();
    forward(FORWARD_SPEED);
  }
  stop();
}

void searchRun()
{
  X = 0, Y = 0; // reset the coordinates
  while (true)
  {
    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);
    lox3.rangingTest(&measure3, false);
    lox4.rangingTest(&measure4, false);
    lox5.rangingTest(&measure5, false);

    // fetch the distance from the sensor
    int leftDistance = measure1.RangeMilliMeter;
    int frontDistance = measure4.RangeMilliMeter;
    int rightDistance = measure3.RangeMilliMeter;

    // set the walls according to this distance
    if (leftDistance < SIDE_MIN_DISTANCE)
      setWalls({X, Y}, orient, LEFT);
    if (frontDistance < FRONT_MIN_DISTANCE)
      setWalls({X, Y}, orient, FORWARD);
    if (rightDistance < SIDE_MIN_DISTANCE)
      setWalls({X, Y}, orient, RIGHT);

    // reset the flood map
    resetFloodMap();
    // update the floodmap based on wallMap
    flooded();

    // find the next point
    Point next = findNext({X, Y}, orient);

    if (next.x == X && next.y == Y)
    {
      // rotate the mouse
      // rotateReverse(leftDistance, rightDistance);
      // set the direction of the mouse
      // orient = getAbsDirection(orient, RIGHT);
      rotateMouse(RIGHT);
      orient = getAbsDirection(orient, RIGHT);
      pathString[pathLength++] = 'R';
      continue;
    }

    int direction = findDirection({X, Y}, next, orient);
    // rotate to the specified direction
    rotateMouse(direction);
    if (direction == LEFT)
      pathString[pathLength++] = 'L';
    else if (direction == RIGHT)
      pathString[pathLength++] = 'R';

    // forward the mouse
    mouseForward(); // forward the mouse until it reaches the next cell
    pathString[pathLength++] = 'F';

    X = next.x;
    Y = next.y;
    orient = getAbsDirection(orient, direction);
    // check whether mouse has reached the destination
    if (isFinished({X, Y}))
    {
      break;
    }
  }
}

void returnToStart()
{
  // first take the full rotate
  rotateMouse(RIGHT);
  rotateMouse(RIGHT);

  for (int i = pathLength - 1; i >= 0; i--)
  {
    char c = pathString[i];
    if (c == 'F')
    {
      mouseForward();
    }
    else if (c == 'L')
    {
      rotateMouse(RIGHT);
    }
    else if (c == 'R')
    {
      rotateMouse(LEFT);
    }
  }
}

void loop()
{

  // log("Starting the program...");
  // blink the LED to indicate the start of the program
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

#if TEST
  test();
#endif

#if SEARCH_RUN
  searchRun();
#endif

#if FAST_RUN
#endif

  while (true)
  {
  }
}
