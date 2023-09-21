#include "Adafruit_VL53L0X.h"

#define FINISHING_X 5
#define FINISHING_Y 5

#define ORIENT_OFFSET 100
#define ROTATE_TIME 530
#define ROTATE_SPEED 120
#define FORWARD_SPEED 110
#define REVERSE_ROTATE_TIME 560
#define LOWER_THERSHOLD 50
#define HIGHER_THRESHOLD 55
#define CELL_SIZE 144
#define MAX_PATH_LENGTH 200
#define FORWARD_MIN_DISTANCE 53
#define SIDE_MIN_DISTANCE 100

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
struct Point {
    int x;
    int y;
};

enum Orient {
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

void initializeMotors();
int forward(int speed, int left, int right, int front);  // rotate the motors in which mouse move forward
void reverse(int speed);  // reverse the motors
void right(int speed);    // turn right
void left(int speed);     // turn left
void stop();              // stop the right and left motors

// MAZE declarations
#define MAZE_SIZE 10
#define MAZE_CELLS (MAZE_SIZE * MAZE_SIZE)
#define UNDEFINED 9999
#define ORIENT_OFFSET 100
#define CELL_SIZE 148

// definitions for represent the orientation of the robot
#define LEFT -1
#define RIGHT 1
#define FORWARD 0

// symbols for represent wall configuration of cells in the maze
#define WALL_NONE 0
#define WALL_LEFT 1
#define WALL_FRONT 2
#define WALL_RIGHT 4
#define WALL_BACK 8

// create two arrays to represent wall map and flood index map with predefine sizes
int wallMap[MAZE_SIZE][MAZE_SIZE];
int floodMap[MAZE_SIZE][MAZE_SIZE];
Orient orient = NORTH; // initial orientation of the mouse
char pathString[MAX_PATH_LENGTH]; // path string to store the path
int pathLength = 0; // length of the path
int finish_val = 0;

// initial coordination of the mouse
int X = 9;
int Y = 0;

// queue declarations
struct Queue {
    Point* queue;
    int front;
    int back;
    int size;
};
#define DEFAULT_SIZE 20
Point queue_arr[DEFAULT_SIZE];
Queue queue = {queue_arr, 0, 0, DEFAULT_SIZE};

// Queue* createQueue(int size = DEFAULT_SIZE);
// void deleteQueue();
void resetQueue();
Point dequeue();
void enqueue(Point point);
bool empty();

// helper function implementations
bool isLeftWall(Point current) {
    return wallMap[current.x][current.y] & WALL_LEFT;
}

bool isRightWall(Point current) {
    return wallMap[current.x][current.y] & WALL_RIGHT;
}

bool isFrontWall(Point current) {
    return wallMap[current.x][current.y] & WALL_FRONT;
}

bool isBackWall(Point current) {
    return wallMap[current.x][current.y] & WALL_BACK;
}

bool isReversed(Orient orient1, Orient orient2) {
    int diff = orient1 - orient2;
    return diff == -2 || diff == 2;
}

// include utility function to run the maze solving algorithm
void initializeMaze(); // initialize the maze array with empty cells
void updateFullFloodArray(); // update the flood index of cells based on wall configutrations
void resetFloodMap(); // reset the flood index array
Point getNext(Point current, Orient orient); // get the position of the cell mouse should go next
// void go(Orient orient, Point point); // move the mouse to the given position with given orientation
void setWalls(Point current, Orient orient, int direction);
bool isFinished(Point current); // check if the mouse has reached the destination
// void optimizedFloodMapFill(Point current, Orient orient, int direction); // update the flood index of cells based on wall configutrations
int findDirection(Point current, Point next, Orient orient); // find the direction which we need to rotate based on current, next point and current orienttation
void print(int X, int Y);
bool inRange(int distance);

Orient getAbsDirection(Orient orient, int direction);
bool isReversed(Orient orient1, Orient orient2);

bool run(); // main running loop of the micromouse
void returnToStart(const char pathString[MAX_PATH_LENGTH], int pathLength); // return to the start point
void rotateMouse(int direction);
void moveMouseForward();
void rotateReverse(int leftD, int rightD);

bool checkFinish(int front, int left, int right, int back);

Point getPoint() {
  if (orient == NORTH) {
    return {X, Y + 1};
  } else if (orient == SOUTH) {
    return {X, Y - 1};
  } else if (orient == EAST) {
    return {X - 1, Y};
  } else {
    return {X + 1, Y};
  }
}

void read_dual_sensors() {

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false);  // pass in 'true' to get debug data printout!
  lox5.rangingTest(&measure5, false);  // pass in 'true' to get debug data printout!


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

  // Serial.printf("Front: %d, Left: %d, Right: %d, Back: %d\n", frontD, leftD, rightD, backD);

  int value = random(2);
  if (frontD > 50) {
      // forward(110, leftD, rightD, frontD);
      moveMouseForward();
      Point p = getPoint();
      X = p.x;
      Y = p.y;
  } else {
      
      int randNum = random(2);

      if (randNum % 2 == 0) {
        if (leftD > 100) {
          stop();
          left(120);
          delay(ROTATE_TIME);
          stop();
          pathString[pathLength++] = 'l';
          orient = getAbsDirection(orient, LEFT);
        } else if (rightD > 100) {
          stop();
          right(120);
          delay(ROTATE_TIME);
          stop();
          pathString[pathLength++] = 'r';
          orient = getAbsDirection(orient, RIGHT);

        } else {
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
          if (!inRange(leftD) || !inRange(rightD)) {
            digitalWrite(in1A, HIGH);
            digitalWrite(in2A, LOW);
            digitalWrite(in1B, HIGH);
            digitalWrite(in2B, LOW);
            if (leftD < LOWER_THERSHOLD) {
                ledcWrite(pwmChannel1, 120);  // 1.65 V
                ledcWrite(pwmChannel2, 120 - 30);  // 1.65 V
            } else {
                ledcWrite(pwmChannel1, 120 - 30);  // 1.65 V
                ledcWrite(pwmChannel2, 120);  // 1.65 V
            }
          }
          delay(150);
          stop();
        }
      } else {
          if (rightD > 100) {
          stop();
          left(120);
          delay(ROTATE_TIME);
          stop();
          pathString[pathLength++] = 'r';
          orient = getAbsDirection(orient, LEFT);

        } else if (leftD > 100) {
          stop();
          right(120);
          delay(ROTATE_TIME);
          stop();
          pathString[pathLength++] = 'r';
          orient = getAbsDirection(orient, RIGHT);

        } else {
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
          if (!inRange(leftD) || !inRange(rightD)) {
            digitalWrite(in1A, HIGH);
            digitalWrite(in2A, LOW);
            digitalWrite(in1B, HIGH);
            digitalWrite(in2B, LOW);
            if (leftD < LOWER_THERSHOLD) {
                ledcWrite(pwmChannel1, 120);  // 1.65 V
                ledcWrite(pwmChannel2, 120 - 30);  // 1.65 V
            } else {
                ledcWrite(pwmChannel1, 120 - 30);  // 1.65 V
                ledcWrite(pwmChannel2, 120);  // 1.65 V
            }
          }
          delay(150);
          stop();
      }
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
  initializeMaze(); // array initialize routines goes here

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false);  // pass in 'true' to get debug data printout!
  lox5.rangingTest(&measure5, false);  // pass in 'true' to get debug data printout!


  int front = measure4.RangeMilliMeter;
  if (front > CELL_SIZE && front < 2*CELL_SIZE) {
    X = 9;
    Y = 0;
  } else {
    X = 0;
    Y = 0;
  }

}

void loop() {

  // lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  // lox2.rangingTest(&measure2, false);
  // lox3.rangingTest(&measure3, false);
  // lox4.rangingTest(&measure4, false);  // pass in 'true' to get debug data printout!
  // lox5.rangingTest(&measure5, false);  // pass in 'true' to get debug data printout!

  // int m1 = measure1.RangeMilliMeter;
  // int m2 = measure2.RangeMilliMeter;
  // int m3 = measure3.RangeMilliMeter;
  // int m4 = measure4.RangeMilliMeter;
  // int m5 = measure5.RangeMilliMeter;

  // Serial.printf("m1: %d, m2: %d, m3; %d, m4; %d, m5: %d\n", m1, m2, m3, m4, m5);
  // delay(2500);
  while (true) {
    read_dual_sensors();
  }
  // if (X == FINISHING_X && Y == FINISHING_Y) {
  //   while (true) {
  //     digitalWrite(BUILTIN_LED, HIGH);
  //     delay(300);
  //     digitalWrite(BUILTIN_LED, LOW);
  //     delay(300);
  //   }
  // }
  
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

bool inRange(int distance) {
  return distance > LOWER_THERSHOLD && distance < HIGHER_THRESHOLD;
}

int get_speed(int distance) {
  return (int) (0.67 * distance + 63);
}

int forward(int speed, int left, int right, int front) {
  
  
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  
  if (front < FORWARD_MIN_DISTANCE) {
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
  if ((inRange(left) && inRange(right)) || (left > CELL_SIZE && right > CELL_SIZE)) {
    ledcWrite(pwmChannel1, speed);  // 1.65 V
    ledcWrite(pwmChannel2, speed);  // 1.65 V
  } else if (left > CELL_SIZE) {
    // int delta_s = get_speed(right);
    if (right < LOWER_THERSHOLD + 5) {
      ledcWrite(pwmChannel1, speed - 40);  // 1.65 V
      ledcWrite(pwmChannel2, speed);  // 1.65 V
    } else {
      ledcWrite(pwmChannel1, speed);  // 1.65 V
      ledcWrite(pwmChannel2, speed - 40);  // 1.65 V
    }
    delay(20);
    return 35;
  } else if (right > CELL_SIZE) {
    int delta_s = get_speed(left);
    if (left < LOWER_THERSHOLD + 5) {
      ledcWrite(pwmChannel1, speed);  // 1.65 V
      ledcWrite(pwmChannel2, speed - 40);  // 1.65 V
    } else {
      ledcWrite(pwmChannel1, speed - 40);  // 1.65 V
      ledcWrite(pwmChannel2, speed);  // 1.65 V
    }
    delay(20);
    return 35;

   }
else {
    if (left < LOWER_THERSHOLD) {
      ledcWrite(pwmChannel1, speed);  // 1.65 V
      ledcWrite(pwmChannel2, speed - 40);  // 1.65 V
      delay(20);
      return 35;
    } else if (right < LOWER_THERSHOLD) {
      ledcWrite(pwmChannel1, speed - 40);  // 1.65 V
      ledcWrite(pwmChannel2, speed);  // 1.65 V
      delay(20);
      return 35;
    } else {
      ledcWrite(pwmChannel1, speed);  // 1.65 V
      ledcWrite(pwmChannel2, speed);  // 1.65 V
    }
  }
  return 0;
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

// queue functions
void resetQueue(){
    queue.front = -1;
    queue.back = -1;

}

Point dequeue() {
    if (queue.front == queue.back) return {0, 0}; // queue is empty

    queue.front = (queue.front + 1) % queue.size;
    return queue.queue[queue.front];
    
}

void enqueue(Point value) {
    if ((queue.back + 1) % queue.size == queue.front) return; // queue is full

    queue.back = (queue.back + 1) % queue.size;
    queue.queue[queue.back] = value;
}

bool empty() {
  return (queue.front == queue.back);
}

// end of queue functions


// utility functions implementations

Point getNextPoint(int x, int y, Orient orient, int direction) {

    // get the absolute direction using the current orientation and the direction
    Orient absOrientation = getAbsDirection(orient, direction);
    
    Point p = {x, y};
    // based on abs direction return the next cell
    switch (absOrientation)
    {
    case NORTH:
        p = {x, y + 1};
        break;
    case EAST:
        p = {x + 1, y};
        break;
    case SOUTH:
        p = {x, y - 1};
        break;
    case WEST:
        p = {x - 1, y};
        break;
    default:
        break;
    }

    return p;
}

Orient getAbsDirection(Orient orient, int direction) {
    int tmp = (orient + direction - ORIENT_OFFSET) % 4;
    if (tmp < 0) tmp += 4;

    return (Orient) (tmp + ORIENT_OFFSET);
}

// implementation of the solving algorithm functions
void initializeMaze() {
    // initialize the maze array with empty cells
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            wallMap[i][j] = WALL_NONE;
            floodMap[i][j] = UNDEFINED;
        }
    }

    // set the boundry walls of the maze
    for (int i = 0; i < MAZE_SIZE; i++) {
        wallMap[i][0] |= WALL_BACK;
        wallMap[i][MAZE_SIZE - 1] |= WALL_FRONT;
        wallMap[0][i] |= WALL_LEFT;
        wallMap[MAZE_SIZE - 1][i] |= WALL_RIGHT;
    }

    // set the destination cell's flood index as zero
    floodMap[FINISHING_X][FINISHING_Y] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y] = 0;
    floodMap[FINISHING_X][FINISHING_Y + 1] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y + 1] = 0;

}

void resetFloodMap() {
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            floodMap[i][j] = UNDEFINED;
        }
    }

    // fill the middle 4 cells with 0
    floodMap[FINISHING_X][FINISHING_Y] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y] = 0;
    floodMap[FINISHING_X][FINISHING_Y + 1] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y + 1] = 0;
}

void updateFullFloodArray() {

    // static Queue *q = createQueue(DEFAULT_SIZE); // create a queue to store the cells to be updated
    // reset the given queue
    resetQueue();

    // add the middle 4 cells to the queue
    enqueue({FINISHING_X, FINISHING_Y});
    enqueue({FINISHING_X + 1, FINISHING_Y});
    enqueue({FINISHING_X, FINISHING_Y + 1});
    enqueue({FINISHING_X + 1, FINISHING_Y + 1});

    // run the while loop until the queue is not empty
    while (!empty()) {
        // fetch the next cell from the queue
        Point current = dequeue();
        // get the flood index of the current position
        int current_flood_index = floodMap[current.x][current.y];

        // check if the cell on the left is not a wall and the flood index is undefined
        if (!isLeftWall(current) && floodMap[current.x - 1][current.y] == UNDEFINED) {
            floodMap[current.x - 1][current.y] = current_flood_index + 1;
            enqueue({current.x - 1, current.y}); // add the cell to the queue
        }
        // check if the cell on the right is not a wall and the flood index is undefined
        if (!isRightWall(current) && floodMap[current.x + 1][current.y] == UNDEFINED) {
            floodMap[current.x + 1][current.y] = current_flood_index + 1;
            enqueue({current.x + 1, current.y}); // add the cell to the queue
        } 
        
        // check if the cell on the front is not a wall and the flood index is undefined
        if (!isFrontWall(current) && floodMap[current.x][current.y + 1] == UNDEFINED) {
            floodMap[current.x][current.y + 1] = current_flood_index + 1;
            enqueue({current.x, current.y + 1}); // add the cell to the queue
        } 
        
        // check if the cell on the back is not a wall and the flood index is undefined
        if (!isBackWall(current) && floodMap[current.x][current.y - 1] == UNDEFINED) {
            floodMap[current.x][current.y - 1] = current_flood_index + 1;
            enqueue({current.x, current.y - 1}); // add the cell to the queue
        }


    }
}

Point getNext(Point current, Orient orient) {

    Point next;

    int northIndex = floodMap[current.x][current.y + 1];
    int eastIndex = floodMap[current.x + 1][current.y];
    int southIndex = floodMap[current.x][current.y - 1];
    int westIndex = floodMap[current.x - 1][current.y];

    int minIndex = UNDEFINED;
    if (minIndex > northIndex && !isFrontWall(current) && !isReversed(orient, NORTH)) {
        minIndex = northIndex;
    }

    if (minIndex > eastIndex && !isRightWall(current) && !isReversed(orient, EAST)) {
        minIndex = eastIndex;
    }

    if (minIndex > southIndex && !isBackWall(current) && !isReversed(orient, SOUTH)) {
        minIndex = southIndex;
    }

    if (minIndex > westIndex && !isLeftWall(current) && !isReversed(orient, WEST)) {
        minIndex = westIndex;
    }
    
    if (minIndex == UNDEFINED) {
        // if no cell is found return the current cell
        return current;
    }

    // get the next cell according to the min flood index
    if (minIndex == northIndex) {
        next = {current.x, current.y + 1};
    } else if (minIndex == eastIndex) {
        next = {current.x + 1, current.y};
    } else if (minIndex == southIndex) {
        next = {current.x, current.y - 1};
    } else if (minIndex == westIndex) {
        next = {current.x - 1, current.y};
    }
    return next;
}

void setWalls(Point current, Orient orient, int direction) {

    // get the next cell according to the directions
    Point next = getNextPoint(current.x, current.y, orient, direction);
    Orient absOrient = getAbsDirection(orient, direction);

    switch (absOrient)
    {
    case NORTH:
        Serial.print("NORTH WALL SET\n");
        wallMap[current.x][current.y] |= WALL_FRONT;
        wallMap[next.x][next.y] |= WALL_BACK;
        break;

    case EAST:
        Serial.print("RIGHT WALL SET\n");
        wallMap[current.x][current.y] |= WALL_RIGHT;
        wallMap[next.x][next.y] |= WALL_LEFT;
        break;
    
    case SOUTH:
        Serial.print("SOUTH WALL SET\n");
        wallMap[current.x][current.y] |= WALL_BACK;
        wallMap[next.x][next.y] |= WALL_FRONT;
        break;

    case WEST:
        Serial.print("LEFT WALL SET\n");
        wallMap[current.x][current.y] |= WALL_LEFT;
        wallMap[next.x][next.y] |= WALL_RIGHT;
        break;
    
    default:
        break;
    }

}

bool isFinished(Point current) {
    // check if the current position is the destination
    return floodMap[current.x][current.y] == 0;
}

int findDirection(Point current, Point next, Orient orient) {
    
    Orient nextCellOrient;

    if (next.y == current.y + 1) nextCellOrient = NORTH;
    else if (next.y == current.y - 1) nextCellOrient = SOUTH;
    else if (next.x == current.x + 1) nextCellOrient = EAST;
    else if (next.x == current.x - 1) nextCellOrient = WEST;

    // return the direction we need to rotate according to current orientation
    int tmp = nextCellOrient - orient;
    if (tmp == 3) return LEFT;
    if (tmp == -3) return RIGHT;
    else return tmp;
}

// end of utility functions
void rotateMouse(int direction) {
  // rotate the mouse according to given target direction
  switch (direction)
  {
  case LEFT:
    left(ROTATE_SPEED); // rotate the motors at given speed
    delay(ROTATE_TIME); // maintain the rotation until specified time is expired
    stop();
    pathString[pathLength++] = 'l'; // update the pathString and pathLength according to the rotation
    break;
  case RIGHT:
    right(ROTATE_SPEED); // rotate the motors at given speed
    delay(ROTATE_TIME); // maintain the rotation until specified time is expired
    stop();
    pathString[pathLength++] = 'r'; // update the pathString and pathLength according to the rotation
    break;
  case FORWARD:
    // nothing to do here
    break;
  default:
    break;
  }
}

void moveMouseForward(void) {
  // get the distance from the front TOF sensor at the beginning
    int frontDistance = measure4.RangeMilliMeter;
    int initialDistance = frontDistance;

    int startTime = millis();
    while (millis() - startTime < 2000) {
        lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
        lox2.rangingTest(&measure2, false);
        lox3.rangingTest(&measure3, false);
        lox4.rangingTest(&measure4, false);  // pass in 'true' to get debug data printout!
        lox5.rangingTest(&measure5, false);  // pass in 'true' to get debug data printout!

        int leftD = measure1.RangeMilliMeter;
        int rightD = measure3.RangeMilliMeter;
        int frontD = measure4.RangeMilliMeter;
        int delta_t = forward(FORWARD_SPEED, leftD, rightD, frontD); // move the mouse forward under given speed // TODO:
        if( delta_t == -1) break;
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

bool run(void) {
  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false);  // pass in 'true' to get debug data printout!
  // reset the flood map
  resetFloodMap();
  
  // Serial.print("%d, %d\n", X, Y);
  // Serial.printf("L: %d, R: %d. F: %d\n", measure1.RangeMilliMeter, measure3.RangeMilliMeter, measure4.RangeMilliMeter);
  bool leftWall = measure1.RangeMilliMeter < SIDE_MIN_DISTANCE; // check whether there is a wall on the left side
  bool rightWall = measure3.RangeMilliMeter < SIDE_MIN_DISTANCE; // check whether there is a wall on the right side
  bool frontWall = measure4.RangeMilliMeter <  FORWARD_MIN_DISTANCE; // check whether there is a wall on the front side

  // update the wall map according to sensor inputs
  if (leftWall) setWalls({X, Y}, orient, LEFT);
  if (rightWall) setWalls({X, Y}, orient, RIGHT);
  if (frontWall) setWalls({X, Y}, orient, FORWARD);
  print(X, Y);
  // update the flood map according to wall map
  updateFullFloodArray();

  // get the next cell to go
  Point next = getNext({X, Y}, orient);

  // TODO: move the mouse to the next cell
      // if point is different from current point rotate the mouse until find the correct orientation
      
      // else 
        // rotate the mouse according to given target orientation
        
        // move the mouse forward until it reaches the next cell
  if (next.x == X && next.y == Y) {
    // rotatet the mouse until it find the appropriate oreintation
    // TODO: change the priority of the rotation to LEFT/RIGHT if changes needed 
    rotateReverse(measure1.RangeMilliMeter, measure3.RangeMilliMeter);
    orient = getAbsDirection(orient, RIGHT); // set the new orientation after rotate
    orient = getAbsDirection(orient, RIGHT); // set the new orientation after rotate
    // update the pathString and pathLength
    pathString[pathLength++] = 'r';
    stop();
    // print(X, Y);
    // delay(5000);

    return true;
  } 
    
    
  // rotate the mouse to the given orientation
  // find the direction we should rotate the mouse
  int direction = findDirection({X, Y}, next, orient);
  // rotate the mouse according to given target direction
  rotateMouse(direction);
  // move the mouse forward until it reaches the next cell
  // move the mouse forward until it reaches the next cell
  moveMouseForward();
  pathString[pathLength++] = 'f'; // update the pathString and pathLength according to the forward movement
  // delay(1000);
  // update the current position of the mouse and orientation
  X = next.x;
  Y = next.y;
  orient = getAbsDirection(orient, direction);
  // check whether mouse has reached the destination
  if (isFinished({X, Y})) {
    return false;
  }

  stop();
  int frontD = measure4.RangeMilliMeter;
  int leftD = measure1.RangeMilliMeter;
  int rightD = measure3.RangeMilliMeter;
  int backD = measure1.RangeMilliMeter;
  // checkFinish(frontD, leftD, rightD, backD);
  return true;
}

void returnToStart(const char pathString[MAX_PATH_LENGTH], int pathLength) {

  // first rotate the mouse reversed
  rotateMouse(RIGHT);
  rotateMouse(RIGHT); // now mouse in the reversed direction

  for (int i = pathLength; i > 0; i--) {
    char c = pathString[i - 1];

    if (c == 'f') moveMouseForward(); // forward the mouse
    else if (c == 'l') rotateMouse(RIGHT); // turn right the mouse
    else if (c == 'r') rotateMouse(LEFT); // turn left the mouse
  }

  // again reverse the direction of the mouse
  rotateMouse(RIGHT);
  rotateMouse(RIGHT);

}

void rotateReverse(int leftD, int rightD) {
    stop();
    right(120);
    delay(REVERSE_ROTATE_TIME);
    right(120);
    delay(REVERSE_ROTATE_TIME);
    stop();

        // adjust the mouse if it is out of range
    if (!inRange(leftD) || !inRange(rightD)) {
        digitalWrite(in1A, HIGH);
        digitalWrite(in2A, LOW);
        digitalWrite(in1B, HIGH);
        digitalWrite(in2B, LOW);
        if (leftD < LOWER_THERSHOLD) {
            ledcWrite(pwmChannel1, 120);  // 1.65 V
            ledcWrite(pwmChannel2, 120 - 30);  // 1.65 V
        } else {
            ledcWrite(pwmChannel1, 120 - 30);  // 1.65 V
            ledcWrite(pwmChannel2, 120);  // 1.65 V
        }
    }
    delay(150);
    stop();
}

void print(int X, int Y) {
  int val = wallMap[X][Y];
  Serial.printf("X: %d, Y: %d --> ", X, Y);
  int isL = val & WALL_LEFT;
  int isR = val & WALL_RIGHT;
  int isF = val & WALL_FRONT;
  int isB = val & WALL_BACK;
  Serial.printf(" LEFT: %d, RIGHt: %d, FRONT: %d, BACK: %d\n", isL, isR, isF, isB);

}

bool checkFinish(int front, int left, int right, int back) {
  if (((front > CELL_SIZE && front < 2*CELL_SIZE) || (back < CELL_SIZE && back > 2*CELL_SIZE)) && ((left > CELL_SIZE && left < 2*CELL_SIZE) || (right > CELL_SIZE && right < 2*CELL_SIZE))) {
    finish_val++;
  } else {
    finish_val = 0;
  }
}
