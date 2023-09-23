#include <Adafruit_VL53L0X.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#define LED 2
/* user configuratins and conatnt definitions */
// program related constants
#define TEST 0
#define FOLLOW_TEST 0
#define SEARCH_RUN 1
#define FAST_RUN 0

#define ROUNDS 1

int FINISHING_X =  8;
int FINISHING_Y = 3;

// Time related constants
#define FORWARD_TIME 2000

#define ROTATE_TIME 440
#define ROTATE_SPEED 120
#define FORWARD_SPEED 110 // 110
#define REVERSE_ROTATE_TIME 560
#define LOWER_THERSHOLD 50
#define HIGHER_THRESHOLD 55
#define CELL_SIZE 144
#define MAX_PATH_LENGTH 200
#define FRONT_MIN_DISTANCE 80
#define SIDE_MIN_DISTANCE 100

#define LEFT_MIN_THRESHOLD 67
#define RIGHT_MIN_THRESHOLD 47
#define LEFT_MAX_THRESHOLD 74
#define RIGHT_MAX_THRESHOLD 53

#define BUFFER_MAX_LENGTH 1024
/* end of config info and constants */

#define pwmChannel1 0
#define pwmChannel2 1	// Selects channel 0
#define resolution 8	// 8-bit resolution, 256 possible values
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

// function headers
void initializeMotors();
void setID();

struct DistanceMetrix
{
	int leftFront;
	int leftBack;
	int rightFront;
	int rightBack;
	int front;
};

// maze related constants and definitions
#define MAZE_SIZE 10
#define MAZE_CELLS (MAZE_SIZE * MAZE_SIZE)
#define UNDEFINED 9999
#define ORIENT_OFFSET 100
#define CELL_SIZE 148

enum Orient
{
	NORTH = ORIENT_OFFSET + 0,
	EAST = ORIENT_OFFSET + 1,
	SOUTH = ORIENT_OFFSET + 2,
	WEST = ORIENT_OFFSET + 3
};

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

// struct for represent the Point in the maze (Cell position)
struct Point
{
	int x;
	int y;
};

// motor related functions
int forward(int speed, DistanceMetrix dt); // rotate the motors in which mouse move forward
void forward(int speed);				   // rotate the motors in which mouse move forward
void reverse(int speed);				   // reverse the motors
void right(int speed);					   // turn right
void left(int speed);					   // turn left
void stop();
bool inRange(int distance); // stop the right and left motors
void rotateReverse(int leftDistance, int rightDistance);

// queue function headers
struct Queue
{
	Point *queue;
	int front;
	int back;
	int size;
};

#define DEFAULT_SIZE 100
Point queue_arr[DEFAULT_SIZE];
Queue queue = {queue_arr, 0, 0, DEFAULT_SIZE};

Queue *getQueue();
void resetQueue();
Point dequeue();
void enqueue(Point point);
bool queueEmpty();

// algorithm related utility functions headers
void initializeMaze();										 // initialize the maze array with empty cells
void resetFloodMap();										 // reset the flood index array
void flooded();												 // flooded the whole flood index map based on new wall structure
void setWalls(Point current, Orient orient, int direction);	 // set the walls of the given point in the given direction
bool isFinished(Point current);								 // check whether the mouse has reached the destination
Point findNext(Point current, Orient orient);				 // find th next point mouse need to go                                                                                // get the position of the cell mouse should go next
int findDirection(Point current, Point next, Orient orient); // check if the mouse has reached the destination
Orient getAbsDirection(Orient orient, int direction);
bool isReversed(Orient orient1, Orient orient2);
bool leftWall(Point p);
bool rightWall(Point p);
bool frontWall(Point p);
bool backWall(Point p);
bool isReversed(Orient orient1, Orient orient2);				 // function for hceck whether the given orientations are in reversed directions
bool isValid(Point p);											 // check whether the given point is valid in the maze
Point getNextPoint(Point current, Orient orient, int direction); // get the next point indicate by the direction and current point

void rotateMouse(int direction);
void mouseForward();
void searchRun();
void returnToStart();
void test();
void followTest();

// declare the 2D int arrays fo represent wall structure and flood index map
int wallMap[MAZE_SIZE][MAZE_SIZE];
int floodMap[MAZE_SIZE][MAZE_SIZE];
int X, Y;
Orient orient;
char pathString[BUFFER_MAX_LENGTH];
int pathLength = 0;

//-----------------------------------------------------------------
AsyncWebServer server(80);

const char *ssid = "Dialog 4G 801";		 // Your WiFi SSID
const char *password = "e35076c3"; // Your WiFi Password

void recvMsg(uint8_t *data, size_t len)
{
	WebSerial.println("Received Data...");
	String d = "";
	for (int i = 0; i < len; i++)
	{
		d += char(data[i]);
	}
	WebSerial.println(d);
	if (d == "ON")
	{
		digitalWrite(LED, HIGH);
	}
	if (d == "OFF")
	{
		digitalWrite(LED, LOW);
	}
}
//-----------------------------------------------------------------

void setup()
{
	Serial.begin(115200);
	//---------------------------------------------------------------
	pinMode(LED, OUTPUT);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	if (WiFi.waitForConnectResult() != WL_CONNECTED)
	{
		Serial.printf("WiFi Failed!\n");
		return;
	}
	Serial.print("IP Address: ");
	Serial.println(WiFi.localIP());
	// WebSerial is accessible at "<IP Address>/webserial" in browser
	WebSerial.begin(&server);
	WebSerial.msgCallback(recvMsg);
	server.begin();
	//---------------------------------------------------------------
	// wait until serial port opens for native USB devices
	while (!Serial)
	{
		delay(1);
	}

	initializeMotors();
	pinMode(SHT_LOX1, OUTPUT);
	pinMode(SHT_LOX2, OUTPUT);
	pinMode(SHT_LOX3, OUTPUT);
	pinMode(SHT_LOX4, OUTPUT);
	pinMode(SHT_LOX5, OUTPUT);

	WebSerial.println("Shutdown pins inited...");

	digitalWrite(SHT_LOX1, LOW);
	digitalWrite(SHT_LOX2, LOW);
	digitalWrite(SHT_LOX3, LOW);
	digitalWrite(SHT_LOX4, LOW);
	digitalWrite(SHT_LOX5, LOW);

	WebSerial.println("Both in reset mode...(pins are low)");

	WebSerial.println("Starting...");
	setID();
}

void loop()
{
#if TEST
	test();
#endif

#if FOLLOW_TEST
	followTest();
#endif
  orient = NORTH;
  X = 0, Y = 0;
#if SEARCH_RUN
	for (int i = 0; i < ROUNDS; i++)
	{
    
    FINISHING_X = 8;
    FINISHING_Y = 3;
		searchRun();
		// inidicate that mouse has reached the destination
		digitalWrite(BUILTIN_LED, HIGH);
		delay(2500);
		digitalWrite(BUILTIN_LED, LOW);
    // set the new finishing x and y
    FINISHING_X = 0;
    FINISHING_Y = 0;
		searchRun();
	}

	while (true)
	{
		// do nothing
	}
#endif
}

void initializeMotors()
{
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

	// initing LOX5
	if (!lox5.begin(LOX5_ADDRESS))
	{
		Serial.println(F("Failed to boot second VL53L0X"));
		while (1)
			;
	}

	Serial.println("SetID end");
}

// motor related functions implementation
void forward(int speed)
{
	digitalWrite(in1A, HIGH);
	digitalWrite(in2A, LOW);
	digitalWrite(in1B, HIGH);
	digitalWrite(in2B, LOW);
	ledcWrite(pwmChannel1, speed); // 1.65 V
	ledcWrite(pwmChannel2, speed); // 1.65 V
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

void left(int speed)
{
	digitalWrite(in1A, HIGH);
	digitalWrite(in2A, LOW);
	digitalWrite(in1B, LOW);
	digitalWrite(in2B, HIGH);
	ledcWrite(pwmChannel1, speed); // 1.65 V
	ledcWrite(pwmChannel2, speed); // 1.65 V
}

void right(int speed)
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

bool inRange(int distance)
{
	return distance > LOWER_THERSHOLD && distance < HIGHER_THRESHOLD;
}

void rotateReverse(int leftD, int rightD)
{
	stop();
	right(120);
	delay(REVERSE_ROTATE_TIME);
	right(120);
	delay(REVERSE_ROTATE_TIME);
	stop();

	// adjust the mouse if it is out of range
	if (!inRange(leftD) || !inRange(rightD))
	{
		digitalWrite(in1A, HIGH);
		digitalWrite(in2A, LOW);
		digitalWrite(in1B, HIGH);
		digitalWrite(in2B, LOW);
		if (leftD < LOWER_THERSHOLD)
		{
			ledcWrite(pwmChannel1, 120);	  // 1.65 V
			ledcWrite(pwmChannel2, 120 - 30); // 1.65 V
		}
		else
		{
			ledcWrite(pwmChannel1, 120 - 30); // 1.65 V
			ledcWrite(pwmChannel2, 120);	  // 1.65 V
		}
	}
	delay(150);
	stop();
}

// int forward(int speed, DistanceMetrix dt)
// {
// 	digitalWrite(in1A, HIGH);
// 	digitalWrite(in2A, LOW);
// 	digitalWrite(in1B, HIGH);
// 	digitalWrite(in2B, LOW);

// 	if (dt.front < FRONT_MIN_DISTANCE)
// 	{
// 		return -1;
// 	}

// 	if (inRange(dt.leftFront) && inRange(dt.rightFront) || ((dt.leftBack > CELL_SIZE || dt.leftFront > CELL_SIZE) && (dt.rightBack > CELL_SIZE || dt.rightFront > CELL_SIZE)))
// 	{
// 		ledcWrite(pwmChannel1, speed); // 1.65 V
// 		ledcWrite(pwmChannel2, speed); // 1.65 V
// 	}
// 	else if (dt.leftBack > CELL_SIZE || dt.leftFront > CELL_SIZE)
// 	{
// 		if (dt.rightFront < LOWER_THERSHOLD)
// 		{
// 			ledcWrite(pwmChannel1, speed - 40); // 1.65 V
// 			ledcWrite(pwmChannel2, speed);		// 1.65 V
// 			delay(20);
// 			return 35;
// 		}
// 		else if (dt.rightFront > HIGHER_THRESHOLD)
// 		{
// 			ledcWrite(pwmChannel1, speed);		// 1.65 V
// 			ledcWrite(pwmChannel2, speed - 40); // 1.65 V
// 			delay(20);
// 			return 35;
// 		}
// 		else
// 		{
// 			ledcWrite(pwmChannel1, speed); // 1.65 V
// 			ledcWrite(pwmChannel2, speed);
// 		}
// 	} else if (dt.rightBack > CELL_SIZE || dt.rightFront > CELL_SIZE)
// 	{
// 		if (dt.leftFront < LOWER_THERSHOLD)
// 		{
// 			ledcWrite(pwmChannel1, speed);		// 1.65 V
// 			ledcWrite(pwmChannel2, speed - 40); // 1.65 V
// 			delay(20);
// 			return 35;
// 		}
// 		else if (dt.leftFront > HIGHER_THRESHOLD)
// 		{
// 			ledcWrite(pwmChannel1, speed - 40); // 1.65 V
// 			ledcWrite(pwmChannel2, speed);		// 1.65 V
// 			delay(20);
// 			return 35;
// 		}
// 		else
// 		{
// 			ledcWrite(pwmChannel1, speed); // 1.65 V
// 			ledcWrite(pwmChannel2, speed);
// 		}
// 	}
// 	else
// 	{
// 		if (dt.leftFront < LOWER_THERSHOLD)
// 		{
// 			ledcWrite(pwmChannel1, speed);		// 1.65 V
// 			ledcWrite(pwmChannel2, speed - 40); // 1.65 V
// 			delay(20);
// 			return 35;
// 		}
// 		else if (dt.rightFront < LOWER_THERSHOLD)
// 		{
// 			ledcWrite(pwmChannel1, speed - 40); // 1.65 V
// 			ledcWrite(pwmChannel2, speed);		// 1.65 V
// 			delay(20);
// 			return 35;
// 		}
// 		else
// 		{
// 			ledcWrite(pwmChannel1, speed); // 1.65 V
// 			ledcWrite(pwmChannel2, speed);
// 		}
// 	}
// 	return 0;
// }

bool isLeftRange(int distance)
{
	return distance > LEFT_MIN_THRESHOLD && distance < LEFT_MAX_THRESHOLD;
}

bool isRightRange(int distance)
{
	return distance > RIGHT_MIN_THRESHOLD && distance < RIGHT_MAX_THRESHOLD;
}

int forward(int speed, DistanceMetrix dt)
{
	digitalWrite(in1A, HIGH);
	digitalWrite(in2A, LOW);
	digitalWrite(in1B, HIGH);
	digitalWrite(in2B, LOW);

  WebSerial.printf("front: %d, left: %d, right: %d, lftBack: %d, rightBack: %d\n", dt.front, dt.leftFront, dt.rightFront, dt.leftBack, dt.rightBack);;
	if (dt.front < FRONT_MIN_DISTANCE)
	{
		return -1;
	}

	if (isLeftRange(dt.leftFront) && isRightRange(dt.rightFront))
	{
		ledcWrite(pwmChannel1, speed); // 1.65 V
		ledcWrite(pwmChannel2, speed); // 1.65 V
	}
	else if (dt.leftFront < CELL_SIZE && dt.rightFront < CELL_SIZE)
	{
		// mouse is not in the middle of the path
		if (dt.leftFront < LEFT_MIN_THRESHOLD && dt.rightFront > RIGHT_MAX_THRESHOLD)
		{
			ledcWrite(pwmChannel1, speed);		// 1.65 V
			ledcWrite(pwmChannel2, speed - 20); // 1.65 V
			// delay(20);
			return 15;
		}
		else if (dt.leftFront > LEFT_MAX_THRESHOLD && dt.rightFront < RIGHT_MIN_THRESHOLD)
		{
			ledcWrite(pwmChannel1, speed - 20); // 1.65 V
			ledcWrite(pwmChannel2, speed);		// 1.65 V
			// delay(20);
			return 15;
		}
		else
		{
			ledcWrite(pwmChannel1, speed); // 1.65 V
			ledcWrite(pwmChannel2, speed); // 1.65 V
		}
	}
  else if (dt.rightFront > CELL_SIZE){
    if ( dt.leftFront > LEFT_MAX_THRESHOLD + 5){
      		ledcWrite(pwmChannel1, speed - 20); // 1.65 V
			    ledcWrite(pwmChannel2, speed);		// 1.65 V
			    // delay(20);
			    return 15;
    }
    else if (dt.leftFront < LEFT_MIN_THRESHOLD){
			ledcWrite(pwmChannel1, speed);		// 1.65 V
			ledcWrite(pwmChannel2, speed - 20); // 1.65 V
			// delay(20);
			return 15;

    }else{
      ledcWrite(pwmChannel1, speed); // 1.65 V
		  ledcWrite(pwmChannel2, speed); // 1.65 V

    }
  }
  else if (dt.leftFront > CELL_SIZE){
    if (dt.rightFront > RIGHT_MAX_THRESHOLD + 5){
      		ledcWrite(pwmChannel1, speed); // 1.65 V
			    ledcWrite(pwmChannel2, speed-20);		// 1.65 V
			    // delay(20);
			    return 15;
    }
    else if (dt.rightFront > RIGHT_MIN_THRESHOLD){
        	ledcWrite(pwmChannel1, speed-20); // 1.65 V
			    ledcWrite(pwmChannel2, speed);		// 1.65 V
			    // delay(20);
			    return 15;

    }
    else{
        ledcWrite(pwmChannel1, speed); // 1.65 V
			  ledcWrite(pwmChannel2, speed);		// 1.65 V

    }
  }
	return 0;
}

// queue functions implementation

// get a pointer to the queue
Queue *getQueue()
{
	return &queue;
}

// reset the queue
void resetQueue()
{
	queue.front = 0;
	queue.back = 0;
}

// get the first element in the queue
Point dequeue()
{
	Point point = queue.queue[queue.front];
	queue.front = (queue.front + 1) % queue.size;
	return point;
}

// insert an element into the queue
void enqueue(Point point)
{
	queue.queue[queue.back] = point;
	queue.back = (queue.back + 1) % queue.size;
}

// check whether the queue is empty
bool queueEmpty()
{
	return queue.front == queue.back;
}

// solver functions implementation
/* -------------------- solver function section ----------------------*/

// function for initialize the wall map and flood index map
void initializeMaze()
{
	for (int i = 0; i < MAZE_SIZE; i++)
	{
		for (int j = 0; j < MAZE_SIZE; j++)
		{
			wallMap[i][j] = WALL_NONE;
			floodMap[i][j] = UNDEFINED;
		}
	}

	// fill  the border of the maze with walls
	for (int i = 0; i < MAZE_SIZE; i++)
	{
		wallMap[0][i] |= WALL_LEFT;				 // fill the left boundary walls
		wallMap[MAZE_SIZE - 1][i] |= WALL_RIGHT; // fill the right boundary walls
		wallMap[i][0] |= WALL_BACK;				 // fill the lower boudnary walls
		wallMap[i][MAZE_SIZE - 1] |= WALL_FRONT; // fill teh higher boundary walls
	}
}

// function for reset the flood map downto initial state
void resetFloodMap()
{
	// first set the index of all cells to undefined state
	for (int i = 0; i < MAZE_SIZE; i++)
	{
		for (int j = 0; j < MAZE_SIZE; j++)
		{
			floodMap[i][j] = UNDEFINED;
		}
	}

	// now set the destination cell flood index to zero
	floodMap[FINISHING_X][FINISHING_Y] = 0;
	floodMap[FINISHING_X][FINISHING_Y + 1] = 0;
	floodMap[FINISHING_X + 1][FINISHING_Y] = 0;
	floodMap[FINISHING_X + 1][FINISHING_Y + 1] = 0;
}

// function for completly flooded the maze with the flood indexes based on the wall map
void flooded()
{
	// first reset the complete flood map
	resetFloodMap();
	// reset the queue also
	resetQueue();

	// add the finishing coordinates to the queue
	enqueue({FINISHING_X, FINISHING_Y});
	enqueue({FINISHING_X, FINISHING_Y + 1});
	enqueue({FINISHING_X + 1, FINISHING_Y});
	enqueue({FINISHING_X + 1, FINISHING_Y + 1});

	while (!queueEmpty())
	{
		// extract the element from the queue
		Point p = dequeue();
		// get the flood index of extracted point
		int floodIndex = floodMap[p.x][p.y];
		int x = p.x, y = p.y;

		// check if the cell on the left is not a wall and the flood index is undefined
		if (!leftWall(p) && floodMap[x - 1][y] == UNDEFINED)
		{
			floodMap[x - 1][y] = floodIndex + 1;
			enqueue({x - 1, y});
		}

		// check if the cell on the right is not a wall and the flood index is undefined
		if (!rightWall(p) && floodMap[x + 1][y] == UNDEFINED)
		{
			floodMap[x + 1][y] = floodIndex + 1;
			enqueue({x + 1, y});
		}

		// check if the cell on the front is not a wall and the flood index is undefined
		if (!frontWall(p) && floodMap[x][y + 1] == UNDEFINED)
		{
			floodMap[x][y + 1] = floodIndex + 1;
			enqueue({x, y + 1});
		}

		// check if the cell on the back is not a wall and the flood index is undefined
		if (!backWall(p) && floodMap[x][y - 1] == UNDEFINED)
		{
			floodMap[x][y - 1] = floodIndex + 1;
			enqueue({x, y - 1});
		}
	}
}

void setWalls(Point current, Orient orient, int direction)
{
	// get the next point and absolute orientation
	Point next = getNextPoint(current, orient, direction);
	Orient absOrt = getAbsDirection(orient, direction);

	int currentWall = WALL_NONE, nextWall = WALL_NONE;

	switch (absOrt)
	{
	case NORTH:
		currentWall = WALL_FRONT;
		nextWall = WALL_BACK;
		break;
	case SOUTH:
		currentWall = WALL_BACK;
		nextWall = WALL_FRONT;
		break;
	case EAST:
		currentWall = WALL_RIGHT;
		nextWall = WALL_LEFT;
		break;
	case WEST:
		currentWall = WALL_LEFT;
		nextWall = WALL_RIGHT;
		break;
	default:
		break;
	}

	wallMap[current.x][current.y] |= currentWall;
	if (isValid(next))
		wallMap[next.x][next.y] |= nextWall;
}

Point findNext(Point point, Orient orient)
{
	// find the cell with the minimum flood index
	int minFloodIndex = UNDEFINED;
	Point next = point;
	int x = point.x, y = point.y;

	if (!leftWall(point) && floodMap[x - 1][y] < minFloodIndex && !isReversed(orient, WEST))
	{
		minFloodIndex = floodMap[x - 1][y];
		next = {x - 1, y};
	}

	if (!rightWall(point) && floodMap[x + 1][y] < minFloodIndex && !isReversed(orient, EAST))
	{
		minFloodIndex = floodMap[x + 1][y];
		next = {x + 1, y};
	}

	if (!frontWall(point) && floodMap[x][y + 1] < minFloodIndex && !isReversed(orient, NORTH))
	{
		minFloodIndex = floodMap[x][y + 1];
		next = {x, y + 1};
	}

	if (!backWall(point) && floodMap[x][y - 1] < minFloodIndex && !isReversed(orient, SOUTH))
	{
		minFloodIndex = floodMap[x][y - 1];
		next = {x, y - 1};
	}

	if (minFloodIndex == UNDEFINED)
		return point;

	return next;
}

int findDirection(Point current, Point next, Orient orient)
{

	Orient nextCellOrient;

	if (next.y == current.y + 1)
		nextCellOrient = NORTH;
	else if (next.y == current.y - 1)
		nextCellOrient = SOUTH;
	else if (next.x == current.x + 1)
		nextCellOrient = EAST;
	else if (next.x == current.x - 1)
		nextCellOrient = WEST;

	// return the direction we need to rotate according to current orientation
	int tmp = nextCellOrient - orient;
	if (tmp == 3)
		return LEFT;
	if (tmp == -3)
		return RIGHT;
	else
		return tmp;
}

bool isFinished(Point current)
{
	int x = current.x, y = current.y;

	return (x == FINISHING_X || x == FINISHING_X + 1) && (y == FINISHING_Y || y == FINISHING_Y + 1);
}

// helper function implementations
bool leftWall(Point p)
{
	return wallMap[p.x][p.y] & WALL_LEFT;
}

bool rightWall(Point p)
{
	return wallMap[p.x][p.y] & WALL_RIGHT;
}

bool frontWall(Point p)
{
	return wallMap[p.x][p.y] & WALL_FRONT;
}

bool backWall(Point p)
{
	return wallMap[p.x][p.y] & WALL_BACK;
}

bool isReversed(Orient orient1, Orient orient2)
{
	int diff = orient1 - orient2;
	return diff == -2 || diff == 2;
}

bool isValid(Point p)
{
	return p.x >= 0 && p.x < MAZE_SIZE && p.y >= 0 && p.y < MAZE_SIZE;
}

Orient getAbsDirection(Orient orient, int direction)
{
	int tmp = (orient + direction - ORIENT_OFFSET) % 4;
	if (tmp < 0)
		tmp += 4;

	return (Orient)(tmp + ORIENT_OFFSET);
}

Point getNextPoint(Point current, Orient orient, int direction)
{
	// get the absolute direction first
	Orient absOrt = getAbsDirection(orient, direction);
	int x = current.x, y = current.y;

	switch (absOrt)
	{
	case NORTH:
		return {x, y + 1};
	case SOUTH:
		return {x, y - 1};
	case EAST:
		return {x + 1, y};
	case WEST:
		return {x - 1, y};
	}

	return current; // never reach to this point
}

/* --------------------- end of solver function section -------------- */

/* other functions */

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
		lox1.rangingTest(&measure1, false);
		lox2.rangingTest(&measure2, false);
		lox3.rangingTest(&measure3, false);
		lox4.rangingTest(&measure4, false);
		lox5.rangingTest(&measure5, false);

		// fetch the distance from the sensor
		DistanceMetrix mat = {
			measure1.RangeMilliMeter,
			measure2.RangeMilliMeter,
			measure3.RangeMilliMeter,
			measure5.RangeMilliMeter,
			measure4.RangeMilliMeter};
		int dd = forward(FORWARD_SPEED, mat);
		if (dd == -1)
			break;
		startTime += dd;
	}

	stop();
}

// search run function implementation
void searchRun()
{
	// X = 0, Y = 0; // reset the coordinates
	// orient = NORTH;
	pathLength = 0;

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

    WebSerial.printf("(%d, %d)\n", X, Y);
	}
}

// function to return to starting positions
void returnToStart()
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

	// first take the full rotate
	// rotateMouse(RIGHT);
	// rotateMouse(RIGHT);
  rotateReverse(leftDistance, rightDistance);

  WebSerial.printf("path length: %d\n", pathLength);
	for (int i = pathLength - 1; i >= 0; i--)
	{
    WebSerial.printf("chracter: %c\n", pathString[i]);
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

	  lox1.rangingTest(&measure1, false);
		lox2.rangingTest(&measure2, false);
		lox3.rangingTest(&measure3, false);
		lox4.rangingTest(&measure4, false);
		lox5.rangingTest(&measure5, false);

		// fetch the distance from the sensor
		leftDistance = measure1.RangeMilliMeter;
		frontDistance = measure4.RangeMilliMeter;
		rightDistance = measure3.RangeMilliMeter;

	// first take the full rotate
	// rotateMouse(RIGHT);
	// rotateMouse(RIGHT);
  rotateReverse(leftDistance, rightDistance);
}

// test function implementation
void test()
{
	lox1.rangingTest(&measure1, false);
	lox2.rangingTest(&measure2, false);
	lox3.rangingTest(&measure3, false);
	lox4.rangingTest(&measure4, false);
	lox5.rangingTest(&measure5, false);

	int front = measure4.RangeMilliMeter;
	int left = measure1.RangeMilliMeter;
	int right = measure3.RangeMilliMeter;

	WebSerial.printf("front : %d, , Left : %d, , Right :  %d\n", front, left, right);
	delay(100);
}

void followTest()
{

	lox1.rangingTest(&measure1, false);
	lox2.rangingTest(&measure2, false);
	lox3.rangingTest(&measure3, false);
	lox4.rangingTest(&measure4, false);
	lox5.rangingTest(&measure5, false);

	int frontDistance = measure4.RangeMilliMeter;
	int leftDistance = measure1.RangeMilliMeter;
	int rightDistance = measure3.RangeMilliMeter;
	int leftBackDistance = measure2.RangeMilliMeter;
	int rightBackDistance = measure5.RangeMilliMeter;

	WebSerial.printf("front: %d, left: %d, right: %d, leftBack: %d, rightBack: %d\n", frontDistance, leftDistance, rightDistance, leftBackDistance, rightBackDistance);

	if (frontDistance > CELL_SIZE)
	{
		mouseForward();
	}
	else if (leftDistance > CELL_SIZE)
	{
		rotateMouse(LEFT);
		mouseForward();
	}
	else if (rightDistance > CELL_SIZE)
	{
		rotateMouse(RIGHT);
		mouseForward();
	}
	else
	{
		rotateReverse(leftDistance, rightDistance);
	}
}