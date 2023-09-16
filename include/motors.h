#ifndef MOTORS_H
#define MOTORS_H

#define pwmChannel1 0
#define pwmChannel2 1    // Selects channel 0
#define resolution 8   // 8-bit resolution, 256 possible values
// #define frequency 30000  // PWM frequency of 1 KHz

// Motor A
#define pwmA 26
#define in1A 33
#define in2A 25

// Motor B
#define pwmB 32
#define in1B 27
#define in2B 14

void initializeMotors();

void forward(int speed); // rotate the motors in which mouse move forward
void reverse(int speed); // reverse the motors
void right(int speed); // turn right
void left(int speed); // turn left
void stop(); // stop the right and left motors

#endif