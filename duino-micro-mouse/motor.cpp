#include "motor.h"
#include <Arduino.h>
#include "user.h"
#include "TOF.h"

#define frequency 30000

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

void forward(int speed, DistanceMetrix dt)
{
    digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);
    digitalWrite(in1B, HIGH);
    digitalWrite(in2B, LOW);

    if (inRange(dt.leftFront) && inRange(dt.rightFront))
    {
        ledcWrite(pwmChannel1, speed); // 1.65 V
        ledcWrite(pwmChannel2, speed); // 1.65 V
    }
    
}