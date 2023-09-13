
#include <ESP32SharpIR.h>

int pwmChannel1 = 0;
int pwmChannel2 = 1;    // Selects channel 0
int frequence = 30000;  // PWM frequency of 1 KHz
int resolution = 8;     // 8-bit resolution, 256 possible values

// Motor A

int pwmA = 13;
int in1A = 14;
int in2A = 12;

// Motor B

int pwmB = 4;
int in1B = 19;
int in2B = 18;

// Speed control potentiometers


// Motor Speed Values - Start at zero

int MotorSpeed1 = 0;
int MotorSpeed2 = 0;

ESP32SharpIR sensor1(ESP32SharpIR::GP2Y0A21YK0F, 26);
ESP32SharpIR sensor2(ESP32SharpIR::GP2Y0A21YK0F, 35);
ESP32SharpIR sensor3(ESP32SharpIR::GP2Y0A21YK0F, 34);

void setup() {
  Serial.begin(115200);
  // Set all the motor control pins to outputs

  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Configuration of channel 0 with the chosen frequency and resolution
  ledcSetup(pwmChannel1, frequence, resolution);
  ledcSetup(pwmChannel2, frequence, resolution);

  // Assigns the PWM channel to pin 23
  ledcAttachPin(pwmA, pwmChannel1);
  ledcAttachPin(pwmB, pwmChannel2);

  // Create the selected output voltage
  // Setting the filter resolution to 0.1
  sensor1.setFilterRate(0.0f);
  sensor2.setFilterRate(0.0f);
  sensor3.setFilterRate(0.0f);
}

void loop() {

  // Print distance in cm
  //Serial.println(sensor.getDistance());

  // Print distance in cm, as a float value

  Serial.print(sensor1.getDistanceFloat());

  Serial.print(sensor2.getDistanceFloat());

  Serial.print(sensor3.getDistanceFloat());
  
  if(sensor1.getDistanceFloat()<15 && sensor2.getDistanceFloat()<11 && sensor2.getDistanceFloat()<15){

  }



  delay(100);
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