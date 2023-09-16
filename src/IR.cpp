#include "IR.h"

ESP32SharpIR sensor1(ESP32SharpIR::GP2Y0A21YK0F, 26);
ESP32SharpIR sensor2(ESP32SharpIR::GP2Y0A21YK0F, 35);
ESP32SharpIR sensor3(ESP32SharpIR::GP2Y0A21YK0F, 34);

void initializeIR() {
  sensor1.setFilterRate(0.0f);
  sensor2.setFilterRate(0.0f);
  sensor3.setFilterRate(0.0f);

}