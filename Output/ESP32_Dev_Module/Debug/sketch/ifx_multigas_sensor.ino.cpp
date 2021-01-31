#include <Arduino.h>
#line 1 "C:\\Users\\PrietoBejar\\Repositories\\github\\boyle_fw_mgs\\sketches\\ifx_multigas_sensor.ino"
#include "Sensor.h"
#include "CommHandler.h"

#define ASIC_INT 4

//CommHandler com;
#line 7 "C:\\Users\\PrietoBejar\\Repositories\\github\\boyle_fw_mgs\\sketches\\ifx_multigas_sensor.ino"
void setup();
#line 17 "C:\\Users\\PrietoBejar\\Repositories\\github\\boyle_fw_mgs\\sketches\\ifx_multigas_sensor.ino"
void loop();
#line 7 "C:\\Users\\PrietoBejar\\Repositories\\github\\boyle_fw_mgs\\sketches\\ifx_multigas_sensor.ino"
void setup()
{
  pinMode(ASIC_INT, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  delay(10); // let serial console settle
  SensorInit(Wire);
}

void loop()
{
  SensorPoll();
  CommRx();
}

