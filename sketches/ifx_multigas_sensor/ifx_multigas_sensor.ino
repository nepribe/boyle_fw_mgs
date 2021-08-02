#include "Sensor.h"
#include "CommHandler.h"

#define ASIC_INT 4

//CommHandler com;
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
