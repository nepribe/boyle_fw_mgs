# 1 "C:\\Users\\PrietoBejar\\Repositories\\github\\boyle_fw_mgs\\sketches\\ifx_multigas_sensor.ino"
# 2 "C:\\Users\\PrietoBejar\\Repositories\\github\\boyle_fw_mgs\\sketches\\ifx_multigas_sensor.ino" 2
# 3 "C:\\Users\\PrietoBejar\\Repositories\\github\\boyle_fw_mgs\\sketches\\ifx_multigas_sensor.ino" 2

#define ASIC_INT 4

//CommHandler com;
void setup()
{
  pinMode(4, 0x05);
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
