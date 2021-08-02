
#include <Dps368.h>
#include "SHTSensor.h"
#include "boyle.h"
#include "Alphasense.h"
#include "CommHandler.h"

#define DPS368_ADDRESS   0x76
 
typedef enum
{
  DPS368 = 1,
  SHT,
  MULTIGAS,
  O3,
  NO2,
  BOYLE
}Sensor_Ids_e;

static bool ascii_data_tx_en = true;
typedef struct{
  SHTSensor::SHTAccuracy sht_accurracy;
  uint8_t dps368_oversampling;
  uint8_t dps368_measurementrate;
  uint16_t sensor_polling_interval;  
}Sensor_Config_s;
bool dps368_init = false;
bool sht31_init = false;
bool alphasense_init = false;
bool boyle_init = true;
char sensors[100];
char error_msg[200];
Sensor_Config_s sensor_config_t;
static bool sensors_start = false;
uint8_t sensors_data[100] = {0};

SHTSensor sht;
AplphasenseADC7794 alphasense;
Boyle boyle;
hw_timer_t * timer;
Dps368 Dps368PressureSensor = Dps368();
Sensor_Config_s sensor_config = { SHTSensor::SHT_ACCURACY_LOW,7,8, 100};

static portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  boyle.timerEvent();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void SensorInit(TwoWire &bus)
{
  Dps368PressureSensor.begin(bus, DPS368_ADDRESS);
  
    memset(sensors, 0 , 100);
    strcat(sensors, "{\"");
    strcat(sensors, "01:dps368");
    strcat(sensors, "\"");
    dps368_init = true;
 
   if (sht.init()) {
      Serial.print("init(): success\n");
      strcat(sensors, ",\"");
      strcat(sensors, "02:sht31x");
      strcat(sensors, "\"");
      sht31_init = true;
  } else {
      Serial.print("sht init(): failed\n");
  }
  
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_LOW); 

  if(boyle.asic_init())
  {
    strcat(sensors, ",\"");
    strcat(sensors, "03:boyle");
    strcat(sensors, "\"}");
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 100000, true);
    timerAlarmEnable(timer);
    boyle_init=true;
  }
  
  if(alphasense.init())
  {
    strcat(sensors, ",\"");
    strcat(sensors, "04:alphasense");
    strcat(sensors, "\"");
    alphasense_init = true;
    Serial.print("alphasense init(): success\n");
  }
  else
  {
    strcat(sensors, "}");
  }
  Serial.println(sensors);
}

void SensorPoll(void)
{
  static uint16_t polling_time = 0;
  if(sensors_start)
  {
    if(boyle_init)
    {
      boyle.asic_poll(sensors_data);
      if(ascii_data_tx_en)
        CommTxDataBoyle(sensors_data);
    }
   
    if((boyle.read_ref_sensor()) || (!boyle_init))
    {
      float temperature;
      float pressure;
      uint8_t oversampling = 7;
      int16_t ret;
      polling_time = 0;
      boyle.set_ref_sensor_readout_ongoing();
      boyle.clear_read_ref_sensor();
      if(dps368_init)
      {
        ret = Dps368PressureSensor.measureTempOnce(temperature, oversampling);
        if (ret != 0)
        {
          //Something went wrong.
          //Look at the library code for more information about return codes
          // report error
          strcpy(error_msg, "dps368 temperature read error");
          CommTxError(error_msg);
        }
        else
        {
          memcpy((uint8_t *)&sensors_data[22], (uint8_t *)&temperature, 4);
        }
  
        ret = Dps368PressureSensor.measurePressureOnce(pressure, oversampling);
        if (ret != 0)
        {
          //Something went wrong.
          //Look at the library code for more information about return codes
          // report error
          strcpy(error_msg, "dps368 temperature read error");
          CommTxError(error_msg);
          //Serial.print("FAIL! ret = ");
          //Serial.println(ret);
        }
        else
        {
          memcpy((uint8_t *)&sensors_data[26], (uint8_t *)&pressure, 4);
        }
        if(ascii_data_tx_en)
          CommTxDataDps368(temperature, pressure);
      }
      if(sht31_init)
      {
        if (sht.readSample()) {
          float humidity,T;
          humidity = sht.getHumidity();
          //Serial.println("humidity");
          //Serial.print(humidity);
          
          memcpy((uint8_t *)&sensors_data[30], (uint8_t *)&humidity, 4);
          T = sht.getTemperature();
          //Serial.println("humidity");
          //Serial.print(humidity);
          memcpy((uint8_t *)&sensors_data[34], (uint8_t *)&T, 4);
          if(ascii_data_tx_en)
            CommTxDatadpsSht31(T, humidity);
            
        } 
        else 
        {
          strcpy(error_msg, "sht31 read error");
          CommTxError(error_msg);
        }
      }
      boyle.clear_ref_sensor_readout_ongoing();
      if(alphasense_init)
      {
        double temp=0; double no2=0; double o3=0;
        if(alphasense.poll(&temp, &no2, &o3) == true)
        {
          memcpy((uint8_t *)&sensors_data[38], (uint8_t *)&temp, 4);
          memcpy((uint8_t *)&sensors_data[42], (uint8_t *)"*\n\r", 3);
          if(ascii_data_tx_en)
            CommTxDatadpsAplhasense(0, 0, 0);
        }
        else
        {
          strcpy(error_msg, "alphasense read error");
          CommTxError(error_msg);
        }
      }
      
    }
    else
    {
      polling_time++;
    }

    if(!ascii_data_tx_en)
    {
      CommTxData(sensors_data, 45);
    }
  }
}
void SensorTxAsciiData(bool en)
{
  ascii_data_tx_en = en;
}
bool SensorGetInfo(char **info)
{
  *info = sensors;
  return true;
}

bool SensorStartAll(void)
{
  sensors_start = true;
  return true;
}
bool SensorStart(char *sid, char *polling)
{
  return true;
}
bool SensorStopAll(void)
{
  sensors_start = false;
  return true;
}
bool SensorStop(char *sid, char *polling)
{
  
    
  return true;
}
bool SensorSetVal(char *sid, char *parameter, char *value)
{
  if(!strcmp(sid, "boyle"))
  {
    //Serial.print(sid); 
    return boyle.SetVal(parameter, value);
  }
  return false;
}
bool SensorGetVal(char *sid, char *parameter, char *value)
{
  return true;
}
