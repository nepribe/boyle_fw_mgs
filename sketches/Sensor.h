#ifndef __SENSOR_H__
#define __SENSOR_H__


#include <Wire.h>
#include <Arduino.h>

void SensorTxAsciiData(bool en);
void SensorInit(TwoWire &bus);
void SensorPoll(void);
bool SensorStartAll(void);
bool SensorStart(char *sid, char *polling);
bool SensorStopAll(void);
bool SensorStop(char *sid, char *polling);
bool SensorGetInfo(char **info);
bool SensorSetVal(char *sid, char *parameter, char *value);
bool SensorGetVal(char *sid, char *parameter, char *value);

#endif
