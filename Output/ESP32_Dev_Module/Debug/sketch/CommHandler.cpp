#include <Arduino.h>

#include "CommHandler.h"
#include "Sensor.h"

#define NUM_OF_CMDS 7
#define NUM_OF_PARAMS 8
#define TX_BUFFER_SIZE 250
#define RX_BUFFER_SIZE 250
#define DATA_BUFFER_SIZE 200

#define FW_VERSION "0.0.1"
#define DEVICE_NAME "MultigasSensor"
#define MANUFACTURER "Infineon"
#define PROTOCOL_UNIQUE_RESOURCE_NUMBER " "

#define JSON_HELLO "{\"pt\":\"%s\",\"uuid\":\"%s\",\"username\":\"%s\"}\n"
#define JSON_DEVICE_INFO "{\"name\":\"%s\",\"manufacturer\":\"%s\",\"fw\":\"%s\",\"baud\":%ld,\"sensors\":[%s]}\n"
#define JSON_GET_PARAM "{\"sid\":\"%s\",\"param\":\"%s\",\"val\":\"%s\"}\n"
#define JSON_ERROR "{\"error\":\"%s\"}\n"
#define JSON_SENSOR_BOYLE_REF "{\"RF1_2\":\"%u\",\"RF3_4\":\"%u\",\"RF5_6\":\"%u\",\"RF7_8\":\"%u\",\"RT1_2\":\"%u\"}\n"
#define JSON_SENSOR_DATA "{\"sensor\":\"%s\",\"data\":[%s]}\n"
#define JSON_SENSOR_BOYLE "{\"counter\":\"%u\",\"RTEMP1\":\"%u\",\"RSENS_1\":\"%u\",\"RSENS_2\":\"%u\",\"RSENS_3\":\"%u\",\"RSENS_4\":\"%u\",\"RTEMP2\":\"%u\",\"RREF_EXT\":\"%u\",\"ASIC_TEMP_0\":\"%lu\",\"ASIC_TEMP_1\":\"%lu\"}"
#define JSON_SENSOR_DPS368 "{\"pressure\":\"%f\",\"temperature\":%f}"
#define JSON_SENSOR_ALPHASENSE "{\"pt1000\":%f,\"NO2\":%f, \"O3\":%f}"
#define JSON_SENSOR_SHT31 "{\"temperature\":%f,\"humidity\":%f}"
#define JSON_SENSOR_BOYLE_AUTOSCALE "{\"R1\":\"%d\",\"R2\":\"%d\",\"R3\":\"%d\",\"R4\":\"%d\",\"R5\":\"%d\",\"R6\":\"%d\",\"R7\":\"%d\",\"R8\":\"%d\",\"R9\":\"%d\",\"R10\":\"%d\",\"R11\":\"%d\",\"R12\":\"%d\"}\n"

  typedef enum{
    SET = 0,
    GET,
    START,
    STOP,
    HELLO,
    INFO,
    ASCII_DATA,
    INVALID_CMD
  }CMDS_T;

const static char cmnds[][10] = {
  "set",
  "get",
  "start",
  "stop",
  "hello",
  "info",
  "asci_data"
};

static bool data_in_s_buffer =false;
static char read_buffer[RX_BUFFER_SIZE];
static char s_buffer[TX_BUFFER_SIZE];
static bool CommtTxHello(void)
{
    if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_HELLO,
      PROTOCOL_UNIQUE_RESOURCE_NUMBER, " ", " ") < 0)
    return false;
    else
      Serial.print(s_buffer);

  return true;
}
static bool CommtTxInfo(char *sensors)
{
  if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_DEVICE_INFO,
      DEVICE_NAME,MANUFACTURER,FW_VERSION,115200,sensors) < 0)
    return false;
   else
   Serial.print(s_buffer);

  return true;
}
bool CommtTxData(char *sensor,uint8_t *d )
{
  if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_SENSOR_DATA,
      sensor,d) < 0)
    return false;
   else
   Serial.print(s_buffer);

  return true;
}
static bool CommtTxParam(char *sid, char *param, char *value)
{
    if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_GET_PARAM,
      sid,param,value) < 0)
    return false;
   else
   Serial.print(s_buffer);

  return true;
}
static uint8_t CommCommand(char *cmd)
{
  uint8_t i=0;
  while(i < (NUM_OF_CMDS))
  {
    int8_t st= strcmp(cmd, cmnds[i]);
    if(!st)
    {
      return i;
    }
    i++;
  }
  return INVALID_CMD;
}
static bool CommParse(char * msg)
{
  char str[100]={0};
  char *cmd;
  char *sid;
  char *parameter;
  char *value;
  char *sensors_info=NULL;
  memcpy((char *)str, (const char *)msg, strlen((const char *)msg));
  cmd  = strtok(str, " \0");
  char * ptr_params = &str[strlen(cmd) + 1];
  //Serial.println(cmd);
  switch(CommCommand(cmd))
  {
    case HELLO:
      CommtTxHello();
      break;
    case INFO:
      if (SensorGetInfo(&sensors_info) == true)
      {  
        CommtTxInfo(sensors_info);
      }
      break;
    case ASCII_DATA:
      parameter  = strtok((char *)ptr_params, " \r\n");
      if(atoi(parameter))
      {
        SensorTxAsciiData(true);
         Serial.println("ascii data enable");
      }
      else
      {
        SensorTxAsciiData(true);
        Serial.println("ascii data disable");
      }
      break;
    case SET:
    {
      sid  = strtok((char *)ptr_params, " ");
      if(sid == NULL)
        return false;
      uint32_t len = strlen(sid);
      parameter  = strtok((char *)&ptr_params[len+1], " :=");
      if(parameter == NULL)
        return false;
      //Serial.print(parameter);
      //Serial.print(" ");
    
      value = strtok (NULL, "\r\n");
      //Serial.print(value);
      
      //if(value == NULL)
      //  return 0;
      if (SensorSetVal(sid, parameter, value) == true)
      {
        Serial.println("ack");
        delay(10);
        if(data_in_s_buffer == true)
        {
          data_in_s_buffer=false;
          Serial.print(s_buffer);
        }
          
      }
      else
      {
        Serial.println("nack");
      }
      break;
    }
    case GET:
    {
      sid  = strtok((char *)ptr_params, " ");
      if(sid == NULL)
        return false;
      
      parameter  = strtok((char *)&ptr_params[2], " :=");
      if(parameter == NULL)
        return false;

      if (SensorGetVal(sid, parameter, value) == 0)
      {
        CommtTxParam(sid, parameter,value);
        //Serial.println("ack");
      }
      else
      {
        Serial.println("nack");
      }
      break;
    }
    case START:
    {
      SensorStartAll();
      break;
    }
    case STOP:
    {
      SensorStopAll();
      break;
    }
    default:
      Serial.println("invalid command");
      break;
  }
  return true;
}
bool CommRx()
{
  static uint8_t idx = 0;
  static boolean newData = false;  

  static boolean  new_packet=false;
  if(Serial.available() > 0) {
    char rc;
    // read the incoming byte:
     rc= Serial.read();
    if(rc == '$')
      new_packet = true;
    else if((rc== '\n') || (rc== '\r'))
    {
      read_buffer[idx++] = '\0';
      new_packet = false;
      //Serial.print(read_buffer);
      if(idx > 3)
        CommParse(read_buffer);
      idx = 0;
    }
    else
    {
      read_buffer[idx++] = rc;
      if(idx > 100)
        idx = 0;
    }
  }
}
bool CommTxDataBoyleAutoScale(unsigned int *d)
{
  if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_SENSOR_BOYLE_AUTOSCALE,d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7],d[8],d[9],d[10],d[11]) < 0)
    return false;
  else
    data_in_s_buffer = true;
  //  Serial.print(s_buffer);
}
bool CommTxDataBoyleRef(uint8_t *d)
{
   if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_SENSOR_BOYLE_REF,d[0],d[1],d[2],d[3],d[4]) < 0)
    return false;
  else
    data_in_s_buffer = true;
  //  Serial.print(s_buffer);
}
bool CommTxDataBoyle(uint8_t *d)
{
  char data_buff[200] = {0x00};
  uint16_t counter = (((uint16_t)d[0] << 8) | d[1]);
  uint16_t rtemp1 = (((uint16_t)d[3] << 8) | d[2]);
  uint16_t rsens1 = (((uint16_t)d[5] << 8) | d[4]);
  uint16_t rsens2 = (((uint16_t)d[7] << 8) | d[6]);
  uint16_t rsens3 = (((uint16_t)d[9] << 8) | d[8]);
  uint16_t rsens4 = (((uint16_t)d[11] << 8) | d[10]);
  uint16_t rtemp2 = (((uint16_t)d[13] << 8) | d[12]);
  uint16_t rref_ext = (((uint16_t)d[15] << 8) | d[14]);
  uint32_t temp0 = (((uint32_t)d[18] << 16) | ((uint32_t)d[17] << 8) | d[16]);
  uint32_t temp1 = (((uint32_t)d[21] << 16) | ((uint32_t)d[20] << 8) | d[19]);
  snprintf(data_buff, DATA_BUFFER_SIZE, JSON_SENSOR_BOYLE,counter,rtemp1,rsens1,rsens2,rsens3,rsens4,rtemp2,rref_ext,temp0,temp1);
  if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_SENSOR_DATA,"boyle",data_buff) < 0)
    return false;
  else
    Serial.print(s_buffer);
}
bool CommTxDataDps368(double temp, double pres)
{
  char data_buff[200] = {0x00};
  snprintf(data_buff, DATA_BUFFER_SIZE, JSON_SENSOR_DPS368,pres, temp);
  if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_SENSOR_DATA,"dps368",data_buff) < 0)
    return false;
  else
    Serial.print(s_buffer);
}
bool CommTxDatadpsAplhasense(double temp, double NO2, double O3)
{
  char data_buff[200] = {0x00};
  snprintf(data_buff, DATA_BUFFER_SIZE, JSON_SENSOR_ALPHASENSE,temp, NO2, O3);
  if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_SENSOR_DATA,"alphasense",data_buff) < 0)
    return false;
  else
    Serial.print(s_buffer);
}
bool CommTxDatadpsSht31(double temp, double humidity)
{
  char data_buff[200] = {0x00};
  snprintf(data_buff, DATA_BUFFER_SIZE, JSON_SENSOR_SHT31,temp,humidity);
  if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_SENSOR_DATA,"sht31",data_buff) < 0)
    return false;
  else
    Serial.print(s_buffer);
}

bool CommTxData(uint8_t *d, uint8_t n)
{
  for (int i=0;i<n;i++)
  {
    Serial.write(d[i]); 
  }
  return true;
}
bool CommTxError(char *  error_msg)
{
  if(snprintf(s_buffer, TX_BUFFER_SIZE, JSON_ERROR,error_msg) < 0)
    return false;
  else
    Serial.print(s_buffer);
  return true;
}
void CommtTx(const char * msg)
{
  Serial.print(msg);
}
