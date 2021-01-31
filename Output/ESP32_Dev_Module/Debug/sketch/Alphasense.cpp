#include <SPI.h>
#include "Alphasense.h"

#define ADC_ID         0x79
#define ADC_MAX_UP     16777216
#define ADC_MAX_BP     8388608

SPIClass * vspi = NULL;

const double V_REF = 1.17;
const double t_t_cal = 27.4;
const double t_v_cal = 326.6;

static const int spiClk = 10000000;

bool AplphasenseADC7794::adc_init()
{
  unsigned long id=0;
  vspi = new SPIClass(VSPI);
  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin();
  pinMode(5, OUTPUT); //VSPI SS
  pinMode(23, INPUT); //VSPI MISO
  adc_reset();
  delay(10);
  spiStart();
  id = adcGetRegisterValue(AD7794_REG_ID, 1);
  spiStop();
  if(( id & 0x0F) != ADC_ID)
  { 
    return true;
  }
  return false;
}
void AplphasenseADC7794::adc_reset()
{
  spiStart();
  for(uint8_t i=0;i<4;i++)
  {
     vspi->transfer(0xFF);
  }
  spiStop();
}

unsigned long AplphasenseADC7794::adcGetRegisterValue(unsigned char regAddress,unsigned char size)
{
  unsigned char data[4]      = {0x00, 0x00, 0x00, 0x00};
  unsigned long receivedData = 0x00;
  unsigned char i            = 0x00; 
    
  data[0] = AD7794_COMM_READ |  AD7794_COMM_ADDR(regAddress); 
  vspi->transfer(data[0]);
  delay(1);
  vspi->transferBytes(NULL, data, size);
  for(i = 0;i < size;i ++)
  {
    receivedData = (receivedData << 8) + data[i];
  }
  return (receivedData);
}
void AplphasenseADC7794::adcSetRegisterValue(unsigned char regAddress,unsigned long regValue,unsigned char size)
{
  unsigned char data[5]      = {0x00, 0x00, 0x00, 0x00, 0x00};  
  unsigned char* dataPointer = (unsigned char*)&regValue;
  unsigned char bytesNr      = size + 1;
    
  data[0] = AD7794_COMM_WRITE |  AD7794_COMM_ADDR(regAddress);
  while(bytesNr > 1)
  {
    data[bytesNr] = *dataPointer;
    dataPointer ++;
    bytesNr --;
  }
  vspi->writeBytes(data,(1 + size));  
}
// wait until the ADC is ready
bool AplphasenseADC7794::spi_wait_for_rdy(uint8_t ch)
{
  uint8_t buffer[10] = {0x00};
  uint8_t delay_timeout = 0;
  while (true) {
    if((!adcGetRegisterValue(AD7794_REG_STAT,1)) &(AD7794_STAT_RDY)) break;
    delay(1);
    delay_timeout++;
    if(delay_timeout > 20)
      return false;    
  }
  // return false if the error bit is set
  if((!adcGetRegisterValue(AD7794_REG_STAT,1)) &(AD7794_STAT_ERR)) {
    Serial.println("read error");
    return false;
  }
  return true;
}
void AplphasenseADC7794::adcSetMode(unsigned long mode)
{
  unsigned long command;
  command = adcGetRegisterValue(AD7794_REG_MODE,2);
  command &= ~AD7794_MODE_SEL(0xFF);
  command |= AD7794_MODE_SEL(mode);
  adcSetRegisterValue(AD7794_REG_MODE,command,2);
}
void AplphasenseADC7794::adcSetChannel(unsigned long channel)
{
  unsigned long command;
  command = adcGetRegisterValue(AD7794_REG_CONF,2);
  command &= ~AD7794_CONF_CHAN(0xFF);
  command |= AD7794_CONF_CHAN(channel);
  adcSetRegisterValue(AD7794_REG_CONF,command,2);
}
void AplphasenseADC7794::adcSetGain(unsigned long gain)
{
  unsigned long command;
  command = adcGetRegisterValue(AD7794_REG_CONF,2);
  command &= ~AD7794_CONF_GAIN(0xFF);
  command |= AD7794_CONF_GAIN(gain);
  adcSetRegisterValue(AD7794_REG_CONF,command,2);
}
void AplphasenseADC7794::adcSetIntReference(unsigned char type)
{
  unsigned long command = 0;
  command = adcGetRegisterValue(AD7794_REG_CONF,2);
  command &= ~AD7794_CONF_REFSEL(AD7794_REFSEL_INT);
  command |= AD7794_CONF_REFSEL(type);
  adcSetRegisterValue(AD7794_REG_CONF,command,2);
}
void AplphasenseADC7794::adcCalibrate(unsigned char mode, unsigned char channel)
{
  unsigned short oldRegValue = 0x0;
  unsigned short newRegValue = 0x0;
   
  adcSetChannel(channel);
  oldRegValue &= adcGetRegisterValue(AD7794_REG_MODE, 2);
  oldRegValue &= ~AD7794_MODE_SEL(0x7);
  newRegValue = oldRegValue | AD7794_MODE_SEL(mode);
  adcSetRegisterValue(AD7794_REG_MODE, newRegValue, 2); 
  spi_wait_for_rdy(channel);
}
unsigned long AplphasenseADC7794::adcSingleConversion(void)
{
  unsigned long command = 0x0;
  unsigned long regData = 0x0;
  
  command  = AD7794_MODE_SEL(AD7794_MODE_SINGLE);
  adcSetRegisterValue(AD7794_REG_MODE, command,2);
  spi_wait_for_rdy(0);
  regData = adcGetRegisterValue(AD7794_REG_DATA, 3);
    return(regData);
}
void inline  AplphasenseADC7794::spiStart()
{
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  digitalWrite(5, LOW); //pull SS slow to rep other end for transfer
}
void inline  AplphasenseADC7794::spiStop()
{
  vspi->endTransaction();
  digitalWrite(5, HIGH); 
}
bool AplphasenseADC7794::adcReadPt100(double *pt1000_v)
{
  unsigned long command = 0x0;
  unsigned long regData = 0x0;
  spiStart();
  command = ((0x10 | gain_1x) << 8) | (0x90 | channel_1);
  adcSetRegisterValue(AD7794_REG_CONF,command,2);
  delay(3);
  command = (0x20 << 8) | 0x0A ;
  adcSetRegisterValue(AD7794_REG_MODE,command,2);
  if(!spi_wait_for_rdy(channel_1))
    return false;
  regData=adcGetRegisterValue(AD7794_REG_DATA, 3);
  spiStop();
  *pt1000_v = adcCalculateChUniPolar((uint8_t *)&regData, gain_1x);
  return true;
}
bool AplphasenseADC7794::adcReadAdcCh6Temp()
{
  
}
bool AplphasenseADC7794::adcReadAdcCh1NO2()
{
  
}
bool AplphasenseADC7794::adcReadAdcCh2O3()
{
  
}
bool AplphasenseADC7794::init()
{
  if(adc_init())
  {
    double value = 0;
    if(adcReadPt100(&value))
    {
      double pt1000_t = pt1000CalculateTemperature(value);
      if((pt1000_t) > 0 && (pt1000_t < 100))
        return true;
    }
    
  }
  return false;
}

double AplphasenseADC7794::adcCalculateChUniPolar(uint8_t *in, Gain_e gain)
{
  double out = ((double)(in[0] << 16 | in[1] << 8 | in[2]) * V_REF)/(ADC_MAX_UP*(1+gain));
  return out;
}
double AplphasenseADC7794::adcCalculateChBiPolar(uint8_t *in, Gain_e gain)
{
  double out = ((double)(in[0] << 16 | in[1] << 8 | in[2]) * V_REF)/(ADC_MAX_UP*(1+gain));
  return out;
}
double AplphasenseADC7794::pt1000CalculateTemperature(double in)
{
  double out = (in * 1000 - t_v_cal) / 1 + t_t_cal;
  return out;
}
double AplphasenseADC7794::AlphasenseCalibrate()
{
  double out = 0.0;
  return out;
}

double AplphasenseADC7794::AlphasenseCalculateNO2(double in)
{
  // covert voltage to ppm values for NO2 here
  double out = 0;
  return out;
}

double AplphasenseADC7794::AlphasenseCalculateO3(double in)
{
  // covert voltage to ppm values for O3 here
  double out = 0;
  return out;
}
bool AplphasenseADC7794::poll(double *t, double *no2, double *o3)
{
  double value = 0;
  //value = adcReadPt100();
  if(adcReadPt100(&value))
  {
    double pt1000_t = pt1000CalculateTemperature(value);
    *t = pt1000_t;
    // read no2

    //read o3
  }
}
void AplphasenseADC7794::cleanup()
{

}
