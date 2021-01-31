
#ifndef __APHASENSE_H__
#define __APHASENSE_H__

#include <Arduino.h>


/*AD7794 Registers*/
#define AD7794_REG_COMM     0 /* Communications Register(WO, 8-bit) */
#define AD7794_REG_STAT     0 /* Status Register      (RO, 8-bit) */
#define AD7794_REG_MODE     1 /* Mode Register        (RW, 16-bit */
#define AD7794_REG_CONF     2 /* Configuration Register (RW, 16-bit)*/
#define AD7794_REG_DATA     3 /* Data Register        (RO, 24-bit) */
#define AD7794_REG_ID       4 /* ID Register        (RO, 8-bit) */
#define AD7794_REG_IO       5 /* IO Register        (RO, 8-bit) */
#define AD7794_REG_OFFSET   6 /* Offset Register      (RW, 24-bit */
#define AD7794_REG_FULLSALE 7 /* Full-Scale Register  (RW, 24-bit */


/* Communications Register Bit Designations (AD7794_REG_COMM) */
#define AD7794_COMM_WEN    (1 << 7)      /* Write Enable */
#define AD7794_COMM_WRITE (0 << 6)      /* Write Operation */
#define AD7794_COMM_READ    (1 << 6)      /* Read Operation */
#define AD7794_COMM_ADDR(x) (((x) & 0x7) << 3)  /* Register Address */
#define AD7794_COMM_CREAD (1 << 2)      /* Continuous Read of Data Register */


/* Status Register Bit Designations (AD7794_REG_STAT) */
#define AD7794_STAT_RDY      (1 << 7) /* Ready */
#define AD7794_STAT_ERR      (1 << 6) /* Error (Overrange, Underrange, absence of a reference voltage) */
#define AD7794_STAT_NOXREF   (1 << 5) /* */
#define AD7794_STAT_SR4      (1 << 4) /* */

/* AD7794_CH_CONV options */
#define AD7794_ADC_CH1_CONVERSION     0 /* CH0 being converted*/
#define AD7794_ADC_CH2_CONVERSION     1 /* CH1 being converted*/
#define AD7794_ADC_CH3_CONVERSION     2 /* CH2 being converted*/
#define AD7794_ADC_CH4_CONVERSION     3 /* CH3 being converted*/
#define AD7794_ADC_CH5_CONVERSION     4 /* CH4 being converted*/
#define AD7794_ADC_CH6_CONVERSION     5 /* CH5 being converted*/
#define AD7794_ADC_CH7_CONVERSION     6 /* CH6 being converted*


/* Mode Register Bit Designations (AD7794_REG_MODE) */
#define AD7794_MODE_SEL(x)    (((x) & 0x7) << 13) /* Operation Mode Select */
#define AD7794_MODE_CLKSRC(x) (((x) & 0x3) << 6)  /* ADC Clock Source Select */
#define AD7794_MODE_RATE(x)   ((x) & 0xF)     /* Filter Update Rate Select */

/* AD7794_MODE_SEL(x) options */
#define AD7794_MODE_CONT          0 /* Continuous Conversion Mode */
#define AD7794_MODE_SINGLE        1 /* Single Conversion Mode */
#define AD7794_MODE_IDLE          2 /* Idle Mode */
#define AD7794_MODE_PWRDN         3 /* Power-Down Mode */
#define AD7794_MODE_CAL_INT_ZERO  4 /* Internal Zero-Scale Calibration */
#define AD7794_MODE_CAL_INT_FULL  5 /* Internal Full-Scale Calibration */
#define AD7794_MODE_CAL_SYS_ZERO  6 /* System Zero-Scale Calibration */
#define AD7794_MODE_CAL_SYS_FULL  7 /* System Full-Scale Calibration */

/* AD7794_MODE_CLKSRC(x) options */
#define AD7794_CLK_INT    0 /* Internal 64 kHz Clk not available at the CLK pin */
#define AD7794_CLK_INT_CO 1 /* Internal 64 kHz Clk available at the CLK pin */
#define AD7794_CLK_EXT    2 /* External 64 kHz Clock */
#define AD7794_CLK_EXT_DIV2 3 /* External Clock divided by 2 */

/* Configuration Register Bit Designations (AD7794_REG_CONF) */
#define AD7794_CONF_VBIAS(x)  (((x) & 0x3) << 14)   /* Bias Voltage Generator Enable */
#define AD7794_CONF_BO_EN     (1 << 13)             /* Burnout Current Enable */
#define AD7794_CONF_UNIPOLAR  (1 << 12)             /* Unipolar/Bipolar Enable */
#define AD7794_CONF_BOOST     (1 << 11)             /* Boost Enable */
#define AD7794_CONF_GAIN(x)   (((x) & 0x7) << 8)    /* Gain Select */
#define AD7794_CONF_REFSEL(x) (((x) & 0x1) << 6)    /* INT/EXT Reference Select */
#define AD7794_CONF_BUF       (1 << 4)              /* Buffered Mode Enable */
#define AD7794_CONF_CHAN(x)   ((x) & 0xF)           /* Channel select */


/* AD7794_CONF_GAIN(x) options */
#define AD7794_GAIN_1       0
#define AD7794_GAIN_2       1
#define AD7794_GAIN_4       2
#define AD7794_GAIN_8       3
#define AD7794_GAIN_16      4
#define AD7794_GAIN_32      5
#define AD7794_GAIN_64      6
#define AD7794_GAIN_128     7

/* AD7794_CONF_REFSEL(x) options */
#define AD7794_REFSEL_EXT1   0 /* External Reference Applied between REFIN1(+) and REFIN2(–). */
#define AD7794_REFSEL_EXT2   1 /* External Reference Applied between REFIN2(+) and REFIN2(–). */
#define AD7794_REFSEL_INT   2 /* Internal 1.17 V Reference Selected. */
#define AD7794_REFSEL_RES   3 /* Reserved*/

/* AD7794_CONF_CHAN(x) options */
#define AD7794_CH_AIN1P_AIN1M   0 /* AIN1(+) - AIN1(-) */
#define AD7794_CH_AIN2P_AIN2M   1 /* AIN2(+) - AIN2(-) */
#define AD7794_CH_AIN3P_AIN3M   2 /* AIN3(+) - AIN3(-) */
#define AD7794_CH_AIN5P_AIN4M   3 /* AIN4(+) - AIN4(-) */
#define AD7794_CH_AIN3P_AIN5M   4 /* AIN5(+) - AIN5(-) */
#define AD7794_CH_AIN6P_AIN6M   5 /* AIN6(+) - AIN6(-) */
#define AD7794_CH_TEMP          6 /* Temp Sensor */
#define AD7794_CH_AVDD_MONITOR  7 /* AVDD Monitor */

/* ID Register Bit Designations (AD7794_REG_ID) */
#define AD7794_ID         0xF
#define AD7794_ID_MASK    0xF

/* IO (Excitation Current Sources) Register Bit Designations (AD7794_REG_IO) */
#define AD7794_IEXCDIR(x)  (((x) & 0x3) << 2)
#define AD7794_IEXCEN(x)  (((x) & 0x3) << 0)

/* AD7793_IEXCDIR(x) options*/
#define AD7794_DIR_IEXC1_IOUT1_IEXC2_IOUT2  0  /* IEXC1 connect to IOUT1, IEXC2 connect to IOUT2 */
#define AD7794_DIR_IEXC1_IOUT2_IEXC2_IOUT1  1  /* IEXC1 connect to IOUT2, IEXC2 connect to IOUT1 */
#define AD7794_DIR_IEXC1_IEXC2_IOUT1    2  /* Both current sources IEXC1,2 connect to IOUT1  */
#define AD7794_DIR_IEXC1_IEXC2_IOUT2    3  /* Both current sources IEXC1,2 connect to IOUT2 */

/* AD7793_IEXCEN(x) options*/
#define AD7794_EN_IXCEN_10uA        1  /* Excitation Current 10uA */
#define AD7794_EN_IXCEN_210uA       2  /* Excitation Current 210uA */
#define AD7794_EN_IXCEN_1mA         3  /* Excitation Current 1mA */


class AplphasenseADC7794
{
public:
  AplphasenseADC7794()
  {
  }
  enum Gain_e { 
    gain_1x = 0x00,
    gain_2x,
    gain_4x,
    gain_8x,
    gain_16x,
    gain_32x,
    gain_64x,
    gain_128x
 };

 enum InputChannel {
    channel_1 = 0x00,
    channel_2,
    channel_3,
    channel_4,
    channel_5,
    channel_6,
    channel_7
 };
  virtual ~AplphasenseADC7794() {
    cleanup();
  }
  bool init();
  bool poll(double *t, double *no2, double *o3);

private:
  void cleanup();
  bool adc_init();
  void adc_reset();
  bool spi_wait_for_rdy(uint8_t ch);
  void adcSetMode(unsigned long mode);
  void adcSetChannel(unsigned long channel);
  void adcSetGain(unsigned long gain);
  void adcSetIntReference(unsigned char type);
  unsigned long adcSingleConversion(void);
  void adcCalibrate(unsigned char mode, unsigned char channel);
  bool spi_read_adc(InputChannel input_channel, Gain_e gain, uint8_t* buffer);
  unsigned long adcGetRegisterValue(unsigned char regAddress,unsigned char size);
  void adcSetRegisterValue(unsigned char regAddress,unsigned long regValue,unsigned char size);
  double adcCalculateChUniPolar(uint8_t *in, Gain_e gain);
  double adcCalculateChBiPolar(uint8_t *in, Gain_e gain);
  double pt1000CalculateTemperature(double in);
  double AlphasenseCalibrate();
  double AlphasenseCalculateNO2(double in);
  double AlphasenseCalculateO3(double in);
  bool adcReadPt100(double *pt1000_v);
  bool adcReadAdcCh6Temp();
  bool adcReadAdcCh1NO2();
  bool adcReadAdcCh2O3();
  void inline spiStart();
  void inline spiStop();
  float mTemperature;
  float mHumidity;
};

#endif
