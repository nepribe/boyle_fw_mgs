#include <Stdio.h>
#include <Wire.h>
//#include "FreqCounter.h"
#include "Dps310.h"


String inString;         // incoming serial String
//String command;
char cmd[30];
byte buf[128];
long adr;
unsigned int reg,data;
double vdd_mv;
long motor_position=0;
char received_string[100];
int motorPin = 9;    // LED connected to digital pin 9
long encoder_count = 0;
long steps = 0;
int wait_time = 5;
int seque_dec[50];
char * pch;
unsigned char sequ_length=0;
int i=0;
unsigned char unlock_pattern[3];
byte i2c_status;
float rtemp_t_offset_calib_temp = 25;

// default programming voltage = 1100 mV (1100 / 1.234 * 2)
// can be changed by "E6" command
int programming_voltage_msb=6;
int programming_voltage_lsb=246;
char adc_byte[3];

unsigned long frq;
int cnt;
int pause_time=10;
char i2c_adr = 0x35;
bool stop = 0;
bool reference_sensors_enabled = 0;
char heater = 0;
int heater_t_off = 20; // 2s off
int heater_t_on = 10; // 1s on
int heater_T_high = 50;
int heater_T_low = 25;
byte heater_temp;
unsigned int timer_100ms = 0;
int st_counter_100ms = 0;

unsigned long timer_clean_pulse_100ms = 0;
unsigned long timer_delay_for_cleaning = 0;
bool heater_enable_in_16x_measurement = 0;
byte heater_toggle_mode = 2;

unsigned long clean_pulse_on_time = 2;
int clean_pulse_temp = 75;

bool trigger = 0;
static unsigned long clean_pulse_off_time = 90;
static unsigned long clean_pulse_phase_shift = 0;
bool heater_stat = 0;
bool heater_stat_old = 0;
bool cleaning_stat = 0;
bool phase_shift_done = 0;

unsigned int ref_sens_interval = 100;
bool ref_sensor_readout_ongoing = 0;

bool demo_board_with_eight_channels = 0;

unsigned int time_per_falling_step;
unsigned int time_per_rising_step;
byte falling_steps;
byte rising_steps;

bool temp_ramping_ongoing = 0;

unsigned int timer_100ms_ramp = 0;

float heater_T_high_stepsize=0;
float heater_T_low_stepsize=0;
byte step_counter=0;


// Dps310 Opject
Dps310 Dps310PressureSensor = Dps310();


#define INT 5
#define SCAN_EN 12
#define SCAN_RST 11
#define SCAN_RST 11

// for daisy chain communication with ADG714
#define SYNC A0
#define SCLK 6 // D6
#define DOUT 7 // D7

// SHT25 I2C address is 0x40(64)
#define Addr 0x40

long dps310_pressure();
//void decode_received_command(bool enable_echo);
//byte sht25_temperature(void);
//byte sht25_humidity(void);


void setup() {

  //
  pinMode(INT, INPUT);    // D5, INT
  pinMode(SCAN_RST, OUTPUT);  // D11, SCAN_RST
  pinMode(SCAN_EN, OUTPUT);  // D12, SCAN_ENABLE
  pinMode(8, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  pinMode(A0, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  digitalWrite(2, HIGH);    // set VDD_IF voltage to 0V

  digitalWrite(12, LOW);    // D12 = SCAN EANBLE
  digitalWrite(11, LOW);    // D11 = SCAN_RST

   
  // start serial port at 115200 bps:
  Serial.begin(115200);
  Serial.setTimeout(10);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    
  }

  

  Wire.begin(); // join i2c bus (address optional for master)
  
  Wire.setClock(300000);

  Wire.beginTransmission(0x90); // transmit to device #44 (0x2c)
  
  Wire.write(0x01);            // sends instruction byte
  
  Wire.write(0x02);            // sends potentiometer value byte
  
  Wire.endTransmission();     // stop transmitting

  Dps310PressureSensor.begin(Wire); // 76 if SDO is pulled low, if floating 77

  ///////////////////////////////////////////////////
  ///// timer1 interrupt setup
  ///////////////////////////////////////////////////

  // interrupt frequency (Hz) = (Arduino clock speed 16,000,000Hz) / (prescaler * (compare match register + 1))
  
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 100ms
  OCR1A = 1562;// = (16*10^6) / (1*1024) * time/s - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
}

void i2c_write_reg(int adr, int reg)
{

  Wire.beginTransmission(adr); // transmit to device #44 (0x2c)
  
  Wire.write(reg);            // sends instruction byte
  
  
  Wire.endTransmission();     // stop transmitting

}

byte i2c_write_reg_data(int adr, int reg, int data)
{
  Wire.beginTransmission(adr); // transmit to device #44 (0x2c)
  
  Wire.write(reg);            // sends instruction byte
  Wire.write(data);            // sends instruction byte
  
  byte stat = Wire.endTransmission();     // stop transmitting

  return stat;
}

void read_adc(int output_data)
{
      char i;

       // start one conversion
       i2c_write_reg(64,8);

       // wait 55 ms, conversion takes 50ms (@20SPS)
       delay(55);

       // read out result (3 bytes)
       i2c_write_reg(64,16);
  
      Wire.requestFrom(64, 3, true);

      i=0;

      while(Wire.available())    // slave may send less than requested
      { 
        int c = Wire.read();    // receive a byte as character

        if(output_data==1)
          Serial.print(c);         // print the character
          Serial.print(" ");         // print the character

        // store result in a global variable for general purpose use
        adc_byte[i] = c;
        i++;
        
      }
     
      Wire.endTransmission();     // stop transmitting

      Serial.println("*"); // echo back data


}


double convert_byte_in_double(char byte3, char byte2, char byte1)
{
    double value=0;
    double ADC_vref = 2.052;
    
    if (byte3 & 128) // check if result is negativ, MSB is sign bit
    {
        byte3 = byte3 & 0x7F; // remove sign bit
        value = (ADC_vref/8388608) * ( (byte3*65536 + byte2*256 + byte1) - 8388608  );
    }
    else
        value = (ADC_vref/8388608) * (byte3*65536 + byte2*256 + byte1);

    return value;
}


void wait(int adr, int echo) // in ms
{
      
      delay(adr);

      if (echo == 1)
      {
        Serial.print(F("wait")); // echo back data
        Serial.println("*"); // echo back data
      }
      
}     

void set_vdd (int adr, int reg, int echo)
{
  // DAC7571: 12bit(4096 values) from 0 to 5V
      // 1 LSB = 5.000V / 4095 = 1.221 mV
      // requested value is in mV
      
      vdd_mv = (double)(adr << 8) + (double)reg;
      vdd_mv = vdd_mv / 1.221; // scale by DAC LSB

      adr = (int)vdd_mv;
      adr = adr & 0xFF00;
      adr = adr >> 8;

      reg = (int)vdd_mv;
      reg = reg & 0x00FF;
      
      i2c_write_reg_data(0x4C, adr, reg);

      // set programming voltage to 0V (for compatibilty reasons with Frömmel board)
      i2c_write_reg_data(0x4D, 0, 0);

      if (echo == 1)
      {
        Serial.print(F("E3 ")); // echo back data
        Serial.print(adr); // echo back data
        Serial.print(" ");         // print the character
        Serial.print(reg); // echo back data
        Serial.print(" ");         // print the character
        Serial.print(vdd_mv); // echo back data
        Serial.print(" ");         // print the character
  
        Serial.println("*"); // echo back data
      }

      
}

void adc_st_16x()
{

//char c[] ="";
unsigned char t,j;
unsigned int msb,lsb;
long time_out = 0;
byte humdidity = 0;
float temperature = 0;
long pressure = 0;
byte temp_bytes[2] = {0, 0};
byte humdidity_bytes[2] = {0, 0};
int measure_reference_sensors;

word batch_id = 0;

char number_of_ASICs = 16;
if(demo_board_with_eight_channels == 1) number_of_ASICs = 8;

// reset heater timers and status
timer_clean_pulse_100ms = 0;
timer_100ms = 0;
heater_stat = 0;
cleaning_stat = 0;
phase_shift_done = 0;
timer_delay_for_cleaning = 0;



  // Start command is sent to all ASIC at the same time (to avoid multiple measurement times)
  // after measurement is done, all ASIC SDA pins are multiplexed to ARDUINO and data is read out
  //
  // start "fake-selftimed-mode"
  // timing will be done by Arduino waits
  // it has to be set with the "SPS" command -> samples per second
  // interrupt pin will be used to check for "measurement finished"

  // connect SDA line from ASIC #1
  multiplex_to_chip(0x00,0x01);
     
 // read status of reg 0x18
 char reg18 = r_reg(i2c_adr, 0x18, 0);


 // connect all SDA lines from all 16 ASICs in parallel
 multiplex_to_chip(0xFF,0xFF);

 // enable interrupt
 i2c_write_reg_data(i2c_adr,0x1A,0x02);

 // clear old interrupt
 i2c_write_reg_data(i2c_adr,0x1C,0x02);

 stop=0;

 // reset heater timer
 timer_100ms = 0;

 // init measure_reference_sensors to setup first measurement at end of heater interval
 measure_reference_sensors = heater_t_on + heater_t_off - 10;
 if (measure_reference_sensors < 1)
 {
  measure_reference_sensors = 1;  // only to cover erronous setting of heater_t_on + heater_t_off
 }
  
  while(1)
  {

    

    // connect all SDA lines from all 16 ASICs in parallel
    multiplex_to_chip(0xFF,0xFF);

     // clear all interrupts
    i2c_write_reg_data(i2c_adr,0x1C,0x07);
 
    // start measurement by setting MEASURE_EN
    if(heater_enable_in_16x_measurement == 1)
    {
      if(heater_toggle_mode == 0)
      {
       // no toggling. heater can be turned on and off by sending the "all_heaters_on" and "all_heaters_off" command before enabling ST mode
       i2c_write_reg_data(i2c_adr,0x18,0x04 | heater);
      } else if (heater_toggle_mode == 1)
      {
        // toggle between T_high and heater off
        i2c_write_reg_data(i2c_adr,0x24, heater_temp);
        i2c_write_reg_data(i2c_adr,0x18,0x04 | heater);
      } else if (heater_toggle_mode == 2)
      {
        // toggle between T_high and T_low
        i2c_write_reg_data(i2c_adr,0x24, heater_temp); // 
        i2c_write_reg_data(i2c_adr,0x18,0x06); // start measurement with heater on
      } else {
        // TODO: unhandled case
      }
      
    } else {
       // heater is turned off
       i2c_write_reg_data(i2c_adr,0x18,0x04); 
       
    }

    // connect SDA line from ASIC #1
    multiplex_to_chip(0x00,0x01);

    // wait till interrupt pin is pulled low
    // and check if stop command was sent
    time_out =0;
    while(digitalRead(INT) == 1)
    {

          time_out++;
          if (time_out > 100000)
          {
            Serial.println(F("Error: no end of measurement interrupt from Boyle !!!"));
            return;
          }
         
    }
    
    // connect all SDA lines from all 16 ASICs in parallel
    multiplex_to_chip(0xFF,0xFF);
    if (Serial.available() > 0) {
 
                 decode_received_command(0);
       
      }
      
    if(stop==1) {
      
      // disbale heater
            
      //multiplex_to_chip(0xFF,0xFF);   // connect all SDA lines from all 16 ASICs in parallel
      //i2c_write_reg_data(i2c_adr,0x18,0x00); // disable heater 

      //heater_enable_in_16x_measurement = 0;
      
      break;
    }

    // st_counter_100ms is handling the delay to adjust sampling rate in units of 100ms
    // st_counter_100ms = pause_time;  // setup st_counter_100ms for ISR(TIMER1_COMPA_vect)
    // while (st_counter_100ms > 0);   // in TIMER1 interrupt st_counter_100ms is decremented every 100ms

    // pause time is set by the SPS command
    delay(pause_time);
    
    
    msb = 0;
    lsb = 0;

    // sending the two byte batch_id before cycling through the ASICs.
    received_string[0] = (batch_id & 0xFF00) >> 8;
    received_string[1] = (batch_id & 0x00FF);
    Serial.write(received_string[0]);
    Serial.write(received_string[1]);
    
    // updating the results of the reference sensors in the output buffer
    // Read 2 bytes of data: humidity msb, humidity lsb
    received_string[20] = humdidity_bytes[0];
    received_string[21] = humdidity_bytes[1];
    
    // 3 bytes: lsb, csb, msb
    received_string[22] = (pressure & 255);
    received_string[23] = (pressure >> 8)  & 255;
    received_string[24] = (pressure >> 16) & 255;
    
    // Read 2 bytes of data: temp msb, temp lsb
    received_string[25] = temp_bytes[0];
    received_string[26] = temp_bytes[1];

    // incrementing batch_id, rolling over is expected. range of 65535 is more than enough to check for dropped batches on the host.
    batch_id++;

    // read result and transmit it to PC
    for(j=0;j<number_of_ASICs;j++)
    {
        // connect SDA line

        msb = 1<<j;
        lsb = msb & 0xFF;
        msb = msb & 0xFF00;
        msb = msb >> 8;
        
        multiplex_to_chip(msb,lsb);
    
        //Wire.endTransmission();     // stop transmitting
        Wire.beginTransmission(i2c_adr); // transmit to device address
        Wire.write(0);               // sends start for block read
        i2c_status = Wire.endTransmission();     // stop transmitting

        if(i2c_status == 0) // slave sent AKN
        {
        
          Wire.requestFrom(i2c_adr, 20);   // read 20 bytes
          i=0;
          while(Wire.available())    // slave may send less than requested
          { 
            received_string[i++]  = Wire.read();    // receive a byte as character
          }

        } else {
          for(i=0;i<20;i++)
          received_string[i] = 0xFF;
        }


        ///////////////////////////////////////////////
        // read out reference sensors when ASIC 16 is connected via the multiplexer. The reference sensors share the same SDA line (SDA16)
        // to keep decoding of the data easier, the values for humidty and pressure are also sent 
        // in the data for ASIC 1 till 15 (just repeated from the last measurement)
        ///////////////////////////////////////////////

        
        if( (j==15) && (measure_reference_sensors > ref_sens_interval) && (reference_sensors_enabled==1) && (demo_board_with_eight_channels == 0)) // read out reference sensors SHT25 & DPS310 (they are connected to SDA16)
        {
           ref_sensor_readout_ongoing = 1; // this bit stops the timer for the heater toggling
           
           sht25_temperature(temp_bytes);
           sht25_humidity(humdidity_bytes);
           pressure = dps310_pressure();

           ref_sensor_readout_ongoing = 0; // this bit enables the timer for the heater toggling afain

           measure_reference_sensors = 0;
        }

        if( (j==0) && (measure_reference_sensors > ref_sens_interval) && (reference_sensors_enabled==1) && (demo_board_with_eight_channels == 1)) // read out reference sensors SHT25 & DPS310
        {
           ref_sensor_readout_ongoing = 1; // this bit stops the timer for the heater toggling
           
           sht25_temperature(temp_bytes);
           sht25_humidity(humdidity_bytes);
           pressure = dps310_pressure();

           ref_sensor_readout_ongoing = 0; // this bit enables the timer for the heater toggling afain

           measure_reference_sensors = 0;
        }

       
        for (i=0;i<27;i++)
        {
          Serial.write(received_string[i]); // echo back data
         
        }

        
        
    
    }

    measure_reference_sensors++;

    // 16x24 bytes = 384 bytes =  6 x 64 bytes 
    // -> send ONE termination data packet (also 64 bytes)
    //Serial.println("this_is_the_end_of_one_valid_data_packet___________________*\r\n");
    Serial.println(F("*\r\n"));

    
  }
   
}

void adc_st()
{

char c[] ="";
int t;
long time_out = 0;
  
  // start "fake-selftimed-mode"
  // timing will be done by Arduino waits
  // it has to be set with the "SPS" command -> samples per second
  // interrupt pin will be used to check for "measurement finished"

 // enable interrupt
 i2c_write_reg_data(i2c_adr,0x1A,0x02);

 // clear old interrupt
 i2c_write_reg_data(i2c_adr,0x1C,0x02);

 stop=0;
 
  while(1)
  {

    if(stop==1) break;
    
     // read status of reg 0x18
     char reg18 = r_reg(i2c_adr, 0x18, 0);

     // clear all interrupts
    i2c_write_reg_data(i2c_adr,0x1C,0x07);
 
    // start measurement by setting MEASURE_EN
    i2c_write_reg_data(i2c_adr,0x18,0x04 | reg18);

    

    // wait till interrupt pin is pulled low
    // and check if stop command was sent
    time_out=0;
    while(digitalRead(INT) == 1)
    {
          time_out++;
          if (time_out > 100000)
          {
            Serial.println(F("Error: no end of measurement interrupt from Boyle !!!"));
            return;
          }
    }
    time_out=0;

    if (Serial.available() > 0) {
 
                 decode_received_command(0);
       
          }

    // st_counter_100ms is handling the delay to adjust sampling rate in units of 100ms
    // st_counter_100ms = pause_time;  // setup st_counter_100ms for ISR(TIMER1_COMPA_vect)
    // while (st_counter_100ms > 0);   // in TIMER1 interrupt st_counter_100ms is decremented every 100ms

    // pause time is set by the SPS command
    delay(pause_time);
    
    

    // read RS1 result and transmit it to PC
    // read command is sending back data

    //Wire.endTransmission();     // stop transmitting
    Wire.beginTransmission(i2c_adr); // transmit to device address
        Wire.write(0);               // sends start for block read
        Wire.endTransmission();     // stop transmitting
        
        Wire.requestFrom(i2c_adr, 20);   // read 20 bytes
        i=0;
        while(Wire.available())    // slave may send less than requested
        { 
          received_string[i++]  = Wire.read();    // receive a byte as character
        }

        for (i=0;i<20;i++)
        {
          Serial.write(received_string[i]); // echo back data
        }
        
    Serial.println(F("*\r\n"));

    
  }
   
}

char r_reg(char adr,char reg, bool transmitt_data_via_usb)
{

        int c=0;
        Wire.beginTransmission(adr); // transmit to device #44 (0x2c)
        Wire.write(reg);            // sends instruction byte
        Wire.endTransmission();     // stop transmitting
        
        Wire.requestFrom(adr, 1);

        while(Wire.available())    // slave may send less than requested
        { 
           c = Wire.read();    // receive a byte as character
          
          if(transmitt_data_via_usb==1) {
            Serial.print(c);         // print the character
            Serial.print(" ");         // print the character
          }
          
        }

       if(transmitt_data_via_usb==1)  Serial.println("*"); // echo back data

       return c;
}

void measure_frequency(bool enable_echo)
{
  /*
  FreqCounter::f_comp=10;   // Cal Value / Calibrate with professional Freq Counter
      FreqCounter::start(100);  // 100 ms Gate Time

      while (FreqCounter::f_ready == 0) {delay(1);}

      frq=FreqCounter::f_freq;

      if(enable_echo==1){

      Serial.print(F("f = "));
      Serial.print(frq/100);
      Serial.println(F(" kHz*"));
      }
      */
}



void multiplex_to_chip(int MSB, int LSB)
{
  // 5 x ADG714 are daisy chained -> 5x8bits have to be written
  //
  char i,k,m;
  int j;
 int bit_test_result;
  
  // REFRESH shift registers
  __asm__("nop\n\t");
  
  PORTC |=(1<< PORTC0); //set A0 (SYNC)
  
  __asm__("nop\n\t");
  
  PORTC &= ~(1<<PORTC0);//clear A0 (SCLK)

  //////////////////////////////////////////////////////////////
  // connect or disconnect SDA and INT pin from chip 9 till 16
  /////////////////////////////////////////////////////////////

  for(m=0;m<2;m++) // do it twice and change bit test field after the MSB was sent
  {

      if(m==1) MSB = LSB;
      MSB = MSB & 0xFF;
        
      for(k=0;k<2;k++) // do it twice because the INT and the SDA switches are 2 x 8 bit
      {

         PORTD &= ~(1<<PORTD7);//clear D7 (DOUT)
        
          j = 128;        
          for(i=0;i<8;i++)
          {
        
                 //bit_test_result = (MSB >> j) & 1;
       
                  //if( bit_test_result == 1)  {
                  
                  bit_test_result = MSB & j;
                  
                  if(bit_test_result > 0)  {

                    PORTD |=(1<< PORTD7); //set D7 (DOUT)
                    
                  }
                  else             {
     
                    PORTD &= ~(1<<PORTD7);//clear D7 (DOUT)
                    
                  }
                  
                  __asm__("nop\n\t");
                  
                  //digitalWrite(SCLK, 1);
                  
                  PORTD |=(1<< PORTD6); //set D6 (SCLK)
                  
                  __asm__("nop\n\t");

                  PORTD &= ~(1<<PORTD6);//clear D6 (SCLK)
                  
                  __asm__("nop\n\t");
                  
                  PORTD &= ~(1<<PORTD7);//clear D7 DOUT)
        
                  j = j>>1;
          }

          
      }
  }

  ////////////////////////////////////////////////////////
  // block select
  ////////////////////////////////////////////////////////
  
  // data is clocked in on FALLING!!! edge
  for(i=0;i<8;i++)
  {
 
         __asm__("nop\n\t");
          PORTD |=(1<< PORTD7); //set D7 (DOUT)
          __asm__("nop\n\t");
          PORTD |=(1<< PORTD6); //set D6 (SCLK)
          __asm__("nop\n\t");
          PORTD &= ~(1<<PORTD6);//clear D6 (SCLK)
          __asm__("nop\n\t");
          PORTD &= ~(1<<PORTD7);//clear D7 (DOUT)
  } 

  // REFRESH shift registers
  __asm__("nop\n\t");
  PORTC |=(1<< PORTC0); //set A0 (SYNC)

  
  
}

void read_rref_settings()
{
    int i,j;
    unsigned int msb,lsb;
    
    char number_of_ASICs = 16;
    if(demo_board_with_eight_channels == 1) number_of_ASICs = 8;
  
    for(j=0;j<number_of_ASICs;j++)
    {
        // connect SDA line

        msb = 1<<j;
        lsb = msb & 0xFF;
        msb = msb & 0xFF00;
        msb = msb >> 8;
        
        multiplex_to_chip(msb,lsb);
    
        //Wire.endTransmission();     // stop transmitting
        Wire.beginTransmission(i2c_adr); // transmit to device address
        Wire.write(0x3B);               // sends start for block read
        i2c_status = Wire.endTransmission();     // stop transmitting

        if (i2c_status == 0) // slave sent AKN
        {
                  
            Wire.requestFrom(i2c_adr, 5);   // read 5 bytes
            i=0;
            while(Wire.available())    // slave may send less than requested
            { 
              received_string[i++]  = Wire.read();    // receive a byte as character
            }
            
        } else {
          for(i=0;i<5;i++) received_string[i]=0xBB; // -> part is broken or missing -> this will cause a fail in the autostatus summary report
        }
       
        for (i=0;i<5;i++)
        {
          Serial.write(received_string[i]); // echo back data
        }

        //delay(500); for debugging to see the multiplexing LEDs stepping
    
    }
   
    Serial.println("*");

}

void measure_pid_t_offset(void)
{

  // 1.)
  // read out reference sensor temperature and compare with ASCI temp
  // if there is no big delta, Rtemp should be at the same temp.
  //
  // 2.)
  // for every ASCI do following:
  // - read Rtemp at known ambient temperature
  // - write offset value in register PID_T_OFFSET register 
  //

  byte i;
  unsigned int msb,lsb;
  byte sht25_bytes[2];
  unsigned int data_buffer[17];
  unsigned int rtemp1;
  unsigned int asic_temp1;
  long time_out = 0;
  bool error;
  long raw_data=123;

  char number_of_ASICs = 16;
  if(demo_board_with_eight_channels == 1) number_of_ASICs = 8;

  //////////////////////////////////////
  // READ temperature from reference sensor
  //////////////////////////////////////
  sht25_temperature(sht25_bytes);
  float reference_temperature = (((sht25_bytes[0] * 256.0 + sht25_bytes[1]) * 175.72) / 65536.0) - 46.85;
  
  // connect SDA lines from ASIC #1 in
  multiplex_to_chip(0x01,0x01);

  // read out register 0x21 to restore its value after autoscaling
  // read status of reg 0x21, echo = 0
  char reg21 = r_reg(i2c_adr, 0x21, 0);
 
  ////////////////////////////////////////////////////////////
  // connect all SDA lines from all 16 ASICs in parallel
  ////////////////////////////////////////////////////////////
  
  multiplex_to_chip(0xFF,0xFF);

  ////////////////////////////////////////////////////////////
  // enable "measurement_finished" interrupt
  ////////////////////////////////////////////////////////////
  
  i2c_write_reg_data(i2c_adr,0x1A,0x02);

  ////////////////////////////////////////////////////////////
  // clear old interrupt
  ////////////////////////////////////////////////////////////
  
  i2c_write_reg_data(i2c_adr,0x1C,0x02);

  ////////////////////////////////////////////////////////////
  // enable RTEMP and ASIC temp measurement
  ////////////////////////////////////////////////////////////
  
  i2c_write_reg_data(i2c_adr,0x21,0x60);

  stop = 0;

  for(i=0;i<number_of_ASICs;i++) // step through all 16 ASICs
  {

    msb = 0;
    lsb = 0;
    
    // connect SDA line of single ASCIS
    msb = 1<<i;
    lsb = msb & 0xFF;
    msb = msb & 0xFF00;
    msb = msb >> 8;
    multiplex_to_chip(msb,lsb);

      
    if(stop==1) break;

    // clear old interrupt
     i2c_write_reg_data(i2c_adr,0x1C,0x02);

    /////////////////////////////////////////////////////////////////////////////////
    // start measurement by setting MEASURE_EN
    /////////////////////////////////////////////////////////////////////////////////
    delay(50);
    i2c_status = i2c_write_reg_data(i2c_adr,0x18,0x04);

    if (i2c_status == 0) // I2C communication is okay, slave sent AKN
    {
      
        // wait till interrupt pin is pulled low
        // and check if stop command was sent
        time_out =0;

        error = 0;

        if(demo_board_with_eight_channels == 1)delay(30);
        else
        {
          
          while(digitalRead(INT) == 1)
          {
    
              time_out++;
              if (time_out > 100000)
              {
                //Serial.println("Error: no end of measurement interrupt from Boyle !!!");
                //return;
                error = 1;
                break;
              }
             
          }
        }
     

      /////////////////////////////////////////////////////////////////////////////////
      // read channel results
      /////////////////////////////////////////////////////////////////////////////////

      if (error == 0) // no timeout occoured -> read the results
      {
        
          
          //Wire.endTransmission();     // stop transmitting
          Wire.beginTransmission(i2c_adr); // transmit to device address
          Wire.write(0);               // send start address for block read
          Wire.endTransmission();     // stop transmitting
          
          Wire.requestFrom(i2c_adr, 17);   // read 14 bytes (0x00 till 0x10)
      
      
          char k=0;
          while(Wire.available())    // slave may send less than requested
          { 
            data_buffer[k++]  = Wire.read();    // receive a byte as character
          }

          if (i==0) {// read the ASIC temperature from chip#1
            
            raw_data = (long)data_buffer[16] * 65536 +  (long)data_buffer[15] * 256 +  (long)data_buffer[14];
            rtemp_t_offset_calib_temp = convert_ASIC_temp_raw_data_to_celsius(float(raw_data));

          }
          

          // read Rtemp and copy this value to the PID_T_OFFSET register
          
          rtemp1 = ((data_buffer[1] << 8) + data_buffer[0] );
          
                   
          if (rtemp1 < 65535){         
            i2c_write_reg_data(i2c_adr,0x2E,data_buffer[0]);
            i2c_write_reg_data(i2c_adr,0x2F,data_buffer[1]);
            Serial.print(rtemp1); // send termination string    
            Serial.print(" ");
          } else { // RTEMP no detected -> set duty cycle to minimum
            i2c_write_reg_data(i2c_adr,0x2E,0); // setting the PID_T_OFFSET to zero is setting the loop duty cycle to 1%
            i2c_write_reg_data(i2c_adr,0x2F,0);
            Serial.print("x "); // send termination string  
          }
          
  
          
      
      }  else Serial.print(F("no_int ")); // send termination string   
      
     
   } else Serial.print(F("no_ackn ")); // send termination string
     
     
  }

   // connect all SDA lines from all 16 ASICs in parallel
    multiplex_to_chip(0xFF,0xFF);

  // restore the content of register 0x21
  i2c_write_reg_data(i2c_adr,0x21,reg21);

  Serial.print(rtemp_t_offset_calib_temp);
  Serial.print(" ");
    
  Serial.println("*"); // send termination string
  
}

void autoscale(double external_rref)
{

  // for every ASCI do following:
  // enable end of measurement interrupt
  // enable all input channels
  // set all rrefs to 0.5k
  // start a measurement
  // check if ADC result < 26000 LSBs:
  //  IF YES keep Rref
  //  ELSE increase Rref
  //  for Rref_extuse all Rrefs: used for calibrating the absulote gain of all Rref-settings

  unsigned char t,j,i,k;
  unsigned int data_buffer[14];
  unsigned int msb,lsb;
  long time_out = 0;
  bool error;

  char rref_rtemp1_index;
  char rref_rsens1_index;
  char rref_rsens2_index;
  char rref_rsens3_index;
  char rref_rsens4_index;
  char rref_rtemp2_index;
  char rref_ext_index;
  double internal_rref[12];
  char best_rext_reference_index;

  unsigned int rtemp1 ; 
    unsigned int rsens1 ;
    unsigned int rsens2 ;
    unsigned int rsens3 ;
    unsigned int rsens4 ;
    unsigned int rtemp2 ; 
    unsigned int rref_ext ; 

    bool trigger;

    char number_of_ASICs = 16;
    if(demo_board_with_eight_channels == 1) number_of_ASICs = 8;

  internal_rref[0] = 500;
  internal_rref[1] = 1000;
  internal_rref[2] = 2000;
  internal_rref[3] = 4000;
  internal_rref[4] = 8000;
  internal_rref[5] = 16000;
  internal_rref[6] = 32000;
  internal_rref[7] = 64000;
  internal_rref[8] = 128000;
  internal_rref[9] = 256000;
  internal_rref[10] = 512000;
  internal_rref[11] = 1024000;

  double min_rref_value = external_rref * 1.2;


  stop = 0;

  // connect SDA lines from ASIC #1 in
  multiplex_to_chip(0x01,0x01);

  // read out register 0x21 to restore its value after autoscaling
  // read status of reg 0x21, echo = 0
  char reg21 = r_reg(i2c_adr, 0x21, 0);


  // connect all SDA lines from all 16 ASICs in parallel
  multiplex_to_chip(0xFF,0xFF);

  // enable "measurement_finished" interrupt
  i2c_write_reg_data(i2c_adr,0x1A,0x02);

  // clear old interrupt
  i2c_write_reg_data(i2c_adr,0x1C,0x02);



  
  // enable all input channels (no ASIC temp measurement)
  i2c_write_reg_data(i2c_adr,0x21,0x3F);
 

 
  for(i=0;i<number_of_ASICs;i++) // step through all 16 ASICs
  {

    // connect SDA line of single ASCIS
    msb = 1<<i;
    lsb = msb & 0xFF;
    msb = msb & 0xFF00;
    msb = msb >> 8;
    multiplex_to_chip(msb,lsb);

    rref_rtemp1_index = 0;
    rref_rsens1_index = 0;
    rref_rsens2_index = 0;
    rref_rsens3_index = 0;
    rref_rsens4_index = 0;
    rref_ext_index = 0;

    
    trigger = 0;
    for(j=0;j<12;j++)   // step through all 12 Rref seetings (0.5k till 1024k)
    {
      
          if(stop==1) break;

          /////////////////////////////////////////////////////////////////////////////////
          // set rrefs
          /////////////////////////////////////////////////////////////////////////////////
          int data  = (rref_rsens2_index<<4) +  rref_rsens1_index;
          i2c_status = i2c_write_reg_data(i2c_adr,0x3B,data);

          if (i2c_status == 0) // I2C communication is okay, slave sent AKN
          {

          data  = (rref_rsens4_index<<4) +  rref_rsens3_index;
          i2c_write_reg_data(i2c_adr,0x3C,data);

          i2c_write_reg_data(i2c_adr,0x3F,rref_rtemp1_index);

          i2c_write_reg_data(i2c_adr,0x40,rref_ext_index);

          delay(1);
          

          /////////////////////////////////////////////////////////////////////////////////
          // start measurement by setting MEASURE_EN
          /////////////////////////////////////////////////////////////////////////////////
          
          i2c_write_reg_data(i2c_adr,0x18,0x04);
      
          // wait till interrupt pin is pulled low
          // and check if stop command was sent
          time_out =0;

          error = 0;
          if(demo_board_with_eight_channels == 1)delay(30);
          else
          {
            
            while(digitalRead(INT) == 1)
            {
      
                time_out++;
                if (time_out > 100000)
                {
                  Serial.println("Error: no end of measurement interrupt from Boyle !!!");
                  return;
                  error = 1;
                  break;
                }
               
            }
          }
          
          
          msb = 0;
          lsb = 0;

        /////////////////////////////////////////////////////////////////////////////////
        // read channel results
        /////////////////////////////////////////////////////////////////////////////////

        if (error == 0) // no timeout occoured -> read the results
        {
          
            
            //Wire.endTransmission();     // stop transmitting
            Wire.beginTransmission(i2c_adr); // transmit to device address
            Wire.write(0);               // send start address for block read
            Wire.endTransmission();     // stop transmitting
            
            Wire.requestFrom(i2c_adr, 14);   // read 14 bytes
        
        
            char k=0;
            while(Wire.available())    // slave may send less than requested
            { 
              data_buffer[k++]  = Wire.read();    // receive a byte as character
            }
           
              rtemp1 = ((data_buffer[1] << 8) + data_buffer[0] ) - 32768 ; 
              rsens1 = ((data_buffer[3] << 8) + data_buffer[2] ) - 32768 ;
              rsens2 = ((data_buffer[5] << 8) + data_buffer[4] ) - 32768 ;
              rsens3 = ((data_buffer[7] << 8) + data_buffer[6] ) - 32768 ;
              rsens4 = ((data_buffer[9] << 8) + data_buffer[8] ) - 32768 ;
              rtemp2 = ((data_buffer[11] << 8) + data_buffer[10] ) - 32768 ; 
              rref_ext = ((data_buffer[13] << 8) + data_buffer[12] ) - 32768 ; 
    
            // check if ADC results are lower than 26000 LSBs (80% of ADC fullscale)
            // and if selected internal rref is 20% higher than external used rref
    
            
    
            if (rtemp1 > 26000 || internal_rref[rref_rtemp1_index] < min_rref_value ) rref_rtemp1_index++;
            if (rsens1 > 26000 || internal_rref[rref_rsens1_index] < min_rref_value ) rref_rsens1_index++;
            if (rsens2 > 26000 || internal_rref[rref_rsens2_index] < min_rref_value ) rref_rsens2_index++;
            if (rsens3 > 26000 || internal_rref[rref_rsens3_index] < min_rref_value ) rref_rsens3_index++;
            if (rsens4 > 26000 || internal_rref[rref_rsens4_index] < min_rref_value ) rref_rsens4_index++;
            if (rtemp2 > 26000 || internal_rref[rref_rtemp2_index] < min_rref_value ) rref_rtemp2_index++;

            
            if (rref_ext < 30000 && trigger == 0) // save the best reference value for REXT
            { 
              best_rext_reference_index = rref_ext_index;
              trigger = 1;
            }
            
            rref_ext_index++;
    
            // clear old interrupt
            i2c_write_reg_data(i2c_adr,0x1C,0x02);
        
        }  else {
          
          rref_ext = 0;
          
        }
        } else {
          
          rref_ext = 0;
        }

        Serial.print(rref_ext);

        Serial.print(" ");

        i2c_write_reg_data(i2c_adr,0x40,best_rext_reference_index);
       
     }
     
     
  }

   // connect all SDA lines from all 16 ASICs in parallel
  multiplex_to_chip(0xFF,0xFF);

  // restore the content of register 0x21
  i2c_write_reg_data(i2c_adr,0x21,reg21);
  
  Serial.println("*"); // send termination string
  
   
}

void sht25_humidity(byte *sht25_result)
{

      
      // Start I2C transmission
      Wire.beginTransmission(Addr);
      // Send humidity measurement command, NO HOLD master
      Wire.write(0xF5);
      // Stop I2C transmission
      i2c_status = Wire.endTransmission(false); // send + rep start

      if(i2c_status == 0)
      {
            
          delay(100);
        
          // Request 2 bytes of data
          Wire.requestFrom(Addr, 2);
        
          // Read 2 bytes of data
          // humidity msb, humidity lsb
          if(Wire.available() == 2)
          {
            sht25_result[0] = Wire.read();
            sht25_result[1] = Wire.read();
            
          }
    
          // Convert the data
          // humidity = (((sht25_result[0] * 256.0 + sht25_result[1]) * 125.0) / 65536.0) - 6.0;

           //Serial.print("humidity = " );
           //Serial.print(humidity);
           //Serial.print(" RH ");
        
           
          
      } else {
        
        sht25_result[0] = 0xFF;
        sht25_result[1] = 0xFF;
        
      }

      
}

void sht25_temperature(byte *sht25_result)
{

      
      Wire.beginTransmission(Addr);
      // Send temperature measurement command, NO HOLD master
      Wire.write(0xF3);
      // Stop I2C transmission
      i2c_status = Wire.endTransmission(false);

      if(i2c_status == 0)
      {
        
      
      delay(100);
    
      // Request 2 bytes of data
      Wire.requestFrom(Addr, 2);
    
      // Read 2 bytes of data
      // temp msb, temp lsb
      if(Wire.available() == 2)
      {
        sht25_result[0] = Wire.read();
        sht25_result[1] = Wire.read();
    
        // Convert the data
        // cTemp = (((sht25_result[0] * 256.0 + sht25_result[1]) * 175.72) / 65536.0) - 46.85;
 

        
      } 
      } else {
        // cTemp = 0;
        sht25_result[0] = 0xFF;
        sht25_result[1] = 0xFF;
      }
      
}

long dps310_pressure(void)
{
      int32_t pressure;
      uint8_t oversampling = 7;
      int16_t ret;

      Dps310PressureSensor.end(); // 
      Dps310PressureSensor.begin(Wire); // 76 if SDO is pulled low, if floating 77
      //delay(1);
      
      
      ret = Dps310PressureSensor.measurePressureOnce(pressure, oversampling);
      if (ret != 0)
      {
        //Something went wrong.
        //Look at the library code for more information about return codes
        return -1;
      }
      else
      {
        return (long)pressure;
      }
}

void update_ramp_parameters()
{
    // update the temp ramp step parameters
    // calculate increment for the rising temperature ramp
    if (rising_steps > 0) heater_T_high_stepsize  = (float)(heater_T_high - heater_T_low) / (float)rising_steps;
    if (falling_steps > 0) heater_T_low_stepsize  = (float)(heater_T_high - heater_T_low) / (float)falling_steps;
}

void decode_received_command(bool enable_echo)
{

    byte sht25_bytes[2];
    
    // get incoming byte:
    inString = Serial.readString();
    inString.toCharArray(received_string, 100); 
    sscanf(received_string, "%s %x %x %x",&cmd , &adr, &reg, &data);

    
    if(!strcmp(received_string,"*IDN?"))
    {
      Serial.println(F("BOYLE*"));
    } 
    else{

      if(!strcmp(cmd,"STOP*"))
      {
            stop=1;
      }

      if(!strcmp(cmd,"autoscale"))
      {

            autoscale(adr);
            
      }

      if(!strcmp(cmd,"measure_pid_t_offset"))
      {
            measure_pid_t_offset();
      }

      if(!strcmp(cmd,"read_rref_settings"))
      {
            read_rref_settings();
      }

      if(!strcmp(cmd,"con"))
      {
            // split 16bit into 2 byte;
            reg = adr& 0xFF;
            adr = adr& 0xFF00;
            adr = adr >> 8;
            
            multiplex_to_chip(adr,reg);
            if(enable_echo==1){
              Serial.print(F("connect_to ")); // echo back data
              Serial.print(adr); // echo back data
              Serial.print(" "); // echo back data
              Serial.print(reg); // echo back data
              Serial.println("*"); // echo back data
            }
      }

      /*if(!strcmp(cmd,"freq*"))
      {
        
        measure_frequency(0);
        delay(100);
        measure_frequency(enable_echo);
      
        
      }*/

      if(!strcmp(cmd,"SPS"))
      {
        pause_time = adr;

        Serial.flush();

        if(enable_echo==1){
          Serial.print(cmd);
          Serial.print(pause_time);
          Serial.println("*");
        }
      }

      if(!strcmp(cmd,"SW"))
      {
        digitalWrite(3, 0);
        digitalWrite(8, 0);

        if(adr == 1) 
        {
          digitalWrite(3, 1);
          i2c_write_reg(75,reg);
        }
        
        if(adr == 2)
        {
          digitalWrite(8, 1);
          i2c_write_reg(75,reg);
        }

        if(adr == 3)
        {
            i2c_write_reg(73,reg);
        }

        if(adr == 4)
        {
            digitalWrite(3, 1);
            digitalWrite(8, 1);
            i2c_write_reg(72,reg);
        }

        if(adr == 5)
        {
            i2c_write_reg(74,reg);
        }

        if(enable_echo==1)
        {
          Serial.println(F("SW")); // echo back data
          Serial.println(adr); // echo back data
          Serial.println(reg); // echo back data
          Serial.println("*"); // echo back data
        }
          
      }

      if(enable_echo==1) 
      {
          if(!strcmp(cmd,"GBR"))
          {
              // clear buffer with "0xFF"
              for(i=0;i<128;i++) buf[i] = 0xFF;
              
              i=0;

              Wire.beginTransmission(i2c_adr); // transmit to device #44 (0x2c)
              Wire.write(reg);            // sends instruction byte
              i2c_status = Wire.endTransmission();     // stop transmitting

              if(i2c_status == 0)
              { // go on if slave did aknowledge 
              
                  Wire.requestFrom(i2c_adr, 32, 0);
                  while(Wire.available())    // slave may send less than requested
                  { 
                    buf[i++]  = Wire.read();    // receive a byte as character
                  }
          
                  Wire.requestFrom(i2c_adr, 32, 0);
                  while(Wire.available())    // slave may send less than requested
                  { 
                    buf[i++]  = Wire.read();    // receive a byte as character
                  }
          
                  Wire.requestFrom(i2c_adr, 32, 0);
                  while(Wire.available())    // slave may send less than requested
                  { 
                    buf[i++]  = Wire.read();    // receive a byte as character
                  }
          
                  Wire.requestFrom(i2c_adr, 32);
                  while(Wire.available())    // slave may send less than requested
                  { 
                    buf[i++]  = Wire.read();    // receive a byte as character
                  }
                
              }
              for (i=0;i<128;i++)
              {
                Serial.write(buf[i]); // echo back data
              }
              Serial.println("*"); // echo back data            
          }
        
      if(!strcmp(cmd,"adc_st*"))
      {
        adc_st();
      }

      if(!strcmp(cmd,"adc_st_16x"))
      {
        // reset all the timings for the ramp, toggling and cleaning pulse
        timer_100ms = 0;
        timer_100ms_ramp = 0;
        temp_ramping_ongoing = 0;
        heater_stat = 1;
        cleaning_stat = 0;
        timer_clean_pulse_100ms = 0;
        timer_delay_for_cleaning = 0;
        phase_shift_done = 0;
        heater_temp = heater_T_low;
        step_counter = 0;
        
        adc_st_16x();
        
      }
      
      if(!strcmp(cmd,"r_reg"))
      {

        r_reg(adr,reg,1);
      }

      if(!strcmp(cmd,"r_reg_sw"))
      {
                
          Wire.requestFrom(adr, 1);

          while(Wire.available())    // slave may send less than requested
          { 
            int c = Wire.read();    // receive a byte as character
            Serial.println(c);         // print the character
          }

        
          Serial.println("*"); // echo back data
          
      }
      
      }
      
      if(!strcmp(cmd,"w_reg"))
      {

          i2c_write_reg(adr,reg);
    
          if(enable_echo==1){
            
            Serial.println(cmd); // echo back data
            Serial.println(adr); // echo back data
            Serial.println(reg); // echo back data
            Serial.println("*"); // echo back data
            
        }
      

      }
      if(!strcmp(cmd,"all_heaters_on"))
      {
          heater_enable_in_16x_measurement = 1;
          heater = 2;
          heater_temp = heater_T_high;
          Serial.println(cmd); // echo back data
          Serial.println("*"); // echo back data
          
      }

      if(!strcmp(cmd,"all_heaters_off"))
      {
          heater_enable_in_16x_measurement = 0;
          heater = 0;
          Serial.println(cmd); // echo back data
          Serial.println("*"); // echo back da
      }

      if(!strcmp(cmd,"heater_t_on"))
      {
          heater_t_on = adr;
          Serial.println(cmd);
          Serial.println(adr); // echo back data
          Serial.println("*"); // echo back data
          timer_100ms = 0;
      }

      if(!strcmp(cmd,"heater_T_high"))
      {
          heater_T_high = (adr - rtemp_t_offset_calib_temp)/2; // divided by 2 because units of 2°
          Serial.println(cmd);
          Serial.println(heater_T_high); // echo back data
          Serial.println("*"); // echo back data
          timer_100ms = 0;

          update_ramp_parameters();
      }

      if(!strcmp(cmd,"heater_T_low"))
      {
          heater_T_low = (adr - rtemp_t_offset_calib_temp)/2;
          Serial.println(cmd);
          Serial.println(heater_T_low); // echo back data
          Serial.println("*"); // echo back data
          timer_100ms = 0;

          update_ramp_parameters();
      }

      if(!strcmp(cmd,"heater_t_off"))
      {
          heater_t_off = adr;
          Serial.println(cmd);
          Serial.println(adr); // echo back data
          Serial.println("*"); // echo back data
          timer_100ms = 0;
      }

      if(!strcmp(cmd,"heater_toggle_mode"))
      {
          heater_toggle_mode = adr;
          Serial.println(cmd);
          Serial.println(adr); // echo back data
          Serial.println("*"); // echo back data
          timer_100ms = 0;
      }

      if(!strcmp(cmd,"clean_on"))
      {
          clean_pulse_on_time = adr;
          Serial.println(cmd);
          Serial.println(adr); // echo back data
          Serial.println("*"); // echo back data
  
      }

      if(!strcmp(cmd,"clean_temp"))
      {
          clean_pulse_temp = (adr - rtemp_t_offset_calib_temp)/2;
          Serial.println(cmd);
          Serial.print(clean_pulse_temp); // echo back data
          Serial.println("*"); // echo back data

      }

      if(!strcmp(cmd,"clean_off"))
      {
          clean_pulse_off_time = adr;
          Serial.println(cmd);
          Serial.println(adr); // echo back data
          Serial.println("*"); // echo back data
      }

      if(!strcmp(cmd,"clean_shift"))
      {
          clean_pulse_phase_shift = adr;
          Serial.println(cmd);
          Serial.println(adr); // echo back data
          Serial.println("*"); // echo back data

      }

      if(!strcmp(cmd,"ref_sens_interval"))
      {
          ref_sens_interval = adr;
          Serial.println(cmd);
          Serial.println(ref_sens_interval); // echo back data
          Serial.println("*"); // echo back data
      }

      if(!strcmp(cmd,"demo_board"))
      {
          demo_board_with_eight_channels = adr;
          Serial.println(cmd);
          Serial.println(adr); // echo back data
          Serial.println("*"); // echo back data
      }

      if(!strcmp(cmd,"w_reg_data"))
      {

        i2c_write_reg_data(i2c_adr,reg,data);
          
        if(enable_echo==1){
          
            Serial.println(cmd); // echo back data
            Serial.println(adr); // echo back data
            Serial.println(reg); // echo back data
            Serial.println(data); // echo back data
            Serial.println("*"); // echo back data

            
        
        }
        
      }

      if(!strcmp(cmd,"reset_i2c"))
      {
        // I2C is causing trouble ... 
        // send 9 pulses on SCL line to device reset

        TWCR = 0; // reset TwoWire Control Register to default, inactive state
        pinMode(A5, OUTPUT); // set SCL pin output

        for(i=0;i<9;i++)
        {
            digitalWrite(A5, HIGH);    // SCL High
            delay(1);
            digitalWrite(A5, LOW);      // SCL Low
            delay(1);
            digitalWrite(A5, HIGH);    // SCL High
              
        }

        Wire.begin(); // join i2c bus (address optional for master)
      }

      if(!strcmp(cmd,"dps310_pressure?")) // single readout of dps310 pressure sensor
      {
      // connect SDA lines for reference sensors (line 15)
      multiplex_to_chip(0x80,0x00);
        long pressure = dps310_pressure();

        if(pressure > 0)
        {
              Serial.print(pressure);
              Serial.println(F("*")); // echo back data
            
        } else {
          
          Serial.print(F("DPS310 not connected"));
          Serial.println(F("*")); // echo back data
          
        }
        
        
      }

      if(!strcmp(cmd,"dps310_temperature?")) // single readout of dps310 temperature sensor
      {

        int32_t temperature;
        int16_t oversampling = 7;
        int16_t ret;

      // connect SDA lines for reference sensors (line 15)
      multiplex_to_chip(0x80,0x00);
        Dps310PressureSensor.end(); // 
        Dps310PressureSensor.begin(Wire); // 76 if SDO is pulled low, if floating 77
        //delay(1);
        
        
        ret = Dps310PressureSensor.measureTempOnce(temperature, oversampling);
        if (ret != 0)
        {
          //Something went wrong.
          //Look at the library code for more information about return codes
          Serial.print(F("DPS310 error"));
          Serial.println(F("*"));
        }
        else
        {
          Serial.print(temperature);
          Serial.println("*");
        }
        
      }

      if(!strcmp(cmd,"sht25_humidity?")) // single readout of sht25 humidity sensor
      {
          // connect SDA lines for reference sensors (line 15)
          multiplex_to_chip(0x80,0x00);
        sht25_humidity(sht25_bytes);
        
        if(sht25_bytes[0] != 0xFF && sht25_bytes[1] != 0xFF)
        {
              //Serial.print("Relative Humidity :");
              float humidity = (((sht25_bytes[0] * 256.0 + sht25_bytes[1]) * 125.0) / 65536.0) - 6.0;
              Serial.print(humidity);
              //Serial.println(" %RH");
              Serial.println("*"); // echo back data
            
        } else {
          
          Serial.print(F("SHT25 not connected"));
          Serial.println(F("*")); // echo back data
          
        }
        
      }

      if(!strcmp(cmd,"sht25_temperature?")) // single readout of sht25 temperature sensor
      {
        // connect SDA lines for reference sensors (line 15)
        multiplex_to_chip(0x80,0x00);
          sht25_temperature(sht25_bytes);

          if(sht25_bytes[0] != 0xFF && sht25_bytes[1] != 0xFF) {
      
          // Output data to Serial Monitor
          //Serial.print("Temperature in Celsius :");
          float temperature = (((sht25_bytes[0] * 256.0 + sht25_bytes[1]) * 175.72) / 65536.0) - 46.85;
          
          Serial.print(temperature);
          Serial.println("*"); // echo back data
          
          } else {
            Serial.print(F("SHT25 not connected"));
            Serial.println("*"); // echo back data
          }
      }
      

      if(!strcmp(cmd,"wait"))
      {

        wait(adr, 1);
        
      }

      if(!strcmp(cmd,"E3")) // set VDD
      {
        set_vdd(adr, reg, 1);
        
      }

      if(!strcmp(cmd,"version?")) // query version string
      {
        Serial.print(F("27.1.2021 ")); // echo back data
        Serial.println("*"); // echo back data
        
      }

      if(!strcmp(cmd,"reference_sensors_enabled")) // enable readout of reference sensors
      {
        reference_sensors_enabled = 1;
        Serial.print(F("reference_sensors_enabled"));
        Serial.println("*"); // acknowledge command with echo      
      }
      
      if(!strcmp(cmd,"reference_sensors_disabled")) // disable readout of reference sensors
      {
        reference_sensors_enabled = 0;  
        Serial.print(F("reference_sensors_disabled"));
        Serial.println("*"); // acknowledge command with echo    
      }

      if(!strcmp(cmd,"init_asic")) // init ASIC, do autoscaling and measure_PID_T_offset
      {
        init_asic();  
        Serial.print(cmd);
        Serial.println("*"); // acknowledge command with echo    
      }

      if(!strcmp(cmd,"E6")) // programming voltage
      {

        // DAC7571: 12bit(4096 values) from 0 to 5V
        // 1 LSB = 5.000V / 4095 = 1.221 mV
        // requested value is in mV
        // -> divide requested value by 1.234
        // this value is multiplied by two again because of hardware resistor divider
        
        vdd_mv = (double)(adr << 8) + (double)reg;
        vdd_mv = vdd_mv / 1.221; // scale by DAC LSB and resistor

        adr = (int)vdd_mv;
        adr = adr & 0xFF00;
        adr = adr >> 8;

        reg = (int)vdd_mv;
        reg = reg & 0x00FF;

        // store value in global variable
        programming_voltage_msb = adr;
        programming_voltage_lsb = reg;
        

      i2c_write_reg_data(0x4D, programming_voltage_msb, programming_voltage_lsb);
        if(enable_echo==1){
      Serial.println("E6*"); // echo back data
        }
      }

      if(!strcmp(cmd,"read_adc"))
      {


        Wire.requestFrom(adr, 3, true);

        while(Wire.available())    // slave may send less than requested
        { 
          int c = Wire.read();    // receive a byte as character
          Serial.println(c);         // print the character
        }

        
        Wire.endTransmission();     // stop transmitting
        


      }

      if(!strcmp(cmd,"read_adc_od"))
      {
        read_adc(1);
      }

      if(!strcmp(cmd,"rising_steps"))
      {
          rising_steps = adr;
          Serial.print(cmd);
          Serial.print(" ");
          Serial.println(adr); // echo back data

          // calculate increment for the rising temperature ramp
          if(rising_steps > 0)
          heater_T_high_stepsize  = (float)(heater_T_high - heater_T_low) / (float)rising_steps;
          
          Serial.print(heater_T_high_stepsize); // echo back data
          Serial.println("*"); // echo back data
      }

      if(!strcmp(cmd,"falling_steps"))
      {
          falling_steps = adr;
          Serial.println(cmd);
          Serial.print(" ");
          Serial.println(adr); // echo back data
          
          // calculate increment for the falling temperature ramp
          if(falling_steps > 0)
          heater_T_low_stepsize  = (float)(heater_T_high - heater_T_low) / (float)falling_steps;
          
          Serial.print(heater_T_low_stepsize); // echo back data
          Serial.println("*"); // echo back data
      }

      if(!strcmp(cmd,"time_per_rising_step"))
      {
          time_per_rising_step = adr;
          Serial.println(cmd);
          Serial.println(adr); // echo back data
          Serial.println("*"); // echo back data
      }

      if(!strcmp(cmd,"time_per_falling_step"))
      {
          time_per_falling_step = adr;
          Serial.println(cmd);
          Serial.println(adr); // echo back data
          Serial.println("*"); // echo back data
      }

  }
}

float convert_ASIC_temp_raw_data_to_celsius(float raw_data)
{
  
            
  float PDMpercent = raw_data/4194304.0;
  float mu = PDMpercent / ( 1.0 + 2.26546 * PDMpercent);
  float temp = 1366.276 * mu - 290.74;

  return temp;
}

void loop() {

  
  // if we get a valid byte
  if (Serial.available() > 0) {

    decode_received_command(1);

  }
  
}

bool init_asic() 
{

  // connect all SDA lines from all 16 ASICs in parallel
  multiplex_to_chip(0xFF,0xFF);
  
  // set all ASIC registers to default values
  i2c_write_reg_data(i2c_adr,0x18,0x00);
  i2c_write_reg_data(i2c_adr,0x19,0x80);
  i2c_write_reg_data(i2c_adr,0x1A,0x02);
  
  i2c_write_reg_data(i2c_adr,0x1B,0x00);
  i2c_write_reg_data(i2c_adr,0x1C,0x00);
  i2c_write_reg_data(i2c_adr,0x1D,0x23);
  i2c_write_reg_data(i2c_adr,0x1E,0x23);
  
  i2c_write_reg_data(i2c_adr,0x1F,0x09);
  i2c_write_reg_data(i2c_adr,0x20,0x00);
  i2c_write_reg_data(i2c_adr,0x21,0x60);
  i2c_write_reg_data(i2c_adr,0x22,0x80);

  i2c_write_reg_data(i2c_adr,0x24,0x25);
  i2c_write_reg_data(i2c_adr,0x25,0x05);
  i2c_write_reg_data(i2c_adr,0x26,0x00);
  i2c_write_reg_data(i2c_adr,0x27,0x00);

  i2c_write_reg_data(i2c_adr,0x28,0x64);
  i2c_write_reg_data(i2c_adr,0x29,0x00);
  i2c_write_reg_data(i2c_adr,0x2A,0xB8);
  i2c_write_reg_data(i2c_adr,0x2B,0x0B);

  i2c_write_reg_data(i2c_adr,0x2C,0x00);
  i2c_write_reg_data(i2c_adr,0x2D,0x00);
  i2c_write_reg_data(i2c_adr,0x2E,0xD4);
  i2c_write_reg_data(i2c_adr,0x2F,0xB0);

  i2c_write_reg_data(i2c_adr,0x30,0xA5);
  i2c_write_reg_data(i2c_adr,0x31,0x01);
  i2c_write_reg_data(i2c_adr,0x32,0x80);
  i2c_write_reg_data(i2c_adr,0x33,0x00);

  i2c_write_reg_data(i2c_adr,0x34,0x00);
  i2c_write_reg_data(i2c_adr,0x35,0x00);
  i2c_write_reg_data(i2c_adr,0x36,0x74);
  i2c_write_reg_data(i2c_adr,0x37,0x00);

  i2c_write_reg_data(i2c_adr,0x38,0x00);
  i2c_write_reg_data(i2c_adr,0x39,0x00);
  i2c_write_reg_data(i2c_adr,0x3A,0x00);
  i2c_write_reg_data(i2c_adr,0x3B,0x88);

  i2c_write_reg_data(i2c_adr,0x3C,0x7A);
  i2c_write_reg_data(i2c_adr,0x3D,0x00);
  i2c_write_reg_data(i2c_adr,0x3E,0x00);
  i2c_write_reg_data(i2c_adr,0x3F,0x03);

  i2c_write_reg_data(i2c_adr,0x40,0x03);

  i2c_write_reg_data(i2c_adr,0x67,0x00);
  i2c_write_reg_data(i2c_adr,0x68,0x00);
  i2c_write_reg_data(i2c_adr,0x69,0x80);
  i2c_write_reg_data(i2c_adr,0x6A,0x00);
  i2c_write_reg_data(i2c_adr,0x6B,0x00);

  i2c_write_reg_data(i2c_adr,0x70,0x00);
  i2c_write_reg_data(i2c_adr,0x71,0x00);
  i2c_write_reg_data(i2c_adr,0x72,0x00);
  i2c_write_reg_data(i2c_adr,0x73,0x00);
  i2c_write_reg_data(i2c_adr,0x74,0x00);
  i2c_write_reg_data(i2c_adr,0x75,0x00);
  i2c_write_reg_data(i2c_adr,0x76,0x00);
  i2c_write_reg_data(i2c_adr,0x77,0x00);
  i2c_write_reg_data(i2c_adr,0x78,0x00);
  i2c_write_reg_data(i2c_adr,0x79,0x00);
  i2c_write_reg_data(i2c_adr,0x7A,0x00);
  i2c_write_reg_data(i2c_adr,0x7B,0x00);
  i2c_write_reg_data(i2c_adr,0x7C,0x00);
  i2c_write_reg_data(i2c_adr,0x7D,0x00);
  i2c_write_reg_data(i2c_adr,0x7E,0x80);
  i2c_write_reg_data(i2c_adr,0x7F,0x00);


  // do autoscaling of the internal reference resistors
  autoscale(2000);

  // measure_PID_T_offset (for the heater loop)
  measure_pid_t_offset();

  return(1);
  
}


ISR(TIMER1_COMPA_vect){//timer1 interrupt 



    
    
  // only toggle when toggling is enabled, no reference sensor is read and no ramping is ongoing
  if (heater_toggle_mode > 0 && ref_sensor_readout_ongoing == 0 && temp_ramping_ongoing == 0)
  {

      timer_100ms++;

      // st_counter is used in adc_st_x16 to synchronize to 100ms timer
      st_counter_100ms--;
  
      ///////////////////////////////////////////////////
      // generate on-off status for heater
      ///////////////////////////////////////////////////
    
      heater_stat_old = heater_stat; // needed to detect heater toggling
      
      if (timer_100ms >= heater_t_on )
      {
        heater_stat = 0;

      }
      if (timer_100ms >= (heater_t_on + heater_t_off) ){
        
        heater_stat = 1;
        timer_100ms = 0;
        
      }
    
      ///////////////////////////////////////////////////
      // wait till the phase shift is reached
      ///////////////////////////////////////////////////
    
      if (timer_delay_for_cleaning >= clean_pulse_phase_shift && phase_shift_done == 0)
      {
             phase_shift_done = 1;
      }
      
      timer_delay_for_cleaning++;
    
       ///////////////////////////////////////////////////
       // generate on-off status for cleaning pulse
       ///////////////////////////////////////////////////
          
      if (phase_shift_done == 1 && clean_pulse_on_time > 0)
      {
          timer_clean_pulse_100ms++;  
        
          if (timer_clean_pulse_100ms >= clean_pulse_on_time  )cleaning_stat = 0;
          if (timer_clean_pulse_100ms >= (clean_pulse_off_time + clean_pulse_on_time) ){
            
              cleaning_stat = 1;
              timer_clean_pulse_100ms = 0;
        
          }
          if (clean_pulse_on_time == 0){ // switch off cleaning feature
              cleaning_stat = 0;
              timer_clean_pulse_100ms = 0;
          }
      }
      
  
  }

  //////////////////////////////////////////////////////////////////////////////
  // if number of ramp steps for rising edge > 1 do a heater ramp
  //////////////////////////////////////////////////////////////////////////////
  
    if (heater_stat > heater_stat_old && rising_steps > 0 ) // check if the heater wants to toggle from LOW to high
    {
        timer_100ms_ramp++;
        temp_ramping_ongoing = 1; // block heater on/off timer
        
        if ( timer_100ms_ramp >= time_per_rising_step ) // increase ramp by one step, if it is time
        {
          step_counter++;
          heater_temp = heater_T_low + heater_T_high_stepsize*step_counter ;
          timer_100ms_ramp = 0;
        }

        if ( step_counter >= rising_steps ) // check if ramp is done
        {
           temp_ramping_ongoing = 0; // enable normal heater timing
           step_counter = 0;
           timer_100ms_ramp = 0;
        
        }
    } else if (heater_stat < heater_stat_old && falling_steps > 0 ) // check if the heater wants to toggle from HIGH to LOW
    {
        timer_100ms_ramp++;
        temp_ramping_ongoing = 1; // block heater on/off timer
        
        if ( timer_100ms_ramp >= time_per_falling_step ) // increase ramp by one step, if it is time
        {
          step_counter++;
          heater_temp = heater_T_high - heater_T_low_stepsize*step_counter ;
          timer_100ms_ramp = 0;
        }

        if ( step_counter >= falling_steps ) // check if ramp is done
        {
           temp_ramping_ongoing = 0; // enable normal heater timing
           step_counter = 0;
           timer_100ms_ramp = 0;
        
        }
    } else temp_ramping_ongoing = 0; // enable normal heater timing


       ///////////////////////////////////////////////////
      // now lets do a logic && and set heater on/off bit and heater temperature
      ///////////////////////////////////////////////////
    if (heater_toggle_mode > 0 && ref_sensor_readout_ongoing == 0 && temp_ramping_ongoing == 0){
      
    
      if ( cleaning_stat == 1)
      {
        heater_temp = clean_pulse_temp;
        heater = 2;
      }
    
      if ( heater_stat == 1 && cleaning_stat == 0)
      {
        heater_temp = heater_T_high;
        heater = 2;
      }
    
      if ( heater_stat == 0 && cleaning_stat == 0)
      {
        heater_temp = heater_T_low;
        heater = 0;
      }

      //step_counter = 0;
      //timer_100ms_ramp = 0;

    }

   
}

  
  
