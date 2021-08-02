#include <Stdio.h>
#include <Wire.h>
#include <Arduino.h>
#include "boyle.h"
#include "CommHandler.h"
#define INT 4
#define SENSOR_POLL_INTERVAL 900

void Boyle::timerEvent()
{
  if (heater_toggle_mode > 0 && ref_sensor_readout_ongoing == 0){

      timer_100ms++;
      ///////////////////////////////////////////////////
      // generate on-off status for heater
      ///////////////////////////////////////////////////
      if (timer_100ms >= heater_t_on) heater_stat = 0;
      if (timer_100ms >= (heater_t_off + heater_t_on)){
        
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
          
      if (phase_shift_done == 1)
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
      ///////////////////////////////////////////////////
      // now lets do a logic and and set heater on/off bit and heater temperature
      ///////////////////////////////////////////////////
    
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
  
  }
}




void Boyle::set_ref_sensor_readout_ongoing()
{
   ref_sensor_readout_ongoing = 1; // this bit stops the timer for the heater toggling
}
void Boyle::clear_ref_sensor_readout_ongoing()
{
   ref_sensor_readout_ongoing = 0; // this bit enables the timer for the heater toggling again
}
bool Boyle::read_ref_sensor()
{
  return read_ref_sensors;
}

void Boyle::clear_read_ref_sensor()
{
  read_ref_sensors = false;
}

void Boyle::i2c_write_reg(int adr, int reg)
{

  Wire.beginTransmission(adr); // transmit to device #44 (0x2c)
  
  Wire.write(reg);            // sends instruction byte
  
  
  Wire.endTransmission();     // stop transmitting

}
byte Boyle::i2c_write_reg_data(int adr, int reg, int data)
{
  Wire.beginTransmission(adr); // transmit to device #44 (0x2c)
  Wire.write(reg);            // sends instruction byte
  Wire.write(data);            // sends instruction byte

  byte stat = Wire.endTransmission();     // stop transmitting

  return stat;
}
void Boyle::wait(int adr, int echo) // in ms
{
  delay(adr);    
}
void Boyle::set_vdd(int adr, int reg, int echo)
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
void Boyle::asic_poll(uint8_t *sensors_data)
{
  static bool first_time = true;
  unsigned char j=0; 
  unsigned int msb,lsb;
  static long time_out = 0;
  static word batch_id = 0;
  char number_of_ASICs = 16;
  static unsigned int measure_reference_sensors = 0; // initialized with 101 so the condition for sensor readout is satisfied on the first iteration.

  if(first_time && (!stop))
  {
    first_time = false;
    if(demo_board_with_eight_channels == 1) number_of_ASICs = 8;
  
    // reset heater timers and status
    timer_clean_pulse_100ms = 0;
    timer_100ms = 0;
    heater_stat = 0;
    cleaning_stat = 0;
    phase_shift_done = 0;
    timer_delay_for_cleaning = 0;
        
    // read status of reg 0x18
    char reg18 = r_reg(i2c_adr, 0x18, 0);
      
    // enable interrupt
    i2c_write_reg_data(i2c_adr,0x1A,0x02);
  
    // clear old interrupt
    i2c_write_reg_data(i2c_adr,0x1C,0x02);
    stop=0;
    // reset heater timer
    timer_100ms = 0;
  }
  
  //while(1)
  if(!stop)
  {
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
      
    } 
    else {
       // heater is turned off
       i2c_write_reg_data(i2c_adr,0x18,0x04); 
       
    }
    // wait till interrupt pin is pulled low
    // and check if stop command was sent
    time_out =0;
    while(digitalRead(INT) == 1)
    {
      delay(1);
      time_out++;
      if (time_out > 1000)
      {
        Serial.println(F("Error: no end of measurement interrupt from Boyle !!!"));
        return;
      }
    }
    
    // pause time is set by the SPS command
    delay(pause_time);
    msb = 0;
    lsb = 0;

    // sending the two byte batch_id before cycling through the ASICs.
    //received_string[0] = (batch_id & 0xFF00) >> 8;
    //received_string[1] = (batch_id & 0x00FF);
    //Serial.write(received_string[0]);
    //Serial.write(received_string[1]);

    //Serial.print(received_string[0], HEX);
    //Serial.print(received_string[1], HEX);
    
    // updating the results of the reference sensors in the output buffer
    // Read 2 bytes of data: humidity msb, humidity lsb
    
    sensors_data[0] = (batch_id & 0xFF00) >> 8;
    sensors_data[1] = (batch_id & 0x00FF);
    // incrementing batch_id, rolling over is expected. range of 65535 is more than enough to check for dropped batches on the host.
    batch_id++;
    //memcpy(sensors_data, (uint8_t *)batch_id, 2);
    // read result and transmit it to PC
    {
        // connect SDA line
        msb = 1<<j;
        lsb = msb & 0xFF;
        msb = msb & 0xFF00;
        msb = msb >> 8;
    
        //Wire.endTransmission();         // stop transmitting
        Wire.beginTransmission(i2c_adr);  // transmit to device address
        Wire.write(0);                    // sends start for block read
        i2c_status = Wire.endTransmission();     // stop transmitting

        if(i2c_status == 0) // slave sent AKN
        {
          Wire.requestFrom(i2c_adr, 20);   // read 20 bytes
          i=0;
          while(Wire.available())    // slave may send less than requested
          { 
            received_string[i++]  = Wire.read();    // receive a byte as character
          }

        }
        else {
          for(i=0;i<20;i++)
          received_string[i] = 0xFF;
        }

        for (i=0;i<20;i++)
        {
          //Serial.write(received_string[i]); // echo back data
          //Serial.print(received_string[i],HEX);
          sensors_data[2+i] = received_string[i];
        }
    }

    if(++measure_reference_sensors == SENSOR_POLL_INTERVAL)
    {
      read_ref_sensors = true;
      measure_reference_sensors = 0;
     // Serial.println("read ef ");
    }
   // Serial.print(measure_reference_sensors);
   // Serial.println();
    
   // 16x24 bytes = 384 bytes =  6 x 64 bytes 
   // -> send ONE termination data packet (also 64 bytes)
   //Serial.println("this_is_the_end_of_one_valid_data_packet___________________*\r\n");
   //    Serial.print("%s", received_string);
   // Serial.println(F("*\r\n"));

  }
  else
  {
    first_time = true;
  }
   
}

char Boyle::r_reg(char adr,char reg, bool transmitt_data_via_usb)
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
        Serial.print(c);         // print the character before commented niels
        Serial.print(" ");         // print the character before commented niels 
      }
   }

   //if(transmitt_data_via_usb==1)
   //   Serial.println("*"); // echo back data

   return c;
}
void Boyle::read_rref_settings()
{
    int i,j;
    unsigned int msb,lsb;
    uint8_t rref_settings[5]={0};
    
    char number_of_ASICs = 1;
    if(demo_board_with_eight_channels == 1) number_of_ASICs = 8;
   
    for(j=0;j<number_of_ASICs;j++)
    {
        // connect SDA line

        msb = 1<<j;
        lsb = msb & 0xFF;
        msb = msb & 0xFF00;
        msb = msb >> 8;
        
        //multiplex_to_chip(msb,lsb);
    
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
            
        } else 
        {
          for(i=0;i<5;i++) received_string[i]=0xBB; // -> part is broken or missing -> this will cause a fail in the autostatus summary report
        }

        
        for (i=0;i<5;i++)
        {
          //Serial.print(received_string[i]); // echo back data
          rref_settings[i] = (uint8_t)received_string[i];
        }
        CommTxDataBoyleRef(rref_settings);

        //delay(500); for debugging to see the multiplexing LEDs stepping
    
    }
   
   // Serial.println("*");

}

void Boyle::measure_pid_t_offset(void)
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

  char number_of_ASICs = 1;
  if(demo_board_with_eight_channels == 1) number_of_ASICs = 8;

  //////////////////////////////////////
  // READ temperature from reference sensor
  //////////////////////////////////////
  //sht25_temperature(sht25_bytes);
  //float reference_temperature = (((sht25_bytes[0] * 256.0 + sht25_bytes[1]) * 175.72) / 65536.0) - 46.85;
  
  // connect SDA lines from ASIC #1 in
  // multiplex_to_chip(0x01,0x01);

  // read out register 0x21 to restore its value after autoscaling
  // read status of reg 0x21, echo = 0
  char reg21 = r_reg(i2c_adr, 0x21, 0);
 
  ////////////////////////////////////////////////////////////
  // connect all SDA lines from all 16 ASICs in parallel
  ////////////////////////////////////////////////////////////
  
  //multiplex_to_chip(0xFF,0xFF);

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

  for(i=0;i<1;i++) // step through all 16 ASICs
  {
    msb = 0;
    lsb = 0;
    // connect SDA line of single ASCIS
    msb = 1<<i;
    lsb = msb & 0xFF;
    msb = msb & 0xFF00;
    msb = msb >> 8;
    //multiplex_to_chip(msb,lsb);
      
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
              delay(1);
              time_out++;
              if (time_out > 1000)
              {
                Serial.println("Error: no int during PID_T_OFFSET !!!");
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
            //Serial.print(rtemp1); // send termination string    
            //Serial.print(" ");
          } else { // RTEMP no detected -> set duty cycle to minimum
            i2c_write_reg_data(i2c_adr,0x2E,0); // setting the PID_T_OFFSET to zero is setting the loop duty cycle to 1%
            i2c_write_reg_data(i2c_adr,0x2F,0);
            //Serial.print("x "); // send termination string  
          }
         
      }
      else
        init_fail =true;
        //Serial.print(F("no_int ")); // send termination string   
      
     
   }
   else 
    no_ack =true;
   //Serial.print(F("no_ackn ")); // send termination string
     
     
  }

   // connect all SDA lines from all 16 ASICs in parallel
   // multiplex_to_chip(0xFF,0xFF);

  // restore the content of register 0x21
  i2c_write_reg_data(i2c_adr,0x21,reg21);

  //Serial.print(rtemp_t_offset_calib_temp);
  //Serial.print(" ");
    
  //Serial.println("*"); // send termination string
  
}
void  Boyle::autoscale(double external_rref)
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
  unsigned int rref_ext[12]={0}; 

  bool trigger;

  char number_of_ASICs = 1;
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
   // read out register 0x21 to restore its value after autoscaling
   // read status of reg 0x21, echo = 0
   char reg21 = r_reg(i2c_adr, 0x21, 0);
  
   // enable "measurement_finished" interrupt
   i2c_write_reg_data(i2c_adr,0x1A,0x02);
  
   // clear old interrupt
   i2c_write_reg_data(i2c_adr,0x1C,0x02);
   // enable all input channels (no ASIC temp measurement)
   i2c_write_reg_data(i2c_adr,0x21,0x3F);
 
  for(i=0;i<1;i++) // step through all 16 ASICs
  {
    // connect SDA line of single ASCIS
    msb = 1<<i;
    lsb = msb & 0xFF;
    msb = msb & 0xFF00;
    msb = msb >> 8;
    //multiplex_to_chip(msb,lsb);

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
            if(demo_board_with_eight_channels == 1)
            {
              delay(30);
            }
            else
            {
              while(digitalRead(INT) == 1)
              {
                  delay(1);
                  time_out++;
                  if (time_out > 1000)
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
                  rref_ext[j] = ((data_buffer[13] << 8) + data_buffer[12] ) - 32768 ; 
      
                  // check if ADC results are lower than 26000 LSBs (80% of ADC fullscale)
                  // and if selected internal rref is 20% higher than external used rref
                  if (rtemp1 > 26000 || internal_rref[rref_rtemp1_index] < min_rref_value ) rref_rtemp1_index++;
                  if (rsens1 > 26000 || internal_rref[rref_rsens1_index] < min_rref_value ) rref_rsens1_index++;
                  if (rsens2 > 26000 || internal_rref[rref_rsens2_index] < min_rref_value ) rref_rsens2_index++;
                  if (rsens3 > 26000 || internal_rref[rref_rsens3_index] < min_rref_value ) rref_rsens3_index++;
                  if (rsens4 > 26000 || internal_rref[rref_rsens4_index] < min_rref_value ) rref_rsens4_index++;
                  if (rtemp2 > 26000 || internal_rref[rref_rtemp2_index] < min_rref_value ) rref_rtemp2_index++;
    
                
                  if (rref_ext[j] < 30000 && trigger == 0) // save the best reference value for REXT
                  { 
                    best_rext_reference_index = rref_ext_index;
                    trigger = 1;
                  }
                  
                  rref_ext_index++;
          
                  // clear old interrupt
                  i2c_write_reg_data(i2c_adr,0x1C,0x02);
          
               }  else 
               {
                    rref_ext[j] = 0; 
               }
        } 
        else 
        {
          Serial.print("I2C error ");
          rref_ext[j] = 0;
        }

        //Serial.print(rref_ext);
        //Serial.print(" ");

        i2c_write_reg_data(i2c_adr,0x40,best_rext_reference_index);
       
     }
  }

  // connect all SDA lines from all 16 ASICs in parallel
  // multiplex_to_chip(0xFF,0xFF);

  // restore the content of register 0x21
  i2c_write_reg_data(i2c_adr,0x21,reg21);
  CommTxDataBoyleAutoScale(rref_ext);
  //Serial.println("*"); // send termination string
   
}
bool Boyle::SetVal(char *parameter, char *value)
{
  long adr;
  unsigned int reg,data;
  if(value != NULL)
    sscanf(value, "%x %x %x", &adr, &reg, &data);

  /*
  Serial.print(parameter); // echo back data
  Serial.print("\t"); // echo back data
  Serial.print(value); // echo back data
  Serial.print("\t"); // echo back data
  Serial.print(adr); // echo back data
  Serial.print("\t"); // echo back data
  Serial.print(reg); // echo back data
  Serial.print("\t"); // echo back data
  Serial.println(data); // echo back data
  */
  //Serial.println(value);
  if(!strcmp(parameter, "STOP*"))
  {
    stop=1;
    return true;
  }
  if(!strcmp(parameter, "START*"))
  {
    stop=0;
    return true;
  }
  
  if(!strcmp(parameter,"autoscale"))
  {
    autoscale(adr);
    return true;
  }
  if(!strcmp(parameter,"measure_pid_t_offset"))
  {
    measure_pid_t_offset(); 
    return true;
  }
  if(!strcmp(parameter,"read_rref_settings"))
  {
    read_rref_settings();
    return true;
  }
  if(!strcmp(parameter,"con"))
  {
    // split 16bit into 2 byte;
    reg = adr& 0xFF;
    adr = adr& 0xFF00;
    adr = adr >> 8;
    //multiplex_to_chip(adr,reg);
    return true;
    /*
    if(enable_echo==1){
      Serial.print(F("connect_to ")); // echo back data
      Serial.print(adr); // echo back data
      Serial.print(" "); // echo back data
      Serial.print(reg); // echo back data
      Serial.println("*"); // echo back data
      }*/
  }
  if(!strcmp(parameter,"SPS"))
  {
    pause_time = adr;
    /*Serial.flush();
    if(enable_echo==1){
      Serial.print(cmd);
      Serial.print(pause_time);
      Serial.println("*");
    }*/
    return true;
  }
  
  if(!strcmp(parameter,"SW"))
  {
    digitalWrite(3, 0);
    digitalWrite(8, 0);
    if(adr == 1) 
    { 
      digitalWrite(3, 1);
      i2c_write_reg(75,reg);
    }
    if(adr == 2){ 
      digitalWrite(8, 1);
      i2c_write_reg(75,reg);
    }
   if(adr == 3){ 
      i2c_write_reg(73,reg);
    }
    if(adr == 4){
      digitalWrite(3, 1);
      digitalWrite(8, 1);
      i2c_write_reg(72,reg);
    }
    if(adr == 5){
      i2c_write_reg(74,reg);
    }
    return true;
    /*
    if(enable_echo==1){
    Serial.println(F("SW")); // echo back data
    Serial.println(adr); // echo back data
    Serial.println(reg); // echo back data
    Serial.println("*"); // echo back data
    }*/
  }
  if(!strcmp(parameter,"GBR"))
  {
    // clear buffer with "0xFF"
    for(i=0;i<128;i++) buf[i] = 0xFF;
    i=0;
    
    Wire.beginTransmission(i2c_adr); // transmit to device #44 (0x2c)
    Wire.write(reg);                 // sends instruction byte
    i2c_status = Wire.endTransmission();     // stop transmitting

    if(i2c_status == 0){ // go on if slave did aknowledge 
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
        return true;
     }
     else
     {
       return false;
        //Serial.println("I2C error, no AKN !!!");
     }
    for (i=0;i<128;i++)
    {
      // send json message instead of directing serial writing
     // Serial.write(buf[i]); // echo back data
    }
    //Serial.println("*"); // echo back data
    return true;
  }
  if(!strcmp(parameter,"GBRA"))
  {
    // clear buffer with "0xFF"
    for(i=0;i<128;i++) buf[i] = 0xFF;
    i=0;

    Wire.beginTransmission(i2c_adr); // transmit to device #44 (0x2c)
    Wire.write(reg);            // sends instruction byte
    i2c_status = Wire.endTransmission();     // stop transmitting
    if(i2c_status == 0)
    { 
      // go on if slave did aknowledge 
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
      return true;
    } 
    else 
    {
      return false;
      //Serial.println("I2C error, no AKN !!!");
    }

    
    for (i=0;i<128;i++)
    {
      Serial.print("reg ");
      Serial.print(i);
      Serial.print(" = ");
      Serial.println(buf[i]);
    }
    Serial.println("*"); // echo back data
    return true;
  }
  if(!strcmp(parameter,"adc_st*"))
  {
    adc_st();
    return true;
  }

  if(!strcmp(parameter,"adc_st_16x"))
  {
    //adc_st_16x();
    return true;
  }
  if(!strcmp(parameter,"r_reg"))
  {
     r_reg(adr,reg,1);
     return true;
  }
  if(!strcmp(parameter,"r_reg_sw"))
  {
    Wire.requestFrom(adr, 1);
    while(Wire.available())    // slave may send less than requested
    {
      int c = Wire.read();    // receive a byte as character
      Serial.println(c);         // print the character
    }
    Serial.println("*"); // echo back data 
    return true;
  }
  if(!strcmp(parameter,"w_reg"))
  {
    i2c_write_reg(adr,reg);
    /*if(enable_echo==1){
      Serial.println(cmd); // echo back data
      Serial.println(adr); // echo back data
      Serial.println(reg); // echo back data
      Serial.println("*"); // echo back data  
    }*/
    return true;
  }
  if(!strcmp(parameter,"all_heaters_on"))
  {
      heater_enable_in_16x_measurement = 1;
      heater = 2;
      heater_temp = heater_T_high;
      //Serial.println(cmd); // echo back data
      //Serial.println("*"); // echo back data
      return true;
  }

  if(!strcmp(parameter,"all_heaters_off"))
  {
      heater_enable_in_16x_measurement = 0;
      heater = 0;
      //Serial.println(cmd); // echo back data
      //Serial.println("*"); // echo back da
      return true;
  }
  if(!strcmp(parameter, "heater_t_on"))
  {
   // Serial.println(adr); 
    //heater_t_on = strtol(value, NULL, 10);
    heater_t_on = adr;
    timer_100ms = 0;
    return true;
  }
  if(!strcmp(parameter,"heater_T_high"))
  {
    //Serial.println(heater_T_high); 
    //Serial.println(adr); 
    heater_T_high = (adr - rtemp_t_offset_calib_temp)/2; // divided by 2 because units of 2°
    //heater_T_high = ((strtol(value, NULL, 10)) - rtemp_t_offset_calib_temp)/2; // divided by 2 because units of 2°
    timer_100ms = 0;
    //Serial.println(heater_T_high); 
    return true;
  }

  if(!strcmp(parameter,"heater_T_low"))
  {
    //Serial.println(heater_T_low); 
    //Serial.println(adr); 
    heater_T_low = (adr - rtemp_t_offset_calib_temp)/2;
    //heater_T_low = ((strtol(value, NULL, 10))  - rtemp_t_offset_calib_temp)/2;
    timer_100ms = 0;
    //Serial.println(heater_T_low); 
    return true;
  }

  if(!strcmp(parameter,"heater_t_off"))
  {
      heater_t_off = adr;
      timer_100ms = 0;
      return true;
  }
  if(!strcmp(parameter,"heater_toggle_mode"))
  {
      heater_toggle_mode = adr;
      timer_100ms = 0;
      return true;
  }

  if(!strcmp(parameter,"clean_on"))
  {
      //Serial.println(adr); 
      clean_pulse_on_time = adr;
      //Serial.println(clean_pulse_on_time); 
      return true;
  }
  if(!strcmp(parameter,"clean_temp"))
  {
     // Serial.println(clean_pulse_temp);
      //Serial.println(rtemp_t_offset_calib_temp);
//Serial.println(adr); 
      clean_pulse_temp = (adr - rtemp_t_offset_calib_temp)/2;
     // Serial.println(clean_pulse_temp); 
      return true;
  }

  if(!strcmp(parameter,"clean_off"))
  {
      clean_pulse_off_time = adr;
      return true;
  }
  if(!strcmp(parameter,"clean_shift"))
  {
      clean_pulse_phase_shift = adr;
      return true;
  }

  if(!strcmp(parameter,"ref_sens_interval"))
  {
      ref_sens_interval = adr;
      return true;
  }
  if(!strcmp(parameter,"w_reg_data"))
  {
    i2c_write_reg_data(i2c_adr,reg,data); 
    /*if(enable_echo==1){
      
        Serial.println(cmd); // echo back data
        Serial.println(adr); // echo back data
        Serial.println(reg); // echo back data
        Serial.println(data); // echo back data
        Serial.println("*"); // echo back data
    }*/
    return true;
  }
  if(!strcmp(parameter,"wait"))
  {
    wait(adr, 1);
    return true;
  }
  if(!strcmp(parameter,"E3")) // set VDD
  {
    set_vdd(adr, reg, 1);
    return true;
  }
  if(!strcmp(parameter,"reference_sensors_enabled")) // enable readout of reference sensors
  {
    reference_sensors_enabled = 1;
    //Serial.print(F("reference_sensors_enabled"));
    //Serial.println("*"); // acknowledge command with echo
    return true;     
  }
  
  if(!strcmp(parameter,"reference_sensors_disabled")) // disable readout of reference sensors
  {
    reference_sensors_enabled = 0;  
    //Serial.print(F("reference_sensors_disabled"));
    //Serial.println("*"); // acknowledge command with echo  
    return true;     
  }
  if(!strcmp(parameter,"init_asic")) // init ASIC, do autoscaling and measure_PID_T_offset
  {
    asic_init();  
    //Serial.print(cmd);
    //Serial.println("*"); // acknowledge command with echo   
    return true;
  }
  if(!strcmp(parameter,"E6")) // programming voltage
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
      /*if(enable_echo==1){
     Serial.println("E6*"); // echo back data
      }*/
      return true;
    }
  if(!strcmp(parameter,"read_adc_od"))
  {
    //read_adc(1);
    return true;
  }
  return false;
}

void Boyle::adc_st()
{
	char c[] = "";
	int t;
	long time_out = 0;
  
	// start "fake-selftimed-mode"
	// timing will be done by Arduino waits
	// it has to be set with the "SPS" command -> samples per second
	// interrupt pin will be used to check for "measurement finished"

	// enable interrupt
	i2c_write_reg_data(i2c_adr, 0x1A, 0x02);

	// clear old interrupt
	i2c_write_reg_data(i2c_adr, 0x1C, 0x02);

	stop = 0;
 
	while (1)
	{
		if (stop == 1) break;
    
		// read status of reg 0x18
		char reg18 = r_reg(i2c_adr, 0x18, 0);

		// clear all interrupts
	   i2c_write_reg_data(i2c_adr, 0x1C, 0x07);
 
		// start measurement by setting MEASURE_EN
		i2c_write_reg_data(i2c_adr, 0x18, 0x04 | reg18);  //niels
		//i2c_write_reg_data(i2c_adr, 0x18, 0x04);

		// wait till interrupt pin is pulled low
		// and check if stop command was sentread_rref_settings
		time_out = 0;
		while (digitalRead(INT) == 1)
		{
			delay(1);  // add
			time_out++;
			if (time_out > 1000) // 1000 instead of 100000
				{
					Serial.println(F("Error: no end of measurement interrupt from Boyle !!!"));
					return;
				}
		}
		time_out = 0;

		if (Serial.available() > 0) {
			CommRx();
		}

		// pause time is set by the SPS command
		delay(pause_time);
   
		// read RS1 result and transmit it to PC
		// read command is sending back data

		//Wire.endTransmission();     // stop transmitting
		Wire.beginTransmission(i2c_adr);  // transmit to device address
		Wire.write(0);                // sends start for block read
		Wire.endTransmission();      // stop transmitting
        
		Wire.requestFrom(i2c_adr, 20);    // read 20 bytes
		i = 0;
		while (Wire.available())    // slave may send less than requested
			{ 
				received_string[i++]  = Wire.read();     // receive a byte as character
			}

		for (i = 0; i < 20; i++)
		{
			Serial.write(received_string[i]);  // echo back data
		}
		Serial.println(F("*\r\n"));
	}
   
}
float Boyle::convert_ASIC_temp_raw_data_to_celsius(float raw_data)
{          
  float PDMpercent = raw_data/4194304.0;
  float mu = PDMpercent / ( 1.0 + 2.26546 * PDMpercent);
  float temp = 1366.276 * mu - 290.74;

  return temp;
}
bool Boyle::asic_init() 
{
  // connect all SDA lines from all 16 ASICs in parallel
  // multiplex_to_chip(0xFF,0xFF);
  uint8_t buff[5]={0};
  
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
  if(no_ack || init_fail)
    return false;

  i2c_write_reg_data(i2c_adr,0x21,0x7F);

  heater_enable_in_16x_measurement = 1;
  heater = 2;
  heater_temp = heater_T_high;

  return true;
  
}
