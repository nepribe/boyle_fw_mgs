#ifndef __BOYLE_H__
#define __BOYLE_H__

#include <Arduino.h>

class Boyle
{
 public:
  bool asic_init();
  void asic_poll(uint8_t *sensors_data);
  void set_ref_sensor_readout_ongoing();
  void clear_ref_sensor_readout_ongoing();
  bool read_ref_sensor();
  void clear_read_ref_sensor();
  void timerEvent();
  void read_rref_settings();
  bool SetVal(char *parameter, char *value);
  Boyle()
  :heater_stat(0),
   cleaning_stat(0),
   phase_shift_done(0),
   stop(0),
   pause_time(10),
   heater(0),
   heater_t_off(300), // 20 2s off 580 for 58 sec
   heater_t_on(300), // 10 1s on
   heater_T_high(130), //org 50
   heater_T_low(50),  //org 25
   heater_temp(0),
   i2c_status(0),
   i2c_adr(0x35),
   timer_clean_pulse_100ms(0),
   timer_100ms(0),
   demo_board_with_eight_channels(0),
   rtemp_t_offset_calib_temp(25),
   timer_delay_for_cleaning(0),
   heater_enable_in_16x_measurement(0),
   heater_toggle_mode(2),
   clean_pulse_off_time(0),
   clean_pulse_phase_shift(0),
   i(0),
   ref_sens_interval(100),
   ref_sensor_readout_ongoing(0),
   reference_sensors_enabled(1),
   clean_pulse_on_time(0),
   clean_pulse_temp(200),
   read_ref_sensors(true),
   no_ack(false),
   init_fail(false),
   programming_voltage_msb(6),
   programming_voltage_lsb(246)
   {}
  private:
  void autoscale(double external_rref);
  void measure_pid_t_offset(void);
  byte i2c_write_reg_data(int adr, int reg, int data);
  char r_reg(char adr,char reg, bool transmitt_data_via_usb);
  float convert_ASIC_temp_raw_data_to_celsius(float raw_data);
  void i2c_write_reg(int adr, int reg);
  void wait(int adr, int echo);
  void set_vdd(int adr, int reg, int echo);
  void adc_st();
  void adc_st_16x();
  bool heater_stat;
  bool cleaning_stat;
  bool phase_shift_done;
  bool stop;
  int pause_time;
  char heater;
  int heater_t_off; // 2s off
  int heater_t_on; // 1s on
  int heater_T_high;
  int heater_T_low;
  byte heater_temp;
  byte i2c_status;
  char i2c_adr;
  int timer_clean_pulse_100ms;
  unsigned int timer_100ms;
  bool demo_board_with_eight_channels;
  float rtemp_t_offset_calib_temp;
  unsigned int timer_delay_for_cleaning;
  bool heater_enable_in_16x_measurement;
  byte heater_toggle_mode;
  unsigned long clean_pulse_off_time;
  unsigned long clean_pulse_phase_shift;
  char received_string[200];
  uint8_t data_buff[100];
  int i=0;
  unsigned int ref_sens_interval;
  bool ref_sensor_readout_ongoing;
  bool reference_sensors_enabled;
  int clean_pulse_on_time;
  int clean_pulse_temp;
  bool read_ref_sensors;
  bool no_ack;
  bool init_fail;
  double vdd_mv;
  volatile byte buf[130];
  int programming_voltage_msb;
int programming_voltage_lsb;
};

#endif
