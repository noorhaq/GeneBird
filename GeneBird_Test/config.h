#ifndef CONFIG_H
#define CONFIG_H

//==================================================
//********************PIN Setup
//==================================================
#define  Motor1 15
#define  Motor2 14
#define  Motor3 12
#define  Motor4 13
#define  LED    16
#define  SCL    5
#define  SDA    4
#define  sigPin 2 //For Testing or setting PPM for other countrollers
#define  PPM_RX_INPUT 11


#define  Battery_Connection A0 //But keep the voltage divider range from 0 to 1v
#define  TOTAL_RX_CHANNEL_NUMBER_PWM 6
#define  TOTAL_RX_CHANNEL_NUMBER_PPM 6
//==================================================
//********************Channel Receiving
//==================================================
#define CPU_MHZ 16
#define CHANNEL_NUMBER 6  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1100  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative

#define  DEBUGPIN  4


//WIFI Configuration
const char *ssid = "WifiPPM";
const char *password = "";


// ===========================================
// ==============
// Throttle Maximum and Minimum
// ===========================================

#define Min_throttle 1100
#define Max_throttle 1800   // Some leaway from Max 2000 for PID tuning

// ===========================================
// =============
// I2C Speed
// ===========================================
//#define I2C_Speed 100000 //100kHz for original WMP
//#define I2C_Speed 400000 //400kHz fast mode, for clones
#define Gyro_Calib_offset 2000     //Default is 2000 but can be increased or decreased to calculate the offset. One thing to notice here is after the offset is calibrated the Gyro values should be zero


//================================================
//**********************PID Controller
//================================================

float pid_p_gain_roll = 1.3;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.04;              //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 18.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).


//=================================================
//*****************************IMU Settings
//=================================================

uint8_t gyro_address = 0x68; //For MPU 6050

float low_battery_warn = 9.5; //Can be set for any value


//=========================================================
//====================
//====================
//=========================================================
uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t check_byte, flip32, start;
uint8_t error, error_counter, error_led;
uint8_t flight_mode, flight_mode_counter, flight_mode_led;
uint8_t takeoff_detected, manual_altitude_change;
uint8_t telemetry_send_byte, telemetry_bit_counter, telemetry_loop_counter;
uint8_t channel_select_counter;
uint8_t level_calibration_on;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t manual_throttle;
int16_t throttle, takeoff_throttle, cal_int;
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;
int16_t acc_axis[4], gyro_axis[4];

int32_t channel_1_start, channel_1, pid_roll_setpoint_base;
int32_t channel_2_start, channel_2, pid_pitch_setpoint_base;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

int32_t acc_alt_integrated;

uint32_t loop_timer, error_timer, flight_mode_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float battery_voltage, dummy_float;


#endif
