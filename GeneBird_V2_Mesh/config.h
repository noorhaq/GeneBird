
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#include <Wire.h>
//#include <//EEPROM.h>

#include <WebSocketsServer.h>
#include <DNSServer.h>
#include <Hash.h>
#include "MSP.h"
#include <painlessMesh.h>
//Mesh Parameters
const char *ssid = "GeneBird";
const char *password = "GeneBird_123";
#define   MESH_PORT       5555

#define CALSTEPS 256 // gyro and acc calibration steps
#define MPU6050_ADDRESS 0x68
#define SMPLRT_DIV 0
#define DLPF_CFG   4

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  = X; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}

#define ACCRESO 4096
#define CYCLETIME 2500
#define CALITIME 10
#define MINTHROTTLE 1000
#define MIDRUD 1500
#define THRCORR 0

#define ROL 0
#define PIT 1
#define THR 2
#define RUD 3
#define AU1 4
#define AU2 5

#define GYRO     0
#define STABI    1
#define RTH      2

#define GYRO_I_MAX 10000.0
#define ANGLE_I_MAX 6000.0

#define FRONT_L 15
#define FRONT_R 14
#define REAR_R 13
#define REAR_L 12

#define RED_LED 2
#define BLUE_LED 16
#define GREEN_LED 0



//==================================Global Variables==================================
extern int16_t accZero[3];
extern float yawRate;
extern float rollPitchRate;
extern float P_PID;
extern float I_PID;
extern float D_PID;
extern float P_Level_PID;
extern float I_Level_PID;
extern float D_Level_PID;
volatile boolean recv = true;

enum ang { ROLL, PITCH, YAW };
static int16_t gyroADC[3];
static int16_t accADC[3];
static int16_t gyroData[3];
static float anglerad[3]    = {0.0, 0.0, 0.0};
static float angle[3]    = {0.0, 0.0, 0.0};
extern int calibratingA;


static int16_t rcCommand[] = {0, 0, 0};

static int8_t flightmode;
static int8_t oldflightmode;
int armed = false;
uint8_t armct = 0;
int debugvalue = 0;


float yawRate = 6.0;
float rollPitchRate = 5.0;

float P_PID = 0.33;    // P8
float I_PID = 0.003;    // I8
float D_PID = 2.8;    // D8

float P_Level_PID = 0.35;   // P8
float I_Level_PID = 0.003;   // I8
float D_Level_PID = 2.8;   // D8

static int16_t axisPID[3];
static int16_t lastError[3] = {0, 0, 0};
static float errorGyroI[3] = {0, 0, 0};
static float errorAngleI[3] = {0, 0, 0};

//----------PID controller----------
int plotct;
int16_t deltab[6][3];
int8_t  deltabpt = 0;
int32_t deltasum[3];

#define CHANNELS 8
int16_t rcValue[CHANNELS];  // in us, center = 1500
static uint16_t servo[4];
uint32_t rxt; // receive time, used for falisave
void buf_to_rc()
{
  rcValue[0] = 1000;
  rcValue[1] = 1000;
  rcValue[2] = 1000;
  rcValue[3] = 1000;
  rcValue[4] = 1000;
  rcValue[5] = 1000;
  rcValue[6] = 1000;
  rcValue[7] = 1000;
}


  uint32_t now, mnow, diff;
/* Set these to your desired credentials. */

#define CPU_MHZ 80
#define CHANNEL_NUMBER 8  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1100  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative


volatile unsigned long next;
volatile unsigned int ppm_running=1;

int ppm[CHANNEL_NUMBER];

const byte captive_portal=0;
const byte DNS_PORT = 53;
const char* serverIndex = "<form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";

extern const char index_html[];

unsigned int alivecount=0;
