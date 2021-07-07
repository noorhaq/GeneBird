
#include "config.h"


extern "C" {
#include <espnow.h>
}
MSP msp;
Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;
IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;
//ESP8266WebServer server = ESP8266WebServer(80);
ESP8266WebServer server (80);
WebSocketsServer webSocket = WebSocketsServer(81);

/*****************************************************************
 * FUNCTION 
 *    SETUP
 *        Initializes the ESP8266 input functions.
 * 
 ******************************************************************/
void setup() {
  Serial.begin(115200); Serial.println();
 
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
 initServo();
   analogWrite(FRONT_L, 0 );
  analogWrite(FRONT_R, 0);
  analogWrite(REAR_R, 0);
  analogWrite(REAR_L, 0);
  WiFi.softAP(ssid,password);
  delay(3000); // give it some time to stop shaking after battery plugin
   
  MPU6050_init();
  MPU6050_readId(); // must be 0x68, 104dec
  IPAddress myIP = WiFi.softAPIP();
  delay(200);
  Serial.print("\r My IP is ");
  Serial.println(myIP);
  delay(100);
 // //EEPROM.begin(64);
  //if (//EEPROM.read(63) != 0x55)
  {
    Serial.println("Need to do ACC calib");
    for (int i = 0; i < 10; i++)
    {
      digitalWrite(GREEN_LED, HIGH);
      delay(300);
      digitalWrite(GREEN_LED, LOW);
      delay(300);
    }
  }
 // else
  {
    ACC_Read(); // //EEPROM is initialized
  }
 // if (//EEPROM.read(62) != 0xAA)
  {
    Serial.println("Need to check and write PID");
    for (int i = 0; i < 10; i++)
    {
      digitalWrite(GREEN_LED, HIGH);
      delay(300);
      digitalWrite(GREEN_LED, LOW);
      delay(300);
    }
  }
 // else
  {
    PID_Read(); // //EEPROM is initialized
  }
  delay(1000);
  digitalWrite(GREEN_LED, HIGH);
   //=======================OTA===========
  ppm_setup();
 
  Serial.println("OTA ready");
  Mesh_setup();
}


void loop()
{
      _loop();
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
    Mesh_loop();
}
//==============================================================================
bool _loop() {

  ppm_loop();
  
  _check();
_Reciever();

 //
  now = millis(); // actual time
  if (debugvalue == 5) mnow = micros();
  
  {
    if (debugvalue == 4) 
{
  Serial.printf("\r%4d %4d %4d %4d \n", rcValue[0], rcValue[1], rcValue[2], rcValue[3]);
}
    if      (rcValue[AU1] < 1500) flightmode = GYRO;
    else flightmode = STABI;
    if (oldflightmode != flightmode)
    {
      zeroGyroAccI();
      oldflightmode = flightmode;
    }

    if (armed > 1500)
    {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BLUE_LED, HIGH);
      rcValue[THR]    -= THRCORR;
      rcCommand[ROLL]  = rcValue[ROL] - MIDRUD;
      rcCommand[PITCH] = rcValue[PIT] - MIDRUD;
      rcCommand[YAW]   = rcValue[RUD] - MIDRUD;
      Serial.println("Armed");
    }
    else
    {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      rcValue[THR] = 1000;
      
    }

    rxt = millis();
  }

  Gyro_getADC();
  ACC_getADC();
  getEstimatedAttitude();
  pid();
  mix();
  if (debugvalue != 6)
  {
    writeServo();
  }
  //=============================================================================================//
  // Failsave part
  if (now > rxt + 90)
  {
    rcValue[THR] = MINTHROTTLE;
    if (debugvalue == 5) Serial.printf("\rRC Failsafe after %d \n", now - rxt);
    rxt = now;
  }
  //===========================================DEBUG============================================//
  // parser part
 _Debug();

  while ( micros() - mnow < CYCLETIME)
  {
    
  }
  return false;
}
