//***************************************
//  WiFi Controlled QuadCopter Component Test Interface
//  ESP8266
// By yours truly Muhammad Noor ul Haq
// Pakistan
//
//**************************************
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Hash.h>
#include "MSP.h"
#include <EEPROM.h>
#include "config.h"
#include <Arduino.h>

char Menu_Input;
volatile unsigned long next;
volatile unsigned int ppm_running=1;
int8_t EEPROM_READ[11]; // To read EEPROM data

int32_t angle_roll_acc_manual_offset , angle_pitch_acc_manual_offset;
//=================================================
//********************Variables
//=================================================
int ppm[CHANNEL_NUMBER];
byte lowByte, highByte, type, clockspeed_ok;
byte roll_axis, pitch_axis, yaw_axis;
byte receiver_check_byte, gyro_check_byte;
int address;
unsigned long timer, timer_1, timer_2, timer_3, timer_4, current_time;

const byte captive_portal=0;
const byte DNS_PORT = 53;
const char* serverIndex = "<form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";

extern const char index_html[];

unsigned int alivecount=0;

MSP msp;

IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;
//ESP8266WebServer server = ESP8266WebServer(80);
ESP8266WebServer server (80);
WebSocketsServer webSocket = WebSocketsServer(81);

void inline ppmISR(void){
  static boolean state = true;

  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    next = next + (PULSE_LENGTH * CPU_MHZ);
    state = false;
    alivecount++;
  } 
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      next = next + ((FRAME_LENGTH - calc_rest) * CPU_MHZ);
      calc_rest = 0;
      digitalWrite(DEBUGPIN, !digitalRead(DEBUGPIN));
    }
    else{
      next = next + ((ppm[cur_chan_numb] - PULSE_LENGTH) * CPU_MHZ);
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
  timer0_write(next);
}

void handleRoot() {
   if(ppm_running==0)
  {
    noInterrupts();
    timer0_isr_init();
    timer0_attachInterrupt(ppmISR);
    next=ESP.getCycleCount()+1000;
    timer0_write(next);
    for(int i=0; i<CHANNEL_NUMBER; i++){
      ppm[i]= CHANNEL_DEFAULT_VALUE;
    }
    ppm_running=1;
    interrupts();
  }
  server.send_P(200,"text/html", index_html);
 }
 
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            ppm[0]=1100; ppm[1]=1500; ppm[2]=1500; ppm[3]=1500;
            ppm[4]=1100; ppm[5]=1100; ppm[6]=1100; ppm[7]=1100;
             break;
        case WStype_CONNECTED: {
            // send message to client
            webSocket.sendTXT(num, "Connected");
            ppm[0]=1100; ppm[1]=1500; ppm[2]=1500; ppm[3]=1500;
            ppm[4]=1100; ppm[5]=1100; ppm[6]=1100; ppm[7]=1100;
        }
            break;
        case WStype_TEXT: {
            if(payload[0]=='g' && payload[1]=='e' && payload[2]=='t'){
/*              msp_analog_t analogdata;
              String senddata="{\"vbat\": ";
              if (msp.request(MSP_ANALOG, &analogdata, sizeof(analogdata))) {
                senddata+=String(analogdata.vbat);
              }
              else
                senddata+="0";
              senddata += "}";
              webSocket.sendTXT(num, senddata);
*/            }
        }
        break;
        case WStype_BIN: {
          ppm[payload[0]]=(payload[1]<<8)+payload[2];
          alivecount=0;
        }
        break;
    }

}

void setup() {
  Serial.begin(57600); //Serial Monitor Display

  pinMode(Motor1,OUTPUT); //Initialing PWM output for Motor Control Pins 
  pinMode(Motor2,OUTPUT); //Initialing PWM output for Motor Control Pins
  pinMode(Motor3,OUTPUT); //Initialing PWM output for Motor Control Pins
  pinMode(Motor4,OUTPUT); //Initialing PWM output for Motor Control Pins

  analogWrite(Motor1,0); //Giving a 0 signal to stop all motors
  analogWrite(Motor2,0); //Giving a 0 signal to stop all motors
  analogWrite(Motor3,0); //Giving a 0 signal to stop all motors
  analogWrite(Motor4,0); //Giving a 0 signal to stop all motors

 // pinMode(Battery_Connection, INPUT_ANALOG); // Initializing Battery Connection Pin for Voltage Measurement
 analogRead(Battery_Connection);
  pinMode(LED, OUTPUT); //Initialing LED 

  Wire.begin(SDA,SCL);
  Wire.setClock (I2C_Speed);          //I2C Initiliazination
    EEPROM_READ_DATA();
  delay(250);


//  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));



  WiFi.softAP(ssid,password,2);

  IPAddress myIP = WiFi.softAPIP();

  if(captive_portal)
    dnsServer.start(DNS_PORT, "*", apIP);
  server.onNotFound(handleRoot);
  server.on("/", handleRoot);

  server.on("/update", HTTP_GET, [](){

    noInterrupts();
    timer0_detachInterrupt();
    ppm_running=0;
    interrupts();
    delay(500);    
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/html", serverIndex);
  });

  server.on("/upload", HTTP_POST, [](){
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
    ESP.restart();
  },[](){
      HTTPUpload& upload = server.upload();
      if(upload.status == UPLOAD_FILE_START){
        WiFiUDP::stopAll();
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if(!Update.begin(maxSketchSpace)){//start with max available size
 //         Update.printError(Serial);
        }
      } 
      else if(upload.status == UPLOAD_FILE_WRITE){
        if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
//          Update.printError(Serial);
        }
      } 
      else if(upload.status == UPLOAD_FILE_END){
        if(Update.end(true)){ //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
//        Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
      }
      yield();
    });
 
  server.begin();
  webSocket.onEvent(webSocketEvent);
  webSocket.begin();

  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(ppmISR);
  next=ESP.getCycleCount()+1000;
  timer0_write(next);
  for(int i=0; i<CHANNEL_NUMBER; i++){
    ppm[i]= CHANNEL_DEFAULT_VALUE;
  }
  interrupts();
  Serial.begin(115200);
  msp.begin(Serial);
}

unsigned long time_now = 0;


//**************************************************************
//**********************
//**************************************************************

void loop() {
Serial.println("Welcome User");
  
  #if F_CPU == 16000000L          //If the clock speed is 16MHz include the next code line when compiling
    clockspeed_ok = 1;            //Set clockspeed_ok to 1
  #endif                          //End of if statement
//==============================================================
  webSocket.loop();
//  ArduinoOTA.handle();
  if(captive_portal)
    dnsServer.processNextRequest();
  server.handleClient();
    // Serial.println("R %d RP %d T %d Y %d L1 %d L2 %d", (int)ppm[0], (int)ppm[1],  (int)ppm[2],  (int)ppm[3],  (int)ppm[4],  (int)ppm[5]); 
  if(alivecount>1000){
    for(int i=0; i<4;i++){
      ppm[i]=Min_throttle;
    }
    for(int i=4; i<8;i++){
      ppm[i]=1000;
    }
  }
    
    Menu();

  yield();
}


void start_gyro(){
  //Setup the L3G4200D or L3GD20H
  //Setup the MPU-6050
  if(type == 1){
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //PWR_MGMT_1 register
    Wire.write(0x00);                                            //Set to zero to turn on the gyro
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x1B);                                            //GYRO_CONFIG register
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
}

void gyro_signalen(){
  if(type == 1){
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x43);                                            //Start reading @ register 43h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address,6);                                 //Request 6 bytes from the gyro
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    gyro_roll=Wire.read()<<8|Wire.read();                        //Read high and low part of the angular data
    if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
    gyro_pitch=Wire.read()<<8|Wire.read();                       //Read high and low part of the angular data
    if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
    gyro_yaw=Wire.read()<<8|Wire.read();                         //Read high and low part of the angular data
    if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration
  }
}

void check_gyro_axes(byte movement){
  byte trigger_axis = 0;
  float gyro_angle_roll, gyro_angle_pitch, gyro_angle_yaw;
  //Reset all axes
  gyro_angle_roll = 0;
  gyro_angle_pitch = 0;
  gyro_angle_yaw = 0;
  gyro_signalen();
  timer = millis() + 10000;    
  while(timer > millis() && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_signalen();
    if(type == 2 || type == 3){
      gyro_angle_roll += gyro_roll * 0.00007;              //0.00007 = 17.5 (md/s) / 250(Hz)
      gyro_angle_pitch += gyro_pitch * 0.00007;
      gyro_angle_yaw += gyro_yaw * 0.00007;
    }
    if(type == 1){
      gyro_angle_roll += gyro_roll * 0.0000611;          // 0.0000611 = 1 / 65.5 (LSB degr/s) / 250(Hz)
      gyro_angle_pitch += gyro_pitch * 0.0000611;
      gyro_angle_yaw += gyro_yaw * 0.0000611;
    }
    
    delayMicroseconds(3700); //Loop is running @ 250Hz. +/-300us is used for communication with the gyro
  }
  //Assign the moved axis to the orresponding function (pitch, roll, yaw)
  if((gyro_angle_roll < -30 || gyro_angle_roll > 30) && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000001;
    if(gyro_angle_roll < 0)trigger_axis = 0b10000001;
    else trigger_axis = 0b00000001;
  }
  if((gyro_angle_pitch < -30 || gyro_angle_pitch > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000010;
    if(gyro_angle_pitch < 0)trigger_axis = 0b10000010;
    else trigger_axis = 0b00000010;
  }
  if((gyro_angle_yaw < -30 || gyro_angle_yaw > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30){
    gyro_check_byte |= 0b00000100;
    if(gyro_angle_yaw < 0)trigger_axis = 0b10000011;
    else trigger_axis = 0b00000011;
  }
  
  if(trigger_axis == 0){
    error = 1;
    Serial.println(F("No angular motion is detected in the last 10 seconds!!! (ERROR 4)"));
  }
  else
  if(movement == 1)roll_axis = trigger_axis;
  if(movement == 2)pitch_axis = trigger_axis;
  if(movement == 3)yaw_axis = trigger_axis;
  
}

void EEPROM_READ_DATA(){
    for(int i = 0; i< 11; i++)
    {
        EEPROM_READ[i] = EEPROM.read(i);
    }
    angle_pitch_acc_manual_offset = EEPROM_READ[0];
    angle_roll_acc_manual_offset = EEPROM_READ[1];
    roll_axis = EEPROM_READ[2];
    pitch_axis = EEPROM_READ[3];
    yaw_axis = EEPROM_READ[4];
    type = EEPROM_READ[5];
    gyro_address = EEPROM_READ[6];
    if((EEPROM_READ[7] != 'N') || (EEPROM_READ[8] != 'O') || (EEPROM_READ[9] != 'O') || (EEPROM_READ[10] != 'R'))
    {
        Serial.println("Please Run Setup Program first.");
        while(1);
    }
}

void Menu(){

    Serial.println("TO check --Channel signals-- Please input 'c' in serial monitor. ");
    Serial.println("TO check --Gyro angles--     Please input 'g' in serial monitor. ");
    Serial.println("TO check --Motor 1--         Please input '1' in serial monitor. ");
    Serial.println("TO check --Motor 2--         Please input '2' in serial monitor. ");
    Serial.println("TO check --Motor 3--         Please input '3' in serial monitor. ");
    Serial.println("TO check --Motor 4--         Please input '4' in serial monitor. ");
    Serial.println("TO       ---quit----         Please input 'q' in serial monitor. ");
    Serial.println("......... ");
    if(Serial.available() >0 ){
        Menu_Input = Serial.read();
        delay(30);
    }
    Check();
}

void Check(){
    switch (Menu_Input)
    {
    case 'c':
       Signals_Check();             //Refers to Wifi Receiving Commands 
        break;
    case 'g':
        Gyro_Check();               //Refers to Gyro Angles
        break;
    case '1':
        Motor_Check(1);
        break;
    case '2':
        Motor_Check(2);
        break;
    case '3':
        Motor_Check(3);
        break;
    case '4':
        Motor_Check(4);
        break;
    case 'q':
        while(1);
        break;

    default:
        Menu();
        break;
    }
}

void Signals_Check(){
      Serial.println(F("System check :-Channels"));
      Serial.println("Please Input q to exit.");
      Serial.println(F("==================================================="));
    do{
    if(Serial.available() >0 ){
        Menu_Input = Serial.read();
        delay(30);
        }
        Serial.print('Roll : ');       Serial.print(ppm[0]);
        Serial.print('\t Pitch : ');   Serial.print(ppm[1] );
        Serial.print('\t Throttle');   Serial.print(ppm[2] );
        Serial.print('\t Yaw');        Serial.print(ppm[3]);
        Serial.print('\t Channel 5');  Serial.print(ppm[4]);
        Serial.print('\t Channel 6');  Serial.print(ppm[5]);
        Serial.print('\t Channel 7');  Serial.print(ppm[6]);
        Serial.print('\t Channel 8');  Serial.println(ppm[7]);
    }while(Menu_Input != 'q');
}

void Gyro_Check(){
    
  Serial.println(F("System check :-Gyro"));
  Serial.println("Please Input q to exit.");
  Serial.println(F("==================================================="));
  delay(100);
        do{
    if(Serial.available() >0 ){
        Menu_Input = Serial.read();
        delay(30);
        }

    gyro_signalen();
     angle_pitch += gyro_pitch * 0.0000611;                                          //Total Angle travelled by Gyro 
      angle_roll += gyro_roll * 0.0000611;                                         //Total Angle travelled by Gyro
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //The Arduino asin function is in radians, so converting to degrees.
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
    
    angle_pitch_acc -= angle_roll_acc_manual_offset ;                                              //Accelerometer calibration value for pitch
    angle_roll_acc -= angle_pitch_acc_manual_offset;                                               //Accelerometer calibration value for roll
  //Using Filter to Dampen the Angle
  //angle_pitch_output = angle_pitch_output * 0.75 + angle_pitch * 0.25;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  //angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
    Serial.print("Pitch: ");  Serial.print(angle_pitch ,0);
    Serial.print(" Roll: ");  Serial.print(angle_roll ,0);
    Serial.print(" Yaw: ");   Serial.println(gyro_yaw / 65.5 ,0);

    }while(Menu_Input != 'q');

}

void Motor_Check(uint8_t motor){
     Serial.println(F("System check :-Motor_Check"));
     Serial.println("Please Input q to exit.");
    Serial.println(F("==================================================="));
    do{
    if(Serial.available() >0 ){
        Menu_Input = Serial.read();
        delay(30);
        }
     manual_throttle = map(manual_throttle,Max_throttle,Min_throttle,0,255);
     switch (motor)
     {
     case 1:
    analogWrite(Motor1,manual_throttle);
         break;
    case 2:
    analogWrite(Motor2,manual_throttle);
         break;
     case 3:
    analogWrite(Motor3,manual_throttle);
         break;
     case 4:
    analogWrite(Motor4,manual_throttle);
         break;
     default:
         break;
     }
    }while(Menu_Input != 'q');

    analogWrite(Motor1,0);
    analogWrite(Motor2,0);
    analogWrite(Motor3,0);
    analogWrite(Motor4,0);

}
