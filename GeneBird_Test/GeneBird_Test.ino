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
#include <ArduinoOTA.h>

char Menu_Input;
volatile unsigned long next;
volatile unsigned int ppm_running=1;
//int8_t EEPROM_READ[11]; // To read EEPROM data

int32_t angle_roll_acc_manual_offset , angle_pitch_acc_manual_offset;
boolean first_angle = false;
int16_t gyro_axis_cal[4];
//==================================================
//********************MPU6050
//==================================================

// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

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
  Serial.begin(115200); //Serial Monitor Display
   // EEPROM.begin(512);
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
//  Wire.setClock (400000);          //I2C Initiliazination
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
  msp.begin(Serial);
 //=======================OTA===========
  
    ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();

    EEPROM_READ_DATA();
 //=============================GYRO START
   Serial.println(F("System check :-Gyro"));
  Serial.println("Please Input q to exit.");
  Serial.println(F("==================================================="));
  delay(100);
    Serial.println(("Searching for MPU-6050 on address 0x68/104"));
    delay(100);
    if(search_gyro(0x68, 0x75) == 0x68){
      Serial.println(("MPU-6050 found on address 0x68"));
      type = 1;
      gyro_address = 0x68;
    }
    
    if(type == 0){
      Serial.println(("Searching for MPU-6050 on address 0x69"));
      delay(100);
      if(search_gyro(0x69, 0x75) == 0x68){
        Serial.println(("MPU-6050 found on address 0x69"));
        type = 1;
        gyro_address = 0x69;
      }
    }
     
    if(type == 0){
      Serial.println(("No gyro device found!!! (ERROR )"));
      error = 1;
    }
    
    else{
      delay(30);
      Serial.println((""));
      Serial.println(("==================================================="));
      Serial.println(("Gyro register settings"));
      Serial.println(("==================================================="));
//      start_gyro(); //Setup the gyro for further use
    }
  //===========================================================================
  
   start_gyro(); //Setup the gyro for further use
     Menu();
     while(Menu_Input !='q'){
          if (Serial.available() > 0)
              {
              Menu_Input = Serial.read();                                       //Read the incomming byte.    
              Check();//Wait for any other bytes to come in.
              }
      }
      
}

unsigned long time_now = 0;


//**************************************************************
//**********************
//**************************************************************

void loop() {
  webSocket.loop();
      ArduinoOTA.handle();
  if(captive_portal)
    dnsServer.processNextRequest();
  server.handleClient();


  yield();
}


void start_gyro(){
  //Setup the MPU-6050
  delay(150);
  I2C_Write(gyro_address, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(gyro_address, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(gyro_address, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(gyro_address, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(gyro_address, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(gyro_address, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(gyro_address, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(gyro_address, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(gyro_address, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(gyro_address, MPU6050_REGISTER_USER_CTRL, 0x00);  delay(300);
  Serial.printf("MPU6050 started at Adddress %d: ", gyro_address);
}

void gyro_signalen(){
  //Read the MPU-6050
  Wire.beginTransmission(gyro_address);
  Wire.write(MPU6050_REGISTER_ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, (uint8_t)14);
  acc_axis[1] = (((int16_t)Wire.read()<<8) | Wire.read());
  acc_axis[2] = (((int16_t)Wire.read()<<8) | Wire.read());
  acc_axis[3]  = (((int16_t)Wire.read()<<8) | Wire.read());
  temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  gyro_axis[1] = (((int16_t)Wire.read()<<8) | Wire.read());
  gyro_axis[2] = (((int16_t)Wire.read()<<8) | Wire.read());
  gyro_axis[3] = (((int16_t)Wire.read()<<8) | Wire.read());
  
  if(cal_int == Gyro_Calib_offset){
    gyro_axis[1] -= gyro_axis_cal[1];                            //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                            //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                            //Only compensate after the calibration.
  }
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  
  //divide each with their sensitivity scale factor
  acc_axis[1] = (double)acc_axis[1];
  acc_axis[2] = (double)acc_axis[2];
  acc_axis[3] = (double)acc_axis[3];
  T = (double)temperature/340+36.53; //temperature formula
  gyro_axis[1] = (double)gyro_axis[1];
  gyro_axis[2] = (double)gyro_axis[2];
  gyro_axis[3] = (double)gyro_axis[3];

// Serial.print("Ax: "); Serial.print(Ax);
//  Serial.print(" Ay: "); Serial.print(Ay);
//  Serial.print(" Az: "); Serial.print(Az);
//  Serial.print(" T: "); Serial.print(T);
//  Serial.print(" Gx: "); Serial.print(Gx);
//  Serial.print(" Gy: "); Serial.print(Gy);
//  Serial.print(" Gz: "); Serial.println(Gz);

      acc_x = acc_axis[1];
      acc_y = acc_axis[2];
      acc_z = acc_axis[3];
  gyro_roll = gyro_axis[1];
 gyro_pitch = gyro_axis[2];
   gyro_yaw = gyro_axis[3];
//  gyro_roll = gyro_axis[roll_axis & 0b00000011];                      
//  if(roll_axis & 0b10000000)gyro_roll *= -1;                          
//  gyro_pitch = gyro_axis[pitch_axis & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
//  if(pitch_axis & 0b10000000)gyro_pitch *= -1;                       
//  gyro_yaw = gyro_axis[yaw_axis & 0b00000011];                       
//  if(yaw_axis & 0b10000000)gyro_yaw *= -1;                           
//
//  acc_x = acc_axis[pitch_axis & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
//  if(pitch_axis & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
//  acc_y = acc_axis[roll_axis & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
//  if(roll_axis  & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
//  acc_z = acc_axis[yaw_axis & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
//  if(yaw_axis   & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.
}

void EEPROM_READ_DATA(){
    for(int i = 0; i< 11; i++)
    {
     //   EEPROM_READ[i] = EEPROM.read(i);
        yield();
    }
    angle_pitch_acc_manual_offset = 0; //EEPROM_READ[0];
    delay(10);
    angle_roll_acc_manual_offset = 0; //EEPROM_READ[1]; 
    delay(10);
    roll_axis = 1; // EEPROM_READ[2];
    delay(10);
    pitch_axis = 2; //EEPROM_READ[3];
    delay(10);
    yaw_axis = 3; //EEPROM_READ[4];
    delay(10);
 //   type = 1;// EEPROM_READ[5];
    delay(10);
 //   gyro_address = 104; //EEPROM_READ[6];
    delay(10);
//    if((EEPROM_READ[7] != 'N') || (EEPROM_READ[8] != 'O') || (EEPROM_READ[9] != 'O') || (EEPROM_READ[10] != 'R'))
//    {
//        Serial.println("Please Run Setup Program first.");
//        while(1){yield();}
//    }
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
        Serial.println("Flash the Flight controlller firmware and enjoy");
        break;

    default:
      //  Menu();
        break;
    }
}

void Signals_Check(){
      Serial.println(("System check :-Channels"));
      Serial.println("Please Input q to exit.");
      Serial.println(("==================================================="));
   while(Serial.read() !='q'){
              webSocket.loop();
          
          if(captive_portal)
            dnsServer.processNextRequest();
          server.handleClient();
          if(alivecount>1000){
            for(int i=0; i<4;i++){
              ppm[i]=Min_throttle;
            }
            for(int i=4; i<8;i++){
              ppm[i]=1000;
            }
          }
        Serial.print("Roll : ");       Serial.print(ppm[0]);
        Serial.print("\t Pitch : ");   Serial.print(ppm[1] );
        Serial.print("\t Throttle : ");   Serial.print(ppm[2] );
        Serial.print("\t Yaw");        Serial.print(ppm[3]);
        Serial.print("\t Chan 5 : ");  Serial.print(ppm[4]);
        Serial.print("\t Chan 6 : ");  Serial.print(ppm[5]);
        Serial.print("\t Chan 7 : ");  Serial.print(ppm[6]);
        Serial.print("\t Chan 8 : ");  Serial.println(ppm[7]);
    yield();
    }
    Menu();
}

void Gyro_Check(){
 while(Serial.read()!='q'){
        if(cal_int != Gyro_Calib_offset){
      Serial.print("Calibrating the gyro");
      //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
      for (cal_int = 0; cal_int < Gyro_Calib_offset ; cal_int ++){                                   //Take 2000 readings for calibration.
        if(cal_int % 100 == 0)Serial.print(".");
        gyro_signalen();                                                                //Read the gyro output.
        gyro_axis_cal[1] += gyro_axis[1];                                               //Ad roll value to gyro_roll_cal.
        gyro_axis_cal[2] += gyro_axis[2];                                               //Ad pitch value to gyro_pitch_cal.
        gyro_axis_cal[3] += gyro_axis[3];                                               //Ad yaw value to gyro_yaw_cal.
        delay(3);                                                                   
      }
      Serial.println(".");
      //Now that we have Gyro_Calib_offset measures, we need to divide by Gyro_Calib_offset to get the average gyro offset.
      gyro_axis_cal[1] /= Gyro_Calib_offset;                                                         //Divide the roll total by Gyro_Calib_offset.
      gyro_axis_cal[2] /= Gyro_Calib_offset;                                                         //Divide the pitch total by Gyro_Calib_offset.
      gyro_axis_cal[3] /= Gyro_Calib_offset;                                                         //Divide the yaw total by Gyro_Calib_offset.
    }
    else{
          gyro_signalen();
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

      //Gyro angle calculations
      angle_pitch += (float)gyro_pitch * 0.0000611;                                           //Calculate the traveled pitch angle and add this to the angle_pitch variable.
      angle_roll += (float)gyro_roll * 0.0000611;                                             //Calculate the traveled roll angle and add this to the angle_roll variable.
      angle_yaw += (float)gyro_yaw * 0.0000611;
      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                         //If the IMU has yawed transfer the roll angle to the pitch angel.
      angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                         //If the IMU has yawed transfer the pitch angle to the roll angel.

    //Accelerometer angle calculations
      acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));           //Calculate the total accelerometer vector.
  if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }     
      if(!first_angle){
        angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
        angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
        first_angle = true;
      }
      else{
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
      }

      //We can't print all the data at once. This takes to long and the angular readings will be off.
      Serial.print("Pitch: ");
      Serial.print(angle_pitch ,0);
      Serial.print(" Roll: ");
      Serial.print(angle_roll ,0);
      Serial.print(" Yaw: ");
      Serial.println(gyro_yaw / 65.5 ,0);
    delay(100);
    }
   }
Menu();
}

byte search_gyro(int gyro_address, int who_am_i){
  Wire.beginTransmission(gyro_address);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  timer = millis() + 100;
  while(Wire.available() < 1 && timer > millis());
  lowByte = Wire.read();
  address = gyro_address;
  return lowByte;
}


void Motor_Check(uint8_t motor){
     Serial.println(F("System check :-Motor_Check"));
     Serial.println("Please Input q to exit.");
    Serial.println(F("==================================================="));
    while(Serial.read()!='q'){
      yield();
        webSocket.loop();  
          if(captive_portal)
            dnsServer.processNextRequest();
          server.handleClient();
      manual_throttle= ppm[2];
     manual_throttle = map(manual_throttle,Min_throttle,Max_throttle,0,255);
     switch (motor)
     {
     case 1:
    analogWrite(Motor1,manual_throttle);
    Serial.printf("Throttle : %d \n",manual_throttle );
         break;
    case 2:
    analogWrite(Motor2,manual_throttle);
    Serial.printf("Throttle : %d \n",manual_throttle );
         break;
     case 3:
    analogWrite(Motor3,manual_throttle);
    Serial.printf("Throttle : %d \n",manual_throttle );
         break;
     case 4:
    analogWrite(Motor4,manual_throttle);
    Serial.printf("Throttle : %d \n",manual_throttle );
         break;
     default:
         break;
     }
    }

    analogWrite(Motor1,0);
    analogWrite(Motor2,0);
    analogWrite(Motor3,0);
    analogWrite(Motor4,0);
    Menu();
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}
