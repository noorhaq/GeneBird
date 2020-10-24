//***************************************
//  WiFi Controlled Quad
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

volatile unsigned long next;
volatile unsigned int ppm_running=1;

boolean Gyro_Temp = false;
byte angle_roll_acc_manual_offset , angle_pitch_acc_manual_offset;
//=================================================
//********************Variables
//=================================================
int ppm[CHANNEL_NUMBER];
byte lowByte, highByte, type, clockspeed_ok;
byte  roll_axis, pitch_axis, yaw_axis;
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
    Serial.print(("Register 0x6B is set to:"));
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
    Serial.print(("Register 0x1B is set to:"));
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
 // timer = millis() + 10000;    
  while( gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_signalen();
    yield();
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
    
    delayMicroseconds(3700); //Loop is running
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
    Serial.println(("No angular motion is detected."));
  }
  else
  if(movement == 1)roll_axis = trigger_axis;
  if(movement == 2)pitch_axis = trigger_axis;
  if(movement == 3)yaw_axis = trigger_axis;
  
}

//===========================================================
//===========================
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
 // Wire.setClock (I2C_Speed);          //I2C Initiliazination

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
    ArduinoOTA.begin();
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
  

Check();
EEPROM_Commit();
}

unsigned long time_now = 0;


//**************************************************************
//**********************
//**************************************************************

void loop() {

//==============================================================
  webSocket.loop();
 ArduinoOTA.handle();
    

 
  if(captive_portal)
    dnsServer.processNextRequest();
  server.handleClient();
     
  if(alivecount>1000){
    for(int i=0; i<4;i++){
      ppm[i]=900;
    }
    for(int i=4; i<8;i++){
      ppm[i]=1100;
    }
    
  }
 
  yield();
}
//========================================
void Check(){
  Serial.println("Welcome");
  
  Serial.println((""));
  Serial.println(("==================================================="));
  Serial.println(("System check"));
  Serial.println(("==================================================="));
  delay(1000);
  Serial.println(("Checking I2C clock speed."));
  delay(1000);
  

  #if F_CPU == 16000000L          //If the clock speed is 16MHz include the next code line when compiling
    clockspeed_ok = 1;            //Set clockspeed_ok to 1
  #endif                          //End of if statement

  if(error == 0){
    //What gyro is connected
    Serial.println((""));
    Serial.println(("==================================================="));
    Serial.println(("Gyro search"));
    Serial.println(("==================================================="));
    delay(2000);
    
    Serial.println(("Searching for MPU-6050 on address 0x68/104"));
    delay(1000);
    if(search_gyro(0x68, 0x75) == 0x68){
      Serial.println(("MPU-6050 found on address 0x68"));
      type = 1;
      gyro_address = 0x68;
    }
    
    if(type == 0){
      Serial.println(("Searching for MPU-6050 on address 0x69"));
      delay(1000);
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
      delay(3000);
      Serial.println((""));
      Serial.println(("==================================================="));
      Serial.println(("Gyro register settings"));
      Serial.println(("==================================================="));
      start_gyro(); //Setup the gyro for further use
    }
  }
  
  //If the gyro is found we can setup the correct gyro axes.
  if(error == 0){
    delay(3000);
    Serial.println((""));
    Serial.println(("==================================================="));
    Serial.println(("Gyro calibration"));
    Serial.println(("==================================================="));
    Serial.println(("Don't move the quadcopter!! Calibration starts in 3 seconds"));
    delay(3000);
    Serial.println(("Calibrating the gyro"));
    Serial.print(("Please wait"));
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
   for (int i =0; i < 100 ; i++){              //Take 2000 readings for calibration.
      if(cal_int % 100 == 0)Serial.print(("."));                //Print dot to indicate calibration.
      gyro_signalen();                                           //Discarding the first 100 values.
      delay(3);                                                  //Discarding the first 100 values.
    }   
   for (cal_int = 0; cal_int < Gyro_Calib_offset ; cal_int ++){              //Take 2000 readings for calibration.
      if(cal_int % 100 == 0)Serial.print(("."));                //Print dot to indicate calibration.
      gyro_signalen();                                           //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                              //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                  //Ad yaw value to gyro_yaw_cal.
      yield();                                                  //Wait 3 milliseconds before the next loop.
    }
    //Calculating average offset.
    gyro_roll_cal /= Gyro_Calib_offset;                                       //Divide the roll total by 2000.
    gyro_pitch_cal /= Gyro_Calib_offset;                                      //Divide the pitch total by 2000.
    gyro_yaw_cal /= Gyro_Calib_offset;                                        //Divide the yaw total by 2000.
    
    gyro_signalen();
    
    //Show the calibration results
    Serial.println((""));
    Serial.print(("Axis 1 offset="));
    Serial.println(gyro_roll_cal);
    Serial.print(("Axis 2 offset="));
    Serial.println(gyro_pitch_cal);
    Serial.print(("Axis 3 offset="));
    Serial.println(gyro_yaw_cal);
    Serial.println((""));
    
    //Spirit Level Value Calculation
    gyro_signalen();
    Serial.println(("Calculating Spirit Level Value of IMU. Don't Move Quadcopter."));
    Serial.println(("...."));
    Serial.println(("...."));
    Serial.println(("...."));
    gyro_roll -= gyro_roll_cal;
    gyro_pitch -= gyro_pitch_cal;
    gyro_yaw -= gyro_yaw_cal;

   angle_pitch += gyro_pitch * 0.0000611;                                          //Total Angle travelled by Gyro 
      angle_roll += gyro_roll * 0.0000611;                                         //Total Angle travelled by Gyro
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //The Arduino asin function is in radians, so converting to degrees.
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
    
  if(!Gyro_Temp){
      angle_roll_acc_manual_offset = angle_pitch_acc;                                              //Accelerometer calibration value for pitch
      angle_pitch_acc_manual_offset = angle_roll_acc; 
      Gyro_Temp = true;
       }
   angle_pitch_acc -= angle_roll_acc_manual_offset ;                                              //Accelerometer calibration value for pitch
   angle_roll_acc -= angle_pitch_acc_manual_offset;                                               //Accelerometer calibration value for roll
  //Using Filter to Dampen the Angle
 // angle_pitch_output = angle_pitch_output * 0.75 + angle_pitch * 0.25;   //Take 90% of the output pitch value and add 10% of the raw pitch value
 // angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
     Serial.println("Are the values without moving the quadcopter 0.");
    Serial.println("If not re run the calibration of gyro and don't move.");
    Serial.println("Move the quadcopter/IMU around to check change in angles.");
    Serial.println("....."); 
    Serial.println(("Press a to stop 100 Gyro values from printing. "));
    delay(1000);
      while(Serial.read()!='a'){
  gyro_signalen();
   Serial.print("Pitch: ");
   Serial.print(angle_pitch ,0);
   Serial.print(" Roll: ");
   Serial.print(angle_roll ,0);
   Serial.print(" Yaw: ");
   Serial.println(gyro_yaw / 65.5 ,0);
   yield();
 }

    Serial.println(".....");
    Serial.print((""));
    Serial.println(("==================================================="));
    Serial.println(("Gyro axes configuration"));
    Serial.println(("==================================================="));
    
    //Detect the left wing up movement
    Serial.println(("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));
    //Check axis movement
    check_gyro_axes(1);
    if(error == 0){
      Serial.println(("OK!"));
      Serial.print(("Angle detection = "));
      Serial.println(roll_axis & 0b00000011);
      if(roll_axis & 0b10000000)Serial.println(("Axis inverted = yes"));
      else Serial.println(("Axis inverted = no"));
      Serial.println(("Put the quadcopter back in its original position"));
      Serial.println(("Press A to countinue. "));
      while(Serial.read()!='a'){yield();}

      //Detect the nose up movement
      Serial.println((""));
      Serial.println((""));
      Serial.println(("Lift the nose of the quadcopter to a 45 degree angle"));
      check_gyro_axes(2);
    }
    if(error == 0){
      Serial.println(("OK!"));
      Serial.print(("Angle detection = "));
      Serial.println(pitch_axis & 0b00000011);
      if(pitch_axis & 0b10000000)Serial.println(("Axis inverted = yes"));
      else Serial.println(("Axis inverted = no"));
      Serial.println(("Put the quadcopter back in its original position"));
      Serial.println(("Press A to countinue. "));
      while(Serial.read()!='a'){yield();}
      
      //Detect the nose right movement
      Serial.println((""));
      Serial.println((""));
      Serial.println(("Rotate the nose of the quadcopter 45 degree to the right"));
      //Check axis movement
      check_gyro_axes(3);
    }
    if(error == 0){
      Serial.println(("OK!"));
      Serial.print(("Angle detection = "));
      Serial.println(yaw_axis & 0b00000011);
      if(yaw_axis & 0b10000000)Serial.println(("Axis inverted = yes"));
      else Serial.println(("Axis inverted = no"));
      Serial.println(("Put the quadcopter back in its original position"));
     Serial.println(("Press A to countinue. "));
      while(Serial.read()!='a'){yield();}
      
    }
  }
  if(error == 0){
    Serial.println((""));
    Serial.println(("==================================================="));
    Serial.println(("LED test"));
    Serial.println(("==================================================="));
    digitalWrite(12, HIGH);
    Serial.println(("The LED should now be lit"));
    Serial.println(("Press A to countinue. "));
      while(Serial.read()!='a'){yield();}
        digitalWrite(12, LOW);
  }
  
  Serial.println((""));
  
  if(error == 0){
    Serial.println(("==================================================="));
    Serial.println(("Final setup check"));
    Serial.println(("==================================================="));
    delay(1000);
    if(gyro_check_byte == 0b00000111){
      Serial.println(("Gyro axes ok"));
    }
    else{
      Serial.println(("Gyro axes verification failed!!! (ERROR 7)"));
      error = 1;
    }
  }     
  


  }


void EEPROM_Commit()
{
// 
//   if(error == 0){
//    //If all is good, store the information in the EEPROM
//    Serial.println((""));
//    Serial.println(("==================================================="));
//    Serial.println(("Storing EEPROM information"));
//    Serial.println(("==================================================="));
//    Serial.println(("Writing EEPROM"));
//
 //   EEPROM.write(0,angle_pitch_acc_manual_offset);
Serial.printf("ANGLE PITCH ACC MANUAL OFFSET : %d \n",angle_pitch_acc_manual_offset);
  delay(100);
 //   EEPROM.write(1,angle_roll_acc_manual_offset); 
      delay(100);
Serial.printf("angle_roll_acc_manual_offset : %d \n",angle_roll_acc_manual_offset);
  //  EEPROM.write(2, roll_axis); 
      delay(100); 
Serial.printf("roll_axis: %d \n",roll_axis);
  //  EEPROM.write(3, pitch_axis);
          delay(100); 
Serial.printf("pitch_axis : %d \n",pitch_axis);
  //  EEPROM.write(4, yaw_axis);
          delay(100);  
Serial.printf("yaw_axis : %d \n",yaw_axis);
      delay(100); 
  //  EEPROM.write(5, type);  
          delay(100); 
Serial.printf("type : %d \n", type);
  //  EEPROM.write(6, gyro_address); 
          delay(100); 
Serial.printf("gyro_address : %d \n",gyro_address);
//    //Write the EEPROM signature
  //  EEPROM.write(7, 'N');  
          delay(100); 
  //  EEPROM.write(8, 'O');
          delay(100);   
  //  EEPROM.write(9, 'O');
          delay(100);  
  //  EEPROM.write(10, 'R');
          delay(100); 
        delay(1000);
    Serial.println(("Done!"));
 //   EEPROM.commit();
//    //To make sure evrything is ok, verify the EEPROM data.
//    Serial.println(("Verify EEPROM data"));
//    delay(1000);
//    if(roll_axis != EEPROM.read(2))error = 1;
//    
//    if(pitch_axis != EEPROM.read(3))error = 1;
//    
//    if(yaw_axis != EEPROM.read(4))error = 1;
//    
//    if(type != EEPROM.read(5))error = 1;
//    
//    if(gyro_address != EEPROM.read(6))error = 1;
//    
//    if('N' != EEPROM.read(7))error = 1;
//    if('O' != EEPROM.read(8))error = 1;
//    if('O' != EEPROM.read(9))error = 1;
//    if('R' != EEPROM.read(10))error = 1;
//  Serial.println(EEPROM.read(5));
//    if(error == 1)Serial.println(("EEPROM verification failed!!! (ERROR )"));
//    else Serial.println(("Verification done"));
    
//  }
  
  
  if(error == 0){
    Serial.println(("Setup is finished."));
  }

}
