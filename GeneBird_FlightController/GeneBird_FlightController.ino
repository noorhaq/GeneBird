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
    EEPROM_READ_DATA();                 //Reading EEPROM data
  delay(250);
    battery_voltage = ( analogRead(Battery_Connection) + 65 ) *1.2317;
    loop_timer = micros();
    start_gyro();           //Starting MPU6050

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

//==========================================================================

//==============================================

//==========================================================================

  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611; 
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);   

 acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
   if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
  
  angle_pitch_acc -= angle_pitch_acc_manual_offset;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= angle_roll_acc_manual_offset;                                                    //Accelerometer calibration value for roll.
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction
  
   //For starting the motors: throttle low and yaw left (step 1).
  if(ppm[2] < 1050 && ppm[3] < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && ppm[2] < 1050 && ppm[3] > 1450){
    start = 2;

    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

 if(start == 2 && ppm[2] < 1050 && ppm[3] > 1950)start = 0;

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(ppm[2] > 1050){ //Do not yaw when turning off the motors.
    if(ppm[3] > 1508)pid_yaw_setpoint = (ppm[3] - 1508)/3.0;
    else if(ppm[3] < 1492)pid_yaw_setpoint = (ppm[3] - 1492)/3.0;
  }
  
  calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(Battery_Connection) + 65) * 0.09853;
  throttle = ppm[2];

  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  MotorWrite_MicroQuad();

  yield();
}

void MotorWrite_MicroQuad()
{
    
    esc_1 = map(esc_1, Min_throttle, Max_throttle, 0, 255);
    esc_2 = map(esc_2, Min_throttle, Max_throttle, 0, 255);
    esc_3 = map(esc_3, Min_throttle, Max_throttle, 0, 255);
    esc_4 = map(esc_4, Min_throttle, Max_throttle, 0, 255);

    analogWrite(Motor1 ,esc_1);
    analogWrite(Motor2 ,esc_2);
    analogWrite(Motor3 ,esc_3);
    analogWrite(Motor4 ,esc_4);
}

void start_gyro(){
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
  //Read the MPU-6050
  if(EEPROM_READ[5] == 1){
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address,14);                                      //Request 14 bytes from the gyro.
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  }

  gyro_roll = gyro_axis[roll_axis & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(roll_axis & 0b10000000)gyro_roll *= -1;                          //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[pitch_axis & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(pitch_axis & 0b10000000)gyro_pitch *= -1;                         //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(yaw_axis & 0b10000000)gyro_yaw *= -1;                           //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.


  acc_x = acc_axis[pitch_axis & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if(pitch_axis & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[roll_axis & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if(roll_axis & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[yaw_axis & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if(yaw_axis & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.
// Serial.print(gyro_roll); Serial.print(" "); Serial.print(gyro_pitch); Serial.print(" "); Serial.print(gyro_yaw); Serial.println();
 //Serial.print(" ROll "); Serial.print(gyro_roll); Serial.print(" Pitch "); Serial.print(gyro_pitch); Serial.print(" YAW "); Serial.print(gyro_yaw); Serial.println();
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

void Gyro_Calculate(){
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
}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}