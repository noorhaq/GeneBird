
void _Debug()
{
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration
    if (ch == 10) Serial.println();
    else if (ch == 'A')
    {
      Serial.println("Doing ACC calib");


      digitalWrite(GREEN_LED, LOW);

      calibratingA = CALSTEPS;
      while (calibratingA != 0)
      {
        delay(CALITIME);
        ACC_getADC();
      }
      ACC_Store();
      Serial.println("ACC calib Done");
      digitalWrite(GREEN_LED, HIGH);
    }
    else if (ch == 'R')
    {
      Serial.print("Act Rate : ");
      Serial.print(yawRate); Serial.print("  ");
      Serial.print(rollPitchRate); Serial.println();
      Serial.println("Act PID :");
      Serial.print(P_PID); Serial.print("  ");
      Serial.print(I_PID); Serial.print("  ");
      Serial.print(D_PID); Serial.println();
      Serial.print(P_Level_PID); Serial.print("  ");
      Serial.print(I_Level_PID); Serial.print("  ");
      Serial.print(D_Level_PID); Serial.println();
    }
    else if (ch == 'D')
    {
      digitalWrite(GREEN_LED, LOW);
      Serial.println("Loading default PID");
      yawRate = 6.0;
      rollPitchRate = 5.0;
      P_PID = 0.33;    // P8
      I_PID = 0.03;    // I8
      D_PID = 2.8;
      P_Level_PID = 0.35;   // P8
      I_Level_PID = 0.03;   // I8
      D_Level_PID = 2.8;
     // PID_Store();
      digitalWrite(GREEN_LED, HIGH);
    }
    else if (ch == 'W')
    {
      char ch = Serial.read();
      int n = Serial.available();
      if (n == 3)
      {
        n = readsernum();
        if      (ch == 'p') {
          P_PID       = float(n) * 0.01 + 0.004;
          Serial.print("pid P ");
          Serial.print(P_PID);
        }
        else if (ch == 'i') {
          I_PID       = float(n) * 0.01 + 0.004;
          Serial.print("pid I ");
          Serial.print(I_PID);
        }
        else if (ch == 'd') {
          D_PID       = float(n) * 0.01 + 0.004;
          Serial.print("pid D ");
          Serial.print(D_PID);
        }
        else if (ch == 'P') {
          P_Level_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid Level P ");
          Serial.print(P_Level_PID);
        }
        else if (ch == 'I') {
          I_Level_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid Level I ");
          Serial.print(I_Level_PID);
        }
        else if (ch == 'D') {
          D_Level_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid Level D ");
          Serial.print(D_Level_PID);
        }
        else Serial.println("unknown command");
      }
      else if (ch == 'S') {
        PID_Store();
        Serial.print("stored in //EEPROM");
      }
      else
      {
        Serial.println("Input format wrong");
        Serial.println("Wpxx, Wixx, Wdxx - write gyro PID, example: Wd13");
        Serial.println("WPxx, WIxx, WDxx - write level PID, example: WD21");
      }
    }
    else if (ch >= '0' && ch <= '9') debugvalue = ch - '0';
    else
    {
      Serial.println("A - acc calib");
      Serial.println("D - write default PID");
      Serial.println("R - read actual PID");
      Serial.println("Wpxx, Wixx, Wdxx - write gyro PID");
      Serial.println("WPxx, WIxx, WDxx - write level PID");
      Serial.println("WS - Store PID in //EEPROM");
      Serial.println("Display data:");
      Serial.println("0 - off");
      Serial.println("1 - Gyro values");
      Serial.println("2 - Acc values");
      Serial.println("3 - Angle values");
      Serial.println("4 - RC values");
      Serial.println("5 - Cycletime");
      Serial.println("6 - Servo Values");
    }

  }
  if      (debugvalue == 1) Serial.printf("\r%4d %4d %4d \n", gyroADC[0], gyroADC[1], gyroADC[2]);
  else if (debugvalue == 2) Serial.printf("\r%5d %5d %5d \n", accADC[0], accADC[1], accADC[2]);
  else if (debugvalue == 3) Serial.printf("\r%3f %3f \n", angle[0], angle[1]);
  else if (debugvalue == 6) Serial.printf("\r%3d %3d %3d %3d \n", servo[0], servo[1], servo[2], servo[3]);
  if (debugvalue == 5)
  {
    diff = micros() - mnow;
    Serial.println(diff);
  }
}

int readsernum()
{
  int num;
  char numStr[3];
  numStr[0] = Serial.read();
  numStr[1] = Serial.read();
  return atol(numStr);
}
