//=====================================OUTPUT=====================//
void mix()
{
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    servo[0] = constrain(rcValue[THR] + axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW], 1000, 2000); //FRONT_L
    servo[1] = constrain(rcValue[THR] - axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW], 1000, 2000); //FRONT_R
    servo[2] = constrain(rcValue[THR] - axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW], 1000, 2000); //REAR_R
    servo[3] = constrain(rcValue[THR] + axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW], 1000, 2000); //REAR_L

  }
  else
  {
    axisPID[0] = 0; axisPID[1] = 0; axisPID[2] = 0;
    servo[0] = 1000; servo[1] = 1000; servo[2] = 1000; servo[3] = 1000;
  }
  servo[0] = map(servo[0], 1000, 2000, 0, 1023);
  servo[1] = map(servo[1], 1000, 2000, 0, 1023);
  servo[2] = map(servo[2], 1000, 2000, 0, 1023);
  servo[3] = map(servo[3], 1000, 2000, 0, 1023);
}

void writeServo()
{

  analogWrite(FRONT_L, servo[0]);
  analogWrite(FRONT_R, servo[1]);
  analogWrite(REAR_R, servo[2]);
  analogWrite(REAR_L, servo[3]);
}

void initServo()
{
  pinMode(FRONT_L, OUTPUT);
  pinMode(FRONT_R, OUTPUT);
  pinMode(REAR_R, OUTPUT);
  pinMode(REAR_L, OUTPUT);

}
