//==================================================RECEIVER==================================================//


void _Reciever()
{
  rcValue[0] = ppm[2]; //ROLL
  rcValue[1] = ppm[3]; //Pitch
  rcValue[2] = ppm[1]; //Throttle
  rcValue[3] = ppm[0]; //YAW
  rcValue[AU1] = ppm[4]; //AUX1
  armed = ppm[5];  //AUX2
}
