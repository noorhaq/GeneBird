//==================================================RECEIVER==================================================//


BLYNK_WRITE(V0)
{
  rcValue[0] = param.asInt();
}
BLYNK_WRITE(V1)
{
  rcValue[1] = param.asInt();
}
BLYNK_WRITE(V2)
{
  rcValue[2] = param.asInt();
}
BLYNK_WRITE(V3)
{
  rcValue[3] = param.asInt();

}

BLYNK_WRITE(V5)
{
  rcValue[AU1] = param.asInt();
}
BLYNK_WRITE(V6)
{
  armed = param.asInt();
}
