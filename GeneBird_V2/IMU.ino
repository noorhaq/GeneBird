//======================================IMU=========================//
enum cart { X, Y, Z };
#define GYRO_SCALE ((1998 * PI)/(32767.0f * 180.0f * 1000.0f)) // MPU6050
#define F_GYRO_SCALE 0.001 * GYRO_SCALE // in usec
/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#define GYR_CMPF_FACTOR 90
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))

typedef struct fp_vector
{
  float X, Y, Z;
} t_fp_vector_def;

typedef union
{
  float A[3];
  t_fp_vector_def V;
} t_fp_vector;


float InvSqrt (float x)
{
  union {
    int32_t i;
    float   f;
  } conv;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float* delta)
{
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}

void normalizeV(struct fp_vector *vec)
{
  float length;
  length = sqrtf(vec->X * vec->X + vec->Y * vec->Y + vec->Z * vec->Z);
  if (length != 0)
  {
    vec->X /= length;
    vec->Y /= length;
    vec->Z /= length;
  }
}

static t_fp_vector EstG = { 0.0, 0.0, (float)ACCRESO }; // x,y,z
static t_fp_vector EstM = { 0.0, 1.0, 0.0 }; // x,y,z
static float accData[3]  = {0, 0, 0};

uint32_t  tnow;

void getEstimatedAttitude()
{
  uint8_t axis;
  float deltaGyroAngle[3];
  uint32_t  tdiff;
  float scale;

  tdiff = micros() - tnow;
  tnow = micros();
  scale = (float)tdiff * F_GYRO_SCALE;

  // Initialization
  for (axis = 0; axis < 3; axis++)
  {
    deltaGyroAngle[axis] = (float)gyroADC[axis] * scale;
    accData[axis]  = (float)accADC[axis];
    gyroData[axis] = gyroADC[axis];
  }

  // AZ, EL
  rotateV(&EstG.V, deltaGyroAngle);

  // Apply complimentary filter (Gyro drift correction)
  for (axis = 0; axis < 3; axis++) EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + accData[axis]) * INV_GYR_CMPF_FACTOR;

  // Attitude of the estimated vector
  float sqGX_sqGZ = sq(EstG.V.X) + sq(EstG.V.Z);
  float invmagXZ  = InvSqrt(sqGX_sqGZ);
  anglerad[ROLL]  = atan2(EstG.V.X , EstG.V.Z);
  anglerad[PITCH] = atan2(EstG.V.Y , invmagXZ * sqGX_sqGZ);
  angle[ROLL]  = 572.95f * anglerad[ROLL];
  angle[PITCH] = 572.95f * anglerad[PITCH];

  // Yaw
  rotateV(&EstM.V, deltaGyroAngle);
#if MAG
  for (axis = 0; axis < 3; axis++)
    EstM.A[axis]  += (imu.magADC[axis] - EstM.A[2 * axis + 1]) << (16 - GYR_CMPFM_FACTOR);
#else
  normalizeV(&EstM.V);
#endif

  float invG = InvSqrt(sqGX_sqGZ + EstG.V.Y * EstG.V.Y);
  anglerad[YAW] = atan2(EstM.V.Z * EstG.V.X - EstM.V.X * EstG.V.Z,
                        (EstM.V.Y * sqGX_sqGZ - (EstM.V.X * EstG.V.X + EstM.V.Z * EstG.V.Z) * EstG.V.Y) * invG );
  angle[YAW] = 572.95f * anglerad[YAW];

}

//===================================================================
//====================================MPU6050===================================//
void i2cRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

void i2cWriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

#define ADDRESS 0x03
int calibratingG = CALSTEPS;
int calibratingA = 0;
int16_t gyroZero[3] = {0, 0, 0};
int16_t accZero[3] = {0, 0, 0};

// ****************
// GYRO common part
// ****************
void GYRO_Common()
{
  static int32_t g[3];
  uint8_t axis, tilt = 0;

  if (calibratingG > 0)
  {
    for (axis = 0; axis < 3; axis++)
    {
      // Reset g[axis] at start of calibration
      if (calibratingG == CALSTEPS) g[axis] = 0;
      // Sum up 512 readings
      g[axis] += gyroADC[axis];
      // Clear global variables for next reading
      gyroADC[axis] = 0;
      gyroZero[axis] = 0;
      if (calibratingG == 1)
      {
        if (g[axis] >= 0) g[axis] += CALSTEPS / 2;
        else            g[axis] -= CALSTEPS / 2;
        gyroZero[axis] = g[axis] / CALSTEPS;
      }
    }
    calibratingG--;
  }

  for (axis = 0; axis < 3; axis++)
    gyroADC[axis] = gyroADC[axis] - gyroZero[axis];
}

// ****************
// ACC common part
// ****************

void ACC_Common()
{
  static int32_t a[3];

  if (calibratingA > 0)
  {
    for (uint8_t axis = 0; axis < 3; axis++)
    {
      // Reset a[axis] at start of calibration
      if (calibratingA == CALSTEPS) a[axis] = 0;
      // Sum up 512 readings
      a[axis] += accADC[axis];
      // Clear global variables for next reading
      accADC[axis] = 0;
      accZero[axis] = 0;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1)
    {
      if (a[0] >= 0) a[0] += CALSTEPS / 2; else a[0] -= CALSTEPS / 2;
      if (a[1] >= 0) a[1] += CALSTEPS / 2; else a[1] -= CALSTEPS / 2;
      if (a[2] >= 0) a[2] += CALSTEPS / 2; else a[2] -= CALSTEPS / 2;
      accZero[0] = a[0] / CALSTEPS;
      accZero[1] = a[1] / CALSTEPS;
      accZero[2] = a[2] / CALSTEPS - ACCRESO;
      Serial.print("  "); Serial.print(accZero[0]); Serial.println();
      Serial.print("  "); Serial.print(accZero[1]); Serial.println();
      Serial.print("  "); Serial.print(accZero[2]); Serial.println();
    }
    calibratingA--;
  }
  accADC[0] -=  accZero[0];
  accADC[1] -=  accZero[1];
  accADC[2] -=  accZero[2];
}

void Gyro_getADC ()
{
  uint8_t rawADC[6];
  i2cRead(MPU6050_ADDRESS, 0x43, 6, rawADC);
  GYRO_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) ,
                    ((rawADC[2] << 8) | rawADC[3]) ,
                    ((rawADC[4] << 8) | rawADC[5]) );
  GYRO_Common();
}

void ACC_getADC ()
{
  uint8_t rawADC[6];
  i2cRead(MPU6050_ADDRESS, 0x3B, 6, rawADC);
  ACC_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) ,
                   ((rawADC[2] << 8) | rawADC[3]) ,
                   ((rawADC[4] << 8) | rawADC[5]) );
  ACC_Common();
}

void MPU6050_readId()
{
  uint8_t id;
  i2cRead(MPU6050_ADDRESS, 0x75, 1, &id);
  if (id == 0x68) Serial.println("6050 ID OK");
  else Serial.println("6050 ID Failed");
}

void MPU6050_init()
{
  Wire.begin();
  Wire.setClock(400000);

  //Gyro_init
  i2cWriteByte(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(50);

  i2cWriteByte(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2cWriteByte(MPU6050_ADDRESS, 0x1A, DLPF_CFG);         //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2cWriteByte(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  delay(50);

  //ACC_init
  i2cWriteByte(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG
  delay(50);
}

//========================================================================================
//========================================================================================
//========================================================================================
void ACC_Read()
{
  accZero[0] = read_int16(0);
  accZero[1] = read_int16(2);
  accZero[2] = read_int16(4);
}
void ACC_Store()
{
  write_int16(0, accZero[0]);
  write_int16(2, accZero[1]);
  write_int16(4, accZero[2]);
  EEPROM.write(63, 0x55);
  EEPROM.commit();
}


//===================================================================
//===================================================================
//===================================================================
void zeroGyroAccI()
{
  for (int axis = 0; axis < 3; axis++)
  {
    errorAngleI[axis] = 0.0;
    errorGyroI[axis] = 0.0;
  }
}
