
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"

#include "rx.h"
#include "time.h"
#include "buzzer.h"
#include "uart_1.h"
#include "i2c_1.h"
#include "mpu.h"
#include "pwm_1.h"
#include "mat.h"
#include "pid.h"
#include "Lights.h"

#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"

#include "utils/uartstdio.c"

#include <string.h>

int main()
{

 	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
 	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
  initTime();
	initUart();
	initI2C();
	initMPU6050();
	initPWM();
	initRX();
	initBuzzer();
	initLEDS();
	
	MPUtestConnection();
	IntMasterEnable();
	SysCtlDelay(3);

	float MAX = 1000.0f, MIN = -1000.0f;
	float  	stabilizePitch = 0,
			PitchOut = 0,
			Pitch,
			stabilizeRoll = 0,
			RollOut = 0,
			Roll = 0,
			Yaw = 0,
			YawOut = 0,
			stabilizeYaw = 0,
			Throttle,
			aux1	=	0,
			aux2	=	0;
//	int		R = 0x2, 	 	G = 0x8,	 	 	B = 0x4;
	float 	motorF,motorR,motorL,motorB; // Motor 0 is bottom right, motor 1 is top right, motor 2 is bottom left and motor 3 is top left
    float 	dtrate;
//		float 	rateRoll, ratePitch, rateYaw;
	double 	last = 0;
	float 	dt = 0.001;
//	float maxX=0, maxY=0;
	
//	int16_t blueled = true;
//	int16_t greenled = false;
//	int16_t k = 1;

//*******ESC CALIBRATION ROUTINE *******//
////
//   writeMotorL(MAX);
//   writeMotorR(MAX);
//   writeMotorB(MAX);
//   writeMotorF(MAX);
//   Delay(5000000);
//   writeMotorR(MIN);
//   writeMotorL(MIN);
//   writeMotorB(MIN);
//   writeMotorF(MIN);
//   Delay(5000000);


//	while(!Armed()){ }
//Remote Control

    while(1)
    {
    	double now = micros();
    	dtrate = (now - last) / 1e6f;
    	last = now;

    	//RollAngle = getRXchannel(RX_ROLL);
     	//PitchAngle = getRXchannel(RX_PITCH);
			
		/*	Roll = getRXchannel(RX_ROLL);
     	Pitch = getRXchannel(RX_PITCH);
     	Yaw = getRXchannel(RX_YAW);
      Throttle = getRXchannel(RX_THROTTLE);
			*/
			Throttle = 500;
			
			aux1 = getRXchannel(RX_AUX1);
			aux2 = getRXchannel(RX_AUX2);

    	aux1 = constrain(aux1,1, 0);
    	aux2 = constrain(aux2,1, 0);
		
		
		
		Yaw = mapf(Yaw, MIN, MAX, 250.f, -250.f);
		Roll = mapf(Roll, MIN, MAX, 80.f, -80.f);
		Pitch = mapf(Pitch, MIN, MAX, -80.f, 80.f);

		Yaw = constrain(Yaw, 250., -250.);
		Roll = constrain(Roll, 80., -80.);
		Pitch = constrain(Pitch, 80., -80.);
//	UARTprintf("Y %d R %d P %d T %d\n" ,(int)(Yaw), (int)(Roll), (int)(Pitch), (int)(Throttle));
		YawOut = ratePID(Yaw, dtrate, 3); //ratePID(point,dt,axis);
		PitchOut = ratePID(Pitch, dtrate, 1);
		RollOut = ratePID(Roll, dtrate, 2);

		RollOut = constrain(RollOut, MAX, MIN);
		YawOut = constrain(YawOut, MAX, MIN);
		PitchOut = constrain(PitchOut, MAX, MIN);

		
		//Yaw = mapf(Yaw, MIN, MAX, MIN_YAW_SCALE, MAX_YAW_SCALE);
    //    Yaw = constrain(Yaw, MAX_YAW_SCALE, MIN_YAW_SCALE);
	   // RollAngle = mapf(RollAngle, MIN, MAX,MIN_TX_SCALE , MAX_TX_SCALE);
	   // RollAngle = constrain(RollAngle, MAX_TX_SCALE, MIN_TX_SCALE);
	   // PitchAngle = mapf(PitchAngle, MIN, MAX, MIN_TX_SCALE, MAX_TX_SCALE);
	 	//PitchAngle = constrain(PitchAngle, MAX_TX_SCALE, MIN_TX_SCALE);
    	if(dataReadyMPU())
			{

			readMPU();

		
				stabilizePitch = stabilizePID(PitchOut, dt, 1);
    		stabilizeRoll = stabilizePID(RollOut, dt, 2);
    		stabilizeYaw = stabilizePID(YawOut, dt, 3);

    		stabilizeRoll = constrain(stabilizeRoll, MAX, MIN);
    		stabilizePitch = constrain(stabilizePitch, MAX, MIN);
    		stabilizeYaw = constrain(stabilizeYaw, MAX, MIN);

   //		    	UARTprintf("Y%d R%d P%d \n" ,(int)(Yaw), (int)(Roll), (int)(Pitch));
    	//	UARTprintf("%d \n", (int)(YawOut));

		//		motorF = Throttle - rateRoll - ratePitch - rateYaw;
    //		motorR = Throttle + rateRoll - ratePitch + rateYaw;
    //		motorB = Throttle - rateRoll + ratePitch + rateYaw;;
    //		motorL = Throttle + rateRoll + ratePitch - rateYaw;
				motorF = Throttle - stabilizeRoll - stabilizePitch + YawOut;
    		motorR = Throttle + stabilizeRoll - stabilizePitch - YawOut;
    		motorB = Throttle + stabilizeRoll + stabilizePitch + YawOut;
    		motorL = Throttle - stabilizeRoll + stabilizePitch - YawOut;
				
    		motorR = constrain(motorR, MAX, MIN);
    		motorL = constrain(motorL, MAX, MIN);
    		motorF = constrain(motorF, MAX, MIN);
    		motorB = constrain(motorB, MAX, MIN);
//UARTprintf("T%d R%d P%d Y%d\n" ,(int)(Throttle), (int)(stabilizeRoll), (int)(stabilizePitch), (int)(YawOut));
			writeMotorL(motorL);
	 		writeMotorR(motorR);
	 		writeMotorF(motorF);
	 		writeMotorB(motorB);
			
			led_B(aux1);
			led_F(aux1);
			led_R(aux2);
			led_L(aux2);
//			    		    	UARTprintf("L %d R%d F%d B%d \n" ,(int)(motorL), (int)(motorR), (int)(motorF), (int)(motorB));
		
			}
   }

}
