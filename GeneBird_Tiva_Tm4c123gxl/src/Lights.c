/*
	*
	*	Lights.c	
	*	Created on 29 June 2020
	*
*/

#include <stdint.h>
#include <stdbool.h>

#include "rx.h"
#include "Lights.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

void initLEDS()
	{
			SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
			SysCtlDelay(3);
			GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	}
	
void led_F(bool enable)
{
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, enable ? GPIO_PIN_4: 0);
}

void led_B(bool enable)
{
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, enable ? GPIO_PIN_5: 0);
}

void led_R(bool enable)
{
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, enable ? GPIO_PIN_6: 0);
}

void led_L(bool enable)
{
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, enable ? GPIO_PIN_7: 0);
}
