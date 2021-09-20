#include <stdint.h>
#include "stm32f4xx.h"

void delay()
{
    for(uint32_t i=0; i<5000000; i++);
}


int main(void)
{
   GPIO_Handle_t Gpioignition, GpioLed, GpioLed1, Gpiouserbutn;
   Gpioignition.pGPIOx= GPIOD;
  Gpioignition.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_12;
   Gpioignition.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
   Gpioignition.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_LOW;
   Gpioignition.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
  Gpioignition.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
   
 
   GPIO_PericlockControl(GPIOD, ENABLE);
 
   GPIO_Init(&Gpioignition);


uint32_t i=0;

   while(i<100)
   {
   
      	        
                GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
                i++;
   }
   
  



   
   return 0;
}
