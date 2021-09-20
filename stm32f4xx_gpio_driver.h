/* GPIO specific Header File*/

#ifndef INC_STM32f4xx_gpio_H_
#define INC_STM32f4xx_gpio_H_
#include "STM32f4xx.h"
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*GPIO Pin number */

#define GPIO_PIN_NO_0  0
#define GPIO_PIN_NO_1  1
#define GPIO_PIN_NO_2  2
#define GPIO_PIN_NO_3  3
#define GPIO_PIN_NO_4  4
#define GPIO_PIN_NO_5  5
#define GPIO_PIN_NO_6  6
#define GPIO_PIN_NO_7  7
#define GPIO_PIN_NO_8  8
#define GPIO_PIN_NO_9  9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/* GPIO_Pinmodes */
#define GPIO_MODE_IN       0
#define GPIO_MODE_OUT      1
#define GPIO_MODE_ALTFN    2
#define GPIO_MODE_ANALOG   3

/*GPIO_PinSpeeds*/
#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MED     1
#define GPIO_SPEED_HIGH    2
#define GPIO_SPEED_VHIGH   3

/*GPIO_PinPinPuPdControl*/
#define GPIO_NO_PUPD       0
#define GPIO_PU            1
#define GPIO_PD            2

/*GPIO_PinOptype*/
#define GPIO_OP_TYPE_PP    0//Output push-pull (reset state)
#define GPIO_OP_TYPE_OD    1//Output open-drain


/*USER DEFINED APIS*/


/*Intialisation and Deintialisation of APIS*/
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/*Peripheral clock*/
void GPIO_PericlockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);

/*DATA Read and Write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx,uint8_t PinNumber );
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WritetoOutputPin(GPIO_RegDef_t* pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WritetoOutputPort(GPIO_RegDef_t* pGPIOx,uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx,uint8_t PinNumber);


#endif
