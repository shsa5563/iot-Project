/*****************************************************************************
 * @file Assignment4.c
 * @brief ADC Samples with DMA Switch, Luminosity Sensor with LETIMER0 of 5 seonds each
 * The luminosity sesnor is monitored for the first two 5 second period and for the third period it goes to EM3
 * @author Shekhar Hulikal Satyanarayana (All Library and Predefined files are from Silicon Labs)
 * @version 1.00
 * I have optimised the code to gain more Energy Score (Optimisation includes coding to the register
 * rather than using predefined functions)
 *  Thus the code might not look the exact same way I had coded in assignment 1
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_letimer.h"
#include "em_system.h"
#include "em_device.h"
#include "em_rtc.h"
#include "em_int.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "em_adc.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "main.h"
#include "TSL2561.h"
#include "em_leuart.h"
#include "circular_Buffer.h"

void unblockSleepMode(sleepstate_enum minimumMode);
void blockSleepMode(sleepstate_enum minimumMode);

int8_t irq_count = 0;
int8_t counter_tempirq = 0 ;
int8_t counter_lightirq = 0;
float convertToCelsius(int32_t adcSample);
void setupI2C(void);
void init_sensor(uint8_t Command_Reg1, uint8_t Reg_Value, uint8_t Read_Write);

void circ_buff(void)
{
_transmit_buffer.buff = allocate_memory(&_transmit_buffer);
			_transmit_buffer.head=_transmit_buffer.buff;
					 	_transmit_buffer.tail=_transmit_buffer.buff;
					 	_transmit_buffer.buff=_transmit_buffer.buff;
					 	_transmit_buffer.num_items= 0;
					 	_transmit_buffer.length = MAX_LEN;
}

/**************************************************************************//**
 * @brief  Defining the LEUART1 initialization data
 ******************************************************************************/

LEUART_Init_TypeDef leuart1Init =
{
  .enable   = LEUART_CMD_TXEN,       /* Activate data reception on LEUn_TX pin. */
  .refFreq  = off,                    /* Inherit the clock frequenzy from the LEUART clock source */
  .baudrate = Baud_Rate,                 /* Baudrate = 9600 bps */
  .databits = LEUART_CTRL_DATABITS_EIGHT,      /* Each LEUART frame containes 8 databits */
  .parity   = LEUART_CTRL_PARITY_NONE,       /* No parity bits in use */
  .stopbits = LEUART_CTRL_STOPBITS_ONE,      /* Setting the number of stop bits in a frame to 2 bitperiods */
};

/**************************************************************************//**
 * @brief  Initialization of LEUART0
 ******************************************************************************/

void initLeuart(void)
{
	if (Enery_Mode == EM3)
	{
	CMU_OscillatorEnable(cmuSelect_LFRCO,true,true);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO);
	}
	else
	{
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	}
  LEUART_Init(LEUART0, &leuart1Init);

  /* Route LEUART1 TX pin to DMA location 0 */
  LEUART0->ROUTE = LEUART_ROUTE_TXPEN |
                   LEUART_ROUTE_LOCATION_LOC0;

  /* Enable GPIO for LEUART1. TX is on D4 */
  GPIO_PinModeSet(EXT_PORT_D,                /* GPIO port */
		  	  	  EXT_PIN_4,                        /* GPIO port number */
                  gpioModePushPull,         /* Pin mode is set to push pull */
                  1);                       /* High idle state */
  LEUART0->CTRL |= LEUART_CTRL_LOOPBK;
  LEUART0->CTRL |= LEUART_CTRL_AUTOTRI;
  NVIC_EnableIRQ(LEUART0_IRQn);
}

/**************************************************************************//**
 * @brief  LEUART0_IRQHandler LEUART0 Interrupt Handler
 ******************************************************************************/

void LEUART0_IRQHandler(void)
{
	INT_Disable();
uint32_t Flags = LEUART0->IF;
LEUART0->IFC = Flags;
LEUART0->IEN &= ~LEUART_IEN_TXBL;
initLeuart();
LEUART0->CMD |= LEUART_CMD_TXEN | LEUART_CMD_RXEN;
if(_leuart_status == 'D')
{
	if (counter_tempirq != 0)
	{
		LEUART0->IEN &= ~LEUART_IEN_TXC;
		if(temp<0)
		{
			temp1 = temp;
			if (CircBuff !=0)
					{
					LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
					}
				else
				{
					LEUART0->TXDATA = (int8_t)((temp-temp1)*10);
				}
		}
		else
		{
			if (CircBuff !=0)
					{
					LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
					}
				else
				{
					LEUART0->TXDATA = (uint8_t)(((float)temp-(int)temp)*10);
				}
		}
		counter_tempirq =0;
		while ((LEUART0->STATUS & LEUART_STATUS_TXC)==0);
		LEUART0->CMD |= LEUART_CMD_TXDIS;
	}
	else if(counter_tempirq == 0)
	{
	LEUART0->IEN |= LEUART_IEN_TXC;
	if (CircBuff !=0)
		{
		LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
		}
	else
	{
	LEUART0->TXDATA = (int8_t)temp;
	}
	while ((LEUART0->STATUS & LEUART_STATUS_TXC)==0);
	counter_tempirq++;
	}

}
else if(_leuart_status == 'L')
{
	if (counter_lightirq!=0)
		{
		LEUART0->IEN &= ~LEUART_IEN_TXC;
		if (CircBuff !=0)
				{
				LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
				}
			else
			{
				LEUART0->TXDATA = (uint8_t)_light_status;
			}
		while ((LEUART0->STATUS & LEUART_STATUS_TXC)==0);
		LEUART0->CMD |= LEUART_CMD_TXDIS;
		counter_lightirq = 0;
		}
	else if(counter_lightirq == 0)
	{
	LEUART0->IEN |= LEUART_IEN_TXC;
	if (CircBuff !=0)
			{
			LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
			}
		else
		{
			LEUART0->TXDATA = (uint8_t)'O';
		}
	while ((LEUART0->STATUS & LEUART_STATUS_TXC)==0);
	counter_lightirq++;
	}
}
// Flags = LEUART0->IF;
//LEUART0->IFC = Flags;
INT_Enable();
}

/**************************************************************************//**
 * @brief  Initialise Sensor Routine - Routine to configure the Luminosity sensor
 *****************************************************************************/
void init_sensor(uint8_t Command_Reg1, uint8_t Reg_Value, uint8_t Read_Write)
{
	 I2C1->TXDATA = (I2C_ADDRESS << 1) | Read_Write;
				  I2C1->CMD = I2C_CMD_START;
				  I2C1->IFC=I2C_IFC_START;//check for start flag in IF reg and then clear it
				  while ((I2C1->IF & I2C_IF_ACK) ==  0);
				  I2C1->IFC = I2C_IFC_ACK;
			      I2C1->TXDATA = Command_Reg1;
				  while ((I2C1->IF & I2C_IF_ACK) ==  0);
				  I2C1->IFC = I2C_IFC_ACK;
				  I2C1->TXDATA = Reg_Value;
				  while ((I2C1->IF & I2C_IF_ACK) ==  0);
				  	  I2C1->IFC = I2C_IFC_ACK;
			I2C1->CMD = I2C_CMD_STOP;
				while ((I2C1->IF & I2C_IF_MSTOP) ==  0);
				I2C1->IFC=I2C_IFC_MSTOP;
}

/**************************************************************************//**
 * @brief  Stabilisation Routine - Routine to give delay
 *****************************************************************************/

void Stabilisation_routine (void)
{
	for(int i =0; i<10000; i++){}
}

/**************************************************************************//**
 * @brief  Slave Setup - To setup the Luminosity Sensor
 *****************************************************************************/

void slave_setup (void)
{
	INT_Disable();
	init_sensor((Command_Reg |timing_Reg),((low_gain<<4) | integration_time),Write);
	init_sensor((Command_Reg |interrupt_cntrl_reg|(interrupt_clear<<4)),((interrupt_control<<4) | persist),Write);
	init_sensor((Command_Reg |threshlowlow_reg),(threshlowlow),Write);
	init_sensor((Command_Reg |threshlowhigh_reg),(threshlowhigh),Write);
	init_sensor((Command_Reg |threshhighlow_reg),(threshhighlow),Write);
	init_sensor((Command_Reg |threshhighhigh_reg),(threshhighhigh),Write);
	init_sensor(Command_Reg,power_LightSensor,Write);
	Stabilisation_routine();
	 INT_Enable();
}


/**************************************************************************//**
 * @brief  Average Sample Routine - To calculating the Average value of the 1000 samples
 * The  convertToCelsius routine was taken from Silicon Labs
 *****************************************************************************/
void Average_Samples ()
{
	uint32_t Average=0;
	for(int i=0; i<ADCSAMPLES ; i++)
		{
		Average += ramBufferAdcData[i];
		}
temp = convertToCelsius(Average/ADCSAMPLES);
}


/**************************************************************************//**
 * @brief  Call-back called when transfer to the DMA is complete. The routine declaration (naming conventions)
 * is taken from silicon labs' sample program. - Changed the code to support the Assignment Requirements
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
	INT_Disable();
	ADC0->CMD = ADC_CMD_SINGLESTOP;
	initLeuart();
	/* Clearing flag to indicate that transfer is complete */
  transferActive = false;
  Average_Samples();
  if((temp>LOW_TEMP_LIMIT)&&(temp<HIGH_TEMP_LIMIT))
  {
	  GPIO_PinOutClear(LED_Port,LED1_Pin);
  }
  else
  {
	  GPIO_PinOutSet(LED_Port,LED1_Pin);
  }
  _leuart_status = 'D';
  if (CircBuff !=0)
  		{
	  put_item_tobuffer(&_transmit_buffer,(int8_t)temp);
			if(temp<0)
			{
				temp1 = temp;
				put_item_tobuffer(&_transmit_buffer,(int8_t)((temp-temp1)*10));
			}
			else
			{
				put_item_tobuffer(&_transmit_buffer,(uint8_t)(((float)temp-(int)temp)*10));
			}
		}
  LEUART0->IEN |= LEUART_IEN_TXBL;
  unblockSleepMode(EM1);
	INT_Enable();
}

/**************************************************************************//**
 * @brief Configure DMA for ADC RAM Transfer - the routine declaration and definition
 * is taken from the Silicon labs - Changes the code to support the Assignment Requirements
 *****************************************************************************/
void setupDma(void)
{
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;

  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Setting up call-back function */
  cb.cbFunc  = transferComplete;
  cb.userPtr = NULL;
  cb.primary = true;

  /* Setting up channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_ADC0_SINGLE;
  chnlCfg.cb        = &cb;
  DMA_CfgChannel(DMA_CHANNEL_ADC, &chnlCfg);

  /* Setting up channel descriptor */
  descrCfg.dstInc  = _DMA_CTRL_SRC_INC_HALFWORD;
  descrCfg.srcInc  = _DMA_CTRL_SRC_INC_NONE;
  descrCfg.size    = _DMA_CTRL_SRC_SIZE_HALFWORD;
  descrCfg.arbRate = _DMA_CTRL_R_POWER_1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_ADC, true, &descrCfg);
}

/**************************************************************************//**
 * @brief ADC0_IRQHandler
 * Interrupt Service Routine for ADC0
 *****************************************************************************/

void ADC0_IRQHandler(void)
{
INT_Disable();
uint32_t flags = ADC0->IF;
ADC0->IFC = flags;

if(count<ADCSAMPLES)
{
	ramBufferAdcData[count] = ADC0->SINGLEDATA;
	count++;
}else
{
	ADC0->CMD = ADC_CMD_SINGLESTOP;	/* Clearing flag to indicate that transfer is complete */
  Average_Samples();
  if((temp>LOW_TEMP_LIMIT)&&(temp<HIGH_TEMP_LIMIT))
  {
	  GPIO_PinOutClear(LED_Port,LED1_Pin);
  }
  else
  {
	  GPIO_PinOutSet(LED_Port,LED1_Pin);
  }
  unblockSleepMode(EM1);
	count=0;
}
INT_Enable();
}

/**************************************************************************//**
 * @brief SetUpADC routine is to configure the ADC
 *****************************************************************************/
void setupAdc(void)
{
  ADC_Init_TypeDef        adcInit       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef  adcInitSingle = ADC_INITSINGLE_DEFAULT;

  adcInit.timebase = ADC_TimebaseCalc(1);
  /* Configure ADC single mode to sample Ref/2 */
  adcInit.prescale = ADC_PrescaleCalc(Pre_Scalar_ADC, 0); /* Set highest allowed prescaler */
  ADC_Init(ADC0, &adcInit);

  adcInitSingle.input     =  _ADC_SINGLECTRL_INPUTSEL_TEMP;  /* Reference */
  adcInitSingle.resolution = _ADC_SINGLECTRL_RES_12BIT;
  adcInitSingle.reference    = _ADC_SINGLECTRL_REF_1V25;
  adcInitSingle.rep = true;
  adcInitSingle.leftAdjust= false;
  ADC_InitSingle(ADC0, &adcInitSingle);

  if(!DMA_Switch)
  {
	  /* Enable ADC Interrupt when Single Conversion Complete */
	 	 	 	    ADC0->IEN = ADC_IEN_SINGLE;
	 	 	 	  /* Enable ADC0 interrupt vector in NVIC*/
	 	 	 	    NVIC_EnableIRQ(ADC0_IRQn);
  }

}

/**************************************************************************//**
 * I claim that I have used blockSleepMode routine, sleep routine , unblockSleepMode routine, converttoCelsius routine from Silicon labs and below shows the permission*/
 /* Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
*****************************************************************************/

/**************************************************************************//**
 * @brief convertToCelsius is a routine to convert to Degree Celsius
 * The entire routine is from Silicon Labs - all credits for Silicon labs
 *****************************************************************************/

float convertToCelsius(int32_t adcSample)
{
float temp;
/*Factory calibration temperature from device information page*/
float cal_temp_0 = (float)((CAL_Value & CAL_Value_Mask)>> SHIFT_TEMP);
float cal_value_0 = (float)((ADC_CAL_VALUE & ADC_CAL_MASK)>>SHIFT_TEMP_V125);
/*Temperature gradient (from datasheet)*/
float t_grad= -6.27;
temp = (cal_temp_0 -((cal_value_0 - adcSample)/t_grad));
return temp;
}

/**************************************************************************//**
 * @brief Sleep Routine
 * To make the MCU sleep in different energy modes - code used from the silicon labs
 *****************************************************************************/
void sleep(void)
{
    if (sleep_block_counter[0] > 0) {
        /* Blocked everything below EM0, so just return */
        return;
    } else if (sleep_block_counter[1] > 0) {
        /* Blocked everything below EM1, enter EM1 */
        EMU_EnterEM1();
    } else if (sleep_block_counter[2] > 0) {
        /* Blocked everything below EM2, enter EM2 */
        EMU_EnterEM2(true);
    } else if (sleep_block_counter[3] > 0){
        /* Blocked everything below EM3, enter EM3 */
        EMU_EnterEM3(true);
    } else
    {
    	EMU_EnterEM3(true);
    }/* Never enter EM4, as Lepoard Gecko as it takes 8 to 12hrs to wake up */
    return;
}

/**************************************************************************//**
 * @brief Block Sleep Mode Routine– to increment the sleep_block_counter for appropriate energy mode
 * code used from the Silicon Labs
 *****************************************************************************/

void blockSleepMode(sleepstate_enum minimumMode)
{
    INT_Disable();
    sleep_block_counter[minimumMode]++;
    INT_Enable();
}

/**************************************************************************//**
 * @brief UnBlock Sleep Mode Routine – to decrement the sleep_block_counter for appropriate energy mode
 * code used from the Silicon Labs
 *****************************************************************************/


void unblockSleepMode(sleepstate_enum minimumMode)
{
    INT_Disable();
    if(sleep_block_counter[minimumMode] > 0) {
        sleep_block_counter[minimumMode]--;
    }
    INT_Enable();
}
/**************************************************************************//**
 * @brief Led Active Routine – For Led On/Off depending upon the Led_Status
 *****************************************************************************/

void Led_Activate(Led_Status LED)
{
	if(LED == on)
		GPIO_PinOutSet(LED_Port,LED0_Pin);
	else
		GPIO_PinOutClear(LED_Port,LED0_Pin);
}


/**************************************************************************//**
 * @brief LETIMER0_IRQHandler
 * Interrupt Service Routine for LETIMER
 *****************************************************************************/
void LETIMER0_IRQHandler(void)
{
  INT_Disable();
  uint32_t flags = LETIMER0->IF;
  LETIMER0->IFC = flags;
  if ((flags & LETIMER_IEN_COMP1))
    {
	  if(Passive_Light_Sense != 0)
	  {
	  ACMP_Enable(ACMP0);
	  while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT));

		GPIO_PinOutSet(LIGHTSENSE_EXCITE_PORT,LIGHTSENSE_EXCITE_PIN); // excite the light sense pin every 3 seconds
	  }
	  	if(DMA_Switch)
	  	{
	  		/* Setting flag to indicate that transfer is in progress
	   * will be cleared by call-back function. */
	  transferActive = true;
	  		/* Starting transfer. Using Basic since every transfer must be initiated
	  	   * by the ADC. */
	  	  DMA_ActivateBasic(DMA_CHANNEL_ADC,
	  	                    true,
	  	                    false,
	  	                    (void *)ramBufferAdcData,
	  	                    (void *)&(ADC0->SINGLEDATA),
	  	                    ADCSAMPLES - 1);
	  	}
	  blockSleepMode(EM1);
	  	ADC_Start(ADC0, ADC_CMD_SINGLESTART);


		if (period_count == 1)
		{
 			//unblockSleepMode(EM2);
			GPIO_PinOutSet(EXT_PORT_D, EXT_PIN_POWER);
			Stabilisation_routine();
			GPIO_ExtIntConfig(EXT_PORT_D, EXT_PIN_INT,EXT_PIN_INT, false, true, true);
			GPIO->IFC = GPIO->IF;
			 /* Enable GPIO_ODD interrupt vector in NVIC */
			NVIC_EnableIRQ(GPIO_ODD_IRQn);
			setupI2C();
			slave_setup();
						 /* Setting up i2c */
		}else if(period_count ==3)
			  		{
					GPIO_ExtIntConfig(EXT_PORT_D, EXT_PIN_INT,EXT_PIN_INT, false, true, false);
//						GPIO_PinModeSet(EXT_PORT_C,EXT_PIN_5,gpioModeDisabled,0);
//						GPIO_PinModeSet(EXT_PORT_C,EXT_PIN_4,gpioModeDisabled,0);
//						GPIO_PinModeSet(EXT_PORT_D,EXT_PIN_INT,gpioModeDisabled,0);
			  			GPIO_PinOutClear(EXT_PORT_D, EXT_PIN_POWER);
						GPIO->IFC = GPIO->IF;
			  			period_count=0;
			  			unblockSleepMode(EM1);
			  			blockSleepMode(Enery_Mode);
			  		}
		if(period_count == 3)
		{
			period_count= 1;
		}else
		{
			period_count++;
		}


    }
  else if ((flags & LETIMER_IEN_UF))
  {
	  if(Passive_Light_Sense !=0)
	  {
	  uint32_t Flag = (ACMP0->STATUS & ACMP_STATUS_ACMPOUT);
	 		ACMP_Disable(ACMP0);
	 		GPIO_PinOutClear(LIGHTSENSE_EXCITE_PORT,LIGHTSENSE_EXCITE_PIN); // excite the light sense pin every 3 seconds
	 	 	  		if (Flag != 0)
	 			{
	 			 if(VDD_LEVEL == UPPER_LIMIT)
	 			 {
	 				Led_Activate(off);
	 				ACMP0->INPUTSEL &= Clear_VDD_Ref;
	 				ACMP0->INPUTSEL |= SET_VDD_Ref_Low;
	 				VDD_LEVEL = LOWER_LIMIT;
	 			 }
	 			}
	 		else if (Flag == 0)
	 		{
	 			if(VDD_LEVEL == LOWER_LIMIT)
	 			{
	 				Led_Activate(on);
	 				ACMP0->INPUTSEL &= Clear_VDD_Ref;
	 				 ACMP0->INPUTSEL |= SET_VDD_Ref_high;
	 				 VDD_LEVEL = UPPER_LIMIT;
	 			}
	 		}
	  }
  }
  INT_Enable();
}
/**************************************************************************//**
 * @brief CMU_setup() Routine : To select/enable the Timer, GPIO ,Clocks,
 * Select the required Oscillator, Select LETIMR0 Clock
 ******************************************************************************/

void CMU_setup(void)
{
	/*Calculating Required Period for Different Frequencies*/
	if (Enery_Mode == EM3)
	{
		Required_Period = period * LOW_FREQ;
	}
	else
	{
		Required_Period = period * HIGH_FREQ;
	}
	/*using code from lecture notes to calculate prescaler value*/
	 /*Using the code to calculate prescalar fromm the lecture notes of proffesor*/
	int A = 1;
		while((Required_Period/(1*(A))) > 65535)
		{
			A*=2;
			Prescalar++;
		}
		CMU->LFAPRESC0 &=0xffffff0f;
			CMU->LFAPRESC0 |=Prescalar <<8;
			Prescalar= 1 << Prescalar;

	if (Enery_Mode == EM3)
	{
		//Enable the Ultra Low Frequency RC Oscillator for 1KHz
		  CMU_OscillatorEnable(cmuOsc_ULFRCO,true,true);
		  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	}
	else
	{
		//Enable the Low Frequency RC Oscillator for 32768Hz
				  CMU_OscillatorEnable(cmuSelect_LFXO,true,true);
				  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	}
	  /* Enable necessary clocks */

	  CMU_ClockEnable(cmuClock_CORELE, true);
	  CMU_ClockEnable(cmuClock_LETIMER0, true);
	  CMU_ClockEnable(cmuClock_GPIO, true);
	  /* Enable HF peripheral clock. */
	  CMU_ClockEnable(cmuClock_HFPER, true);
	  /* Enable clock for ACMP0. */
	  CMU_ClockEnable(cmuClock_ACMP0, true);
	  CMU_ClockEnable(cmuClock_DMA, true);
	  CMU_ClockEnable(cmuClock_ADC0, true);
	  CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART1 clock */


	  /* Enabling clock to the I2C, GPIO, LE */
	  CMU_ClockEnable(cmuClock_I2C1, true);
}


/**************************************************************************//**
/ * @brief  Setup I2C - Setting up the I2C
 *****************************************************************************/
void setupI2C(void)
{
  // Using default settings
  I2C_Init_TypeDef i2cInit = {                                                                         \
		  true,                    /* Enable when init done */                    \
		  true,                    /* Set to master mode */                       \
		  0,                       /* Use currently configured reference clock */ \
		  I2C_FREQ_FAST_MAX,   /* Set to fast rate */      \
		                           /* within I2C spec */                          \
		  _I2C_CTRL_CLHR_FAST      /* Set to use 11:3 low/high duty cycle */       \
		};

  /* Using PC4 (SDA) and PC5 (SCL) */
  GPIO_PinModeSet(EXT_PORT_C, EXT_PIN_5, gpioModeWiredAnd, 1);
  GPIO_PinModeSet(EXT_PORT_C, EXT_PIN_4, gpioModeWiredAnd, 1);

  for (int i = 0; i < 9; i++)
    {
      GPIO_PinModeSet(EXT_PORT_C, EXT_PIN_5, gpioModeWiredAnd, 0);
      GPIO_PinModeSet(EXT_PORT_C, EXT_PIN_5, gpioModeWiredAnd, 1);
    }

  /* Enable pins at location 1 */
  I2C1->ROUTE = I2C_ROUTE_SDAPEN |
                I2C_ROUTE_SCLPEN |
                (0 << _I2C_ROUTE_LOCATION_SHIFT);

  /* Initializing the I2C */
 I2C_Init(I2C1, &i2cInit);
 if (I2C1->STATE & I2C_STATE_BUSY){
 I2C1->CMD = I2C_CMD_ABORT;
 }
}


/**************************************************************************//**
 * @brief GPIO_setup() Routine : To set up the GPIO Pins and Ports for LED0
 ******************************************************************************/

void GPIO_setup(void)
{
	  GPIO_PinModeSet(LED_Port,LED0_Pin,gpioModePushPull,0);
	  GPIO_PinModeSet(LED_Port,LED1_Pin,gpioModePushPull,0);
	  /* Initialize the 2 GPIO pins of the light sensor setup. */
	  GPIO_PinModeSet(LIGHTSENSE_EXCITE_PORT, LIGHTSENSE_EXCITE_PIN, gpioModePushPull, 0);
	  GPIO_PinModeSet(LIGHTSENSE_SENSOR_PORT, LIGHTSENSE_SENSOR_PIN, gpioModeDisabled, 0);


	  /* Configure PE1 as input with filter enable */
	  GPIO_PinModeSet(EXT_PORT_D, EXT_PIN_INT, gpioModeInput, 1);
	  GPIO_PinModeSet(EXT_PORT_D,EXT_PIN_POWER,gpioModePushPull,0);
}


/**************************************************************************//**
 * @brief GPIO_ODD_IRQHandler() Routine : To latch up the LED depending on the sensor values
 ******************************************************************************/


void GPIO_ODD_IRQHandler(void)
{
	INT_Disable();
	uint32_t flags = GPIO->IF;
  /* clear flag for PE1 interrupt */
  GPIO->IFC = flags;

	I2C1->TXDATA = (I2C_ADDRESS << 1) | Write; //write
		 I2C1->CMD = I2C_CMD_START;
		 I2C1->IFC=I2C_IFC_START;
		  	while ((I2C1->IF & I2C_IF_ACK) ==  0);
		  		  	  I2C1->IFC = I2C_IFC_ACK;

	 I2C1->TXDATA = (0xA0)|(interrupt_clear<<4)|0x0C;
	 while ((I2C1->IF & I2C_IF_ACK) ==  0);
	 I2C1->IFC = I2C_IFC_ACK;

	I2C1->CMD = I2C_CMD_START;
	I2C1->TXDATA = (I2C_ADDRESS << 1) | Read; //write
	 while ((I2C1->IF & I2C_IF_ACK) ==  0);
	  	 I2C1->IFC = I2C_IFC_ACK;

	  	while ((I2C1->IF & I2C_IF_RXDATAV) ==  0);
	  	uint32_t  low_data =  I2C1->RXDATA;
	  I2C1->CMD =I2C_CMD_ACK;

	  	while ((I2C1->IF & I2C_IF_RXDATAV) ==  0);
uint32_t high_data =  I2C1->RXDATA;
I2C1->CMD =I2C_CMD_NACK;

uint32_t rx_data = high_data<<8 |low_data;

I2C1->CMD = I2C_CMD_STOP;
				while ((I2C1->IF & I2C_IF_MSTOP) ==  0);
				I2C1->IFC=I2C_IFC_MSTOP;
				initLeuart();
				_leuart_status = 'L';
				if (CircBuff !=0)
				{
					put_item_tobuffer(&_transmit_buffer,'O');
				}
				  LEUART0->IEN |= LEUART_IEN_TXBL;
if(rx_data < (threshlowhigh<<8 | threshlowlow))
{
   GPIO_PinOutSet(gpioPortE, 2);
   _light_status = 'N';
   if (CircBuff !=0)
   				{
	   	   	   	   put_item_tobuffer(&_transmit_buffer,'N');
   				}
}
else if (rx_data > (threshhighhigh<<8 | threshhighlow))
{
	GPIO_PinOutClear(gpioPortE, 2);
	_light_status = 'F';
	if (CircBuff !=0)
					{
					put_item_tobuffer(&_transmit_buffer,'F');
					}
}
GPIO->IFC = GPIO->IF;
INT_Enable();

}


/**************************************************************************//**
 * @brief Calibartion routine for the ULFCRO
 ******************************************************************************/


void ULFRCO_Calibration()
{    /*Code to calculate the LFXO Count through TIMER0 & TIMER1*/

	//set up clocks for LFXO
		CMU_OscillatorEnable(cmuOsc_LFXO,true,true);

	    CMU_ClockEnable(cmuClock_CORELE, true); /* Enable CORELE clock */
	    CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO); /* Select LFXO as clock source for LFA for EM0 to EM2 */


	    /* Select TIMER0 parameters */
			  TIMER_Init_TypeDef timer0Init =
			  {
			    .enable     = false,
			    .debugRun   = true,
			    .prescale   = timerPrescale1,
			    .clkSel     = timerClkSelHFPerClk,
			    .fallAction = timerInputActionNone,
			    .riseAction = timerInputActionNone,
			    .mode       = timerModeUp,
			    .dmaClrAct  = false,
			    .quadModeX4 = false,
			    .oneShot    = false,
			    .sync       = false,
			  };


			  /* Select TIMER1 parameters */
			  	  TIMER_Init_TypeDef timer1Init =
			  	  {
			  	    .enable     = false,
			  	    .debugRun   = true,
			  	    .prescale   = timerPrescale1,
			  	    .clkSel     = timerClkSelCascade,
			  	    .fallAction = timerInputActionNone,
			  	    .riseAction = timerInputActionNone,
			  	    .mode       = timerModeUp,
			  	    .dmaClrAct  = false,
			  	    .quadModeX4 = false,
			  	    .oneShot    = false,
			  	    .sync       = true,
			  	  };


			  //Clear all timer0 and timer1 interrupts
			  int  IntFlags0=TIMER0->IF;
			  TIMER0->IFC=IntFlags0;

			  /* Enable TIMER0 and TIMER1 interrupt vector in NVIC */
			  NVIC_EnableIRQ(TIMER0_IRQn);
			   //set LED on time
				LETIMER0->CNT=HIGH_FREQ; // set count for 1s                //Load count for 1s in ULFRCO
		          while(LETIMER0->SYNCBUSY!=0);
				TIMER0->CNT=0x0000;  //Set Timer0 initial count to 0
				TIMER1->CNT=0x0000;  //Set Timer1 initial count to 0

			//Clear all interrupts
			        int IntFlags=LETIMER0->IF;
			        LETIMER0->IFC=IntFlags;

			//Enable clock for timers
			CMU_ClockEnable(cmuClock_LETIMER0, true); /*Enable clock for LETIMER0*/
			CMU_ClockEnable(cmuClock_TIMER0, true); /*Enable clock for TIMER0*/
			CMU_ClockEnable(cmuClock_TIMER1, true); /*Enable clock for TIMER0*/

		   /* Initialise  TIMER0,TIMER1 */
			TIMER_Init(TIMER0, &timer0Init);
			TIMER_Init(TIMER1, &timer1Init);

			//Enable timers to start count
			LETIMER_Enable(LETIMER0, true);
			 TIMER_Enable(TIMER0, true);
			 TIMER_Enable(TIMER1, true);

			 while(LETIMER0->CNT!=0x0000); // wait till count = 0
			 TIMER_Enable(TIMER0, false);
			 TIMER_Enable(TIMER1, false);
			LETIMER_Enable(LETIMER0, false);

	        //The 32 bit count is put into a variable (Timer0 = last 16 bits ; Timer1 count = first 16 bits)
	        float LFXO_COUNT=((TIMER1->CNT << 16)|(TIMER0->CNT));

	    /*Code to calculate the ULFCRO Count through TIMER0 & TIMER1*/
	        TIMER0->CNT=0x000;
	        TIMER1->CNT=0x000;

	        //Set necessary clocks for ULFRCO
	          CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);
	          CMU_OscillatorEnable(cmuOsc_LFXO,false,false);
	          LETIMER0->CNT=LOW_FREQ;                   //Load count for 1s in ULFRCO
	          while(LETIMER0->SYNCBUSY!=0);

	          //Enable clock for timers
	          CMU_ClockEnable(cmuClock_LETIMER0, true); /*Enable clock for LETIMER0*/
	          CMU_ClockEnable(cmuClock_TIMER0, true); /*Enable clock for TIMER0*/
	          CMU_ClockEnable(cmuClock_TIMER1, true); /*Enable clock for TIMER0*/


	       LETIMER_Enable(LETIMER0, true);
			 TIMER_Enable(TIMER0, true);
			 TIMER_Enable(TIMER1, true);
	       while(LETIMER0->CNT!=0x0000); // wait untill count = 0
			 TIMER_Enable(TIMER0, false);
			 TIMER_Enable(TIMER0, false);
	       LETIMER_Enable(LETIMER0, false);
	        //The 32 bit count is put into a variable (Timer0 = last 16 bits ; Timer1 count = first 16 bits)
	        float ULFRCO_COUNT=((TIMER1->CNT << 16)|(TIMER0->CNT));

	      Osc_Ratio=LFXO_COUNT/ULFRCO_COUNT;
	      CMU_ClockEnable(cmuClock_TIMER0, false); /*Enable clock for TIMER0*/
	      CMU_ClockEnable(cmuClock_TIMER1, false); /*Enable clock for TIMER0*/

}

/**************************************************************************//**
 * @brief  LETIMER_setup
 * Configures and starts the LETIMER0
 *****************************************************************************/
void LETIMER_setup(void)
{
  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit =
  {
  .enable         = false,                   /* Start counting when init completed. */
  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
  .out0Pol        = 0,                      /* Idle value for output 0. */
  .out1Pol        = 0,                      /* Idle value for output 1. */
  .ufoa0          = letimerUFOANone,         /* PWM output on output 0 */
  .ufoa1          = letimerUFOANone,       /* Pulse output on output 1*/
  .repMode        = letimerRepeatFree       /* Count until stopped */
  };

  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit);
  if (Enery_Mode == EM3)
   {
	  // As per the discussion with the proffesor, I implemented the same way he likes to implement the code
	  //Calling the ULFRCO routine from LETIMER_Setup
	  if (ULFRCO_Calibrate)
      ULFRCO_Calibration();
	  if((int)period*Osc_Ratio< period)
		  {
		  Required_Period = (period*LOW_FREQ)* ((int)Osc_Ratio +1)/Prescalar; // for ULFRCO 1kHz is the frequency, so multipled by 1000
		  }
	  else
	  Required_Period = (period*LOW_FREQ)* Osc_Ratio/Prescalar; // for ULFRCO 1kHz is the frequency, so multipled by 1000
	  if((int)on_duty*Osc_Ratio< on_duty)
	  		  {
		  On_Duty_Cycle = (on_duty*LOW_FREQ)*((int)Osc_Ratio +1)/Prescalar; // for ULFRCO 1kHz is the frequency, so multipled by 1000
	  		  }
  else
	  {
	  On_Duty_Cycle = (on_duty*LOW_FREQ)*(Osc_Ratio)/Prescalar; // for ULFRCO 1kHz is the frequency, so multipled by 1000

	  }
   }
  else
      {
	  Required_Period = (period*HIGH_FREQ)/Prescalar; // for LFA 32768Hz is the frequency, so multipled by 32768
	  On_Duty_Cycle = (on_duty*HIGH_FREQ)/Prescalar;
      }
 /* Enable underflow interrupt */
  LETIMER_CompareSet(LETIMER0,0,Required_Period); //Put Period to COMP0
  LETIMER0->IEN |= LETIMER_IF_UF;//LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
    /* Enable COMP1 interrupt */
  LETIMER_CompareSet(LETIMER0,1,On_Duty_Cycle); //Put OnDutyCycle to COMP1
  LETIMER0->IEN |= LETIMER_IF_COMP1;//LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);
  while(LETIMER0->SYNCBUSY!=0);
  blockSleepMode(Enery_Mode);
  /* Enable LETIMER0 interrupt vector in NVIC*/
  NVIC_EnableIRQ(LETIMER0_IRQn);
  }


/**************************************************************************//**
 * @brief  Setup the Analog Comparator
 *****************************************************************************/
 void setupACMP(void)
{
  /* ACMP configuration constant table. */
  ACMP_Init_TypeDef initACMP =
  {
    .fullBias = false,                 /* fullBias */
    .halfBias = true,                  /* halfBias  selecting half bias for lesser power consumption*/
    .biasProg =  0x0,                  /* biasProg =0 to get least current*/
    .interruptOnFallingEdge =  true,  /* interrupt on rising edge */
    .interruptOnRisingEdge =  true,   /* interrupt on falling edge */
    .warmTime = acmpWarmTime256,       /* 256 cycle warmup to be safe */
    .hysteresisLevel = acmpHysteresisLevel5, /* hysteresis level 5 */
    .inactiveValue = false,            /* inactive value */
    .lowPowerReferenceEnabled = true, /* low power reference selected for lesser power consumption */
    .vddLevel = (uint32_t)VDD_LEVEL,                  /* VDD level */
    .enable = false                    /* Don't request enabling. */
  };
  /* Configure ACMP. */
  ACMP_Init(ACMP0, &initACMP);
  /* Set up ACMP negSel to VDD, posSel is controlled by light sensor. */
  ACMP_ChannelSet(ACMP0, NEGATIVE_INPUT, POSITIVE_INPUT);

}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	  /* Align different chip revisions */
	  CHIP_Init();
	  CMU_setup();/* Setup CMU*/
	  circ_buff();
	  GPIO_setup();/* Setup GPIO */
	  LETIMER_setup();/* Initialize LETIMER */
	  setupACMP();
	  setupDma();
	  setupAdc();
	  initLeuart();
	  LETIMER_Enable(LETIMER0, true);
	  while(1)
	  {
		  INT_Disable();
		    if ( transferActive )
		    {
		      EMU_EnterEM1();
		    }
		    INT_Enable();

		    /* Exit the loop if transfer has completed */
		    if ( !transferActive )
		    {
		    	sleep();//Make the MCU to sleep in respective Energy Mode
		    }
	  }

}

