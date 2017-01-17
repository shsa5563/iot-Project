/*
 * main.h
 *
 *  Created on: 21-Sep-2016
 *      Author: Satyanarayana
 *      This header file includes the #define statements for DMA, LOW and HIGH temperature limits
 */

#ifndef MAIN_H_
#define MAIN_H_
#define Passive_Light_Sense 0
#define DMA_Switch true
#define LOW_TEMP_LIMIT 15
#define HIGH_TEMP_LIMIT 35
#define CircBuff 0
long count =0; // The count for number of ramBufferAdcData array elements
#define NUM_SLEEP_MODES 5 // Number of Sleep Modes
typedef enum {
    EM0 = 0,
    EM1 = 1,
    EM2 = 2,
    EM3 = 3,
    EM4 = 4
} sleepstate_enum;
#define period 6.25
#define on_duty 0.004
#define LOW_FREQ 1000
#define HIGH_FREQ 32768
int Required_Period, On_Duty_Cycle;
int Prescalar =0;
typedef enum {
    On = 1,
    Off =0
} Led_Status;

#define NEGATIVE_INPUT acmpChannelVDD
#define POSITIVE_INPUT acmpChannel6
#define off 0
#define on 1
#define LED_Port gpioPortE
#define LED0_Pin 2
#define LED1_Pin 3
#define Enery_Mode EM3

#define LIGHTSENSE_EXCITE_PORT   gpioPortD
#define LIGHTSENSE_EXCITE_PIN    6U
#define LIGHTSENSE_SENSOR_PORT   gpioPortC
#define LIGHTSENSE_SENSOR_PIN    6U

#define LOWER_LIMIT 0x02
#define UPPER_LIMIT 0x3D
uint32_t VDD_LEVEL = LOWER_LIMIT;
#define ULFRCO_Calibrate  true
uint32_t sleep_block_counter[NUM_SLEEP_MODES] = {0};

float Osc_Ratio = 1;
float temp ;
int8_t temp1;

char _leuart_status;
char _light_status;
#define Baud_Rate 9600

#define DMA_CHANNEL_ADC       0
/* ADC Transfer Data */
#define ADCSAMPLES                        500
volatile uint16_t ramBufferAdcData[ADCSAMPLES];

/* DMA callback structure */
DMA_CB_TypeDef cb;

/* Temperature #Define statements*/
#define CAL_Value DEVINFO->CAL
#define CAL_Value_Mask  _DEVINFO_CAL_TEMP_MASK //0x00FF0000UL
#define SHIFT_TEMP  _DEVINFO_CAL_TEMP_SHIFT //16
#define ADC_CAL_VALUE DEVINFO->ADC0CAL2
#define ADC_CAL_MASK  _DEVINFO_ADC0CAL2_TEMP1V25_MASK//0xFFF00000UL
#define SHIFT_TEMP_V125 _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT //20
#define Clear_VDD_Ref 0xFFFFC0FF
#define SET_VDD_Ref_high 0x00003D00
#define SET_VDD_Ref_Low 0x00000200
uint32_t period_count =1;
/* Transfer Flag */
volatile bool transferActive;

#define EXT_PORT_C gpioPortC
#define EXT_PORT_D gpioPortD
#define EXT_PIN_5 5
#define EXT_PIN_4 4
#define EXT_PIN_INT 1
#define EXT_PIN_POWER 0
#define MODE_WIRE gpioModeWiredAnd
#define Pre_Scalar_ADC 1300000//975000
#endif /* MAIN_H_ */
