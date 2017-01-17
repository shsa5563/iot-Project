/*
 * TSL2561.h
 *
 *  Created on: 03-Oct-2016
 *      Author: Satyanarayana
 */

#ifndef TSL2561_H_
#define TSL2561_H_
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_i2c.h"
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "em_int.h"

#define I2C_ADDRESS                     0x39
#define Write 0x00
#define Read 0x01
#define Command_Reg 0x80
#define power_LightSensor 0x03
#define timing_Reg 0x01
#define low_gain 0x00
#define integration_time 0x01
#define interrupt_cntrl_reg 0x06
#define interrupt_control 0x01
#define interrupt_clear 0x04
#define persist 0x04
#define threshlowlow_reg 0x02
#define threshlowhigh_reg 0x03
#define threshhighlow_reg 0x04
#define threshhighhigh_reg 0x05
#define threshlowlow 0x0f
#define threshlowhigh 0x00
#define threshhighlow 0x00
#define threshhighhigh 0x08
#define word_mode 0x01


#endif /* TSL2561_H_ */
