/*
 * structs.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#ifndef STRUCTS_H
#define STRUCTS_H

#include <stdint.h>

#include "adc.h"
#include "calibration.h"
#include "can.h"
#include "drv8323.h"
#include "foc.h"
#include "fsm.h"
#include "gpio.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "spi.h"
#include "tim.h"

typedef struct {
} GPIOStruct;

typedef struct {
} COMStruct;

/* Global Structs */
extern ControllerStruct controller;
extern ObserverStruct observer;
extern COMStruct com;
extern FSMStruct state;
extern EncoderStruct comm_encoder;
extern DRVStruct drv;
extern PreferenceWriter prefs;
extern CalStruct comm_encoder_cal;
extern CANTxMessage can_tx;
extern CANRxMessage can_rx;

#endif
