/*
    FLipWare - AsTeRICS Foundation
    Copyright (c) Benjamin Aigner
    For more info please visit: https://www.asterics-foundation.org

    Module: lpwFuncs.h - Header file for battery and power management functions

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; See the GNU General Public License:
    http://www.gnu.org/licenses/gpl-3.0.en.html
*/

#ifndef LPW_FUNCS_H
#define LPW_FUNCS_H

#include "hardware/xosc.h"
#include "hardware/uart.h"
#include "powermanagement/sleep.h"
#include "FlipWare.h"
#include "display.h"
#include "sleep.h"
#include <pico/types.h>

/**
 * @name Pin Definitions
 * @brief Constants defining GPIO pin assignments for various functionalities.
 */
#define V_BATT_VD_SWITCH 0
#define V_BATT_MEASURE 28
#define CHARGE_STAT 1
#define EN3V3 7
#define dormantInterruptPin 4   ///< GPIO11, interrupt for dormant state

#define battRefreshRate 1000 // in ms; ONLY IN MULTIPLES OF TEN

// THERE ARE CHANGED PINS THAT NEED TO BE REDEFINED, see FABI schematic

/**
 * @name Time Definitions
 * @brief Constants for inactivity time configurations.
 */
#define inactivityTimeMinutes 0
#define inactivityTimeSeconds 10

/**
 * @name Power Rail Functions
 */
void enable3V3();
void disable3V3();

/**
 * @name Battery Management Functions
 */
void checkBMSStat();
void initPowerSave();
void deinitPowerSave();
uint16_t readPercentage();

/**
 * @name Power Source Functions
 */
void enableBattMeasure();
void disableBattMeasure();

/**
 * @name Inactivity Management Functions
 */
void inactivityDetector();
void disableInactivityDetector();
void inactivityHandler();
void dormantUntilInterrupt(int interruptPin);
void userInterrupt();

/**
 * @name Utility Functions
 */
timespec newTimer(uint8_t minutes, long long seconds, long nanoseconds);

#endif // LPW_FUNCS_H
