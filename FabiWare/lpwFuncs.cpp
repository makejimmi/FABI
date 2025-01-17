#include "lpwFuncs.h"

/**
 * @name checkBMSStat
 * @brief Monitors battery management system (BMS) status and updates battery data.
 * @return none
 */

/**
 * Function is able to calculate % of battery but only if not started by battery but
 * by USB. Patches underway...
*/
void checkBMSStat() {
  if(sensorData.battStateCounter >= 100) {
    uint16_t check = sensorData.battPercentSum/sensorData.battStatusSum;
    if( sensorData.currentBattPercent != check ){
      sensorData.currentBattPercent = sensorData.battPercentSum/sensorData.battStatusSum;
    } 
    displayUpdate();
    sensorData.battPercentSum = 0;
    sensorData.battStateCounter = 0;
    sensorData.battStatusSum = 0;
  }
  switch(sensorData.battStateCounter++%2){
   case 0: gpio_disable_pulls(CHARGE_STAT); gpio_pull_up(CHARGE_STAT); 
    if(gpio_get(CHARGE_STAT)){
      sensorData.battStatusSum++; sensorData.battPercentSum += readPercentage();
    } break;
   case 1: gpio_disable_pulls(CHARGE_STAT); gpio_pull_down(CHARGE_STAT); 
    if(gpio_get(CHARGE_STAT)){
      sensorData.battStatusSum++; sensorData.battPercentSum += readPercentage();
    } break;
  }
}

/**
 * @name enable3V3
 * @brief Enables the 3.3V power rail.
 * @return none
 */
void enable3V3() {
    pinMode(EN3V3, OUTPUT);
    digitalWrite(EN3V3, HIGH);
}

/**
 * @name disable3V3
 * @brief Disables the 3.3V power rail.
 * @return none
 */
void disable3V3() {
    digitalWrite(EN3V3, LOW);
    pinMode(EN3V3, INPUT);
}

/**
 * @name readPercentage
 * @brief Reads and calculates the battery percentage from the ADC input.
 * @return uint16_t Battery percentage (0-100%).
 */
uint16_t readPercentage() {
    // The voltage divider reduces battery voltage by 50% for ADC measurement.
    // A fully charged battery (4.2V) corresponds to ~651 ADC value (63.63% of 1024).
    // Mapping ADC value (518(==3.2V)-640) to battery percentage (0-100%).
    return map(analogRead(V_BATT_MEASURE), 518, 672, 0, 100);
    //return analogRead(V_BATT_MEASURE);
}

/**
 * @name initPowerSave
 * @brief Initializes power-saving mechanisms and prepares the system for battery operation.
 * @return none
 */
void initPowerSave() {
    gpio_init(CHARGE_STAT);
    gpio_init(V_BATT_VD_SWITCH);
    gpio_set_dir(V_BATT_VD_SWITCH, false);
    gpio_init(V_BATT_VD_SWITCH);
    gpio_set_dir(V_BATT_VD_SWITCH, false);
    //enableBattMeasure();
    enable3V3();
    //inactivityDetector();
}

/**
 * @name deinitPowerSave
 * @brief Placeholder for deinitializing power-saving mechanisms.
 * @return none
 */
void deinitPowerSave() {}

/**
 * @name disableBattMeasure
 * @brief Disables battery measurement circuitry.
 * @return none
 */
void disableBattMeasure() {
    pinMode(V_BATT_VD_SWITCH, OUTPUT);
    digitalWrite(V_BATT_VD_SWITCH, LOW);
    pinMode(V_BATT_MEASURE, OUTPUT);
    digitalWrite(V_BATT_MEASURE, LOW);
}

/**
 * @name enableBattMeasure
 * @brief Enables battery measurement circuitry.
 * @return none
 */
void enableBattMeasure() {
    pinMode(V_BATT_VD_SWITCH, INPUT);
    pinMode(V_BATT_MEASURE, INPUT);
}

/**
 * @name inactivityDetector
 * @brief Sets up the AON (Always On) timer to detect user inactivity and trigger power-saving modes.
 * @param state Current device state.
 * @return none
 */
void inactivityDetector() {
    if (aon_timer_is_running()) {
        aon_timer_stop();
        aon_timer_disable_alarm();
    }

    timespec inactivityTime = newTimer(inactivityTimeMinutes, inactivityTimeSeconds, 0);
    timespec startingTime = newTimer(0, 0, 0);

    aon_timer_enable_alarm(&inactivityTime, inactivityHandler, false);
    aon_timer_start(&startingTime);
}

void disableInactivityDetector(){  
    if (aon_timer_is_running()) {
        aon_timer_stop();
        aon_timer_disable_alarm();
    }
}

/**
 * @name dormantUntilInterrupt
 * @brief Puts the device into dormant mode until an interrupt is triggered.
 * @param interruptPin GPIO pin used for waking up the device.
 * @return none
 */
void dormantUntilInterrupt(int interruptPin) {
    sleep_run_from_lposc(); // Runs the low-power oscillator for minimal power usage (RP2350 ONLY).
    // sleep_run_from_xosc(); // Runs the crystal oscillator (RP2040 & RP2350)
    sleep_goto_dormant_until_edge_high(interruptPin);
    sleep_power_up();
}

/**
 * @name inactivityHandler
 * @brief Handles user inactivity by transitioning the device to dormant mode.
 * @return none
 */
void inactivityHandler() {
    disableBattMeasure();
    disable3V3();
    dormantUntilInterrupt(dormantInterruptPin);
    delay(1000);
    setup();
}

/**
 * @name userInterrupt
 * @brief Handles user interaction interrupts and resets inactivity detection.
 * @return none
 */
void userInterrupt() {
    inactivityDetector();
}

/**
 * @name newTimer
 * @brief Creates a new timer struct with the given time parameters.
 * @param minutes Timer duration in minutes.
 * @param seconds Timer duration in seconds.
 * @param nanoseconds Timer duration in nanoseconds.
 * @return timespec Timer struct.
 */
timespec newTimer(uint8_t minutes, long long seconds, long nanoseconds) {
    timespec ts; timespec nullTs;

    uint8_t _minutes = (minutes > 0 && minutes <= 256) ? minutes * 60 : 0;
    long long _seconds = (seconds >= LONG_LONG_MIN && seconds <= LONG_LONG_MAX) ? seconds + _minutes : _minutes;
    long _nanoseconds = (nanoseconds >= LONG_MIN && nanoseconds <= LONG_MAX) ? nanoseconds : 0;

    ts.tv_sec = _seconds;
    ts.tv_nsec = _nanoseconds;
    return ts;
}
