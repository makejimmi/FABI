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
  
  static uint16_t battStateCounter, battStatusSum, amountOfReads, battPercentSum;
  static int8_t prevBattValue = -2;
  
  if(battStateCounter >= battRefreshRate) {
    
    int8_t battValue;
    
    if(battStatusSum != battStateCounter){
      switch(battStatusSum){
        case (uint16_t) (battRefreshRate*0.8):        // CHARGING
          battValue = 127; break;
        default:                                      // USB - no battery
          battValue = -1; break; 
      }
    } else {
      battValue = battPercentSum/amountOfReads; //slotSettings.bt = 2; // BT
    }
    
    if(battValue != prevBattValue){ // statement to prevent display from refreshing too often
      sensorData.currentBattPercent = battValue;
      displayUpdate();
      prevBattValue = battValue;
    }
    
    amountOfReads = 0; battStateCounter = 0;
    battPercentSum = 0; battStatusSum = 0;
  }
  
  switch(battStateCounter++%10){
   case 0: gpio_disable_pulls(CHARGE_STAT); gpio_pull_up(CHARGE_STAT); 
    battStatusSum++; break;
   case 4:
    if(gpio_get(CHARGE_STAT)){
      battStatusSum++; battPercentSum += readPercentage(); amountOfReads++;
    } break;
   case 5: gpio_disable_pulls(CHARGE_STAT); gpio_pull_down(CHARGE_STAT); 
    battStatusSum++; break;
   case 9:
    if(gpio_get(CHARGE_STAT)){
      battStatusSum++; battPercentSum += readPercentage(); amountOfReads++;
    } break;
   default: battStatusSum++; break;
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
    // 518 = 3.2V (0%)  |  682 = 4.2V (100%)
    return map(analogRead(V_BATT_MEASURE), 518, 682, 0, 100);
}

/**
 * @name initPowerSave
 * @brief Initializes power-saving mechanisms and prepares the system for battery operation.
 * @return none
 */
void initPowerSave() {
    gpio_init(CHARGE_STAT);
    //enableBattMeasure();
    enable3V3();
    //inactivityDetector();
}

/**
 * @name deinitPowerSave
 * @brief Placeholder for deinitializing power-saving mechanisms.
 * @return none
 */
void deinitPowerSave() {
    disable3V3();
    disableBattMeasure();
}

/**
 * @name disableBattMeasure
 * @brief Disables battery measurement circuitry.
 * @return none
 */
void disableBattMeasure() {
    gpio_set_dir(V_BATT_VD_SWITCH, true);
    gpio_set_dir(V_BATT_VD_SWITCH, true);
}

/**
 * @name enableBattMeasure
 * @brief Enables battery measurement circuitry.
 * @return none
 */
void enableBattMeasure() {
    gpio_init(CHARGE_STAT);
    gpio_init(V_BATT_VD_SWITCH);
    gpio_set_dir(V_BATT_VD_SWITCH, false);
    gpio_init(V_BATT_VD_SWITCH);
    gpio_set_dir(V_BATT_VD_SWITCH, false);
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
    delay(500);
}

/**
 * @name inactivityHandler
 * @brief Handles user inactivity by transitioning the device to dormant mode.
 * @return none
 */
void inactivityHandler() {
    displayClear();
    deinitPowerSave();
    sensorData.currentBattPercent = 0;
    dormantUntilInterrupt(dormantInterruptPin);
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
