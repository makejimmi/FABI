/*
     FabiWare - AsTeRICS Foundation
     For more info please visit: https://www.asterics-foundation.org

    Module: lpwFuncs.cpp - C module for battery and power management functions

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; See the GNU General Public License:
   http://www.gnu.org/licenses/gpl-3.0.en.html

*/

#include "FlipWare.h"
#include "gpio.h"
#include "display.h"
#include "pico/cyw43_arch.h"

#include "hardware/pwm.h"
#include "hardware/vreg.h"

#include "tone.h"
#include "keys.h"
#include "sensors.h"

extern "C" {
  #include "../lib/power/sleep.h"
}

unsigned long inactivityTime=0;  // measures user inactivity (in ms)
static bool core1dormantSwitch = false;

/**
 * @name detectUSB
 * @brief Reads the state of defined CYW43 VBUS Pin (2)
 * @return true : USB Connected (VBUS Presence detected)
 *         false : USB Disconnected (VBUS Presence not detected)
 */
bool detectUSB(){
  sensorData.usbConnected = cyw43_arch_gpio_get(CYW43_WL_GPIO_VBUS_PIN);
  static bool prevValue = !sensorData.usbConnected;
  
  if (prevValue != sensorData.usbConnected){
    #ifdef DEBUG_BATTERY_MANAGEMENT
     if (sensorData.usbConnected) Serial.println("USB Connected");
     else Serial.println("USB Disconnected");  // note that this only makes sense if Serial connected still present!
    #endif
    prevValue = sensorData.usbConnected;
  }
  return sensorData.usbConnected;
}

/**
 * @name batteryPresenceDetector
 * @brief Uses a state-machine to check for the battery's presence.
 *        There are three modes that need to be differentiated: 
 *          a. HIGH Z : Shutdown, No battery present 
 *          b. LOW : Charging (Preconditioning, Constant-Current Fast Charge, Constant Voltage) 
 *          c. HIGH : Charge Complete, Standby
 * @return MCPSTAT_HIGHZ : No battery is present / MCP is in shutdown.
 *         MCPSTAT_LOW   : The battery is present, preconditioning / charging.
 *         MCPSTAT_HIGH  : The battery is present, fully charged - standby mode.
 *         MCPSTAT_UNDEFINED  : Do nothing - default return value when state is being determined.
 */
uint8_t batteryPresenceDetector(){
  static uint16_t battStateCounter=0;  // determines when to pull up/down, to read
  static uint16_t  battStateSum=0;  // accumulates digital reads of the MCP Stat pin which eitehr floats, is LOW or HIGH.
  static uint8_t state = MCPSTAT_UNDEFINED; 

  switch (battStateCounter++) {
    case 0:
      battStateSum=0;  // reset accumulator variable
      gpio_disable_pulls(CHARGE_STAT_PIN); gpio_pull_up(CHARGE_STAT_PIN); break;
    case 1:
      battStateSum += gpio_get(CHARGE_STAT_PIN); break;
    case 2:
      gpio_disable_pulls(CHARGE_STAT_PIN); gpio_pull_down(CHARGE_STAT_PIN); break;
    case 3:
      battStateSum += gpio_get(CHARGE_STAT_PIN); 

      // now we can determine the battery state:
      if (battStateSum == 0) {                           // battery is charging
        state = MCPSTAT_LOW;
        #ifdef DEBUG_BATTERY_MANAGEMENT
          Serial.println("LOW   :\tBattery is Charging"); 
        #endif
      }
      else if (battStateSum == 2) {                    // charging complete or standby
        state = MCPSTAT_HIGH;
        #ifdef DEBUG_BATTERY_MANAGEMENT
          Serial.println("HIGH  :\tBattery Charge Completed"); 
        #endif
      } 
      else {
        state = MCPSTAT_HIGHZ;     // shutdown or no battery present
        #ifdef DEBUG_BATTERY_MANAGEMENT
          Serial.println("HIGH Z:\tBattery not present or in shutdown Mode");
        #endif
      }
      battStateCounter=0; // reset state counter!
    break;
  }

  sensorData.MCPSTAT=state;
  return state;
}

/**
 * @name getBatteryPercentage
 * @brief returns an averages battery level (averaging count is defined by the constant BATTERY_AVERAGING).
 * @return 0 - 100  : The average of the battery-% read over last n reads.
 *         -1       : Undefined battery level (no battery present / MCP Stat is HighZ).
 */
int8_t getBatteryPercentage(){
  static int battSum=0, battReadCounter=0, result=0;
  if(sensorData.MCPSTAT == MCPSTAT_HIGHZ) return (-1);
  battReadCounter++;
  battSum += readPercentage();
  if (battReadCounter >= BATTERY_AVERAGING ){ 
    result = battSum / BATTERY_AVERAGING;
    battSum = 0; battReadCounter = 0;
  }
  return result;
}

/**
 * @name readPercentage
 * @brief Reads and calculates the battery percentage using ADC input.
 * @return uint16_t Battery percentage (0-100%).
 */
uint16_t readPercentage() {
  int16_t value = map(analogRead(V_BATT_MEASURE_PIN), 518, 682, 0, 100);   // map ADC value to battery percentage range
  return constrain(value, 0, 100); 
}

/**
 * @name performBatteryManagement
 * @brief called periodically, to update battery status
 * @return none
 */
void performBatteryManagement()  {
  detectUSB();
  batteryPresenceDetector();
  sensorData.currentBattPercent = getBatteryPercentage();
  if (!isDisplayAvailable()) batteryDisplay();
  #ifdef DEBUG_BATTERY_MANAGEMENT
    Serial.println("Battery level="+String(sensorData.currentBattPercent));
  #endif

  // check user inactivity, possibly initiate power save mode
  if (!sensorData.usbConnected) {
    inactivityTime += BATTERY_UPDATE_INTERVAL;
    if (inactivityTime >= (inactivityTimeMinutes*60000 + inactivityTimeSeconds*1000))
      inactivityHandler();  // time to go to sleep...
  } else inactivityTime=0;
}

/**
 * @name enable3V3
 * @brief Enables the 3.3V power rail for peripherals like LCD, Neopixel or external sensors
 */
void enable3V3() {
  gpio_init(LDO_ENABLE_PIN);
  gpio_set_dir(LDO_ENABLE_PIN, true);
  gpio_put(LDO_ENABLE_PIN, true);
}

/**
 * @name disable3V3
 * @brief Disables the 3.3V power rail
 */
void disable3V3() {
  gpio_put(LDO_ENABLE_PIN, false);
  gpio_set_dir(LDO_ENABLE_PIN, false);
  // gpio_deinit(LDO_ENABLE_PIN);
}

/**
 * @name initBattery
 * @brief Initializes battery-related power-saving mechanisms and peripherals.
 */
void initBattery() {
  detectUSB();
  gpio_init(CHARGE_STAT_PIN);
  enableBattMeasure();
}

/**
 * @name deinitBattery
 * @brief Deinitializes battery-related peripherals.
 */
void deinitBattery() {
  gpio_deinit(CHARGE_STAT_PIN);
  disableBattMeasure();
}

/**
 * @name disableBattMeasure
 * @brief Disables the battery measurement circuitry.
 */
void disableBattMeasure() {
  gpio_set_dir(V_BATT_MEASURE_PIN, false);
  gpio_deinit(V_BATT_MEASURE_PIN);
  gpio_disable_pulls(V_BATT_MEASURE_PIN);
}

/**
 * @name enableBattMeasure
 * @brief Enables the battery measurement circuitry.
 */
void enableBattMeasure() {
  gpio_init(CHARGE_STAT_PIN);
  gpio_init(V_BATT_VD_SWITCH_PIN);
  gpio_set_dir(V_BATT_VD_SWITCH_PIN, false);
  gpio_init(V_BATT_MEASURE_PIN);
  gpio_set_dir(V_BATT_MEASURE_PIN, false);
}

/**
 * @name core1dormantManager
 * @brief Manages core1 when preparing for / getting out of dormant mode.
 * @todo eliminate the need for a global variable, perhaps by adding it (global var) into slotSettings struct...
 */
void core1dormantManager(bool core0signal) {
  
  if (core1dormantSwitch && !core0signal) {
      // deinit stuff
      Wire1.flush(); 
      Wire1.endTransmission();        // I2C Sensors (NAUT, DPS)
      Wire1.end(); 

      core1dormantSwitch = false; // preparation done

      while(!core1dormantSwitch); // wait for signal from core0
      
      // enable Wire1 I2C interface (used by Core1 for sensors)
      #ifndef FLIPMOUSE
        Wire1.setSDA(PIN_WIRE1_SDA_);
        Wire1.setSCL(PIN_WIRE1_SCL_);
      #endif
      Wire1.begin();
      Wire1.setClock(400000);  // use 400kHz I2C clock

      #ifdef DEBUG_ACTIVITY_LED
        pinMode(LED_BUILTIN,OUTPUT);
      #endif

      initSensors();
      if (getForceSensorType()==FORCE_NAU7802)
        setSensorBoard(slotSettings.sb); // apply sensorboard settings
      core1dormantSwitch = false;
  }

  if (core0signal) {
      core1dormantSwitch = true;
      displayMessage((char*) "1");
      while(core1dormantSwitch);
  }
}

/**
 * @name dormantUntilInterrupt
 * @brief Puts the device into dormant mode until a specified GPIO interrupt wakes it up.
 * @param interruptPin GPIO pin to monitor for the interrupt.
 */
void dormantUntilInterrupt(int8_t *wake_interrupt_gpios, int8_t amt_gpios) {
  sleep_run_from_lposc(); // use low-power oscillator for minimal power consumption

  hw_set_bits(&powman_hw->vreg_ctrl, POWMAN_PASSWORD_BITS | POWMAN_VREG_CTRL_UNLOCK_BITS);
  // Wait for any prior change to finish before making a new change
  while (powman_hw->vreg & POWMAN_VREG_UPDATE_IN_PROGRESS_BITS)
      tight_loop_contents();
  hw_set_bits(&powman_hw->bod, ~(0x00000001)); // disable brownout detection

  // Enter deep sleep --------------------------------------------------------
  sleep_goto_dormant_until_edge_high(wake_interrupt_gpios, amt_gpios);  // Wait for rising edge

  // Post-wake restoration ---------------------------------------------------
  hw_set_bits(&powman_hw->vreg_ctrl, POWMAN_PASSWORD_BITS | POWMAN_VREG_CTRL_UNLOCK_BITS);
  // Wait for any prior change to finish before making a new change
  while (powman_hw->vreg & POWMAN_VREG_UPDATE_IN_PROGRESS_BITS)
      tight_loop_contents();
  hw_set_bits(&powman_hw->bod, 0x00000001); // enable brownout detection

  sleep_power_up(); // restore sys clocks after waking up (using rosc -> jump starts processor)
  delay(400); // allow some time for system to stabilize after restoring sys clocks
}

/**
 * @name deinitDormant
 * @brief Restores full system operation after dormant mode.
 * @details Reinits: CYW43, Peripherals, Comm. protocols, Core1 sensor processing.
 * @todo implement core1dormantManager, load saved slot setting
 */
void deinitDormant() {
  cyw43_arch_init();      // WiFi / BT chip CYW43
  app_alarm_pool = alarm_pool_create(2, 64);
  enable3V3();            // power raail
  delay(50);
  Serial.begin(115200);
  #ifdef DEBUG_DELAY_STARTUP
    delay(3000);
  #endif

  initBattery();
  
  #ifdef FLIPMOUSE
    // TODO: FlipMouse specific initializations?
  #else
    Wire.setSDA(PIN_WIRE0_SDA_); 
    Wire.setSCL(PIN_WIRE0_SCL_);
  #endif
  Wire.begin();

  initGPIO();
  initIR();
  initButtons();
  initDebouncers();
  initStorage();
  initAudio();

  readFromEEPROMSlotNumber(0, false);     // load default slot settings
  
  #ifndef FLIPMOUSE
    MouseBLE.begin(moduleName);
    KeyboardBLE.begin("");
    #ifdef FABIJOYSTICK_ENABLED
      JoystickBLE.begin("");
    #endif
  #endif
  setKeyboardLayout(slotSettings.kbdLayout);

  initDisplay(); delay(10);
  if (isDisplayAvailable()) displayUpdate();

  core1dormantManager(true);

  userActivity();
  makeTone(TONE_STARTUP,0);  // announce readyness!
}

/**
 * @name initDormant
 * @brief Prepares system for dormant mode.
 * @todo Have core1 take over init / deinit of Wire1 using core1dormantManager()
 * @note GPIO map for high power consuming components
 *        SWCP  : NEOPIXEL (10);                                                          | 10
 *        PWM   : BUZZER (2), Audio_Signal (6), IR_LED (14)                               | 2, 6, 14
 *        ADC   : EXT1 (26) : EXT2 (27), V_BATT_MEASURE (28)                              | 26, 27, 28
 *        SIO   :                                                                         | 0, 1, 3, 4, 7, 8, 9, 11, 15, 16 
 *        I2C   : MPRLS Sensors (GPIO5), SDA_INT (12) : SCL_INT (13), SDA (18) : SCL (19) | 5, 12, 13, 18, 19
 */
void initDormant() {
  #ifdef DEBUG_BATTERY_MANAGEMENT
    Serial.println("Preparing for dormant mode...");
  #endif

  // Misc
  globalSettings.buzzerMode = 0;  // disable, 1 = only height, 2 = height and count, ...
  globalSettings.audioVolume = 0; // deactivate audio output

  #ifdef AUDIO_SIGNAL_PIN
    pwm_set_enabled(pwm_gpio_to_slice_num(AUDIO_SIGNAL_PIN), false);
  #endif
  #ifdef TONE_PIN
    pwm_set_enabled(pwm_gpio_to_slice_num(TONE_PIN), false);
  #endif

  // stop running processes ----------------------------
  pauseDisplayUpdates(1);            // pause display updates
  alarm_pool_destroy(app_alarm_pool);

  #ifdef IR_LED_PIN
    stop_IR_command();  // stop any running ir command
  #endif

  // power down peripherals ----------------------------
  MouseBLE.end();       // does nothing when not running
  KeyboardBLE.end();
  delay(100);

  cyw43_arch_deinit();  // deinit the cyw43 module
  clearLeds();          // clear Neopixel

  // core1dormantManager(true);     // <- todo
  
  if(isDisplayAvailable()) {        // clear and disconnect OLED Display
    displayMessage((char*)"ByeBye");
    pauseDisplayUpdates(1);         // pause display updates
    delay(2000);                    // time for the user to read the message
    displayClear();                 // clear the display <- go in front of stopping I2C bus
  }

  // I2C handling, Wire0 (12, 13 - INT) and Wire1 (18, 19 - "")
  Wire1.flush();                  // move Wire1 init / deinit to core1dormantManager!!
  Wire1.endTransmission();        // I2C Sensors (NAUT, DPS)
  Wire1.end(); 

  Wire.flush(); delay(10);
  Wire.endTransmission(); delay(50);        // I2C Display
  Wire.end(); delay(100);

  // All other pins
  disablePins(); delay(50);
  Serial.end(); delay(500);
}


void inactivityHandler() {
  initDormant();
  dormantUntilInterrupt(input_map, NUMBER_OF_PHYSICAL_BUTTONS);
  watchdog_reboot(0, 0, 10);  // cause a watchdog reset to wake everything up!
  // deinitDormant();         // <- todo, in the meantime use watchdog reset
  while (1) { continue; } 
}

/**
 * @name userActivity
 * @brief Resets the inactivity counter upon user interaction.
 */
void userActivity() { // Call of this function can be found in line 181, buttons.cpp
  inactivityTime=0;   // reset the inactivity counter!
}

/**
 * @name disablePins
 * @brief Disable all pins.
 */
void disablePins() {
  for (int pin = 0; pin < 28; pin++) {
    if (pin == 23 || pin == 24 || pin == 25) continue;
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);
    digitalWrite(pin, LOW);
    delay(5);
  }
}