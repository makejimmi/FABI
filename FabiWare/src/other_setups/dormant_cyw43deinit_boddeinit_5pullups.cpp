#include <Arduino.h>
// #include "hardware/vreg.h"

// extern "C"{
//     #include "sleep.h"
// }
// #include <pico/cyw43_arch.h>
// #include "hardware/structs/usb.h"
// #include "pico/multicore.h"

// // #define STANDALONE
// #define PCB
// #define LED LED_BUILTIN
// #define CYW43_WL_GPIO_VBUS_PIN 2
// #define PHYSICAL_BUTTON_MAP {11, 9, 8, 4, 3}

// #ifdef STANDALONE
//     #define BUTTON BOOTSEL
// #endif
// #ifdef PCB
//     #define BUTTON 4
// #endif

// static bool prepareDormant = false;

// void core1dormant() {
//     if (prepareDormant) {
//         // deinit stuff
//         while(prepareDormant);
//         // init stuff
//     }
// }

// void setup()
// {   
//     pinMode(LED, OUTPUT);
//     digitalWrite(LED, HIGH);
//     pinMode(BUTTON, INPUT_PULLUP);
//     Serial.begin(115200);
//     delay(3000); Serial.println("Started core0");
// } 

// void loop()
// {
//     // Active mode: check for button press to re-enter dormant
//     if (digitalRead(BUTTON) == LOW)
//     {
//         delay(80); // Simple debounce
        
//         // Wait for button release to prevent multiple triggers
//         if (digitalRead(BUTTON) == LOW) {
//             while (digitalRead(BUTTON) == LOW) delay(10);
//             prepareDormant = true;

//             // Inform that PICO is going DORMANT
//             Serial.println("Going to sleep");
//             digitalWrite(LED, LOW); pinMode(LED, INPUT);
//             pinMode(BUTTON, INPUT);

//             // Close Serial before entering dormant
//             Serial.flush(); delay(10);
//             Serial.end(); delay(500);

//             // Disable Module for Wireless functionalities
//             cyw43_arch_deinit();
    
//             // Prepare for DORMANT
//             sleep_run_from_lposc();
    
//             hw_set_bits(&powman_hw->vreg_ctrl, POWMAN_PASSWORD_BITS | POWMAN_VREG_CTRL_UNLOCK_BITS);
//             // Wait for any prior change to finish before making a new change
//             while (powman_hw->vreg & POWMAN_VREG_UPDATE_IN_PROGRESS_BITS)
//                 tight_loop_contents();
//             hw_set_bits(&powman_hw->bod, ~(0x00000001)); // disable brownout detection

//             // Enter DORMANT (wait for button press to wake; drive pin to LOW to wake)
//             int8_t gpios[5] = PHYSICAL_BUTTON_MAP;
//             sleep_goto_dormant_until_edge_high(gpios, 5); // wait for rising edge interrupt

//             hw_set_bits(&powman_hw->vreg_ctrl, POWMAN_PASSWORD_BITS | POWMAN_VREG_CTRL_UNLOCK_BITS);
//             // Wait for any prior change to finish before making a new change
//             while (powman_hw->vreg & POWMAN_VREG_UPDATE_IN_PROGRESS_BITS)
//                 tight_loop_contents();
//             hw_set_bits(&powman_hw->bod, 0x00000001); // enable brownout detection
    
//             // Wake up
//             sleep_power_up(); delay(400);
            
//             cyw43_arch_init();
//             delay(100);
//             pinMode(LED, OUTPUT); digitalWrite(LED, HIGH);
//             pinMode(BUTTON, INPUT_PULLUP);

//             // Reopen Serial after waking
//             Serial.begin(115200); delay(2000); // Allow time for Serial initialization
//             Serial.println("Good morning");
//             prepareDormant = false;
//         }
//     }
// }

// void setup1() {
// }

// void loop1() {
//     core1dormant();
// }

// // -----------------------------------------------------------------

// // DORMANT Mode, cyw43 disabled, brownout detection disabled
// // Current Consumption; 4.2V; 30s;

// // As the FABI module needs both cores to run all of it's functionalities and for less complications
// // during further development, only both core operation will be tested and measured.

// // The reason why the uController on the PCB draws more current than the standalone Pico2W presumably is
// // because of the various resistances that lie in the path (e.g. p-CH MOSFET).

// // Directly hooking up the Pico2W on the PCB via VSYS pin on 4.2V, 
// // reduces the current consumption by aprox. 100 uA.

// // All Button Pins ought to be able to wake up the
// // uController -> 5 Pull Ups instead of 1 = higher current consumption

// // -----------------------------------------------------------------
// // Both Cores

// // Pico2W as standalone uC:
// //    - 1: 415.28 uA, max. 213.34 mA, 12.46 mC (CHARGE)
// //    - 2: 405.32 uA, max. 207.91 mA, 12.16 mC (CHARGE)
// //    - 3: 406.14 uA, max. 208.69 mA, 12.18 mC (CHARGE)
// //    - 4: 401.97 uA, max. 211.01 mA, 12.06 mC (CHARGE)
// //    - 5: 408.94 uA, max. 209.46 mA, 12.27 mC (CHARGE)
// // Averages: 407.534 uA, max. 210.082 mA, 12.226 mC

// // Pico2W on custom PCB:
// //    - 1: 715.99 uA, max. 83.84 mA, 21.48 mC (CHARGE)
// //    - 2: 716.00 uA, max. 82.33 mA, 21.48 mC (CHARGE)
// //    - 3: 712.73 uA, max. 82.33 mA, 21.38 mC (CHARGE)
// //    - 4: 716.47 uA, max. 81.57 mA, 21.49 mC (CHARGE)
// //    - 5: 713.21 uA, max. 82.33 mA, 21.40 mC (CHARGE)
// // Averages: 714.88 uA, max. 82.48 mA, 21.446 mC