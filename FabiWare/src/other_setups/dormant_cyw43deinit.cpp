// #include <Arduino.h>
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
    
//             // Close Serial before entering dormant
//             Serial.flush(); delay(10);
//             Serial.end(); delay(500);

//             // Disable Module for Wireless functionalities
//             cyw43_arch_deinit();

//             pinMode(BUTTON, INPUT_PULLUP);
    
//             // Prepare for DORMANT
//             sleep_run_from_lposc();
    
//             hw_set_bits(&powman_hw->vreg_ctrl, POWMAN_PASSWORD_BITS | POWMAN_VREG_CTRL_UNLOCK_BITS);
//             // Wait for any prior change to finish before making a new change
//             while (powman_hw->vreg & POWMAN_VREG_UPDATE_IN_PROGRESS_BITS)
//                 tight_loop_contents();
//             hw_set_bits(&powman_hw->bod, ~(0x00000001)); // disable brownout detection

//             // Enter DORMANT (wait for button press to wake; drive pin to LOW to wake)
//             int8_t gpios[] = {4};
//             sleep_goto_dormant_until_edge_high(gpios, 1);

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
//         }
//     }
// }

// void setup1() {
// }

// void loop1() {
//     core1dormant();
// }

// // -----------------------------------------------------------------

// // DORMANT Mode, cyw43 disabled
// // Current Consumption; 4.2V; 30s;

// // As the FABI module needs both cores to run all of it's functionalities and for less complications
// // during further development, only both core operation will be tested and measured.

// // The reason why the uController on the PCB draws more current than the standalone Pico2W presumably is
// // because of the various resistances that lie in the path (e.g. p-CH MOSFET).

// // Directly hooking up the Pico2W on the PCB via VSYS pin, reduces the current consumption by about 100 uA.

// // -----------------------------------------------------------------
// // Both Cores

// // Pico2W as standalone uC:
// //    - 1: 416.11 uA, max. 219.54 mA, 12.48 mC (CHARGE)
// //    - 2: 396.04 uA, max. 208.69 mA, 11.88 mC (CHARGE)
// //    - 3: 396.79 uA, max. 212.56 mA, 11.90 mC (CHARGE)
// //    - 4: 398.13 uA, max. 214.11 mA, 11.94 mC (CHARGE)
// //    - 5: 406.45 uA, max. 213.34 mA, 12.19 mC (CHARGE)
// // Averages: 402.704 uA, max. 213,648 mA, 12.078 mC

// // Pico2W on custom PCB:
// //    - 1: 672.40 uA, max. 76.27 mA, 20.17 mC (CHARGE)
// //    - 2: 666.06 uA, max. 76.27 mA, 19.98 mC (CHARGE)
// //    - 3: 661.73 uA, max. 78.54 mA, 19.85 mC (CHARGE)
// //    - 4: 660.13 uA, max. 79.30 mA, 19.80 mC (CHARGE)
// //    - 5: 655.54 uA, max. 77.78 mA, 19.67 mC (CHARGE)
// // Averages: 663.172 uA, max. 77.632 mA, 19.894 mC