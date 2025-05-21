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
//             int8_t gpios[] = {4};
//             sleep_goto_dormant_until_edge_high(gpios, 1); // wait for rising edge interrupt

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

// // DORMANT Mode, cyw43 disabled, brownout detection disabled
// // Current Consumption; 4.2V; 30s;

// // As the FABI module needs both cores to run all of it's functionalities and for less complications
// // during further development, only both core operation will be tested and measured.

// // The reason why the uController on the PCB draws more current than the standalone Pico2W presumably is
// // because of the various resistances that lie in the path (e.g. p-CH MOSFET).

// // Directly hooking up the Pico2W on the PCB via VSYS pin on 4.2V, 
// // reduces the current consumption by aprox. 100 uA.

// // -----------------------------------------------------------------
// // Both Cores

// // Pico2W as standalone uC:
// //    - 1: 438.56 uA, max. 223.42 mA, 13.16 mC (CHARGE)
// //    - 2: 437.05 uA, max. 221.87 mA, 13.11 mC (CHARGE)
// //    - 3: 438.19 uA, max. 227.31 mA, 13.15 mC (CHARGE)
// //    - 4: 451.71 uA, max. 222.65 mA, 13.55 mC (CHARGE)
// //    - 5: 434.25 uA, max. 220.32 mA, 13.03 mC (CHARGE)
// // Averages: 439.952 uA, max. 223.114 mA, 13.200 mC

// // Pico2W on custom PCB:
// //    - 1: 577.34 uA, max. 99.01 mA, 17.32 mC (CHARGE)
// //    - 2: 571.59 uA, max. 98.25 mA, 17.15 mC (CHARGE)
// //    - 3: 564.61 uA, max. 99.01 mA, 16.94 mC (CHARGE)
// //    - 4: 589.74 uA, max. 99.01 mA, 17.69 mC (CHARGE)
// //    - 5: 586.47 uA, max. 99.77 mA, 17.59 mC (CHARGE)
// // Averages: 577.95 uA, max. 99.01 mA, 17.338 mC
