// #include <Arduino.h>
// #include "sleep.h"

// // #define STANDALONE
// #define PCB
// #define LED LED_BUILTIN

// #ifdef STANDALONE
//     #define BUTTON BOOTSEL
// #endif
// #ifdef PCB
//     #define BUTTON 4
// #endif

// void setup()
// {   
//     pinMode(LED, OUTPUT);
//     digitalWrite(LED, HIGH);
//     pinMode(BUTTON, INPUT_PULLUP);
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

//             // Inform that PICO is going DORMANT
//             Serial.println("Going to sleep");
//             digitalWrite(LED, LOW); pinMode(LED, INPUT);
    
//             // Close Serial before entering dormant
//             Serial.flush(); delay(10);
//             Serial.end(); delay(1000);
    
//             // Prepare for DORMANT
//             sleep_run_from_lposc();
    
//             // Enter DORMANT (wait for button press to wake)
//             int8_t gpios[] = {4};
//             sleep_goto_dormant_until_edge_high(gpios, 1);
    
//             // Wake up
//             sleep_power_up(); delay(400);
            
//             // Reopen Serial after waking
//             pinMode(LED, OUTPUT); digitalWrite(LED, HIGH);
//             Serial.println("Good morning");
//             pinMode(BUTTON, INPUT_PULLUP);
//             Serial.begin(115200); delay(2000); // Allow time for Serial initialization
//         }
//     }
// }

// void setup1() {}
// void loop1() {}

// // -----------------------------------------------------------------

// // DORMANT Mode
// // Current Consumption; 4.2V; 30s;

// // -----------------------------------------------------------------

// // Pico2W as standalone uC:

// //    - 1: 4.17 mA, max. 170.09 mA, 124.91 mC (CHARGE); Normal: 9.84 mA
// //    - 2: 4.16 mA, max. 143.23 mA, 124.87 mC (CHARGE); Normal: 12.57 mA
// //    - 3: 4.30 mA, max. 165.47 mA, 128.91 mC (CHARGE); Normal: 10.39 mA
// //    - 4: 4.24 mA, max. 171.63 mA, 127.22 mC (CHARGE); Normal: 10.34 mA
// //    - 5: 4.12 mA, max. 173.17 mA, 123.24 mC (CHARGE); Normal: 9.62 mA
// // Averages: 4.20 mA, max. 164.72 mA, 125.83 mC, Normal 10.55 mA

// // Pico2W on custom PCB:
// //    - 1: 5.86 mA, max. 86.87 mA, 175.88 mC (CHARGE); Normal: 13.31 mA
// //    - 2: 5.94 mA, max. 88.34 mA, 178.32 mC (CHARGE); Normal: 13.43 mA
// //    - 3: 5.90 mA, max. 88.38 mA, 177.09 mC (CHARGE); Normal: 13.44 mA
// //    - 4: 5.92 mA, max. 87.63 mA, 178.68 mC (CHARGE); Normal: 13.54 mA
// //    - 5: 5.96 mA, max. 87.63 mA, 178.87 mC (CHARGE); Normal: 13.42 mA
// // Averages: 5.92 mA, max. 87.77 mA, 177.77 mC, Normal 13.43 mA

// // -----------------------------------------------------------------

// // Both Cores

// // Pico2W as standalone uC:
// //    - 1: 3.71 mA, max. 181.64 mA, 111.17 mC (CHARGE); Normal: 9.09 mA
// //    - 2: 3.61 mA, max. 180.10 mA, 108.44 mC (CHARGE); Normal: 8.74 mA
// //    - 3: 3.64 mA, max. 180.87 mA, 109.15 mC (CHARGE); Normal: 9.18 mA
// //    - 4: 3.63 mA, max. 183.95 mA, 108.79 mC (CHARGE); Normal: 8.89 mA
// //    - 5: 3.59 mA, max. 185.50 mA, 107.60 mC (CHARGE); Normal: 8.50 mA
// // Averages: 3.64 mA, max. 182.41 mA, 109.03 mC, Normal 8.88 mA

// // Pico2W on custom PCB:
// //    - 1: 5.93 mA, max. 88.38 mA, 178.07 mC (CHARGE); Normal: 14.63 mA
// //    - 2: 5.97 mA, max. 86.47 mA, 179.07 mC (CHARGE); Normal: 14.69 mA
// //    - 3: 5.97 mA, max. 86.87 mA, 178.98 mC (CHARGE); Normal: 14.69 mA
// //    - 4: 5.97 mA, max. 88.38 mA, 179.16 mC (CHARGE); Normal: 15.04 mA
// //    - 5: 5.98 mA, max. 88.38 mA, 179.32 mC (CHARGE); Normal: 15.03 mA
// // Averages: 5.96 mA, max. 87.70 mA, 178.92 mC, Normal 14.82 mA