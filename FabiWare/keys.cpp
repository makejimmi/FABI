/*
     Flexible Assistive Button Interface (FABI) - AsTeRICS Foundation - http://www.asterics-foundation.org
     for controlling HID functions via momentary switches and/or serial AT-commands  
     More Information: https://github.com/asterics/FABI

     Module: keys.cpp - keyboard and keycode support

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License, see:
     http://www.gnu.org/licenses/gpl-3.0.en.html
 
*/

#include "fabi.h"
#include "keys.h"
#include <avr/pgmspace.h>



int storedKeys[HID_REPORT_KEY_COUNT]={0};  // arrays for keycodes of currently pressed keys
extern const int usToDE[];                 // translation map for keycodes, see below


// this keymap associates keycode-strings to the actual key codes
//
const keymap_struct keymap[] PROGMEM  = {   
  {"SHIFT", KEY_LEFT_SHIFT},
  {"CTRL", KEY_LEFT_CTRL},
  {"ALT", KEY_LEFT_ALT},
  {"RIGHT_ALT", KEY_RIGHT_ALT},
  {"GUI", KEY_LEFT_GUI},
  {"RIGHT_GUI", KEY_RIGHT_GUI},
  {"UP", KEY_UP},
  {"DOWN", KEY_DOWN},
  {"LEFT", KEY_LEFT},
  {"RIGHT", KEY_RIGHT},
  {"ENTER", KEY_ENTER},
  {"SPACE", KEY_SPACE},
  {"ESC", KEY_ESC},
  {"BACKSPACE", KEY_BACKSPACE},
  {"TAB", KEY_TAB},
  {"CAPS_LOCK", KEY_CAPS_LOCK},
  {"F1", KEY_F1},
  {"F2", KEY_F2},
  {"F3", KEY_F3},
  {"F4", KEY_F4},
  {"F5", KEY_F5},
  {"F6", KEY_F6},
  {"F7", KEY_F7},
  {"F8", KEY_F8},
  {"F9", KEY_F9},
  {"F10", KEY_F10},
  {"F11", KEY_F11},
  {"F12", KEY_F12},
  {"F13", KEY_F13},
  {"F14", KEY_F14},
  {"F15", KEY_F15},
  {"F16", KEY_F16},
  {"F17", KEY_F17},
  {"F18", KEY_F18},
  {"F19", KEY_F19},
  {"F20", KEY_F20},
  {"F21", KEY_F21},
  {"F22", KEY_F22},
  {"F23", KEY_F23},
  {"F24", KEY_F24},
  {"INSERT", KEY_INSERT},
  {"HOME", KEY_HOME},
  {"PAGE_UP", KEY_PAGE_UP},
  {"DELETE", KEY_DELETE},
  {"END", KEY_END},
  {"PAGE_DOWN", KEY_PAGE_DOWN},
  {"A", KEY_A},
  {"B", KEY_B},
  {"C", KEY_C},
  {"D", KEY_D},
  {"E", KEY_E},
  {"F", KEY_F},
  {"G", KEY_G},
  {"H", KEY_H},
  {"I", KEY_I},
  {"J", KEY_J},
  {"K", KEY_K},
  {"L", KEY_L},
  {"M", KEY_M},
  {"N", KEY_N},
  {"O", KEY_O},
  {"P", KEY_P},
  {"Q", KEY_Q},
  {"R", KEY_R},
  {"S", KEY_S},
  {"T", KEY_T},
  {"U", KEY_U},
  {"V", KEY_V},
  {"W", KEY_W},
  {"X", KEY_X},
  {"Y", KEY_Y},
  {"Z", KEY_Z},
  {"1", KEY_1},
  {"2", KEY_2},
  {"3", KEY_3},
  {"4", KEY_4},
  {"5", KEY_5},
  {"6", KEY_6},
  {"7", KEY_7},
  {"8", KEY_8},
  {"9", KEY_9},
  {"0", KEY_0},
  {"PLUS", 184},
  {"MINUS", 47},
  {"ASTERISK", 125},
  {"SLASH", 38},
  {"DOT", 46},
  {"COLON", 62},
  {"SEMICOLON", 60},
  {"HASH", 0xba},
  {"KP_PLUS", 0xdf},
  {"KP_MINUS", 0xde},
  {"KP_ASTERISK", 0xdd},
  {"KP_SLASH", 0xdc},
};

#define KEYMAP_ELEMENTS (sizeof keymap / sizeof keymap[0])

/**
   @name getKeycode
   @param char* acttoken
   @return int

   returns a keycode for a given keycode-string (acttoken)
*/
int getKeycode(char* acttoken)
{
    keymap_struct keyRAM;    
    if (!strncmp(acttoken, "KEY_", 4)) {
      acttoken += 4;
      for (int i = 0; i < KEYMAP_ELEMENTS; i++) {
        // Serial.print("scanning for ");  Serial.println(keymap[i].token);
        // if (!strcmp_FM(acttoken,(uint_farptr_t_FM)keymap[i].token)) {

        memcpy_P( &keyRAM, &keymap[i], sizeof(keymap[0]));
        if (!strcmp(acttoken, keyRAM.token)) {
          // Serial.print ("found "); Serial.println (keyRAM.key);
          return(keyRAM.key);
        }
      }
    }
    return(0);
}

/**
   @name getNextKeyName
   @param char* keyNames
   @param char* singleKeyName
   @return uint16_t

   stores the first keycode-string into "singleKeyName" and returns its lenght
   Note: this function is called multiple times in order to 
   tokenizes a string which contains multiple keycode-stings (eg. "KEY_A KEY_B")
 
*/
uint16_t getNextKeyName(char* keyNames, char* singleKeyName)
{
  int i=0,j=0;
  while (keyNames[i]==' ') i++;
  while ((keyNames[i]!=' ') && (keyNames[i])) {
     singleKeyName[j++]=keyNames[i++];
  }
  singleKeyName[j]=0;
  return(i);
}

/**
   @name storeKey
   @param int k
   @return none

   adds a keycode to an array, in order to keep track of currently pressed keys
 
*/
void storeKey(int k) {
  for (int i=0;i<HID_REPORT_KEY_COUNT;i++) {
    if (storedKeys[i]==k) return;  // already stored
    if (storedKeys[i]==0) {storedKeys[i]=k; return;} // store new key    
  }
}

/**
   @name removeKey
   @param int k
   @return none

   removes a keycode to an array, in order to keep track of currently pressed keys
 
*/
void removeKey(int k) {
  for (int i=0;i<HID_REPORT_KEY_COUNT;i++) {
    if (storedKeys[i]==k) {storedKeys[i]=0; return;} // remove key    
  }
}

/**
   @name keyStored
   @param int k
   @return int

   returns if a given key (keycode) is pressed (0: no / 1:yes)
*/
int keyStored(int k) {
  for (int i=0;i<HID_REPORT_KEY_COUNT;i++) {
    if (storedKeys[i]==k) return (1);  // found
  }
  return(0);
}


/**
   @name pressSingleKeys
   @param char* keyNames
   @return none

   press sequence of supported single keys 
   keyNames is a string which contains the key identifiers eg. "KEY_CTRL KEY_C" for Ctrl-C
*/
void pressSingleKeys(char* keyNames)
{
  int len;
  char singleKeyName[20];   // e.g. KEY_A
  while (len=getNextKeyName(keyNames,singleKeyName))
  {
    int kc=getKeycode(singleKeyName);
    if (kc) {
     keyboardPress(kc);
     storeKey(kc);
     // Serial.print ("press key ");  Serial.println (kc);
    }
    keyNames+=len;
  }
}

/**
   @name releaseSingleKeys
   @param char* keyNames
   @return none

   release sequence of supported single keys 
   keyNames is a string which contains the key identifiers eg. "KEY_CTRL KEY_C" for Ctrl-C
*/
void releaseSingleKeys (char * keyNames)
{
  int len;
  char singleKeyName[20];
  while (len=getNextKeyName(keyNames,singleKeyName))
  {
    int kc=getKeycode(singleKeyName);
    if (kc) {
      keyboardRelease(kc);
      removeKey(kc);
    }
    keyNames+=len;
  }
}


/**
   @name toggleSingleKeys
   @param char* keyNames
   @return none

   toggle sequence of supported single keys 
   keyNames is a string which contains the key identifiers eg. "KEY_CTRL KEY_C" for Ctrl-C
*/
void toggleSingleKeys(char* keyNames)
{
  int len;
  char singleKeyName[20];   // e.g. KEY_A
  while (len=getNextKeyName(keyNames,singleKeyName))
  {
    int kc=getKeycode(singleKeyName);
    if (kc) {
      if (keyStored(kc)) { keyboardRelease(kc); removeKey(kc); }
      else { keyboardPress(kc); storeKey(kc);}
      // Serial.print ("toggle key ");  Serial.println (kc);
    }
    keyNames+=len;
  }
}



/**
   @name writeTranslatedKeys
   @param char * str
   @param int len
   @return none
   
   write a sequence of characters, translated to locale using modifier keys
*/
void writeTranslatedKeys(char * str, int len)
{
   int k;
   for (int i=0; i<len; i++) {
      if (KEYBOARD_LAYOUT == KBD_DE)
         k=pgm_read_word_near(&(usToDE[(uint8_t)str[i]]));  // get the translated keycode (DE layout)
      else k=str[i];
      
      // Serial.print ("char:"); Serial.print(str[i]); Serial.print(" -> ");Serial.print(k);Serial.print(" (");
      // if (k&MOD_ALTGR) Serial.print("AltGr + "); if (k&MOD_SHIFT) Serial.print("Shift + "); 
      // Serial.print((char)(k&0xff)); Serial.println(")");

      if (k&MOD_ALTGR) keyboardPress(KEY_RIGHT_ALT); 
      if (k&MOD_SHIFT) keyboardPress(KEY_LEFT_SHIFT); 
      keyboardPress(k&0xff); 
      keyboardRelease(k&0xff); 
      if (k&MOD_SHIFT) keyboardRelease(KEY_LEFT_SHIFT); 
      if (k&MOD_ALTGR) keyboardRelease(KEY_RIGHT_ALT);
   }
}


/**
   @name sendToKeyboard
   @param char * writeKeystring
   @return none
   
   write a string to the keyboard, replacing special keys (identified by "KEY_NAME") with their keycodes
*/ 
void sendToKeyboard(char * writeKeystring)
{
    char singleKeyName[20];
    char * actpos = writeKeystring;
    char * specialKeyLocation=strstr(actpos,"KEY_");

    while (specialKeyLocation) {
        // write all normal characters until the special key position
        writeTranslatedKeys (actpos, specialKeyLocation-actpos);
        //extract name of special key
        getNextKeyName(specialKeyLocation,singleKeyName);
        int kc=getKeycode(singleKeyName);
        if (kc)  {
           keyboardPress(kc);
           keyboardRelease(kc);
        }
        // continue after special key name
        actpos= specialKeyLocation+strlen(singleKeyName);
        specialKeyLocation=strstr(actpos,"KEY_");
    }
    // write remainder of normal characters   
    writeTranslatedKeys(actpos, strlen(actpos));
}


/**
   @name release_all
   @param none
   @return none
   
   releases all previously pressed keys and mouse buttons
   stop mouse movement
*/
void release_all() 
{
  // Serial.println("release all!");
  keyboardReleaseAll();  //Keyboard.releaseAll();
  leftMouseButton = 0;
  rightMouseButton = 0;
  middleMouseButton = 0;
  moveX = 0;
  moveY = 0;
}



// here comes a character translation table - this works only for DE by now ...
const int usToDE[] PROGMEM = 
{
//  0,  0,  0,  0,  0,  0,  0,  0, BS, TB, CR,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  8,  9, 10,  0,  0, 13,  0,  0,

    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,

// BL,  !,  Ä,  §,     $,  %,  /,       ä,        ),  =,  (,  `,  ,,  ß,  .,  -,
//          "                  &        /         (   )   *   +       -       /
   32, 33, 64,  '\\', 36, 37, 94, MOD_SHIFT+'\\', 42, 40,125,184, 44, 47, 46, 38,

//  0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  Ö,  ö, ;,  ´,  :,  _,
//                                          :   ;  <  |    >   =        
   48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 62, 60, 0, 41,  0, 95,

//      ",          A,  B,  C,  D,  E,  F,  G,  H,  I,  J,  K,  L,  M,  N,  O,
//      @
    MOD_ALTGR+'q', 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,

//  P,  Q,  R,  S,  T,  U,  V,  W,  X,  Z,  Y,      ü,                #,                +,          &,  ?,
//                                      Y   Z       [                 \                 ]           ^   _
   80, 81, 82, 83, 84, 85, 86, 87, 88, 90, 89,  MOD_ALTGR+KEY_8,  MOD_ALTGR+'-',  MOD_ALTGR+KEY_9, 96, 63,

//  ^,  a,  b,  c,  d,  e,  f,  g,  h,  i,  j,  k,  l,  m,  n,  o,
//  `
   43, 97, 98, 99,100,101,102,103,104,105,106,107,108,109,110,111,

//  p,  q,  r,  s,  t,  u,  v,  w,  x,  z,  y,         Ü,         ,           *,        °,  0,
//                                      y   z          {          |           }         ~
  112,113,114,115,116,117,118,119,120,122,121,  MOD_ALTGR+KEY_7,  0,  MOD_ALTGR+KEY_0,  0,  0,


// 
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  '\"',  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  ':',  0,  0,  0,  0,  0,  MOD_SHIFT+'[',  0,  0,  '-',
    0,  0,  0,  0,  '\'',  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  ';',  0,  0,  0,  0,  0,  '[',  0,  0,  0,

};
