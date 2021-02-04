//BUTTON BOX 
//USE w ProMicro
//Tested in WIN10 + Assetto Corsa
//AMSTUDIO
//20.11.12

#include <Keypad.h>
#include <Joystick.h>

#include <Adafruit_NeoPixel.h>  //inclusion of Adafruit's NeoPixel (RBG addressable LED) library
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIN            15 // Which pin on the Arduino is connected to the NeoPixels?
#define NUMPIXELS      25 // How many NeoPixels are attached to the Arduino? 13 total, but they are address from 0,1,2,...12.

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int colorUpdate = 0;   //setting a flag to only update colors once when the mode is switched. 
const int b = 100;       // Brightness control variable. Used to divide the RBG vales set for the RGB LEDs. full range is 0-255. 255 is super bright
                       // In fact 255 is obnoxiously bright, so this use this variable to reduce the value. It also reduces the current draw on the USB

#define X1 20
#define Y1 21
//#define X1 PB0
//#define Y1 PD5

#define ENABLE_PULLUPS
#define NUMROTARIES 4
#define NUMBUTTONS 20
#define NUMROWS 4
#define NUMCOLS 5

#define ASYNC_UPDATE_MILLIS 20

byte buttons[NUMROWS][NUMCOLS] = {
  {0, 1, 2, 3, 4},
  {5, 6, 7, 8, 9},
  {10,11,12,13,14},
  {15,16,17,18,19},
};


struct rotariesdef 
{
  byte pin1;
  byte pin2;
  byte ccwchar;
  byte cwchar;
  byte halfstep;
  byte pulldown;
  signed char ccw_count;
  signed char cw_count;
  volatile unsigned char state;
};

rotariesdef rotaries[NUMROTARIES] {
  {1,0,24,25,1,0,0, 0},   /* propwash dual axis 0 - halfstep */
  {2,3,26,27,1,0,0, 0},   /* propwash dual axis 1 - halfstep */
  {4,5,28,29,0,0,0, 0},   /* standard encoder */
  {6,7,30,31,0,0,0, 0},   /* standard encoder */
};

#define DIR_CCW 0x10
#define DIR_CW 0x20

/* half-step rotary states
 * if pins are reading 00, one click would transition to pins reading 11
 * if pins are reading 11, one click would transition to pins reading 00
 *
 * one bit changes state before the other for CW and CCW transitions
 * whichever pin changes first determines the direction.
 */
enum {
  Rh_START_LO = 0,
  Rh_CCW_BEGIN,
  Rh_CW_BEGIN,

  Rh_START_HI,
  Rh_CW_BEGIN_HI,
  Rh_CCW_BEGIN_HI,

  Rh_MAX
};

const unsigned char ttable_half[Rh_MAX][4] = {
  // pin bits - transistions from 00 to 00 or 11
  //  00                   01              10             11
  // Rh_START_LO (00) - usually either both on or both off
  {Rh_START_LO,           Rh_CW_BEGIN,    Rh_CCW_BEGIN,   Rh_START_HI},
  // Rh_CCW_BEGIN (was at 10)
  {Rh_START_LO,           Rh_START_LO,    Rh_CCW_BEGIN,   Rh_START_HI | DIR_CCW},
  // Rh_CW_BEGIN  (was at 01)
  {Rh_START_LO,           Rh_CW_BEGIN,    Rh_START_LO,    Rh_START_HI | DIR_CW},

  // pin bits - transistions from 11 to 00 or 11
  //  00                   01              10              11
  // Rh_START_HI (11) - usually either both on or both off
  {Rh_START_LO,           Rh_CCW_BEGIN_HI, Rh_CW_BEGIN_HI, Rh_START_HI},
  // Rh_CW_BEGIN_HI (was at 10)
  {Rh_START_LO | DIR_CW,  Rh_START_HI,     Rh_CW_BEGIN_HI, Rh_START_HI},
  // Rh_CCW_BEGIN_HI (was at 01)
  {Rh_START_LO | DIR_CCW, Rh_CCW_BEGIN_HI, Rh_START_HI,    Rh_START_HI},
};

/* X-step rotaries */
enum {
  R_START = 0x0,
  R_CW_FINAL,
  R_CW_BEGIN,
  R_CW_NEXT,
  R_CCW_BEGIN,
  R_CCW_FINAL,
  R_CCW_NEXT,

  R_MAX
};

const unsigned char ttable[R_MAX][4] = {
  // pin bits - transistions
  //  00       01           10           11
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

byte rowPins[NUMROWS] = {LED_BUILTIN_RX,LED_BUILTIN_TX,19,18}; //utilise les 2 pins du pro micro dans le bas pas de PAD
byte colPins[NUMCOLS] = {14,16,10,8,9}; 

Keypad buttbx = Keypad( makeKeymap(buttons), rowPins, colPins, NUMROWS, NUMCOLS); 

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 32, 0,
  true, true, false, false, false, false,
  false, false, false, false, false);

unsigned long LastSendTime;

//TODO
void setColorsMode0(){
  if (colorUpdate == 0){                                           // have the neopixels been updated?
      for(int i=0;i<NUMPIXELS;i++)
      {      //  Red,Green,Blue                      // pixels.Color takes RGB values; range is (0,0,0) to (255,255,255)
        pixels.setPixelColor(i, pixels.Color(255,   0,    0));      // Moderately bright red color.
        delay(2);
      }                                               // Delay for a period of time (in milliseconds).
      pixels.show();                                             // This pushes the updated pixel color to the hardware.
                                               
      colorUpdate=1;   }                                           // Mark the color flag so neopixels are no longer updated in the loop
}

void setColorsMode1(){
  if (colorUpdate == 0){                                     // have the neopixels been updated?
      pixels.setPixelColor(0,  pixels.Color( 80,  0,200));    //gradient mix
      pixels.setPixelColor(1,  pixels.Color( 10,  0,200));    //gradient mix
      pixels.setPixelColor(2,  pixels.Color( 20,  0,200));
      pixels.setPixelColor(3,  pixels.Color( 40,  0,200));
      pixels.setPixelColor(4,  pixels.Color( 60,  0,200));
      pixels.setPixelColor(5,  pixels.Color( 80,  0,200));
      pixels.setPixelColor(6,  pixels.Color(100,  0,200));
      pixels.setPixelColor(7,  pixels.Color(120,  0,200));
      pixels.setPixelColor(8,  pixels.Color(140,  0,200));
      pixels.setPixelColor(9,  pixels.Color(160,  0,200));
      pixels.setPixelColor(10, pixels.Color(180,  0,200));
      pixels.setPixelColor(11, pixels.Color(200,  0,200));
      pixels.setPixelColor(12, pixels.Color(220,  0,200));
      delay(50);
      pixels.show();
      colorUpdate=1;              }                           // neoPixels have been updated. 
                                                              // Set the flag to 1; so they are not updated until a Mode change
}
//

void MonJoystick()
{
    Joystick.setXAxis(analogRead(X1) - 512);
    Joystick.setYAxis(analogRead(Y1) - 512);
}

void setup() {
  Serial.begin(9600); 
  Joystick.begin();
  Joystick.setXAxisRange(512, -512);
  Joystick.setYAxisRange(-512, 512);
#ifdef ASYNC_UPDATE_MILLIS
  buttbx.setDebounceTime(2*ASYNC_UPDATE_MILLIS);
#else
  Joystick.begin(true);
#endif
  for (int i=0;i<NUMROTARIES;i++) {
    pinMode(rotaries[i].pin1, INPUT);
    pinMode(rotaries[i].pin2, INPUT);
    if ( rotaries[i].pulldown ) {
      digitalWrite(rotaries[i].pin1, LOW);
      digitalWrite(rotaries[i].pin2, LOW);
    } else {
      digitalWrite(rotaries[i].pin1, HIGH);
      digitalWrite(rotaries[i].pin2, HIGH);
    }
  }

  pixels.begin(); // This initializes the NeoPixel library.
  LastSendTime = millis();
}

int CheckAllEncoders(void);
int CheckAllButtons(void);

void loop() { 

  int changes = 0;
  unsigned long now;

  CheckAllEncoders();

  setColorsMode0();                         //indicate what mode is loaded by changing the key colors
  MonJoystick();
  now = millis();

#ifdef ASYNC_UPDATE_MILLIS
  if ( (signed long)(now - LastSendTime) > ASYNC_UPDATE_MILLIS ) {
    /* do the clicks */
    for (int i=0;i<NUMROTARIES;i++) {
      if ( rotaries[i].cw_count > 0 ) {        /* clockwise clicks */
        if ( !(rotaries[i].cw_count & 1) ) {   /* EVEN: start click */
          Joystick.setButton(rotaries[i].cwchar, 1);
        } else {
          Joystick.setButton(rotaries[i].cwchar, 0);
        }
        rotaries[i].cw_count--;
        changes++;
      }
      if ( rotaries[i].ccw_count > 0 ) { /* counter-clockwise clicks */
        if ( !(rotaries[i].ccw_count & 1) ) {   /* EVEN: start click */
          Joystick.setButton(rotaries[i].ccwchar, 1);
        } else {
          Joystick.setButton(rotaries[i].ccwchar, 0);
        }
        rotaries[i].ccw_count--;
        changes++;
      }
    }
  }
  changes += CheckAllButtons();

  if ( changes ) {
    Joystick.sendState();
    LastSendTime = now /* millis() */;
  } else {
    //delay(1);
  }
#else
  changes += CheckAllButtons();
#endif
}

int CheckAllButtons(void) {
  int changes = 0;
  if (buttbx.getKeys())
    {
       for (int i=0; i<LIST_MAX; i++)   
        {
           if ( buttbx.key[i].stateChanged )   
            {
              switch (buttbx.key[i].kstate) {  
                    case PRESSED:
                              pixels.setPixelColor(buttbx.key[i].kcode, pixels.Color(0,255,255));
                              colorUpdate = 0;                                    //call the color update to change the color back to Mode settings
                              pixels.show();                                      //update the color after the button press
                    case HOLD:
                              Joystick.setButton(buttbx.key[i].kchar, 1);
                              changes++;
                              
                              break;
                    case RELEASED:


                    case IDLE:
                              Joystick.setButton(buttbx.key[i].kchar, 0);
                              changes++;
                              break;
            }
           }   
         }
     }
  return changes;
}


unsigned char rotary_process(int _i) {
   unsigned char pinstate = (digitalRead(rotaries[_i].pin2) << 1) | digitalRead(rotaries[_i].pin1);
   if ( rotaries[_i].halfstep ) {
      rotaries[_i].state = ttable_half[rotaries[_i].state & 0xf][pinstate];
   } else {
      rotaries[_i].state = ttable[rotaries[_i].state & 0xf][pinstate];
   }
  return (rotaries[_i].state & 0x30);
}

int CheckAllEncoders(void) 
{
  int changes = 0;
  for (int i=0;i<NUMROTARIES;i++) {
    unsigned char result = rotary_process(i);
    if (result == DIR_CCW) {
      changes++;
      #ifdef ASYNC_UPDATE_MILLIS
        rotaries[i].ccw_count += 2;
        rotaries[i].state &= 0xf; /* clear the CW/CCW state as we've added to click count */
        /* and cancel the opposite rotation, note we get rid
         * of everything except the LSB so it may "wind down" to 0
         */
        rotaries[i].cw_count &= 0x1;
      #else
        Joystick.setButton(rotaries[i].ccwchar, 1); delay(50); Joystick.setButton(rotaries[i].ccwchar, 0);
      #endif
    };
    if (result == DIR_CW) {
      changes++;
      #ifdef ASYNC_UPDATE_MILLIS
        rotaries[i].cw_count += 2;
        rotaries[i].state &= 0xf; /* clear the CW/CCW state as we've added to click count */
        /* and cancel the opposite rotation, note we get rid
         * of everything except the LSB so it may "wind down" to 0
         */
        rotaries[i].ccw_count &= 0x1;
      #else
        Joystick.setButton(rotaries[i].cwchar, 1); delay(50); Joystick.setButton(rotaries[i].cwchar, 0);
      #endif
    };
  }
  return changes;
}
