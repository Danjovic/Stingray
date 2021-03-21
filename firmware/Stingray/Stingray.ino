/*******************************************************************************

         _____ __  _
        / ___// /_(_)___  ____ __________ ___  __
        \__ \/ __/ / __ \/ __ `/ ___/ __ `/ / / /
       ___/ / /_/ / / / / /_/ / /  / /_/ / /_/ /
      /____/\__/_/_/ /_/\__, /_/   \__,_/\__, /
                       /____/           /____/

  Nintendo Wii Classic controller adapter for Atari 5200
  Danjovic 2020 - danjovic@hotmail.com - https://hackaday.io/danjovic

  13 March 2020 - Basic Release
  29 March 2020 - Bug fixed and code improvement, thanks Chris Belcher for the feedback
                - Pull down of all the axes lines on startup
                - Fix deadlock condition on Vac control 
  
  This adapter lets you play Atari 5200 using a Nintendo Wii Classic controller yet providing full keypad control.

  Main features are:

    # Analog Controller (using precise timing);
    # Digital Controller on D-PAD just like a Masterplay adapter;
    # Full keypad emulation;
    # Auto detected second controller adapter for games like Robotron
    # Independent Top/Bottom buttons on second controller. full keypad control.


  The circuit is built around an Arduino Nano and two analog multiplexers.
  Interface with analog joystick is performed using precise timing method: The internal analog comparator is used to detect the moment that the pokey chip in Atari 5200 releases the POT lines to start charging the timing capacitors. The comparator triggers an interrupt before the voltage across the capacitors reach the Vih threshold of POKEY POT lines. Then the AVR forces the POT lines LOW during an interval of time proportional to the position of the corresponding axis. At a given moment the POT line is pushed HIGH, exceeding Vih threshold immediately which makes POKEY capture a counting value that is equivalent to the potentiometer position.

  Keypad presses are emulated by activating a pair of analog multiplexers. Only 1 (one) keypress can be simulated at a time.
  When there is no key pressed the "output" multiplexer is inhibited.


  Buttons are mapped as follows:

    +----------+--Zl key (select)--+
    | Button   | Pressed | Released|
    +----------+---------+---------+
    | D-PAD UP |    2    |   none  |
    | D-PAD DW |   none  |   none  |
    | D-PAD LF |    1    |   none  |
    | D-PAD RG |    3    |   none  |
    |  Minus   |    4    |    *    |
    |  Home    |    5    |    0    |
    |  Plus    |    6    |    #    |
    |    Y     |    7    |   none  |
    |    X     |    8    |   none  |
    |    A     |    9    |   none  |
    |    B     |   none  |   none  |
    |  LEFT    |   none  |   none  |
    |  RIGHT   |  START  |   none  |
    |   Zr     |  PAUSE  |  PAUSE  |
    +----------+---------+---------+

  Keypresses are emulated by activation of the following combination of signals.
  The 'none' key state is issued by rising the INHIBT signal to disabe the output MUX.

        +------- Output Mux -MSbits-----+
  Input |   7       6       5       8
   Mux  |  0 0  |  0 1  |  1 0  |  1 1  |
   B A  +-------+-------+-------+-------+
   0 0  |   1   |   4   |   7   |   *   | 3
   0 1  |   2   |   5   |   8   |   0   | 2
   1 0  |   3   |   6   |   9   |   #   | 1
   1 1  | start | pause | reset |  none | 4
        +-------+-------+-------+-------+

  CAV Voltage on PIN 9 is connected to INT1 IRQ pin and is used to promptly turn off the timing pins whenever CAV drops to zero.
  A zener diode as added to protect the AVR input, as the CAV voltage can reach up to 6.4Volts.



    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/


#include <NintendoExtensionCtrl.h>  
// By Dave Madison. Use version 0.8.1 or newer
// https://github.com/dmadison/NintendoExtensionCtrl
#include <avr/pgmspace.h>


 
/*******************************************************************************
           _      __ _      _ _   _
        __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
       / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
       \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/

*/

//#define DEBUG 1  // run debug only to check values as it messes with temporization

#define _key1        ((0<<3)|(0<<2)|(0<<1)|(0<<0)) // 0
#define _key2        ((0<<3)|(0<<2)|(0<<1)|(1<<0)) // 1
#define _key3        ((0<<3)|(0<<2)|(1<<1)|(0<<0)) // 2
#define _keyStart    ((0<<3)|(0<<2)|(1<<1)|(1<<0)) // 3
#define _key4        ((0<<3)|(1<<2)|(0<<1)|(0<<0)) // 4
#define _key5        ((0<<3)|(1<<2)|(0<<1)|(1<<0)) // 5
#define _key6        ((0<<3)|(1<<2)|(1<<1)|(0<<0)) // 6
#define _keyPause    ((0<<3)|(1<<2)|(1<<1)|(1<<0)) // 7
#define _key7        ((1<<3)|(0<<2)|(0<<1)|(0<<0)) // 8
#define _key8        ((1<<3)|(0<<2)|(0<<1)|(1<<0)) // 9
#define _key9        ((1<<3)|(0<<2)|(1<<1)|(0<<0)) // 10
#define _keyReset    ((1<<3)|(0<<2)|(1<<1)|(1<<0)) // 11
#define _keyAsterisk ((1<<3)|(1<<2)|(0<<1)|(0<<0)) // 12
#define _key0        ((1<<3)|(1<<2)|(0<<1)|(1<<0)) // 13
#define _keyHash     ((1<<3)|(1<<2)|(1<<1)|(0<<0)) // 14
#define _keyNone     ((1<<3)|(1<<2)|(1<<1)|(1<<0)) // 15

//      Signal Name     Arduino pin
#define inputMuxApin         8
#define inputMuxBpin         9
#define outputMuxApin        10
#define outputMuxBpin        11
#define outputMuxInhibtPin   12

#define potX1pin             5
#define potY1pin             4
#define fireTop1pin          2
#define fireBottom1pin       3

#define potX2pin             A0
#define potY2pin             A3
#define fireTop2pin          A2
#define fireBottom2pin       A1

#define caVinputPin          13

#define pinDetectDual        A6
#define thresholdDetectDual  256

// Bit mapped buttons
#define btnUp     (1<<0)
#define btnDown   (1<<1)
#define btnLeft   (1<<2)
#define btnRight  (1<<3)
#define btnMinus  (1<<4)
#define btnHome   (1<<5)
#define btnPlus   (1<<6)
#define btnStart  (1<<7)
#define btnY      (1<<8)
#define btnX      (1<<9)
#define btnA      (1<<10)
#define btnB      (1<<11)
#define btnLFT    (1<<12)
#define btnRGT    (1<<13)
#define btnZr     (1<<14)
#define btnZl     (1<<15)

#define btnMode btnZl



/*******************************************************************************

        _ __  __ _ __ _ _ ___ ___
       | '  \/ _` / _| '_/ _ (_-<
       |_|_|_\__,_\__|_| \___/__/

*/
#define pushX1() do { pinMode(potX1pin,OUTPUT); digitalWrite(potX1pin,HIGH); } while (0)
#define pushX2() do { pinMode(potX2pin,OUTPUT); digitalWrite(potX2pin,HIGH); } while (0)
#define pushY1() do { pinMode(potY1pin,OUTPUT); digitalWrite(potY1pin,HIGH); } while (0)
#define pushY2() do { pinMode(potY2pin,OUTPUT); digitalWrite(potY2pin,HIGH); } while (0)

#define dualJoystick() analogRead(pinDetectDual)< thresholdDetectDual



/*******************************************************************************
                       _            _
        __ ___ _ _  __| |_ __ _ _ _| |_ ___
       / _/ _ \ ' \(_-<  _/ _` | ' \  _(_-<
       \__\___/_||_/__/\__\__,_|_||_\__/__/

*/
enum controllerType {
  _Classic = 0,
  _Nunchuck,
  _unKnown = 0xff
};

enum { // operationMode
  JOYSTICK = 0,
  TRACKBALL
};

const  uint8_t gamma3 [256] PROGMEM = {  // non linar curve 
  5,  5,  5,  5,  5,  5,  8,  8,  8,  8,  9,  9, 10, 10, 10, 11,
 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 16, 17, 17, 18, 19, 20,
 21, 21, 22, 23, 24, 25, 25, 26, 27, 28, 28, 29, 30, 30, 31, 32,
 32, 34, 35, 36, 38, 39, 40, 41, 43, 44, 45, 46, 47, 48, 49, 50,
 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 63, 65, 67, 68, 70, 72,
 73, 75, 76, 78, 79, 81, 82, 84, 85, 87, 89, 91, 93, 94, 96, 98,
 99,101,102,104,105,106,107,109,110,110,111,111,111,112,112,112,
113,113,113,113,113,114,114,114,114,114,114,114,114,114,114,114,
114,114,114,114,114,114,114,114,114,114,114,115,115,115,115,115,
116,116,116,117,117,117,118,118,119,121,122,123,124,126,127,129,
130,132,134,135,137,139,141,143,144,146,147,149,150,152,153,155,
156,158,160,161,163,165,167,168,169,170,171,172,173,174,175,176,
178,179,180,181,182,183,184,185,187,188,189,190,192,193,194,196,
196,197,198,198,199,200,200,201,202,203,203,204,205,206,207,207,
208,209,210,211,211,212,213,214,214,215,215,215,216,216,217,217,
217,218,218,218,219,219,220,220,220,220,220,220,220,220,220,220 };

/*******************************************************************************
                     _      _    _
       __ ____ _ _ _(_)__ _| |__| |___ ___
       \ V / _` | '_| / _` | '_ \ / -_|_-<
        \_/\__,_|_| |_\__,_|_.__/_\___/__/

*/
static volatile uint8_t CAVoff;
static volatile uint8_t operationMode;
static volatile uint8_t hline = 0;
static volatile uint8_t potX1value = 0;
static volatile uint8_t potX2value = 0;
static volatile uint8_t potY1value = 0;
static volatile uint8_t potY2value = 0;

//uint8_t combinedXaxis;  // ponderated value of both stick movement on X axis
//uint8_t combinedYaxis;  // ponderated value of both stick movement on Y axis

uint8_t  leftXaxis;
uint8_t rightXaxis;
uint8_t  leftYaxis;
uint8_t rightYaxis;

uint8_t singleControllerX, singleControllerY; // range 0..255
uint8_t dualControllerX1,  dualControllerY1;  // range 0..255
uint8_t dualControllerX2,  dualControllerY2;  // range 0..255

uint16_t combinedButtons = 0;

ExtensionPort controller;
Nunchuk::Shared nchuk(controller);  // Read Nunchuk formatted data from the port
ClassicController::Shared classic(controller);  // Read Classic Controller formatted data from the port



/*******************************************************************************
        _     _                         _
       (_)_ _| |_ ___ _ _ _ _ _  _ _ __| |_ ___
       | | ' \  _/ -_) '_| '_| || | '_ \  _(_-<
       |_|_||_\__\___|_| |_|  \_,_| .__/\__/__/
                                  |_|
*/

// Timer 2 interrupt occurs at each 64us
ISR (TIMER2_COMPA_vect) {
  if (hline < 227) {
    if (hline == potX1value) pushX1();
    if (hline == potX2value) pushX2();
    if (hline == potY1value) pushY1();
    if (hline == potY2value) pushY2();
    hline++;

  } else if (hline == 227) {
    //last line, release all pins to make possible to detect next discharge
    pinMode(potX1pin, INPUT);
    pinMode(potX2pin, INPUT);
    pinMode(potY1pin, INPUT);
    pinMode(potY2pin, INPUT);

    ACSR |= (1 << ACIE) // reactivate analog comparator
            | (1 << ACI); // clear any pending interrupt bit
    hline++;           // increment to the next line. will stay in this count
  }                   // until hline is reset on the next comparation
}

// Analog comparator, triggers when detect that pokey released POT inputs to charge
ISR (ANALOG_COMP_vect) {
  //  pulseFlag();
  // Todo: write directly to DDR/PORT registers to speed up ISR servicing.
  pinMode(potX1pin, OUTPUT);  // hold axis pin in LOW state
  digitalWrite(potX1pin, LOW);
  pinMode(potY1pin, OUTPUT);  // hold axis pin in LOW state
  digitalWrite(potY1pin, LOW);
  pinMode(potX2pin, OUTPUT);  // hold axis pin in LOW state
  digitalWrite(potX2pin, LOW);
  pinMode(potY2pin, OUTPUT);  // hold axis pin in LOW state
  digitalWrite(potY2pin, LOW);

  hline = 0;
  ACSR &= ~(1 << ACIE); // disable comparator
  // ACSR &= (1<<ACI);  // clear pending interrupt bit
}


/*******************************************************************************
        ___      _
       / __| ___| |_ _  _ _ __
       \__ \/ -_)  _| || | '_ \
       |___/\___|\__|\_,_| .__/
                         |_|
*/
void setup() {

  // setup I/O pins
  pinMode(fireTop1pin    , INPUT) ; // Trigger pins all open
  pinMode(fireBottom1pin , INPUT) ;
  pinMode(fireTop2pin    , INPUT) ;
  pinMode(fireBottom2pin , INPUT) ;

  pinMode(potX1pin, INPUT) ; // Potentiometer pins as inputs
  pinMode(potY1pin, INPUT) ;
  pinMode(potX2pin, INPUT) ;
  pinMode(potY2pin, INPUT) ;

  digitalWrite(potX1pin, LOW); // disable pullups on potentiometer pins
  digitalWrite(potX2pin, LOW);
  digitalWrite(potY1pin, LOW);
  digitalWrite(potY2pin, LOW);

  pinMode(outputMuxBpin, OUTPUT); // All keyboard mux pins as outputs
  pinMode(outputMuxApin, OUTPUT);
  pinMode(inputMuxBpin , OUTPUT);
  pinMode(inputMuxApin , OUTPUT);
  pinMode(outputMuxInhibtPin , OUTPUT);

  digitalWrite(outputMuxInhibtPin, HIGH); // Disable mux (no key pressed on keypad)

  pinMode(caVinputPin, INPUT);

  operationMode = JOYSTICK;  // TODO implement PS/2 mouse trackball emulation

  // Setup Timer2
  TCCR2A = (0 << WGM20) // WGM[2..0] = 010 CTC mode, counts up overflow on OCR2A
           | (1 << WGM21); //
  TCCR2B = (0 << WGM22) //
           | (0 << CS20) // CS[2..0] = 010 Prescaler clock/8
           | (1 << CS21) //
           | (0 << CS22); //
  OCR2A = 127;          // reload value for 15748Hz (63.5us)

  TIMSK2 = (0 << OCIE2B) //  Enable interrupt on match
           | (1 << OCIE2A) //
           | (0 << TOIE2); //

  // Setup Analog Comparator
  ADCSRB = 0;                   // (Disable) ACME: Analog Comparator Multiplexer Enable
  ACSR = (1 << ACI)             // (Clear) Analog Comparator Interrupt Flag
         | (1 << ACIE)           // Analog Comparator Interrupt Enable
         | (1 << ACIS1) | (1 << ACIS0); // Trigger on rising edge


  sei();  // enable interrupts
#ifdef DEBUG
  Serial.begin(9600);
#endif
}




/*******************************************************************************
        __  __      _        _
       |  \/  |__ _(_)_ _   | |   ___  ___ _ __
       | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ \
       |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
                                          |_|
*/
void loop() {
  controller.begin();
  if (controller.connect()) { // valid controller was connected
    while (controller.update()) {
      ExtensionType conType = controller.getControllerType();
      switch (conType) {
        case (ExtensionType::Nunchuk):
          mapNunchuckData();
          break;
        case (ExtensionType::ClassicController):
          mapClassicData();
          break;
        default:
#ifdef DEBUG
          Serial.println("Other controller connected!");
#endif
          disableOutputs();
      } // Switch
      processControllerData();      
      delay(3);  // between controller samples
    } // while
    disableOutputs();
  } else { // Controller not connected}
#ifdef DEBUG
    Serial.println("No controller found!");
#endif
    disableOutputs();
  }
  delay(200);  // after controller sampling reading error
}



/*******************************************************************************
         __              _   _
        / _|_  _ _ _  __| |_(_)___ _ _  ___
       |  _| || | ' \/ _|  _| / _ \ ' \(_-<
       |_|  \_,_|_||_\__|\__|_\___/_||_/__/

*/

void setKeypad ( uint8_t keyCode) {
#ifdef DEBUG
  char key[16] = {'1', '2', '3', 'S', '4', '5', '6', 'P', '7', '8', '9', 'R', '*', '0', '#', 'N'};

  Serial.print (" Key:");
  Serial.print (keyCode);
  Serial.print ("->");
  Serial.println (key[keyCode]);
#endif
  if (keyCode & (1 << 3) ) digitalWrite(outputMuxBpin, HIGH); else digitalWrite(outputMuxBpin, LOW);
  if (keyCode & (1 << 2) ) digitalWrite(outputMuxApin, HIGH); else digitalWrite(outputMuxApin, LOW);

  if (keyCode & (1 << 1) ) digitalWrite(inputMuxBpin, HIGH); else digitalWrite(inputMuxBpin, LOW);
  if (keyCode & (1 << 0) ) digitalWrite(inputMuxApin, HIGH); else digitalWrite(inputMuxApin, LOW);

  if ( keyCode == _keyNone ) {
    digitalWrite(outputMuxInhibtPin, HIGH); // inhibt output
  } else {
    digitalWrite(outputMuxInhibtPin, LOW);
  }
}


//
//
void assertTopFire2(void) {
  digitalWrite(fireTop2pin, LOW); // turn off pullup
  pinMode(fireTop2pin, OUTPUT);
}

//
//
void releaseTopFire2(void) {
  pinMode(fireTop2pin, INPUT);
  digitalWrite(fireTop2pin, HIGH); // turn on pullup
}

//
//
void assertBottomFire2(void) {
  digitalWrite(fireBottom2pin, LOW); // turn off pullup
  pinMode(fireBottom2pin, OUTPUT);
}

//
//
void releaseBottomFire2(void) {
  pinMode(fireBottom1pin, INPUT);
  digitalWrite(fireBottom2pin, HIGH); // turn on pullup
}

//
//
void assertTopFire1(void) {
  digitalWrite(fireTop1pin, LOW); // turn off pullup
  pinMode(fireTop1pin, OUTPUT);
}

//
//
void releaseTopFire1(void) {
  pinMode(fireTop1pin, INPUT);
  digitalWrite(fireTop1pin, HIGH); // turn on pullup
}

//
//
void assertBottomFire1(void) {
  digitalWrite(fireBottom1pin, LOW); // turn off pullup
  pinMode(fireBottom1pin, OUTPUT);
}

//
//
void releaseBottomFire1(void) {
  pinMode(fireBottom1pin, INPUT);
  digitalWrite(fireBottom1pin, HIGH); // turn on pullup
}

//
//
void disableOutputs() {
#ifdef DEBUG
  Serial.println("Disable Outputs");
#endif
  potX1value = 255;
  potY1value = 255;
  potX2value = 255;
  potY2value = 255;
}


//
//
void mapNunchuckData() {

  uint8_t leftXaxis, leftYaxis, rightXaxis, rightYaxis;

  //  Get button State
  combinedButtons = 0;
  if (nchuk.buttonZ()      ) combinedButtons |= btnA;     // (1<<4 )
  if (nchuk.buttonC()      ) combinedButtons |= btnB;     // (1<<5 )

  // Get Analog Values
  leftXaxis = nchuk.joyX() ;    
  leftYaxis = 255 - nchuk.joyY() ;  // Invert Y Axis

  //map(value, fromLow, fromHigh, toLow, toHigh)
  rightXaxis = map( nchuk.rollAngle(), -180, 180, 0, 255);
  rightYaxis = map(nchuk.pitchAngle(), -180, 180, 0, 255);

  singleControllerX = leftXaxis;
  dualControllerX1  = leftXaxis;

  singleControllerY = leftYaxis;
  dualControllerY1  = leftYaxis;  
  
  dualControllerX2  = rightXaxis;
  dualControllerY2  = rightYaxis;
}


//
//
void mapClassicData() {

#define MIN 0
#define MAX 255

  //  Get button State
  combinedButtons = 0;
  if (classic.dpadUp()     ) combinedButtons |= btnUp;    // (1<<0 )
  if (classic.dpadDown()   ) combinedButtons |= btnDown;  // (1<<1 )
  if (classic.dpadLeft()   ) combinedButtons |= btnLeft;  // (1<<2 )
  if (classic.dpadRight()  ) combinedButtons |= btnRight; // (1<<3 )
  if (classic.buttonA()    ) combinedButtons |= btnA;     // (1<<4 )
  if (classic.buttonB()    ) combinedButtons |= btnB;     // (1<<5 )
  if (classic.buttonX()    ) combinedButtons |= btnX;     // (1<<6 )
  if (classic.buttonY()    ) combinedButtons |= btnY;     // (1<<7 )
  if (classic.buttonL()    ) combinedButtons |= btnLFT;   // (1<<8 )
  if (classic.buttonR()    ) combinedButtons |= btnRGT;   // (1<<9 )
  if (classic.buttonZL()   ) combinedButtons |= btnZl;    // (1<<10) -> Mode button
  if (classic.buttonZR()   ) combinedButtons |= btnZr;    // (1<<11)
  if (classic.buttonMinus()) combinedButtons |= btnMinus; // (1<<13)
  if (classic.buttonHome() ) combinedButtons |= btnHome;  // (1<<14)
  if (classic.buttonPlus() ) combinedButtons |= btnPlus;  // (1<<12)



  // Get Analog values -
  // After version 0.8.1 the extension controller libary return in the range 0-255
  // for either original or knockoff controllers.
  //

  // Read analog controller axes and convert them to the -128..+127 range
  int16_t joyLX = classic.leftJoyX() - 128;
  int16_t joyLY = 128 - classic.leftJoyY();   // invert Y axis
  int16_t joyRX = classic.rightJoyX() - 128;
  int16_t joyRY = 128 - classic.rightJoyY();  // invert Y axis

#if DEBUG > 1
  Serial.print(" LX"); Serial.print (joyLX);
  Serial.print(" LY"); Serial.print (joyLY);
  Serial.print(" RX"); Serial.print (joyRX);
  Serial.print(" RY"); Serial.print (joyRY);
#endif

  // Combine Left and Right controllers for use in single controller mode
  int16_t Xcomb = joyLX + joyRX;
  int16_t Ycomb = joyLY + joyRY;

  if (Xcomb > 127) Xcomb = 127;    // saturate in the range -128..+127
  if (Xcomb < -128) Xcomb = -128;

  if (Ycomb > 127) Ycomb = 127;    // saturate in the range -128..+127
  if (Ycomb < -128) Ycomb = -128;


  switch ( combinedButtons & (btnLeft | btnRight) ) { // left / right
    case (btnLeft): // left
      singleControllerX = MIN;
      dualControllerX1 = MIN;
      break;
    case (btnRight): // right
      singleControllerX = MAX;
      dualControllerX1 = MAX;
      break;
    default: // up+down or none, convert value back to the range 0..255
      singleControllerX = 128 + Xcomb;
      dualControllerX1  = 128 + joyLX;
  }


  switch ( combinedButtons & (btnUp | btnDown)) { // up/down
    case (btnUp): // up
      singleControllerY = MIN;
      dualControllerY1 = MIN;
      break;
    case (btnDown): // down
      singleControllerY = MAX;
      dualControllerY1 = MAX;
      break;
    default: // up+down or none, convert value back to the range 0..255
      singleControllerY = 128 + Ycomb;
      dualControllerY1  = 128 + joyLY;
  }

  dualControllerX2  = 128 + joyRX;
  dualControllerY2  = 128 + joyRY;

}


//
//
void  processControllerData() {
  if (combinedButtons & btnMode) {
    combinedButtons &= ~btnMode;  // clear Mode button bit
    switch (combinedButtons) {
      case btnUp:    // keypad 2
        setKeypad (_key2);
        break;
      //        case btnDown:  // None
      //           setKeypad (_keyNone);
      //           break;
      case btnLeft:  // keypad 1
        setKeypad (_key1);
        break;
      case btnRight: // keypad 3
        setKeypad (_key3);
        break;
      case btnA:     // keypad 9
        setKeypad (_key9);
        break;
      case btnB:     // keypad 0
        setKeypad (_key0);
        break;
      case btnX:     // keypad 8
        setKeypad (_key8);
        break;
      case btnY:     // keypad 7
        setKeypad (_key7);
        break;
      //      case btnL:     // None
      //         setKeypad (_keyNone);
      //         break;
      case btnRGT:     // Reset
        setKeypad (_keyReset);
        break;
      //        case btnZl:     // This is the modekey itself
      //           setKeypad (_key6);
      //           break;
      case btnZr:     // Start
        setKeypad (_keyStart);
        break;
      case btnMinus: // keypad 4
        setKeypad (_key4);
        break;
      case btnHome:     // keypad 5
        setKeypad (_key5);
        break;
      case btnPlus:     // keypad 6
        setKeypad (_key6);
        break;

      default:       // None or multiple
        setKeypad (_keyNone);
    } // switch(combinedButtons)

  } else { // Mode button released
    switch (combinedButtons) { // TODO add a mask for other bits.
      case btnMinus:     // keypad *
        setKeypad (_keyAsterisk);
        break;
      case btnHome:     // keypad 0
        setKeypad (_key0);
        break;
      case btnPlus:     // keypad #
        setKeypad (_keyHash);
        break;
      case btnZr: // Pause
        setKeypad (_keyPause);
        break;
      default:       // None or multiple
        setKeypad (_keyNone);
    } // switch

    // Take care of buttons
    if (dualJoystick()) {
      if (combinedButtons &  btnX) assertTopFire2()   ; else releaseTopFire2();
      if (combinedButtons &  btnY) assertBottomFire2(); else releaseBottomFire2();

      if (combinedButtons &  (btnA | btnRGT )) assertTopFire1()   ; else releaseTopFire1();
      if (combinedButtons &  (btnB | btnLFT )) assertBottomFire1(); else releaseBottomFire1();
    } else {
      if (combinedButtons &  (btnA | btnRGT | btnX )) assertTopFire1()   ; else releaseTopFire1();
      if (combinedButtons &  (btnB | btnLFT | btnY )) assertBottomFire1(); else releaseBottomFire1();
    }
  }



  // Check for trackball / joystick query.
  // The system 5200 drops voltage on pin 9 to differentiate a trackball from a joystick.
  // +----------------+-------------+-------------+
  // |Controller/Pin 9| Pot X value | Pot Y value |
  // +----------+-----+-------------+-------------+
  // |          | Vcc |   0 - 227   |   0 - 227   |
  // | Joystick +-----+-------------+-------------+
  // |          |  0  |    > 227    |    >227     |
  // +----------+-----+-------------+-------------+
  // |          | Vcc |   0 - 227   |   0 - 227   |
  // | Trackball+-----+-------------+-------------+
  // |          |  0  |    114      |     114     |
  // +----------+-----+-------------+-------------+
  //


  // associate values 
  switch (operationMode ) {

    case TRACKBALL:
      if (digitalRead(caVinputPin)) { // VAC on, do normal temporization

          // TODO process trackball data
          potX1value = pgm_read_byte(gamma3 + dualControllerX1);
          potY1value = pgm_read_byte(gamma3 + dualControllerY1);

          potX2value =  114; // middle range
          potY2value =  114;         

      }  else {  // VAC off, set middle value
        potX1value = 114;
        potY1value = 114;

        potX2value =  114;
        potY2value =  114;
      } // else

      break;

    case JOYSTICK:
    default:
      if (digitalRead(caVinputPin)) { // VAC on, do normal temporization

        // Populate analog axes information. Values will be used by interrupts
        if (dualJoystick()) {
          // Dual adapter connected, map different values for pot1/pot2
          potX1value = pgm_read_byte(gamma3 + dualControllerX1);
          potY1value = pgm_read_byte(gamma3 + dualControllerY1);

          potX2value =  pgm_read_byte(gamma3 + dualControllerX2);
          potY2value =  pgm_read_byte(gamma3 + dualControllerY2);
        } else {
          // Not using dual controller adapter, use combined stick values
          potX1value = pgm_read_byte(gamma3 + singleControllerX);
          potY1value = pgm_read_byte(gamma3 + singleControllerY);

          potX2value =  128;  // middle range
          potY2value =  128;  // middle range
         
#ifdef DEBUG        
        Serial.print("!");
#endif
          
        } // else
      } else {  // VAC off, do overvalue
       
        potX1value = 228;
        potY1value = 228;

        potX2value =  228;
        potY2value =  228;

#ifdef DEBUG        
        Serial.print("@");
#endif 

        
      } // else
  } // switch


#if DEBUG > 0
  Serial.print(" potX1:"); Serial.print(potX1value);
  Serial.print(" potY1:"); Serial.print(potY1value);
  Serial.print("/potX2:"); Serial.print(potX2value);
  Serial.print(" potY2:"); Serial.print(potY2value);
  Serial.print(" detect:");  Serial.print(dualJoystick());
  Serial.print(" "); Serial.print(combinedButtons, BIN);
  //  Serial.println();
#endif

}




//
