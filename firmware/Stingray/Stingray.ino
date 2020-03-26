/*******************************************************************************

         _____ __  _
        / ___// /_(_)___  ____ __________ ___  __
        \__ \/ __/ / __ \/ __ `/ ___/ __ `/ / / /
       ___/ / /_/ / / / / /_/ / /  / /_/ / /_/ /
      /____/\__/_/_/ /_/\__, /_/   \__,_/\__, /
                       /____/           /____/

  Nintendo Wii Classic controller adapter for Atari 5200
  Danjovic 2020 - danjovic@hotmail.com - https://hackaday.io/danjovic

  13 March 2020

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



/*******************************************************************************
           _      __ _      _ _   _
        __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
       / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
       \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/

*/

//#define DEBUG

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
     _              _   _                         _       _
   / _|_  _ _ _  __| |_(_)___ _ _    _ __ _ _ ___| |_ ___| |_ _  _ _ __  ___ ___
  |  _| || | ' \/ _|  _| / _ \ ' \  | '_ \ '_/ _ \  _/ _ \  _| || | '_ \/ -_|_-<
  |_|  \_,_|_||_\__|\__|_\___/_||_| | .__/_| \___/\__\___/\__|\_, | .__/\___/__/
                                    |_|                       |__/|_|
*/




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

const uint8_t gammaCurve[64] = { // gamma = 2.0
  5  , 12 , 18 , 24 , 30 , 36 , 42 , 47 , 52 , 57 , 62 , 66 , 71 , 75 , 78 , 82 ,
  86 , 89 , 92 , 95 , 97 , 100, 102, 104, 106, 107, 109, 110, 111, 112, 112, 112,
  113, 113, 113, 114, 114, 115, 117, 118, 120, 122, 124, 126, 129, 131, 134, 138,
  141, 145, 149, 153, 157, 162, 167, 172, 177, 182, 188, 194, 200, 207, 213, 220
};


/*******************************************************************************
                     _      _    _
       __ ____ _ _ _(_)__ _| |__| |___ ___
       \ V / _` | '_| / _` | '_ \ / -_|_-<
        \_/\__,_|_| |_\__,_|_.__/_\___/__/

*/
static volatile uint8_t CAVoff;
static volatile uint8_t hline = 0;
static volatile uint8_t potX1value = 0;
static volatile uint8_t potX2value = 0;
static volatile uint8_t potY1value = 0;
static volatile uint8_t potY2value = 0;

int8_t combinedXaxis;  // ponderated value of both stick movement on X axis
int8_t combinedYaxis;  // ponderated value of both stick movement on Y axis
int8_t  leftXaxis;
int8_t rightXaxis;
int8_t  leftYaxis;
int8_t rightYaxis;

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


// Pin change interrupt, trigger by Cav line change state.
ISR (PCINT0_vect) {
  if (digitalRead(caVinputPin)) {  // rising edge ?
    // Enable detection of pokey release pin for charging
    pinMode(potX1pin, INPUT);
    pinMode(potX2pin, INPUT);
    pinMode(potY1pin, INPUT);
    pinMode(potY2pin, INPUT);
    CAVoff = 0;	  // flag that CAV voltage is present
    ACSR |= (1 << ACIE) // reactivate analog comparator
            | (1 << ACI); // clear any pending interrupt bit
  } else { // Falling edge
    // force all outputs to zero
    pinMode(potX1pin, OUTPUT);
    pinMode(potX2pin, OUTPUT);
    pinMode(potY1pin, OUTPUT);
    pinMode(potY2pin, OUTPUT);
    digitalWrite(potX1pin, LOW); 
    digitalWrite(potX2pin, LOW);
    digitalWrite(potY1pin, LOW);
    digitalWrite(potY2pin, LOW);

    hline = 255; // make sure that no action will be taking at Hline interrupt
    CAVoff = 1;	// flag that CAV voltage has dropped
    ACSR |= (1 << ACI); // clear any pending interrupt on analog comparator
  }
}


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
  pinMode(potX1pin, OUTPUT);  // hold axis pin in LOW state
  digitalWrite(potX1pin, LOW);

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

  // Setup Pin Change Interrupt for Cav
  PCICR = (1 << PCIE0) // Pin change interrupt enable for PCINT0..PCINT7
          | (0 << PCIE1) // Pin change interrupt enable for PCINT8..PCINT14
          | (0 << PCIE2); // Pin change interrupt enable for PCINT16..PCINT23

  PCMSK0 = (0 << PCINT0) // Pin change enable for PB0 pin
           | (0 << PCINT1) // Pin change enable for PB1 pin
           | (0 << PCINT2) // Pin change enable for PB2 pin
           | (0 << PCINT3) // Pin change enable for PB3 pin
           | (0 << PCINT4) // Pin change enable for PB4 pin
           | (1 << PCINT5) // Pin change enable for PB5 pin - Vac voltage is connected here
           | (0 << PCINT6) // Pin change enable for PB6 pin
           | (0 << PCINT7); // Pin change enable for PB7 pin


  PCIFR = (1 << PCIF0) // Clear any pending interrupt flag
          | (0 << PCIF1)
          | (0 << PCIF2);


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
    } // while
    disableOutputs();
  } else { // Controller not connected}
#ifdef DEBUG    
    Serial.println("No controller found!");
#endif
    disableOutputs();
  }
  delay(200);
}



/*******************************************************************************
         __              _   _
        / _|_  _ _ _  __| |_(_)___ _ _  ___
       |  _| || | ' \/ _|  _| / _ \ ' \(_-<
       |_|  \_,_|_||_\__|\__|_\___/_||_/__/

*/

void setKeypad ( uint8_t keyCode) {
  char key[16] = {"123S456P789R*0#N"};
#ifdef DEBUG
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
    potX1value = 255;
    potY1value = 255;
    potX2value = 255;
    potY2value = 255;
#endif  
}


//
//
void mapNunchuckData() {

  //  Get button State
  combinedButtons = 0;
  if (nchuk.buttonZ()      ) combinedButtons |= btnA;     // (1<<4 )
  if (nchuk.buttonC()      ) combinedButtons |= btnB;     // (1<<5 )



  // Get Analog Values
  leftXaxis = nchuk.joyX() >> 2;    // get values from left stick [0..63]
  leftYaxis = nchuk.joyY() >> 2;

  //map(value, fromLow, fromHigh, toLow, toHigh)
  rightXaxis = map( nchuk.rollAngle(), -180, 180, 0, 63);
  rightYaxis = map(nchuk.pitchAngle(), -180, 180, 0, 63);

  combinedXaxis = leftXaxis;
  combinedYaxis = leftYaxis;



}

//
//
void mapClassicData() {
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


  // Get Analog Values
  leftXaxis = classic.leftJoyX();       // get values from left stick [0..63]
  leftYaxis = 63-classic.leftJoyY();
  rightXaxis = (classic.rightJoyX() << 1); // get the values for right [0..31]
  rightYaxis = ( (31-classic.rightJoyY()) << 1); // and double then for [0..62]


  // If D-Pad is pressed without a mode key then the D-Pad direction overimposes left X/Y axis
  if ((combinedButtons & btnMode) == 0 ) {
    // vertical axis, inverted (up=63,down=0)
    if ( (combinedButtons & btnUp) && !(combinedButtons & btnDown) )
      leftYaxis = 0;
    else if ( !(combinedButtons & btnUp) && (combinedButtons & btnDown) )
      leftYaxis = 63;

    // horizontal axis, normal (left=0, right=63)
    if ( (combinedButtons & btnLeft) && !(combinedButtons & btnRight) )
      leftXaxis = 0;
    else if ( !(combinedButtons & btnLeft) && (combinedButtons & btnRight) )
      leftXaxis = 63;
  } // btnMode


  // compute resulting X axis position
  combinedXaxis = leftXaxis + rightXaxis - 32;
  if (combinedXaxis > 63) combinedXaxis = 63;
  if (combinedXaxis < 0)  combinedXaxis = 0;

  // compute resulting Y axis position
  combinedYaxis = leftYaxis + rightYaxis - 32;
  if (combinedYaxis > 63) combinedYaxis = 63;
  if (combinedYaxis < 0)  combinedYaxis = 0;

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
  // Take care of directionals


  // Populate analog axes information. Values will be used by interrupts
  if (dualJoystick()) {
    // Dual adapter connected, map different values for pot1/pot2
    potX1value = gammaCurve[uint8_t(leftXaxis)];
    potY1value = gammaCurve[uint8_t(leftYaxis)];

    potX2value = gammaCurve[uint8_t(rightXaxis)];
    potY2value = gammaCurve[uint8_t(rightYaxis)];

  } else {
    // Not using dual controller adapter, use combined stick values
    potX1value = gammaCurve[uint8_t(combinedXaxis)];
    potY1value = gammaCurve[uint8_t(combinedYaxis)];

    potX2value = potX1value;
    potY2value = potY1value;
  }

#ifdef DEBUG
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
