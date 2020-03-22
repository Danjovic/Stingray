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




/*******************************************************************************
 *         _      __ _      _ _   _             
 *      __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
 *     / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
 *     \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/
 *                                              
 */

//#define DEBUG 1
 
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
#define inputMuxA         8
#define inputMuxB         9
#define outputMuxA        10
#define outputMuxB        11
#define outputMuxInhibt   12

#define potX1             5 
#define potY1             4 
#define fireTop1          3 
#define fireBottom1       2 

#define potX2             A0
#define potY2             A3
#define fireTop2          A2
#define fireBottom2       A1

#define caVinput          13

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
#define btnLFT    (1<<13)
#define btnRGT    (1<<14)
#define btnZr     (1<<14)
#define btnZl     (1<<15)

#define buttonMode buttonZL
/*******************************************************************************
 *                                
 *      _ __  __ _ __ _ _ ___ ___ 
 *     | '  \/ _` / _| '_/ _ (_-< 
 *     |_|_|_\__,_\__|_| \___/__/ 
 *                                
 */
#define pushX1() do { pinMode(potX1,OUTPUT); digitalWrite(potX1,HIGH); } while (0)
#define pushX2() do { pinMode(potX2,OUTPUT); digitalWrite(potX2,HIGH); } while (0)
#define pushY1() do { pinMode(potY1,OUTPUT); digitalWrite(potY1,HIGH); } while (0)
#define pushY2() do { pinMode(potY2,OUTPUT); digitalWrite(potY2,HIGH); } while (0)



/*******************************************************************************
 *   _              _   _                         _       _                     
 * / _|_  _ _ _  __| |_(_)___ _ _    _ __ _ _ ___| |_ ___| |_ _  _ _ __  ___ ___
 *|  _| || | ' \/ _|  _| / _ \ ' \  | '_ \ '_/ _ \  _/ _ \  _| || | '_ \/ -_|_-<
 *|_|  \_,_|_||_\__|\__|_\___/_||_| | .__/_| \___/\__\___/\__|\_, | .__/\___/__/
 *                                  |_|                       |__/|_|           
 */
uint8_t ScanGenesis(void);
void assertTopFire(void);
void releaseTopFire(void);
void assertBottomFire(void);
void releaseBottomFire(void);
void setMinimumY(void);
void setMaximumY(void);
void setMiddleY(void); 
void setMinimumX(void);
void setMaximumX(void);
void setMiddleX(void); 



/*******************************************************************************
 *                     _            _      
 *      __ ___ _ _  __| |_ __ _ _ _| |_ ___
 *     / _/ _ \ ' \(_-<  _/ _` | ' \  _(_-<
 *     \__\___/_||_/__/\__\__,_|_||_\__/__/
 *                                         
 */
enum controllerType {
  _Classic=0,
  _Nunchuck,
  _unKnown =0xff
  };


/*******************************************************************************
 *                   _      _    _        
 *     __ ____ _ _ _(_)__ _| |__| |___ ___
 *     \ V / _` | '_| / _` | '_ \ / -_|_-<
 *      \_/\__,_|_| |_\__,_|_.__/_\___/__/
 *                                        
 */
static volatile uint8_t CAVoff;
static volatile uint8_t hline=0;
static volatile uint8_t potX1value = 0;
static volatile uint8_t potX2value = 0;
static volatile uint8_t potY1value = 0;
static volatile uint8_t potY2value = 0;

uint8_t combinedXaxis;  // ponderated value of both stick movement on X axis
uint8_t combinedYaxis;  // ponderated value of both stick movement on Y axis
uint8_t leftXaxis;      // individual stick values in the range of 5-220 (min-max)
uint8_t rightXaxis;
uint8_t leftXaxis;
uint8_t leftYaxis;

uint16_t combinedButtons = 0;

ExtensionPort controller;
Nunchuk::Shared nchuk(controller);  // Read Nunchuk formatted data from the port
ClassicController::Shared classic(controller);  // Read Classic Controller formatted data from the port


/*******************************************************************************
 *      _     _                         _      
 *     (_)_ _| |_ ___ _ _ _ _ _  _ _ __| |_ ___
 *     | | ' \  _/ -_) '_| '_| || | '_ \  _(_-<
 *     |_|_||_\__\___|_| |_|  \_,_| .__/\__/__/
 *                                |_|          
 */
/*
// External interrupt, triggers every time CAV voltage changes state.
ISR (INT1_vect, ISR_NAKED) { 
  asm volatile (
    "push __tmp_reg__\n\t"         //
    "in __tmp_reg__,__SREG__\n\t"  //  Save Context
    "push __tmp_reg__\n\t"         //  
    "clr __tmp_reg__\n\t"         // temp_reg=0 CAVoff = false
    "sbic %[PORTIN],3 \n\t"       // CAV has dropped?
    "rjmp _STORE\n\t"             // no update value
    "cbi %[PORTPOTS],2\n\t"       // yes, force pin Y to zero
    "sbi %[DDRPOTS],2\n\t"
    "cbi %[PORTPOTS],3\n\t"       // yes, force pin X to zero
    "sbi %[DDRPOTS],3\n\t"
    "inc __tmp_reg__\n\t"         // temp_reg=1 CAVoff = true                        
    "_STORE:\n\t"
     "sts %[flagCAVoff],__tmp_reg__\n\t" // store CAVoff flag value
    "pop __tmp_reg__\n\t"           //
    "out __SREG__,__tmp_reg__\n\t"  // Restore Context
    "pop __tmp_reg__\n\t"           //
    "reti \n\t"
    :[flagCAVoff] "=m"(CAVoff)
    :[PORTIN]"I" (_SFR_IO_ADDR(PIND)),[DDRPOTS]"I" (_SFR_IO_ADDR(DDRB)) ,[PORTPOTS]"I" (_SFR_IO_ADDR(PORTB)) 
    );  
  }
*/

// Pin change interrupt, trigger by Cav line change state.
ISR (PCINT0_vect) {
	if (digitalRead(caVinput)) {  // rising edge ?
       // Enable detection of pokey release pin for charging
       pinMode(potX1,INPUT); 
	   pinMode(potX2,INPUT); 
	   pinMode(potY1,INPUT);	   
	   pinMode(potY2,INPUT);
       CAVoff = 0;	  // flag that CAV voltage is present 	   
       ACSR |= (1<<ACIE)  // reactivate analog comparator
              |(1<<ACI);  // clear any pending interrupt bit	   
 	} else { // Falling edge
	   // Disable detection of pokey release pin for charging
       pushX1();   // force all pins in charged state
       pushX2();
       pushY1();
       pushY2();
	   hline=255;   // make sure that no action will be taking at Hline interrupt
       CAVoff = 1;	// flag that CAV voltage has dropped
	   ACSR |= (1<<ACI);  // clear any pending interrupt on analog comparator	   
	}
}


// Timer 2 interrupt occurs at each 64us
ISR (TIMER2_COMPA_vect) {
  if (hline<227) {
       if (hline == potX1value) pushX1(); 
       if (hline == potX2value) pushX2(); 
       if (hline == potY1value) pushY1(); 
       if (hline == potY2value) pushY2(); 	   
       hline++;
      
    } else if (hline==227) {
       //last line, release all pins to make possible to detect next discharge
       pinMode(potX1,INPUT); 
	   pinMode(potX2,INPUT); 
	   pinMode(potY1,INPUT);	   
	   pinMode(potY2,INPUT);	

       ACSR |= (1<<ACIE)  // reactivate analog comparator
              |(1<<ACI);  // clear any pending interrupt bit
       hline++;           // increment to the next line. will stay in this count       
      }                   // until hline is reset on the next comparation
  } 

// Analog comparator, triggers when detect that pokey released POT inputs to charge
ISR (ANALOG_COMP_vect) {
//  pulseFlag();    
  pinMode(potX1,OUTPUT);   // hold axis pin in LOW state
  digitalWrite(potX1,LOW); 

  hline=0;
  ACSR &=~(1<<ACIE);  // disable comparator
 // ACSR &= (1<<ACI);  // clear pending interrupt bit
}


/*******************************************************************************
 *      ___      _             
 *     / __| ___| |_ _  _ _ __ 
 *     \__ \/ -_)  _| || | '_ \
 *     |___/\___|\__|\_,_| .__/
 *                       |_|   
 */
void setup() {
	
  // setup I/O pins
  pinMode(fireTop1    ,INPUT) ; // Trigger pins all open
  pinMode(fireBottom1 ,INPUT) ; 
  pinMode(fireTop2    ,INPUT) ;
  pinMode(fireBottom2 ,INPUT) ;  
  
  pinMode(potX1,INPUT) ; // Potentiometer pins as inputs 
  pinMode(potY1,INPUT) ; 
  pinMode(potX2,INPUT) ;
  pinMode(potY2,INPUT) ;
  
  digitalWrite(potX1,LOW); // disable pullups on potentiometer pins 
  digitalWrite(potX2,LOW); 
  digitalWrite(potY1,LOW); 
  digitalWrite(potY2,LOW); 
  
  pinMode(outputMuxB,OUTPUT); // All keyboard mux pins as outputs
  pinMode(outputMuxA,OUTPUT); 
  pinMode(inputMuxB ,OUTPUT); 
  pinMode(inputMuxA ,OUTPUT);   
  pinMode(muxInhibt ,OUTPUT);
 
  digitalWrite(muxInhibt,HIGH); // Disable mux (no key pressed on keypad)
 
  pinMode(caVinput,INPUT);
 
   // Setup Timer2
  TCCR2A = (0<<WGM20)   // WGM[2..0] = 010 CTC mode, counts up overflow on OCR2A
          |(1<<WGM21);  //
  TCCR2B = (0<<WGM22)   //
          |(0<<CS20)    // CS[2..0] = 010 Prescaler clock/8
          |(1<<CS21)    //
          |(0<<CS22);   //           
  OCR2A = 127;          // reload value for 15748Hz (63.5us)
 
  TIMSK2 = (0<<OCIE2B)  //  Enable interrupt on match
          |(1<<OCIE2A)  //
          |(0<<TOIE2);  //

  // Setup Analog Comparator
  ADCSRB = 0;                   // (Disable) ACME: Analog Comparator Multiplexer Enable
  ACSR = (1<<ACI)               // (Clear) Analog Comparator Interrupt Flag
        |(1<<ACIE)              // Analog Comparator Interrupt Enable
        |(1<<ACIS1)|(1<<ACIS0); // Trigger on rising edge

  // Setup Pin Change Interrupt for Cav 
  PCICR = (1<<PCIE0)   // Pin change interrupt enable for PCINT0..PCINT7 
        | (0<<PCIE1)   // Pin change interrupt enable for PCINT8..PCINT14 
        | (0<<PCIE2);  // Pin change interrupt enable for PCINT16..PCINT23

  PCMSK0 = (0<<PCINT0)  // Pin change enable for PB0 pin 
         | (0<<PCINT1)  // Pin change enable for PB1 pin 
         | (0<<PCINT2)  // Pin change enable for PB2 pin 
         | (0<<PCINT3)  // Pin change enable for PB3 pin 
         | (0<<PCINT4)  // Pin change enable for PB4 pin 
         | (1<<PCINT5)  // Pin change enable for PB5 pin - Vac voltage is connected here
         | (0<<PCINT6)  // Pin change enable for PB6 pin 
         | (0<<PCINT7); // Pin change enable for PB7 pin   


  PCIFR = (1<<PCIF0)   // Clear any pending interrupt flag 
        | (0<<PCIF1)  
        | (0<<PCIF2); 		

 
  sei();  // enable interrupts
#ifdef DEBUG
  Serial.begin(9600);
#endif
}




/*******************************************************************************
 *      __  __      _        _                  
 *     |  \/  |__ _(_)_ _   | |   ___  ___ _ __ 
 *     | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ \
 *     |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
 *                                        |_|   
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
          Serial.println("Other controller connected!");
          disableOutputs();
      } // Switch
      processControllerData();
    } // while
    disableOutputs();
  } else { // Controller not connected}
    Serial.println("No controller found!");
    disableOutputs();
  }
  delay(200);
}



/*******************************************************************************
 *       __              _   _             
 *      / _|_  _ _ _  __| |_(_)___ _ _  ___
 *     |  _| || | ' \/ _|  _| / _ \ ' \(_-<
 *     |_|  \_,_|_||_\__|\__|_\___/_||_/__/
 *                                         
 */

void setKeypad ( uint8_t keyCode) {
  char key[16]={"123S456P789R*0#N"};
#ifdef DEBUG  
  Serial.print (" Key:");
  Serial.print (keyCode);
  Serial.print ("->");  
  Serial.println (key[keyCode]);
#endif
  if (keyCode & (1<<3) ) digitalWrite(outputMuxB,HIGH); else digitalWrite(outputMuxB,LOW);
  if (keyCode & (1<<2) ) digitalWrite(outputMuxA,HIGH); else digitalWrite(outputMuxA,LOW); 
  
  if (keyCode & (1<<1) ) digitalWrite(inputMuxB,HIGH); else digitalWrite(inputMuxB,LOW);
  if (keyCode & (1<<0) ) digitalWrite(inputMuxA,HIGH); else digitalWrite(inputMuxA,LOW);

  if ( keyCode == _keyNone ) {
    digitalWrite(outputMuxInhibt,HIGH);  // inhibt output 
  } else {
    digitalWrite(outputMuxInhibt,LOW);   
  }
}

//
//
void setMinimumX(void) {
  if (!CAVoff) {
    digitalWrite(potXfull,HIGH);
    pinMode(potXfull,OUTPUT);
  }
}

//
//
void setMaximumX(void) {
  if (!CAVoff) {
    pinMode(potXhalf,INPUT);
    digitalWrite(potXhalf,LOW); // turn off pullup
    pinMode(potXfull,INPUT);
    digitalWrite(potXfull,LOW);  // turn off pullup
  } 
}

//
//
void setMiddleX(void){
  if (!CAVoff) {
    pinMode(potXfull,INPUT);
    digitalWrite(potXfull,LOW);  // turn off pullup   
    digitalWrite(potXhalf,HIGH);    
    pinMode(potXhalf,OUTPUT);
  } 
}

//
//
void setMinimumY(void) {
  if (!CAVoff) {
    digitalWrite(potYfull,HIGH);
    pinMode(potYfull,OUTPUT);
  }
}

//
//
void setMaximumY(void) {
  if (!CAVoff) {
    pinMode(potYhalf,INPUT);
    digitalWrite(potYhalf,LOW); // turn off pullup
    pinMode(potYfull,INPUT);
    digitalWrite(potYfull,LOW);  // turn off pullup
  } 
}

//
//
void setMiddleY(void){
  if (!CAVoff) {
    pinMode(potYfull,INPUT);
    digitalWrite(potYfull,LOW);  // turn off pullup   
    digitalWrite(potYhalf,HIGH);    
    pinMode(potYhalf,OUTPUT);
  } 
}

//
//
void assertTopFire(void) {
  digitalWrite(fireTop,LOW); // turn off pullup
  pinMode(fireTop,OUTPUT);   
}

//
//
void releaseTopFire(void) {
  pinMode(fireTop,INPUT);
  digitalWrite(fireTop,HIGH); // turn on pullup
}

//
//
void assertBottomFire(void) {
  digitalWrite(fireBottom,LOW); // turn off pullup
  pinMode(fireBottom,OUTPUT);   
}

//
//
void releaseBottomFire(void) {
  pinMode(fireBottom,INPUT);
  digitalWrite(fireBottom,HIGH); // turn on pullup
}

//
//
void disableOutputs() {
  Serial.println("Disable Outputs");
}

//
//


//
//
void mapNunchuckData() { 
    nchuk.printDebug();  
}

//
//
void mapClassicData() {

//  deal with buttons
combinedButtons=0;
if (classic.dpadUp()     ) combinedButtons |= btnUp;    // (1<<0 )
if (classic.dpadDown()   ) combinedButtons |= btnDown;  // (1<<1 )
if (classic.dpadLeft()   ) combinedButtons |= btnLeft;  // (1<<2 )
if (classic.dpadRight(   ) combinedButtons |= btnRight; // (1<<3 )
if (classic.buttonA()    ) combinedButtons |= btnA;     // (1<<4 )
if (classic.buttonB()    ) combinedButtons |= btnB;     // (1<<5 )
if (classic.buttonX()    ) combinedButtons |= btnX;     // (1<<6 )
if (classic.buttonY()    ) combinedButtons |= btnY;     // (1<<7 )
if (classic.buttonL()    ) combinedButtons |= btnL;     // (1<<8 )
if (classic.buttonR()    ) combinedButtons |= btnR;     // (1<<9 )
if (classic.buttonZL()   ) combinedButtons |= btnZL;    // (1<<10) -> Mode button
if (classic.buttonZR()   ) combinedButtons |= btnZR;    // (1<<11)
if (classic.buttonMinus()) combinedButtons |= btnMinus; // (1<<13)
if (classic.buttonHome() ) combinedButtons |= btnHome;  // (1<<14)
if (classic.buttonPlus() ) combinedButtons |= btnPlus;  // (1<<12)

// deal with POTs


}

//
//
void  processControllerData() {
if (combinedButtons & buttonMode) {
      combinedButtons &= ~buttonMode;  // clear Mode button bit 
      switch(combinedButtons) {
        case buttonUp:    // keypad 2
           setKeypad (_key2);
           break;
//        case buttonDown:  // None
//           setKeypad (_keyNone);
//           break;
        case buttonLeft:  // keypad 1
           setKeypad (_key1);
           break;
        case buttonRight: // keypad 3
           setKeypad (_key3);
           break;
        case buttonA:     // keypad 9
           setKeypad (_key9);
           break; 
        case buttonB:     // keypad 0
           setKeypad (_key0);
           break;
        case buttonX:     // keypad 7
           setKeypad (_key7);
           break;
        case buttonY:     // keypad 8
           setKeypad (_key8);
           break;    
//      case buttonL:     // None    
//         setKeypad (_keyNone);
//         break;  
        case buttonR:     // Reset
           setKeypad (_keyReset);
           break;
//        case buttonZL:     // This is the modekey itself
//           setKeypad (_key6);
//           break;  
        case buttonZR:     // Pause
           setKeypad (_keyPause);
           break;    
        case buttonMinus: // keypad 4
           setKeypad (_key4);
           break;
        case buttonHome:     // keypad 5
           setKeypad (_key5);
           break;  
        case buttonPlus:     // keypad 6
           setKeypad (_key6);
           break;

        default:       // None or multiple
           setKeypad (_keyNone);    
      } // switch(combinedButtons)

    } else { // Mode button released
      switch(combinedButtons) {  // TODO add a mask for other bits.
        case buttonMinus:     // keypad * 
           setKeypad (_keyAsterisk);
           break;    
        case buttonHome:     // keypad 0
           setKeypad (_key0);
           break;    
        case buttonPlus:     // keypad #
           setKeypad (_keyHash);
           break;
        case buttonZR: // Pause
           setKeypad (_keyPause);
           break;
        default:       // None or multiple
           setKeypad (_keyNone);
      } // switch
      
      // Take care of buttons
  if (dualJoystick) {
     if (combinedButtons &  buttonX) assertTopFire2()   ; else releaseTopFire2();
      if (combinedButtons &  buttonY) assertBottomFire2(); else releaseBottomFire2();

      if (combinedButtons &  (buttonA | buttonR ) assertTopFire1()   ; else releaseTopFire1();
      if (combinedButtons &  (buttonB | buttonR ) assertBottomFire1(); else releaseBottomFire1();
  } else {
       if (combinedButtons &  (buttonA | buttonL | buttonX ) assertTopFire1()   ; else releaseTopFire1();
       if (combinedButtons &  (buttonB | buttonR | buttonY ) assertBottomFire1(); else releaseBottomFire1();
  }
} 
      // Take care of directionals
  
   if ( (combinedButtons & buttonUp) & !(combinedButtons & buttonDown) )
      setYpot(5);
   else if ( !(combinedButtons & buttonUp) & (combinedButtons & buttonDown) )
           setYpot(220);
   else if (!dualJoystick) {
           setYpot(combinedYaxis ); // set at sum of both y axes
    } else { // Dual Joystick
        setYpot ( rightYaxis );  // set only value from right stick for dual joystick operation    
    }

   if ( (combinedButtons & buttonLeft) & !(combinedButtons & buttonRight) )
      setXpot(5);
   else if ( !(combinedButtons & buttonLeft) & (combinedButtons & buttonRight) )
           setXpot(220);
   else if (!dualJoystick) {
           setXpot( combinedXaxis ); // set at sum of both y axes
   } else { // Dual Joystick
       setXpot ( rightXaxis );  // set only value from right stick for dual joystick operation    
   }

   // deal with pot values for dual mode 
   if (dualJoystick) {
   setYpot2(leftYaxis);
   setXpot2(leftXaxis);
   } else {
   setYpot2(227);
   setXpot2(227);
   }
}




//
