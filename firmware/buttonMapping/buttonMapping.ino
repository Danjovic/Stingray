/*
  Stingray  -  Controller Logic
*/

#include <NintendoExtensionCtrl.h>

ExtensionPort controller;
Nunchuk::Shared nchuk(controller);  // Read Nunchuk formatted data from the port
ClassicController::Shared classic(controller);  // Read Classic Controller formatted data from the port


#define pinDetectDual A0

#define thresholdDetectDual 256


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

uint8_t gammaCurve[64] = { // gamma = 2.0
5  , 12 , 18 , 24 , 30 , 36 , 42 , 47 , 52 , 57 , 62 , 66 , 71 , 75 , 78 , 82 , 
86 , 89 , 92 , 95 , 97 , 100, 102, 104, 106, 107, 109, 110, 111, 112, 112, 112, 
113, 113, 113, 114, 114, 115, 117, 118, 120, 122, 124, 126, 129, 131, 134, 138, 
141, 145, 149, 153, 157, 162, 167, 172, 177, 182, 188, 194, 200, 207, 213, 220 
};



uint16_t combinedButtons = 0;
uint8_t potX1,potX2,potY1,potY2;
  int8_t combinedXaxis;  // ponderated value of both stick movement on X axis
  int8_t combinedYaxis;  // ponderated value of both stick movement on Y axis
  int8_t leftXaxis;      
  int8_t leftYaxis;
  int8_t rightXaxis;
  int8_t rightYaxis; 


#define DEBUG


void setup() {
#ifdef DEBUG  
  Serial.begin(9600);
#endif  
  controller.begin();
}

void loop() {
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

void disableOutputs() {
#ifdef DEBUG
  Serial.println("Disable Outputs");
    potX1 = 255;
    potY1 = 255;

    potX2 = 255;
    potY2 = 255;
#endif  
}

void  processControllerData() {
//  Serial.println("Process Data");
  // Populate analog axes information. Values will be used by interrupts
  if (analogRead(pinDetectDual)< thresholdDetectDual) { 
    // Dual adapter connected, map different values for pot1/pot2
    potX1 = gammaCurve[uint8_t(leftXaxis)];
    potY1 = gammaCurve[uint8_t(leftYaxis)];

    potX2 = gammaCurve[uint8_t(rightXaxis)];
    potY2 = gammaCurve[uint8_t(rightYaxis)];

    } else {
      // Not using dual controller adapter, use combined stick values
      potX1 = gammaCurve[uint8_t(combinedXaxis)];
      potY1 = gammaCurve[uint8_t(combinedYaxis)];
  
      potX2 = potX1;
      potY2 = potY1;      
      }  

#ifdef DEBUG
   Serial.print(" potX1:"); Serial.print(potX1);
   Serial.print(" potY1:"); Serial.print(potY1);
   Serial.print("/potX2:"); Serial.print(potX2);
   Serial.print(" potY2:"); Serial.print(potY2);  
   Serial.print(" detect:");  Serial.print((analogRead(pinDetectDual)< thresholdDetectDual));   
   Serial.print(" ");Serial.print(combinedButtons,BIN);
   Serial.println();
#endif
  
}

void mapNunchuckData() { 
  
  //  Get button State
  combinedButtons=0;
  if (nchuk.buttonZ()      ) combinedButtons |= btnA;     // (1<<4 )
  if (nchuk.buttonC()      ) combinedButtons |= btnB;     // (1<<5 )



  // Get Analog Values 
  leftXaxis=nchuk.joyX()>>2;        // get values from left stick [0..63]
  leftYaxis=nchuk.joyY()>>2;
  
 //map(value, fromLow, fromHigh, toLow, toHigh)    
  rightXaxis= map( nchuk.rollAngle(),-180,180,0,63);
  rightYaxis= map(nchuk.pitchAngle(),-180,180,0,63);

  combinedXaxis=leftXaxis;
  combinedYaxis=leftYaxis;
 

    
}

void mapClassicData() {
  //  Get button State
  combinedButtons=0;
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
  leftXaxis=classic.leftJoyX();         // get values from left stick [0..63]
  leftYaxis=classic.leftJoyY(); 
  rightXaxis=(classic.rightJoyX()<<1);  // get the values for right [0..31]
  rightYaxis=(classic.rightJoyY()<<1);  // and double then for [0..62]


  // If D-Pad is pressed without a mode key then the D-Pad direction overimposes left X/Y axis
  if ((combinedButtons & btnMode)==0 ) {
   // vertical axis, inverted (up=63,down=0)
   if ( (combinedButtons & btnUp) && !(combinedButtons & btnDown) )
       leftYaxis=63;
   else if ( !(combinedButtons & btnUp) && (combinedButtons & btnDown) )
           leftYaxis=0;

   // horizontal axis, normal (left=0, right=63)
   if ( (combinedButtons & btnLeft) && !(combinedButtons & btnRight) )
      leftXaxis=0;
   else if ( !(combinedButtons & btnLeft) && (combinedButtons & btnRight) )
           leftXaxis=63; 
    } // btnMode


  // compute resulting X axis position
  combinedXaxis = leftXaxis+rightXaxis-32;
  if (combinedXaxis>63) combinedXaxis = 63;
  if (combinedXaxis<0)  combinedXaxis = 0;

  // compute resulting Y axis position
  combinedYaxis = leftYaxis+rightYaxis-32;
  if (combinedYaxis>63) combinedYaxis = 63;
  if (combinedYaxis<0)  combinedYaxis = 0;
  combinedYaxis = 63-combinedYaxis;    // invert Y (make up=0, down=63)
  
}  
