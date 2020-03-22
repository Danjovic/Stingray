# Stingray
Nintendo Classic controller adapter with keypad emulation for Atari 5200

 This project This adapter lets you play Atari 5200 using a Wii Classic controller yet providing full keypad control.
 
Main features are 

-Analog Controller (using precise timing);
-Digital Controller on D-PAD just like a Masterplay adapter;
-Full keypad emulation;
-Auto detected second controller adapter for games like Robotron
-Independent Top/Bottom buttons on second controller. 
	
The circuit is built around an Arduino Nano and two analog multiplexers. 

Interface with Nintendo Classic controller uses I2C communication. Analog joystick information is provided to Atari 5200 using the "time precise" method and uses the internal Analog Comparator of the AVR to detect the moment the Pokey chip releases the capacitors to charge and starts counting time.

The analog multiplexers are used to emulate one of the 15 possible keys pressed. Buttons and controllers are mapped as follows:

![Button Mapping](/doc/mapping.png)

The adapter for the second controller consists in a USB cord with a Male USB type A in one edge and a DB-15 female connector on another. The wiring is below:

![Dual Adapter Wiring](/doc/dualAdapter.jpg)

The circuit also provides a mini-DIN connector with pull up resistors. At the moment this is a provision for emulation of trackball using a PS/2 mouse. 

![pcb](/doc/stingray-pcb.png)
