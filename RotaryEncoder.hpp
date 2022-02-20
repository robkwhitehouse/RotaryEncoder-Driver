#ifndef RotaryEncoder_hpp
#define RotaryEncoder_hpp
/*
  Device handler for a rotary encoder
  R K Whitehouse - Nov 2021
  
  The encoder requires two input pins. It will send a stream of pulses
  to each pin while being rotated. The number of pulses indicates the degree
  of rotation. Typically around 20-25 per full rotation.
  The two streams of  pulses are identical except for the relative phase.
  If the pulses on pin A are in advance of those on pin B then it is a clockwise rotation and vice-versa.
  The pulse duration is a minimum of around 20ms (knob turned very quickly) and a
  practical maximum around 200ms (knob turned slowly).
  Although in theory there is no maximum duration.
  
  Most endcoders are mechanical and tend to suffer (sometimes very badly) from contact bounce
  So it is essential to perform some form of filtering to remove higher freqency transient pulses
  This should be done in both hardware (with a capacitor) and in software (here).
  
  Typically these transient pulses will be less than 100us duration.
  
  This driver assumes that anything with duration greater than 5ms is a valid pulse and
  ignores anything with shorter duration. This can be tweaked if necessary (see below). 
  
  The driver is not interrupt driven and the "scan() method should be called every  millisecond 
  (you might get awy with doing this every 2ms)
  
*/
 
#define MIN_PERIOD 5  // milliseconds
#include "TaskScheduler.h"

//Forward declarations
class RotaryEncoder;
RotaryEncoder *instance;
static void encoderIntHandler();
static void buttonIntHandler();
static void enableDebounceDelayTerminate();
extern Scheduler runner;

// -- Main class definition 
class RotaryEncoder {
   public:
     //  -- constructor
     RotaryEncoder(uint8_t _pinA, uint8_t _pinB, uint8_t _pinC) {
       pinA = _pinA;
       pinB = _pinB;
       pinC = _pinC;
       pinMode(pinA,INPUT_PULLUP);
       pinMode(pinB,INPUT_PULLUP);
       pinMode(pinC,INPUT_PULLUP);
       instance = this;         
     }

//Must call this during setup()     
     void begin() { 
       attachInterrupt(digitalPinToInterrupt(pinA), encoderIntHandler, FALLING); 
       attachInterrupt(digitalPinToInterrupt(pinC), buttonIntHandler, LOW);
     }        

//Returns number of clicks since previous call     
     int getPulseCount () {  // is positive for clockwise steps, negative for anticlocwise clicks
       int retVal = pulseCount;
       pulseCount = 0;
       return(retVal);
     }

//Allow ISR access to members
     friend void encoderIntHandler();

     volatile int pulseCount;
     uint8_t pinA, pinB, pinC; 
     volatile long deBounceStart, deBounceEnd;
     volatile bool inDebounceDelay = false;
};


static void encoderIntHandler() {
       int pinBval;  
       
   if ( instance->inDebounceDelay == false ) { 
       // initiate de-bounce delay (ignore further interrupts for 2ms)
       instance->inDebounceDelay = true;
       instance->deBounceStart = micros();
       instance->deBounceEnd = instance->deBounceStart + 1000;   
       // --- Get the current pin states and increment pulseCount accordingly
       pinBval = digitalRead(instance->pinB);
       if ( pinBval == 0 ) instance->pulseCount++; //clockwise rotation
       else instance->pulseCount--;
   }
}

static void buttonIntHandler() {

  if ( instance->inDebounceDelay == false ) { 
       // initiate de-bounce delay (ignore further interrupts for 2ms)
       instance->inDebounceDelay = true;
       instance->deBounceStart = micros();
       instance->deBounceEnd = instance->deBounceStart + 1000;
   }
}       

#endif

