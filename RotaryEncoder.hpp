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
  
  The driver uses two interrupts, one for the rotary pulses  and one
  for the push button. Either interrupt will put the encoder into the "active"
  state. While active the "scan() method should be called at (max) 3ms intervals
  
  The number of rotary pulses counted is artifically incremented if the 
  knob is rotated quickly.  
  
*/
 
#define DEBOUNCE_INTERVAL 5000  // 5 milliseconds
#define LONG_PRESS_INTERVAL 3000000 //3 seconds
#define ACTIVITY_TIMEOUT 10000000 //10 seconds
#define BUTTON_UP false

#include "TaskScheduler.h"
#include "StateMachine.hpp"

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
       pinA = _pinA; //Rotary "data" 
       pinB = _pinB; //Rotary "clock"
       pinC = _pinC; //Pushbutton

       instance = this;  //Needed by intrrupt handlers        
     }

//Must call this during setup()     
     void begin(bool _accel=true) { 
       pinMode(pinA,INPUT_PULLUP);
       pinMode(pinB,INPUT_PULLUP);
       pinMode(pinC,INPUT_PULLUP);
       accel = _accel;
       attachInterrupt(digitalPinToInterrupt(pinA), encoderIntHandler, RISING); //rotary motion are we only inetrested in one edge
       attachInterrupt(digitalPinToInterrupt(pinC), buttonIntHandler, CHANGE); //push button - we want to time down and up
       
     }        

//Returns number of clicks since previous call     
     int getPulseCount () {  // is positive for clockwise steps, negative for anticlocwise clicks
       int retVal = pulseCount;
       pulseCount = 0;
       return(retVal);
     }
     
//Returns true if there has been recent activity (in last 60 secs)
    bool isActive() {
      return(active);
    }
    
//Called every time through loop() if encoder is active - must be non-blocking and quick
    void scan() {
      long now = 0;
      
      //What time is it now?
      now = micros();
      
      //Check for recent activity
      if ( now > lastActivity + ACTIVITY_TIMEOUT || now < lastActivity ) {
        active = false;
        lastActivity = 0;
      }
    
      //Check for end of de-bounce interval
      if (inDebounceDelay) {
        if ( now > deBounceEnd ) {
          inDebounceDelay = false; 
        } else return; //In debounce - ignore all events
      }

      //Has the button state changed? (recorded by int handler);
      if (buttonDown != buttonState ) {
      Serial.println("Button change");
        if (buttonDown) 
          pressStart = now; //New button press started
        else 
          pressEnd = now; 
        if (!buttonDown) { //Button released
          if ( now - pressStart > LONG_PRESS_INTERVAL )
            encoderEvent = LONGPRESS;
          else
            //Short press event
            encoderEvent = SHORTPRESS;
          eventQueue.push(&encoderEvent);
        } 
        buttonState = buttonDown; //Save current state         
      }
    } // End of scan() method
    
    void dumpState() { //output state variables (for debug)
      char buff[128];
      sprintf(buff, "active: %d, lastActivity %d, inDebounceDelay: %d, buttonDown: %d, buttonState: %d\n",
          active,lastActivity,inDebounceDelay,buttonDown,buttonState);
      Serial.print(buff);
    }

     //Allow ISR access to members
     friend void encoderIntHandler();
     friend void buttonIntHandler();

     //Properties
     volatile int pulseCount;
     uint8_t pinA, pinB, pinC; 
     volatile long deBounceEnd, lastActivity, rotaryPulseStart;
     volatile bool inDebounceDelay = false;
     volatile bool active = false;
     volatile bool buttonDown = BUTTON_UP;
     volatile bool buttonState = BUTTON_UP;
     long pressStart, pressEnd;  //Measures button press
     bool accel = true;
     bool pulseStarted = false;
}; //end of RotaryEncoder class definition

//Interrupt Handlers

//Called on edge (on pinA) - rotary motion
static void encoderIntHandler() {
   int pinBval, increment;
   long now, pulseDuration; 
   bool pulseReceived;
    
   now = micros();
   instance->active = true;
   instance->lastActivity = now;    //Start activity timer
   
   //Main body only executed if not in de-bounce period
   if ( instance->inDebounceDelay == false ) {
   
       // initiate de-bounce delay and set end time
       instance->inDebounceDelay = true;  //DebounceDelay is terminated in scan()
       instance->deBounceEnd = now + DEBOUNCE_INTERVAL; 
      
       // Work out if we have received a complete pulse
       pulseReceived = false;
       if (!instance->pulseStarted) { //start of pulse
         instance->pulseStarted = true;
         instance->rotaryPulseStart = now;
       } else { //end of pulse
         instance->pulseStarted = false;
         pulseDuration = now - instance->rotaryPulseStart;
         pulseReceived = true;
       }
  
       // --- Get the current pin states and increment pulseCount accordingly
       increment = 1;
       if ( pulseReceived ) {
         if(instance->accel)  //calculate increment        
           increment = 1 + (1000000 / (3*pulseDuration)); //If using encoder speed add a factor 
       }
           
       //Need to look at the CLK pin to work out diretion of rotation  
       pinBval = digitalRead(instance->pinB);
       if ( pinBval == 0 ) instance->pulseCount += increment; //clockwise rotation
       else instance->pulseCount -= increment;
       if (instance->pulseCount < 0) instance->pulseCount = 0;
   }
}

//Called on falling and rising edges of the button pin
static void buttonIntHandler() {
  long now;
  
  instance->active = true;
  if ( instance->inDebounceDelay == false ) { 
       // initiate de-bounce delay (ignore further interrupts for a while)
       instance->inDebounceDelay = true;
       now = micros();
       instance->lastActivity = now;
       instance->deBounceEnd = now + DEBOUNCE_INTERVAL;
       instance->buttonDown = !digitalRead(instance->pinC); //record current button position, up or down
   }
}       

#endif

