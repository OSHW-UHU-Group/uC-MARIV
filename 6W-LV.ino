
#include <BasicStepperDriver.h>        // for 2-wired drived bipolar stepper motors
#include <Stepper.h>                   // for 4-wired unipolar and bipolar stepper motors
#include <Servo.h>

/* Code for automatic actuation of 6 way - linear actuated injection valve
 * developed by J.D. Mozo for the Open Source Hardware Group at the Universidad de Huelva www.uhu.es/OSHW/
 * and the Aplied Electrochemistry Department of the Scientific and Technological Research Center of Huelva www.uhu.es/CCTH/
 * more info about wiring and hardware associated to this sketch can be found at www.uhu.es/OSHW/Projects/6w-valve.htm
 * 
 * BasicStepperDriver library is licensed under the MIT license (Copyright (c) 2015 Laurentiu Badea) https://github.com/laurb9/StepperDriver/
 * Stepper library is the general Arduino library https://www.arduino.cc/en/Reference/Stepper
 * 
 * October, 2021. CC BY-NC-SA v.4 licensed
 */

//pin assignement
const int SwitchPinLOAD = 6;           // pin for switch connection LOAD position (stand-alone operation without PC)
const int SwitchPinINJ = 7;            // pin for switch connection INJECT position (stand-alone operation without PC)
const int SolenoidPin = 9;             // pin for solenoid activation (This pin must be boosted with an amplifier)
const int ServoPin = 9;                // pin for servo activation
const int Stepper1 = 9;                // pin A (A1) for stepper motor control
const int Stepper2 = 10;               // pin B (A2) for stepper motor control
const int Stepper3 = 11;               // pin C (B1) for stepper motor control
const int Stepper4 = 12;               // pin D (B2) for stepper motor control
const int StepPin = 11;                // pin to make a Step DRV8825
const int DirPin = 12;                 // pin for Direction DRV8825
const int EnablePin = 9;               // ENABLE pin for DC motor
const int DC1Pin = 10;                 // pins for setting direction at DC motor
const int DC2Pin = 11;                 // pins for setting direction at DC motor 

//other constants
const char MODE = 'd';                 // automation mode (m = manual; s=solenoid; v=servo; p=stepper; r=DRV8825; d=DC motor)
const int ServoPosI = 15;              // stores servo position for INJECTION setting (ADJUST VALUE TO CHANGE STATE)
const int ServoPosL = 55;              // stores servo position for LOAD setting (ADJUST VALUE TO CHANGE STATE)
const int StepsNum = 72;               // steps number to change valve position with 2-wire stepper motor (ADJUST VALUE TO CHANGE STATE)
const float RPM2 = 68.0;               // stores speed rotation for 2-wired Steeper Motors in RPM (ADJUST VALUE FOR FASTEST MOVEMENT)
const int SetSteps = 72;               // steps number to change valve position with 4-wire stepper motor (ADJUST VALUE TO CHANGE STATE)
const long RPM4 = 68;                  // stores speed rotation for 4-wired Steeper Motors in RPM (ADJUST VALUE FOR FASTEST MOVEMENT)
const int ActiveTime = 2000;           // time to activate the DC motor (msec) (ADJUST VALUE TO CHANGE STATE)

//variable declaration
bool State = false;                    // valve state (false = LOAD; true = INJECTION)
bool PrevState = false;
char serialData = 0;                   // stores data read from USB (one byte 0 - 255)
bool Direction = false;                // stores rotation direction on DC motor(false = CW; true = CCW)

Servo myservo;                                                         // creates servo object 
Stepper myStepper = Stepper(200,Stepper1,Stepper2,Stepper3,Stepper4);  // creates 4-wire stepper object
BasicStepperDriver DRVstepper(200, DirPin, StepPin);                   // creates 2-wire stepper object

/* USB command set  ------------------------------------------------------------------------------
 *    t       == toggle                            toggle the valve state
 */


void setup() {
  // initialize the digital pins as input or output.
  //             ------------------------------      switch pins for stand alone function
  pinMode(SwitchPinLOAD, INPUT_PULLUP);              // Only two connection are needed (digital pin and GND)
  pinMode(SwitchPinINJ, INPUT_PULLUP);               // Only two connection are needed (digital pin and GND)
  
  switch (MODE) {
    case 's':  //------------------------------      initialize solenoid
      pinMode(SolenoidPin, OUTPUT);                  // This pin must be boosted with an amplifier
      break;
    case 'v':  //------------------------------      intialize servo motor
      pinMode(ServoPin,OUTPUT);
      myservo.attach(ServoPin);
      myservo.write(ServoPosI);
      break;
    case 'p':  //------------------------------      initialize 4wire stepper motor (bipolar and unipolar)
      myStepper.setSpeed(RPM4);                      // in RPM
      break;
    case 'r':  //------------------------------      initialize 2wire driver stepper motor
      DRVstepper.begin(RPM2,1);                      // RPM and MICROSTEPPING (microstepping is hardwired)
      break;
    case 'd':  //------------------------------      initialize DC motor
      pinMode(EnablePin, OUTPUT);
      pinMode(DC1Pin,OUTPUT);
      pinMode(DC2Pin,OUTPUT);
      digitalWrite (EnablePin, LOW);
      break;
  }  
  
  // initialize the serial port
  Serial.begin(9600);

}

void loop() {
  /* check if data is waiting on serial port         --------------------------------------------------------
   *  For synchronization purposes, the state can be changed towards the Arduino USB port. 
   *  If Toggle switches are mounted there are mandatories, to use the USB control they must be removed.
   *  For more information about toggle switches wiring you can visit www.uhu.es/OSHW/Projects/6w-valve.htm
   *  Alternative wiring for making compatible both methods can be implemented by using two independent pushbuttons. 
   */
   
  while (Serial.available() > 0) {
    serialData = Serial.read();
    switch (serialData) {
      case 't':                                      // toggle state (ONLY IF SWITCHES ARE UNMOUNTED)
        State = !State;
        Serial.print("Ok, ");                        // Handshaking Ok
        Serial.print(serialData);
        Serial.println(State, DEC);
        break;
    }
    int serialDatatmp = Serial.read();               // empty serial port (CR)
  }

  // check if switches are pressed to change State   --------------------------------------------------------
  if (digitalRead(SwitchPinLOAD) == LOW && State){   // only if LOAD is required and State is INJ = true
    State = false;}                                  // rest position (load valve)
  if (digitalRead(SwitchPinINJ) == LOW && !State){   // only if INJ is required and State is LOAD = false
    State = true;}                                   // activated position (inject)

  
  // activate or not according MODE and State        --------------------------------------------------------
  switch (MODE) {
    case 'm':  //------------------------------      manual mode (nothing to do)
      break;
    case 's':  //------------------------------      solenoid actuated --------------------------------------checked
      if (State){                                    // if State = true
        digitalWrite(SolenoidPin, HIGH);}
      else {                                         // if State = false
        digitalWrite(SolenoidPin, LOW);}
      break;
    case 'v':  //------------------------------      servo actuated -----------------------------------------checked
      if (State){                                    // set angle according State
        myservo.write(ServoPosI);}                   // INJ STATE
      else {                                         
        myservo.write(ServoPosL);}                   // LOAD STATE 
      break;
    case 'p':  //------------------------------      4-wire stepper actuated --------------------------------checked (unip)
      if (State != PrevState){                       // actuates only when State has changed
        PrevState = State;
        if (State){                                  // set direction according State
          myStepper.step(SetSteps);}
        else {
          myStepper.step(-SetSteps);}
      }
      break;
    case 'r':  //------------------------------      2-wire stepper motor actuated --------------------------checked
      if (State != PrevState){                       // actuates only when State has changed
        PrevState = State;
        if (State){                                  // set direction according State
          DRVstepper.move(StepsNum);}
        else {
          DRVstepper.move(-StepsNum);}
      }
      break;
    case 'd':  //------------------------------      DC motor actuated --------------------------------------checked
      if (State){                                    // set direction according State value
        digitalWrite (DC1Pin, HIGH);
        digitalWrite (DC2Pin, LOW);}
      else {
        digitalWrite (DC1Pin, LOW);
        digitalWrite (DC2Pin, HIGH);
      }
      if (State != PrevState){
        digitalWrite (EnablePin, HIGH);              // for positive logical level (change if needed)
        delay(ActiveTime);
        digitalWrite (EnablePin, LOW);               // disable when finish
        PrevState = State;
      }
      break;
  }   

}
