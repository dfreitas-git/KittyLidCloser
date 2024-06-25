#include <Arduino.h>

/*
 Sketch to control our kitty feeder.
 Timer controls when a small winch will reel in a cord that pulls the lid closed.
 This is so she won't be able to eat wet food that sat out too long.
 Momentary switch opens lid/starts timer  or closes lid/stops timer.
 LED flashes to indicate countdown timer is active.  LED off when timer is not running.
 Pot controls the delay before lid-closingtime (0-4 hours).

 Dan Freitas
 6/22/2024
 
*/


//#define DEBUG  // Comment out before installing final code

// Prototypes
boolean checkSwitch(int pin);
void stopMotor();
void startMotor(int powerPercent, unsigned int duration);
int scalePower(int powerPercent, int adj);
void openLid();
void closeLid();

// Constants
const int STOPPED   = 0;
const int RUNNING   = 1;
const int OPENED    = 1;
const int CLOSED    = 0;
const int FORWARD   = 1;
const int PUSHED    = 0;
const int ON        = 1;
const int OFF       = 0;
const int ACTIVATED = 0;
const int REVERSE   = -1;

// Set pin numbers
const int TIMER_POT     = 0;
const int START_STOP    = 2;
const int TIMER_LED     = 3;
const int BIA           = 5;  // Motor B control input
const int BIB           = 9;  // Motor B control input
const int LID_OPENED_SW = 6;  // Switch to detect when the lid is fully open
const int LID_CLOSED_SW = 7;  // Switch to detect when the lid is fully closed

#ifdef DEBUG
 unsigned long maxTime = 10000;     // Short timeout for testing
#else
unsigned long maxTime = 14400000;     // 4 hours in milliseconds
#endif

unsigned long stopTime;               // Time in milliseconds when the lid close
unsigned long curTime;                // Current time during the countdown
unsigned long ledTime;                // The next time that we will turn on or off the LED
int blinkTime = 500;                  // on/off time for led in milliseconds

// Motor control
const int MOTOR_ADJ = 5;   // plus or minus power percent to trim motor speed
const int OPEN_STOP_DELAY = 100;  // To make up for magnetic switches switch point
const int CLOSE_STOP_DELAY = 15;  // To make up for magnetic switches switch point
unsigned long motorStopTime;
unsigned long motorStartTime;
unsigned long MAX_MOTOR_RUN_TIME = 7000;   //If it runs longer than this, something went wrong...  Set assuming 50% motor speed with 9v supply

boolean ledState;
boolean lidState;
boolean timerState;


void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  // Need this delay as the Arduino resets itself after opening the Serial connection
  // (else we'd execute the setup/loop twice)
  delay(1000);
  pinMode(START_STOP, INPUT_PULLUP);
  pinMode(LID_OPENED_SW, INPUT_PULLUP);
  pinMode(LID_CLOSED_SW, INPUT_PULLUP);
  pinMode(BIA, OUTPUT); 
  pinMode(BIB, OUTPUT);
  
  // If the lid is not open, open it!
  if(checkSwitch(LID_OPENED_SW) != ACTIVATED) {
    openLid();
  }
  timerState = STOPPED;
  lidState = OPENED;
}

void loop() {

  unsigned long percentOfMax;
  float timeDelta;

  // Lid open, counting down until the time to pop the lid open
  if(timerState == RUNNING && checkSwitch(START_STOP) != PUSHED){

    // Just counting down till the time to close the lid
    curTime=millis();

    // Blinking the LED to signify the timer is counting down
    if(curTime >= ledTime) {
      if(ledState == ON) {
        digitalWrite(TIMER_LED,OFF);
        ledState = OFF;
      } else {
        digitalWrite(TIMER_LED,ON);
        ledState = ON;
      }
      ledTime = millis() + blinkTime;
    }

    // When we hit the stop time, close the lid
    if(curTime >= stopTime) {
      timerState = STOPPED;
      #ifdef DEBUG
      Serial.println(" Time is up, close the lid ");
      #endif

      closeLid();
      digitalWrite(TIMER_LED,OFF);
    }

  // Lid open, counting down, just want to stop the timer
  } else if(lidState == OPENED && timerState == RUNNING && checkSwitch(START_STOP) == PUSHED){
      #ifdef DEBUG
      digitalWrite(TIMER_LED,OFF);
      Serial.println(" Stopping Timer");
      #endif
    // Wait for the switch to be released
    while(checkSwitch(START_STOP) == PUSHED) {
    }
    timerState = STOPPED;
    digitalWrite(TIMER_LED,OFF);

  // Just open the lid
  } else if(lidState == CLOSED && timerState == STOPPED && checkSwitch(START_STOP) == PUSHED) {
      #ifdef DEBUG
      Serial.println(" Open the lid");
      #endif
     openLid();

  // Reset the timer
  } else if(lidState == OPENED && timerState == STOPPED && checkSwitch(START_STOP) == PUSHED) {

    // See how long to set the timer for
    percentOfMax = analogRead(TIMER_POT);          // reads the value of the potentiometer (value between 0 and 1023)
    percentOfMax = map(percentOfMax, 0, 1023, 0, 100);     // scale it to use it with the timer (0 to 100% of the max time)
    curTime=millis();
    ledTime = curTime + blinkTime;
    timeDelta = (percentOfMax/100.0) * maxTime;
    stopTime = curTime + timeDelta;
    #ifdef DEBUG
    Serial.print(" Starting timer.  Will close Lid after ");
    Serial.print(timeDelta/1000/60);
    Serial.println(" minutes");
    #endif
    // Wait for the switch to be released
    while(checkSwitch(START_STOP) == PUSHED) {
    }
    timerState = RUNNING;
  }
}

//*******************************
// Function to stop the motor(s)
//*******************************
void stopMotor() {
    analogWrite(BIA, 0);
    analogWrite(BIB, 0);
    #ifdef DEBUG
    Serial.println(" Motor stopping!");
    #endif
}

//**************************************************************************
// Function to turn on the motor. 
// The motor power range is from -100% to +100% (negative for reverse)
// Duration is the number of milliseconds to run (will be blocking), if set
// to zero, then the motor(s) are started and stopping is controlled outside
// this function.
//**************************************************************************
void startMotor(int powerPercent, unsigned int duration) {
       
    // Now go turn on, or change speed 
    if(powerPercent == 0) {
        analogWrite(BIA, 0);
        analogWrite(BIB, 0);
    } else if(powerPercent > 0) {
        analogWrite(BIA, scalePower(powerPercent,MOTOR_ADJ));
        analogWrite(BIB, 0);
    } else {
        analogWrite(BIA, 0);
        
        // flip power around with abs as we need positive PWM setting
        analogWrite(BIB, abs(scalePower(powerPercent,MOTOR_ADJ)));
    }
    #ifdef DEBUG
    Serial.print("startMotor.  ");
    Serial.print("    motorPower: ");
    Serial.println(powerPercent);
    #endif

    if(duration != 0) {
        delay(duration);
    }
} 

//**********************************************************************************************************
// Convert % to absolute (0-255) pwm setting.  Also, motor power has adj factor in case of inbalanced motors
//**********************************************************************************************************
int scalePower(int powerPercent, int adj) {
    int scaledPower = (powerPercent + adj)/100.0*255;

    // Take care of case of + and - Percent (forward and reverse)
    if(scaledPower < 0) {
        if(scaledPower < -255) {
            scaledPower = -255;
        } else if(scaledPower > 0 || powerPercent == 0) {
            scaledPower = 0;
        }
    } else {
        if(scaledPower > 255) {
            scaledPower = 255;
        } else if(scaledPower < 0 || powerPercent == 0) {
            scaledPower = 0;
        }
    }
    return(scaledPower);
}

//**********************************************************************************************************
// Open the lid.  The limit switch will stop the motor
//**********************************************************************************************************
void openLid() {
  if(checkSwitch(LID_OPENED_SW) != ACTIVATED) {

      // start the motor (let run for 1000ms to let the magnetic switch separate)
    motorStartTime=millis();
    startMotor(50,1000);
    while(checkSwitch(LID_OPENED_SW) != ACTIVATED) {

      // Motor should never run this long.  Must be something wrong.
      if(millis() - motorStartTime >= MAX_MOTOR_RUN_TIME) {
        #ifdef DEBUG
        Serial.println(" Motor Error While Opening Lid!  Stopping Motor");
        #endif
        stopMotor();
        while(1) {
           // Stop Everything.  Need a reset.
        }
      }
    }
    delay(OPEN_STOP_DELAY);
    stopMotor();
  }
  lidState = OPENED;
}

//**********************************************************************************************************
// Close the lid.  The limit switch will stop the motor
//**********************************************************************************************************
void closeLid() {
  if(checkSwitch(LID_CLOSED_SW) != ACTIVATED) {

    // start the motor (let run for 1000ms to let the magnetic switch separate)
    motorStartTime=millis();
    startMotor(-50,1000);
    while(checkSwitch(LID_CLOSED_SW) != ACTIVATED) {
      // Motor should never run this long.  Must be something wrong.
      if(millis() - motorStartTime >= MAX_MOTOR_RUN_TIME) {
        #ifdef DEBUG
        Serial.println(" Motor Error While Closing Lid!  Stopping Motor");
        #endif
        stopMotor();
        while(1) {
           // Stop Everything.  Need a reset.
        }
      }
    }
    delay(CLOSE_STOP_DELAY);
    stopMotor();
  }
  lidState = CLOSED;
}


//**********************************************************************************************************
// Switch debounce 
//**********************************************************************************************************
boolean checkSwitch(int pin) {
  boolean state;
  boolean prevState;
  int debounceDelay = 20;
  prevState = digitalRead(pin);
  for(int counter=0; counter < debounceDelay; counter++) {
    delay(1);
    state = digitalRead(pin);
    if(state != prevState) {
      counter=0;
      prevState=state;
    }
  }
  // At this point the switch state is stable
  if(state == HIGH) {
    return true;
  } else {
    return false;
  }
}