/*
 * Attiny85 chip between intercom VCC and ground
 * Incoming ring signal is on 1M circuit port -> PB2
 */

// Includes -----------

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

// Constants ----------

#define MEASURE 0
#define WAIT_AFTER_RING 1
#define PICK_UP 2
#define WAIT_AFTER_PICK_UP 3
#define OPEN 4
#define CLOSE 5
#define HANG_UP 6

#define WDT_16_MS 0b00000000
#define WDT_32_MS 0b00000001
#define WDT_64_MS 0b00000010
#define WDT_125_MS 0b00000011
#define WDT_250_MS 0b00000100
#define WDT_500_MS 0b00000101
#define WDT_1000_MS 0b00000110
#define WDT_4000_MS 0b00100000
#define WDT_8000_MS 0b00100001

#define UNTIL_INTERRUPT 99
#define NO_SLEEP 100

static const byte pinPickUp = 0;                      // PB0 - IC pin 5
static const byte pinOpenDoor = 1;                    // PB1 - IC pin 6
static const byte pinRing = 2;                        // PB2 - IC pin 7
static const byte pinUnused3 = 3;                     // PB3 - IC pin 2
static const byte pinLed = 4;                         // PB4 - IC pin 3
static const byte pinReset = 5;                       // PB5 - IC pin 1

static const byte debounceTime = 20;                       // debounce in ms
static const byte waitTimeAfterRingStart1 = WDT_4000_MS;   // 4s 
static const byte waitTimeAfterRingStart2 = WDT_1000_MS;   // 1s 
static const byte waitTimeAfterPickUp = WDT_1000_MS;       // 1s * 3 = 3s (see pickUpCounterMax)
static const byte waitTimeAfterOpen = WDT_1000_MS;         // 1s
static const byte waitTimeAfterClose = WDT_1000_MS;        // 1s

static const byte pickUpCounterMax = 3;

// Variables ----------

byte sleepTime = UNTIL_INTERRUPT;
byte pickUpSleepCounter = 0;

volatile byte nextAction = MEASURE;

// Functions ----------

byte debounceRing(void) {

  // Bouncing is caused by the hook switch
  // if the ring signal arrives in that moment
  // when the phone receiver is put down.
  // This function decides what is the actual state 
  // of the circuit after eliminating bouncing
  // It also determines the direction of the pin change
  
  byte currentValue = 0;
  byte previousValue = 0;
  unsigned long now = millis();

  // Bounce means we read different values within
  // the debouncing timeframe
  do {
    currentValue = digitalRead(pinRing);
           
    // On bounce, reset time-out
    if (currentValue != previousValue) {
      now = millis();
    }
    previousValue = currentValue;
  } while (millis()-now <= debounceTime);

  return currentValue;
}

void sleepNow(const byte interval) {
  if (interval != NO_SLEEP) { 
  
    // Set WDT timer for delay type of sleeps
    if (interval != UNTIL_INTERRUPT) {
      noInterrupts();					// disable interrupts
      MCUSR = 0;                        // clear WDT reset flags
      WDTCR = bit (WDCE) | bit (WDE);   // allow changes, disable reset
      WDTCR = bit (WDIE) | interval;    // set interrupt mode and sleep time 
      wdt_reset();                      // start Watch Dog Timer
      interrupts();						// re-enable interrupts
    }
    sleep_bod_disable();                // turn off Brown Out Detector during sleep
    power_timer0_disable();             // turn off Timer0 during sleep
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();                        // go to sleep - next line will be executed first after wake up
    power_timer0_enable();              // turn back Timer0 when awake, it's needed for millis()
  }
}

void blinkLed(byte pin, const int onTime, const int times) {  
  for (int i=times; i>0; i--) {
    long now = millis();
    digitalWrite(pin, HIGH);
    while (millis() - now < onTime) {
     ;; // wait
    }
    digitalWrite(pin, LOW);
    while (millis() - now < onTime * 2) {
     ;; // wait
    }
  }
}

// Interrupt Service Routines ----------

ISR(PCINT0_vect) {

    // ISR for pin change interrupt on PB2
    sleep_disable();
    wdt_disable();
    bitClear(GIMSK, PCIF); // disable pin change interrupt
    nextAction = MEASURE;
}

ISR(WDT_vect) {
  
  // ISR for the Watchdog Timer interrupt
  sleep_disable();
  wdt_disable();
}

// Setup function ----------

void setup(void) {

  // Turn on internal pull-up resistors
  // for unused pins too, for power save
  pinMode(pinPickUp, OUTPUT);                      // PB0 - IC pin 5
  pinMode(pinOpenDoor, OUTPUT);                    // PB1 - IC pin 6
  pinMode(pinRing, INPUT_PULLUP);                  // PB2 - IC pin 7
  pinMode(pinUnused3, INPUT_PULLUP);               // PB3 - IC pin 2
  pinMode(pinLed, OUTPUT);                         // PB4 - IC pin 3
  pinMode(pinReset, INPUT_PULLUP);                 // PB5 - IC pin 1

  // Turn off unnecessary modules:
  // Timer1, USI, ADC
  ADCSRA = 0;  
  PRR = bit(PRTIM1) | bit(PRUSI) | bit(PRADC);

  // Enable pin change interrupt on PB2
  bitSet(PCMSK, PCINT2);
  bitSet(GIMSK, PCIE);
  interrupts();

  // Flash 3 times to signal restart 
  blinkLed(pinLed, 100, 3);
}

// Main loop ----------

void loop(void) {
  
  // NOTE: nextAction is also set from ringInterrupt() ISR function

  switch(nextAction) {

    case MEASURE:
    
      if(debounceRing() == LOW) {         // ring signal present
        nextAction = WAIT_AFTER_RING;
        sleepTime = waitTimeAfterRingStart1;
      } else {                            // no ring signal
        nextAction = HANG_UP;
        sleepTime = NO_SLEEP;
      }
      bitSet(GIFR, PCIF);                 // clear interrupt flag 
      bitSet(GIMSK, PCIE);                // re-enable pin change interrupt
      break;

    case WAIT_AFTER_RING:                 // wait more before picking up
    
      sleepTime = waitTimeAfterRingStart2;
      nextAction = PICK_UP;
      break;

    case PICK_UP:

      if(digitalRead(pinRing) == LOW) {   // still ringing
        digitalWrite(pinPickUp, HIGH);    // pick up receiver
        sleepTime = NO_SLEEP;
        nextAction = WAIT_AFTER_PICK_UP;
        pickUpSleepCounter = 0;
      } else {                            // caller hung up
        sleepTime = NO_SLEEP;
        nextAction = HANG_UP;
      }
      break;

    case WAIT_AFTER_PICK_UP:
    
      if (pickUpSleepCounter < pickUpCounterMax) {
        sleepTime = waitTimeAfterPickUp;
        nextAction = WAIT_AFTER_PICK_UP;
        pickUpSleepCounter++;
      } else {
        sleepTime = NO_SLEEP;
        nextAction = OPEN;
      }
      break;

    case OPEN:
    
      if(digitalRead(pinRing) == LOW) {   // still ringing
        digitalWrite(pinOpenDoor, HIGH);  // press door open button
        sleepTime = waitTimeAfterOpen;
        nextAction = CLOSE;
      } else {
        sleepTime = NO_SLEEP;             // caller hung up
        nextAction = HANG_UP;
      }
      break;

    case CLOSE:
    
      digitalWrite(pinOpenDoor, LOW);     // release door open button 
      sleepTime = waitTimeAfterClose;     
      nextAction = HANG_UP;
      break;

    case HANG_UP:
     
      digitalWrite(pinOpenDoor, LOW);     // release door open button - just in case
      digitalWrite(pinPickUp, LOW);       // put down receiver
      sleepTime = UNTIL_INTERRUPT;        // sleep until pinchange interrupts
      nextAction = MEASURE;
      break;
  } // end of switch
  
  sleepNow(sleepTime); 
}
