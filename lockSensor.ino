/*                 JeeNode / JeeNode USB / JeeSMD 
 -------|-----------------------|----|-----------------------|----       
|       |D3  A1 [Port2]  D5     |    |D3  A0 [port1]  D4     |    |
|-------|IRQ AIO +3V GND DIO PWR|    |IRQ AIO +3V GND DIO PWR|    |
| D1|TXD|                                           ---- ----     |
| A5|SCL|                                       D12|MISO|+3v |    |
| A4|SDA|   Atmel Atmega 328                    D13|SCK |MOSI|D11 |
|   |PWR|   JeeNode / JeeNode USB / JeeSMD         |RST |GND |    |
|   |GND|                                       D8 |BO  |B1  |D9  |
| D0|RXD|                                           ---- ----     |
|-------|PWR DIO GND +3V AIO IRQ|    |PWR DIO GND +3V AIO IRQ|    |
|       |    D6 [Port3]  A2  D3 |    |    D7 [Port4]  A3  D3 |    |
 -------|-----------------------|----|-----------------------|----
*/

#include <JeeLib.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>

#define DEBUG   1   // set to 1 to display each loop() run

#define NODE_ID  6
#define GROUP_ID 39

#define MEASURE_PERIOD  5000 // how often to measure, in tenths of seconds
#define ACKS_ENABLED    0   // if enabled the senor will wait for an ack
#define RETRY_PERIOD    10  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     5   // maximum number of times to retry
#define ACK_TIME        10  // number of milliseconds to wait for an ack

Port lock (1);
Port ledRed (3);
Port ledGreen (4);
Port blank (2);

static byte state;

ISR(PCINT2_vect) {}
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static void serialFlush () {
    #if ARDUINO >= 100
        Serial.flush();
    #endif  
    delay(2); // make sure tx buf is empty before going back to sleep
}

static void ledOn() {
  ledGreen.digiWrite(state);
  ledRed.digiWrite(!state);
}

static void ledOff() {
  ledGreen.digiWrite(0);
  ledRed.digiWrite(0);
}

static void ledBlink() {
  ledOn();
  Sleepy::loseSomeTime(100);
  ledOff();
}

// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | NODE_ID))
            return 1;
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
    return 0;
}

static byte sendState(byte withAck) {
  rf12_sleep(RF12_WAKEUP); ledOn();
  
  while (!rf12_canSend()) 
    rf12_recvDone();
  
  rf12_sendStart(0, &state, sizeof state);
  rf12_sendWait(2);
  
  byte acked = 0;
  if(withAck) acked = waitForAck(); 
  
  rf12_sleep(RF12_SLEEP); ledOff();
  return acked;
}

static void retryAck() {
  for (byte i = 0; i < RETRY_LIMIT; ++i) {
    if(sendState(ACKS_ENABLED)) {
      #if DEBUG
        Serial.print(" ack ");
        Serial.println((int) i);
        serialFlush();
      #endif
      return;
    }
    delay(RETRY_PERIOD * 100);
  }
  #if DEBUG
    Serial.println(" no ack!");
    serialFlush();
  #endif
}

void setup() {
  #if DEBUG
    Serial.begin(57600);
    Serial.print("\n[lockSensor.2]");
    serialFlush();
  #endif
  
  rf12_initialize(NODE_ID, RF12_868MHZ, GROUP_ID);
  
  lock.mode(INPUT);
  lock.digiWrite(1); //Pull-up on
  
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);
  
  ledGreen.mode(OUTPUT);
  ledRed.mode(OUTPUT);
  
  state = lock.digiRead();
  sendState(0);
}

void loop() {
  
  byte locked = lock.digiRead();
  #if DEBUG
    Serial.println(state ? "LOCKED" : "UNLOCKED");
  #endif
  
  if(state != locked) {
    state = locked;
    
    if(ACKS_ENABLED && !sendState(ACKS_ENABLED)) {
      retryAck();
    } 
  }
  else {
    ledBlink();
  }
   
  Sleepy::loseSomeTime(MEASURE_PERIOD);
}
