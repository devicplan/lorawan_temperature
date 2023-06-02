/*******************************************************************************
 * Jens Dietrich 31.05.2023 Lora-Radio-Node-V1.0 mit T-Sensor DS1820 
 * Clockerror = https://www.thethingsnetwork.org/forum/t/using-lmic-setclockerror-on-mcci-lmic-howto/39776
 * 31.05.2023 Stromsparfunktionen neu 5,5µA
 *  - ADC wird vor Sleep aus und danach wieder eingeschaltet 
 *  - BOD per ISP ausgeschaltet
 *  - Batteriespannungsmessung mit hexausgabe Byte 4 und 5 (adc wert auf 1114 angepasst)
 *  - one or two byte send = sendintervall in minutes (start after reset = 1 minutes, max ist 1440 Mintes)
 *    send 0x3c = 60 minutes send 0x0168 = 360 minutes / 6 hour 
 * 
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 * 
 *******************************************************************************/

#include <lmic.h>                                                                         // MCCI LoRaWAN LMIC library V 4.1.1 (11/2022)
#include <hal/hal.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <OneWire.h>

OneWire  ds(8);
uint16_t temperatur = 0;

char LED = 4;                                                                             // led red
uint8_t rec_b = 0;                                                                        // received bytes
uint16_t rec_t = 1;                                                                        // send intervall in minutes

#define debug                                                                             // auskommentieren fuer Debugausgabe

// void startSleeping(void);
// void mess(void);

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
static const u1_t PROGMEM APPEUI[8]={0x01, 0x20, 0x0A0, 0x04, 0x11, 0x01, 0x89, 0xA0};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={0xB2, 0x73, 0x25, 0xD0, 0x8E, 0xD5, 0xC3, 0x70};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
static const u1_t PROGMEM APPKEY[16] = {0xE8, 0x5A, 0xA5, 0x8F, 0xA7, 0x18, 0x98, 0xC6, 0xF6, 0xB2, 0xBA, 0x3B, 0x10, 0xC1, 0xBE, 0xCF};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

//static uint8_t mydata[] = "Hello, world!";
static uint8_t mydata[] = "000000";
static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;                                                         // send intervall in seconds multiple sendfactor

const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 5, LMIC_UNUSED_PIN},
};

float messen() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);                                 // preparation to measure internal 1.1 volts
  delay(10);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  uint8_t low  = ADCL;  
  uint8_t high = ADCH;
  long result = (high<<8) | low;
  float vcc = (1105 * 1023L / result) + 0;                                                // 0 or 534 is blocking voltage diode batt->IC
  analogReference(DEFAULT);                                                               // reset to Vcc as reference
  delay(10);
  return vcc;
}

#ifdef debug
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}
#endif

void onEvent (ev_t ev) {
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("F1"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("F2"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("F3"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("F4"));
            break;
        case EV_JOINING:
            Serial.println(F("J-ING"));
            break;
        case EV_JOINED:
            Serial.println(F("J-NED"));
#ifdef debug
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print(F("netid: "));
              Serial.println(netid, DEC);
              Serial.print(F("devaddr: "));
              Serial.println(devaddr, HEX);
              Serial.print(F("AppSKey: "));
              digitalWrite(LED, HIGH);                                                    // join ok - led off              
             for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print(F("NwkSKey: "));
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
#endif
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("F5"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("F6"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("F7"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("R ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              rec_b = LMIC.dataLen;
              Serial.print(F(" bytes of payload: 0x"));                                   // display rx data
              for (int i = 0; i < LMIC.dataLen; i++) {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                  Serial.print(F("0"));
                }
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
              }
              Serial.println();

              if (rec_b == 1)  {                                                          // read one byte
                rec_t = LMIC.frame[LMIC.dataBeg + 0];                                     // read new send timeer
                if (rec_t < 1)  rec_t = 1;
              }
              if (rec_b == 2)  {                                                          // read two byte
                rec_t = LMIC.frame[LMIC.dataBeg + 1];                                     // read low byte send timeer
                rec_t = rec_t + (LMIC.frame[LMIC.dataBeg + 0] * 256);                     // read high byte send timeer
                if (rec_t < 1)  rec_t = 1;                                                // min one minutes
                if (rec_t > 1440)  rec_t = 1440;                                          // max one day
              }
            }

            digitalWrite(LED, LOW);                                                       // short led pulse 
            delay(40);
            digitalWrite(LED, HIGH);             
            uint8_t adcbackup = ADCSRA;                                                   // push adc parameter
            ADCSRA = 0;                                                                   // adc switch off before standby
            delay(500);
            for(uint16_t t=0;t<((TX_INTERVAL * rec_t)/8);t++)  {                          // interval see above
              startSleeping();
            }
            ADCSRA = adcbackup;                                                           // pop adc parameter
            delay(500);
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);

            break;
        case EV_LOST_TSYNC:
            Serial.println(F("F8"));
            break;
        case EV_RESET:
            Serial.println(F("F9"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("F10"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("F11"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("F12"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("F13"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("F14"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("F15"));
            break;

        default:
            Serial.print(F("F16: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    uint16_t sp;
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("F17"));
    } else {
        mess();
        sp = messen();

        mydata[0] = (temperatur & 0xFF00)>>8;                 // temperatur
        mydata[1] = (temperatur & 0x00FF);
        mydata[2] = (rec_t & 0xFF00)>>8;                      // send intervall in minutes
        mydata[3] = (rec_t & 0x00FF);
        mydata[4] = (sp & 0xFF00)>>8;                         // 3,3 volt spannung
        mydata[5] = (sp & 0x00FF);
        LMIC_setTxData2(1, mydata, 6, 0);        

        Serial.println(F("P queued"));
    }
}

void setup() {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW); 
    Serial.begin(9600);
    delay(2000);
    Serial.println(F("Start"));
    os_init();
    LMIC_reset();
//    LMIC_setDrTxpow(DR_SF7,1);                              // not work !!! 
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

void mess()  {
  Serial.println("...messen");
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  float celsius;
  
  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
    return;
  }
   
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);                                            // start conversion, with parasite power on at the end
  
  delay(1000);                                                  // maybe 750ms is enough, maybe not
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);                                               // Read Scratchpad

  for ( i = 0; i < 9; i++) {                                    // we need 9 bytes
    data[i] = ds.read();
  }

  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  if (cfg == 0x00) raw = raw & ~7;                              // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3;                         // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1;                         // 11 bit res, 375 ms 
  celsius = (float)raw / 1.60;
  if(celsius < 0)  {
    temperatur = 0 - celsius;
    temperatur = 0xffff - temperatur;
  }
  else  {
    temperatur = celsius;
  }
  ds.reset_search();
}

ISR (WDT_vect) {
   wdt_disable(); 
}

void startSleeping() {
    // clear various "reset" flags
    MCUSR = 0;                                                  // allow changes, disable reset, enable Watchdog interrupt
    WDTCSR = bit (WDCE) | bit (WDE);                            // set interval (see datasheet p55)
    WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);              // set WDIE, and 8 seconds delay
    //WDTCSR = bit (WDIE) | bit (WDP2) | bit (WDP1);            // same with 1 second
    wdt_reset();                                                // start watchdog timer
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);                       // prepare for powerdown  
    sleep_enable();  
    sleep_cpu ();                                               // power down !
}
