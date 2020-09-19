/*******************************************************************************
 * Wasserstandsanzeiger fÃ¼r Regenzisterne
 * surasto.de 2019
 * 
 * The code works on an Arduino with Draguino Shield
 * It is based on the lmic-library
 * 
 * Measureemnt by Ultrasonic Sensor
 * Full = 5cm
 * Empty = 60cm
 * 
 * Formula for % of full:
 * PercentFull = 100 - (Measurement - 5) 
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>

u1_t NWKSKEY[16] = { 0xD9, 0x2E, 0xCF, 0xF7, 0xC0, 0x60, 0x84, 0x3F, 0x64, 0xC8, 0x7D, 0x70, 0x46, 0x1B, 0x5D, 0xA6 };
u1_t APPSKEY[16] = { 0x83, 0xBC, 0xF4, 0xCB, 0x14, 0x0B, 0x24, 0x9C, 0xA7, 0xC2, 0xDB, 0x5A, 0x4A, 0x6C, 0xC0, 0x7B };
static const u4_t DEVADDR = 0x260118C2 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

float percentFull;

CayenneLPP lpp(51);

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

// Pin mapping ultrasonic sensor
const int pingPin = A1;
const int start_signal = A0;

//==================== LMIC-Functions ============================================

void onEvent (ev_t ev) {
    
   lpp.reset();
   lpp.addAnalogInput(1, percentFull);

   if( percentFull < 101 ) {
   if (ev == EV_TXCOMPLETE) {
    Serial.println("AHORA!!!!!!!!!!!!!!!!!!!!!!!!!");
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
   }
   }
}

void do_send(osjob_t* j){

    if( percentFull < 101 ) {
      // Check if there is not a current TX/RX job running
      if (LMIC.opmode & OP_TXRXPEND) {
          Serial.println(F("OP_TXRXPEND, not sending"));
      } else {
          // Prepare upstream data transmission at the next possible time.
          LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
          Serial.println(F("Packet queued"));
      }
      // Next TX is scheduled after TX_COMPLETE event.
    }  
}


//============================== Ultrasonic Measurement =========================

float measurePercentage() {
  long duration, cm;

  pinMode(pingPin,OUTPUT);     // Pins vorbereiten
  pinMode(start_signal,OUTPUT);
  digitalWrite(start_signal,HIGH);
  delayMicroseconds(20);

  digitalWrite(start_signal,LOW);  // Starte Messung mit fallender Flanke
  digitalWrite(pingPin,LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin,HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin,LOW);
  pinMode(pingPin,INPUT);

  duration = pulseIn(pingPin,HIGH);  // Messung der VerzÃ¶gerung bis Echo
  cm = duration / 29 / 2 ;
  Serial.print(cm);
  Serial.println("cm");
  
  return (float) 100 - (cm - 5);    // See formula in headline comment
}

//===================== Arduino setup and loop ========================================

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    os_init();
    LMIC_reset();
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);   
}

void loop() {
    os_runloop_once();
    percentFull = measurePercentage();
    delay(1000);
    
    if (percentFull > 0) {
        Serial.print(percentFull);
        Serial.println("%");

    }
}
