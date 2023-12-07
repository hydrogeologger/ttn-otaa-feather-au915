/*******************************************************************************
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
 * the The Things Network. It's pre-configured for the Adafruit
 * Feather M0 LoRa.
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

#include <arduino_lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Serial1 works with standby mode.
#define serial Serial
#define SERIAL_BAUD 9600
#define USE_SERIAL 1
#define SERIAL_BLOCKING 1

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
#endif /* COMPILE_REGRESSION_TEST */

// Application Identifier (u1_t[8]) in lsb format
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { FILLMEIN };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// End-device Identifier (u1_t[8]) in lsb format
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { FILLMEIN };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// Application Key (u1_t[16]) in msb format
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { FILLMEIN };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// Payload to send to gateway
static uint8_t payload[] = "Hello, world!";
static osjob_t sendjob;

// Pin mapping
//
// Adafruit BSPs are not consistent -- m0 express defs ARDUINO_SAMD_FEATHER_M0,
// m0 defs ADAFRUIT_FEATHER_M0
//
#if defined(ARDUINO_SAMD_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0)
// Pin mapping for Adafruit Feather M0 LoRa, etc.
// /!\ By default Adafruit Feather M0's pin 6 and DIO1 are not connected.
// Please ensure they are connected.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};
#elif defined(ARDUINO_AVR_FEATHER32U4)
// Pin mapping for Adafruit Feather 32u4 LoRa, etc.
// Just like Feather M0 LoRa, but uses SPI at 1MHz; and that's only
// because MCCI doesn't have a test board; probably higher frequencies
// will work.
// /!\ By default Feather 32u4's pin 6 and DIO1 are not connected. Please
// ensure they are connected.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather 32U4 LoRa, in dB
    .spi_freq = 1000000,
};
#elif defined(ARDUINO_CATENA_4551)
// Pin mapping for Murata module / Catena 4551
const lmic_pinmap lmic_pins = {
        .nss = 7,
        .rxtx = 29,
        .rst = 8,
        .dio = { 25,    // DIO0 (IRQ) is D25
                 26,    // DIO1 is D26
                 27,    // DIO2 is D27
               },
        .rxtx_rx_active = 1,
        .rssi_cal = 10,
        .spi_freq = 8000000     // 8MHz
};
#else
# error "Unknown target"
#endif /* BSP Target */

// A buffer for printing log messages.
static constexpr int MAX_MSG = 256;
static char msg[MAX_MSG];

// A printf-like function to print log messages prefixed by the current
// LMIC tick value. Don't call it before os_init();
//
// The RTC timestamps will start at 00:00:00, but will update to UTC
// if the DeviceTimeReq is answered.
void log_msg(const char *fmt, bool show_ticks=false, ...) {
#if USE_SERIAL
    // snprintf(msg, MAX_MSG, "%02d:%02d:%02d / ", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    // serial.write(msg, strlen(msg));
    if (show_ticks) {
        snprintf(msg, MAX_MSG, "%ld: ", os_getTime());
        serial.write(msg, strlen(msg));
    }
    va_list args;
    va_start(args, show_ticks);
    vsnprintf(msg, MAX_MSG, fmt, args);
    va_end(args);
    serial.write(msg, strlen(msg));
    serial.println();
    msg[0] = '\0'; // clear string memory after use
#endif /* USE_SERIAL */
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        serial.print('0');
    serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            log_msg("EV_SCAN_TIMEOUT", true);
            break;
        case EV_BEACON_FOUND:
            log_msg("EV_BEACON_FOUND", true);
            break;
        case EV_BEACON_MISSED:
            log_msg("EV_BEACON_MISSED", true);
            break;
        case EV_BEACON_TRACKED:
            log_msg("EV_BEACON_TRACKED", true);
            break;
        case EV_JOINING:
            log_msg("EV_JOINING", true);
            break;
        case EV_JOINED:
            log_msg("EV_JOINED", true);
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              #if USE_SERIAL
                serial.print("netid: ");
                serial.println(netid, DEC);
                serial.print("devaddr: ");
                serial.println(devaddr, HEX);
                serial.print("AppSKey: ");
                for (size_t i=0; i<sizeof(artKey); ++i) {
                    if (i != 0)
                    serial.print("-");
                    printHex2(artKey[i]);
                }
                serial.println("");
                serial.print("NwkSKey: ");
                for (size_t i=0; i<sizeof(nwkKey); ++i) {
                        if (i != 0)
                                serial.print("-");
                        printHex2(nwkKey[i]);
                }
                serial.println();
              #endif /* USE_SERIAL */
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	        // size, we don't use it in this example.
            // LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            log_msg("EV_JOIN_FAILED", true);
            break;
        case EV_REJOIN_FAILED:
            log_msg("EV_REJOIN_FAILED", true);
            break;
        case EV_TXCOMPLETE:
            log_msg("EV_TXCOMPLETE (includes waiting for RX windows)", true);
            if (LMIC.txrxFlags & TXRX_ACK) { log_msg("Received ack", true); }
            if (LMIC.dataLen) {
              if (LMIC.txrxFlags & TXRX_PORT) {
                log_msg("FPort: %d, Received %d bytes of payload", false, \
                        LMIC.frame[LMIC.dataBeg-1], LMIC.dataLen);
              } else {
                log_msg("Received %d bytes of payload", false, LMIC.dataLen);
              }
            } else if (LMIC.dataBeg) {
            #if USE_SERIAL
                serial.print(F("MAC message received: "));
                for (unsigned i = 0; i < LMIC.dataBeg; ++i) {
                    serial.print(F(" "));
                    printHex2(LMIC.frame[i]);
                }
                serial.println();
            #endif /* USE_SERIAL */
            } /* LMIC.dataBeg */
            log_msg("adrAckReq: %d,  adrChanged: %d", false,
                    LMIC.adrAckReq,
                    LMIC.adrChanged);
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            log_msg("EV_LOST_TSYNC", true);
            break;
        case EV_RESET:
            log_msg("EV_RESET", true);
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            log_msg("EV_RXCOMPLETE", true);
            break;
        case EV_LINK_DEAD:
            log_msg("EV_LINK_DEAD", true);
            break;
        case EV_LINK_ALIVE:
            log_msg("EV_LINK_ALIVE", true);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            log_msg("EV_TXSTART", true);
            break;
        case EV_TXCANCELED:
            log_msg("EV_TXCANCELED", true);
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            log_msg("EV_JOIN_TXCOMPLETE: no JoinAccept", true);
            break;

        default:
            log_msg("Unknown event: %u", true, (unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Schedule the next application level uplink so the interval is constant
    // rather than related to the end of an uplink/downlink.
    // Clear any current scheduled copy of the job because there should never be
    // more than one.
    os_clearCallback(&sendjob);
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        log_msg("OP_TXRXPEND, not sending");
    } else {
        // Prepare upstream data transmission at the next possible time.
        lmic_tx_error_t txDataError;
        txDataError = LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);

        if (txDataError == LMIC_ERROR_SUCCESS) {
            log_msg("Packet queued");
        } else if (txDataError == LMIC_ERROR_TX_BUSY) {
            log_msg("Packet not sent, LMIC busy sending other message");
        } else if (txDataError == LMIC_ERROR_TX_TOO_LARGE) {
            log_msg("Packet too large for current datarate");
        } else if (txDataError == LMIC_ERROR_TX_NOT_FEASIBLE) {
            log_msg("Packet unsuitable for current datarate");
        } else {
            log_msg("Queued message failed to send for other reason than data len");
        }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    delay(5000);
    #if USE_SERIAL
        serial.begin(SERIAL_BAUD);
        #if SERIAL_BLOCKING
        while (! serial);
        #endif /* SERIAL_BLOCKING */
        log_msg("Starting");
    #endif /* USE_SERIAL */

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Disable link-check mode and ADR, because ADR tends to complicate testing.
    LMIC_setLinkCheckMode(0); // LinkADRReq
    // Enable or disable data rate adaptation. Should be turned off if the device is mobile.
    LMIC_setAdrMode(0);
    // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
    // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
    LMIC_setDrTxpow(DR_SF7,14);
    // in the AU, with TTN, it saves join time if we start on subband 1 (channels 8-15). This will
    // get overridden after the join by parameters from the network. If working with other
    // networks or in other regions, this will need to be changed.
    LMIC_selectSubBand(1);
    // Define device power source, MCMD_DEVS_EXT_POWER if battery powered
    LMIC_setBatteryLevel(MCMD_DEVS_BATT_NOINFO);


    log_msg("Joining");
    LMIC_startJoining();
}

void loop() {
    os_runloop_once();

    // Let radio do its thing before consider doing anything else. Prioritise TX/RX
    if (LMIC.opmode & (OP_POLL | OP_TXDATA | OP_TXRXPEND)) { return; }


    if (!os_queryTimeCriticalJobs(sec2osticks(TX_INTERVAL))) {
        // Start job (sending automatically starts OTAA too if not yet joined)
        os_setCallback(&sendjob, do_send);
    }
}
