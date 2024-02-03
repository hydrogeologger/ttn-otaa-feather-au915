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
#include <RTCZero.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Serial1 works with standby mode.
#define serial Serial
#define SERIAL_BAUD 9600
#define USE_SERIAL 1
#define SERIAL_BLOCKING 0
#define DEBUG 0

#define TZ_OFFSET 10 // Timezone offset in hours

/* Battery Configuration */
// #define BATTERY_ADC_PIN A7
#define BATTERY_MAP_VAL_MIN 3.09 * 100 // 3.09V at 0%, 3.2V at 10%
#define BATTERY_MAP_VAL_MAX 4.2 * 100 // 4.2V
#define ADC_REF_VOLTAGE 3.3 // 3.3V
#define BATT_ADC_TO_VOLT(adcVal) (adcVal * 2 * ADC_REF_VOLTAGE / 1024)
#define BATT_ADC_TO_INT_SHIFT(adcVal) (BATT_ADC_TO_VOLT(adcVal) * 100)

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
#elif defined(ARDUINO_NRF52832_FEATHER)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840, esp32-s2 and 328p
  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
#endif

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
static const u1_t PROGMEM APPEUI[8]= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// End-device Identifier (u1_t[8]) in lsb format
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { 0xF3, 0xC2, 0xE8, 0x23, 0x2C, 0x3A, 0x23, 0x74 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// Application Key (u1_t[16]) in msb format
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x75, 0x91, 0x12, 0xF7, 0xCE, 0xD1, 0x23, 0x82, 0x1A, 0x06, 0xDB, 0x51, 0x2A, 0x90, 0x92, 0xD2 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// EPOCH Time Defines
#define GPS_EPOCH_OFFSET 315964800 // GPS Epoch offset in seconds
#define LEAP_SECONDS_1980 19 // Leap seconds from 1970 to 1080
#define LEAP_SECONDS_NOW 37
#define HOURS_TO_SECONDS(hrs) (hrs * 60 * 60)

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
    .dio = {3, 10, LMIC_UNUSED_PIN},
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

// LMIC Device State
#define STATE_LOW_POWER 0
#define STATE_IDLE 1
static uint8_t state = STATE_IDLE;

static uint32_t userUTCTime; // Seconds since the UTC epoch
RTCZero rtc; // Declare real time clock

// A buffer for printing log messages.
static constexpr int MAX_MSG = 256;
static char msg[MAX_MSG];

#if DEBUG && USE_SERIAL
static u2_t prev_opmode = OP_NONE;

void PrintOpmode(uint16_t opmode, char sep = ',') {
    serial.print(F("opmode="));
    serial.print("0x");
    serial.print(opmode, HEX);
    if (sep != 0)
        serial.print(sep);
}

void PrintBinaryStrZeroPad(int value, int8_t num_bits, bool newline = false){
    //ZeroPadding = nth bit, e.g for a 16 bit number nth bit = 16
    while(--num_bits >= 0){
        if((value & (1 << num_bits)) > 0) {
            serial.write('1');
        } else {
            serial.write('0');
        }
    }
    if (newline) serial.write("\n");
}
#endif /* DEBUG && USE_SERIAL */

// A printf-like function to print log messages prefixed by the current
// LMIC tick value. Don't call it before os_init();
//
// The RTC timestamps will start at 00:00:00, but will update to UTC
// if the DeviceTimeReq is answered.
void log_msg(const char *fmt, bool show_ticks=false, ...) {
#if USE_SERIAL
    snprintf(msg, MAX_MSG, "%02d:%02d:%02d / ",
            rtc.getHours(),
            rtc.getMinutes(),
            rtc.getSeconds());
    serial.write(msg, strlen(msg));
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

void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess) {
    // Explicit conversion from void* to uint32_t* to avoid compiler errors
    uint32_t *pUserUTCTime = (uint32_t *) pVoidUserUTCTime;

    // A struct that will be populated by LMIC_getNetworkTimeReference.
    // It contains the following fields:
    //  - tLocal: the value returned by os_GetTime() when the time
    //            request was sent to the gateway, and
    //  - tNetwork: the seconds between the GPS epoch and the time
    //              the gateway received the time request
    lmic_time_reference_t lmicTimeReference;

    if (flagSuccess != 1) {
        log_msg("user_request_network_time_callback: Not a success");
        return;
    }

    // Populate "lmic_time_reference"
    flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
    if (flagSuccess != 1) {
        log_msg("user_request_network_time_callback: LMIC_getNetworkTimeReference didn't succeed");
        return;
    }

    // Update userUTCTime, considering the difference between the GPS and UTC
    // epoch, and the leap seconds
    *pUserUTCTime = lmicTimeReference.tNetwork + GPS_EPOCH_OFFSET + \
            LEAP_SECONDS_1980 - LEAP_SECONDS_NOW;

    {
    // Add the delay between the instant the time was transmitted and
    // the current time
    // Current time, in ticks
    ostime_t ticksNow = os_getTime();
    // Time when the request was sent, in ticks
    ostime_t ticksRequestSent = lmicTimeReference.tLocal;
    uint32_t requestDelaySec = osticks2ms(ticksNow - ticksRequestSent) / 1000;
    *pUserUTCTime += requestDelaySec;
    // All that gets the sketch to somewhere near UTC.
    }

    // Update the system time with the time read from the network
    #if (TZ_OFFSET != 0)
        rtc.setEpoch(*pUserUTCTime + HOURS_TO_SECONDS(TZ_OFFSET));
    #else
        rtc.setEpoch(*pUserUTCTime);
    #endif /* (TZ_OFFSET != 0) */

    log_msg("The current local time from network is: %02d/%02d/%02d %02d:%02d:%02d", false,
            rtc.getYear(),
            rtc.getMonth(),
            rtc.getDay(),
            rtc.getHours(),
            rtc.getMinutes(),
            rtc.getSeconds());
}

/*
 * This function is used to set the alarm to a relative time in the future, such as when
 * sleeping between LMIC tasks.
 */
void set_delta_alarm(uint32_t delta_seconds) {
    int32_t ss = (int32_t)rtc.getSeconds();
    int32_t mm = (int32_t)rtc.getMinutes();
    int32_t hh = (int32_t)rtc.getHours();

    // Sanity check.
    if (delta_seconds < 1) {
        delta_seconds = 1;
    }

    int32_t delta = delta_seconds;
    int32_t hh_delta = delta / 3600; delta -= (hh_delta * 3600);
    // Will always be less than 1 hour.
    int32_t mm_delta = delta / 60; delta -= (mm_delta * 60);
    // Will always be less than 1 minute.
    int32_t ss_delta = delta;

    ss += ss_delta;
    if (ss > 59) {
        ss = ss % 60;
        mm_delta++;
    }

    mm += mm_delta;
    if (mm > 59) {
        mm = mm % 60;
        hh_delta++;
    }

    hh = (hh + hh_delta) % 24;

    log_msg("Delta(s) = %d, wake at %02d:%02d:%02d", true,
            delta_seconds,
            hh,
            mm,
            ss);

    rtc.setAlarmTime((uint8_t)(hh & 0xff), (uint8_t)(mm & 0xff), (uint8_t)(ss & 0xff));
    rtc.enableAlarm(RTCZero::MATCH_HHMMSS);
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
                serial.println("Uplink frequency channels:");
                for (uint8_t i = 0; i <= 4; i++) {
                    if (i < 4) {
                        snprintf(msg, MAX_MSG,
                                "\t0x%04X channel=%u-%u\n",
                                LMIC.channelMap[i], i*16, (i+1)*16-1);
                    } else {
                        snprintf(msg, MAX_MSG,
                                "\t0x%02X   channel=%u-%u\n",
                                LMIC.channelMap[i], i*16, (i+1)*16-8-1);
                    }
                    serial.write(msg, strlen(msg));
                }
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
            #if DEBUG && USE_SERIAL
            {
                u1_t sf = getSf(LMIC.rps) + 6; // 1 == SF7
                u1_t bw = getBw(LMIC.rps);
                u1_t cr = getCr(LMIC.rps);
                snprintf(msg, MAX_MSG,
                        "freq=%u.%u, DR%d(SF%d BW%d), CR=4/%d, IH=%d rps=%04x\n",
                        LMIC.freq / 1000000,
                        (LMIC.freq % 1000000) / 100000,
                        LMIC.datarate,
                        sf,
                        bw == BW125 ? 125 : (bw == BW250 ? 250 : 500),
                        cr == CR_4_5 ? 5 : (cr == CR_4_6 ? 6 : (cr == CR_4_7 ? 7 : 8)),
                        getIh(LMIC.rps),
                        LMIC.rps
                );
                serial.write(msg, strlen(msg));
            }
            #endif /* DEBUG && USE_SERIAL */
            if (LMIC.txrxFlags & TXRX_ACK) { log_msg("Received ack", true); }
            #if DEBUG && USE_SERIAL
            log_msg("adrAckReq: %d,  adrChanged: %d", false,
                    LMIC.adrAckReq,
                    LMIC.adrChanged);
            if (LMIC.dataBeg) {
                serial.print(F("MAC message received: "));
                for (unsigned i = 0; i < LMIC.dataBeg + LMIC.dataLen; i++) {
                    if (i == LMIC.dataBeg) serial.print(F(","));
                    serial.print(F(" "));
                    printHex2(LMIC.frame[i]);
                }
                serial.println();
                if (LMIC.dataLen) {
                    if (LMIC.txrxFlags & TXRX_PORT) {
                        log_msg("FPort: %d, Received %d bytes of payload", false, \
                                LMIC.frame[LMIC.dataBeg-1], LMIC.dataLen);
                    } else {
                        log_msg("Received %d bytes of payload", false, LMIC.dataLen);
                    }
                    for (unsigned i = 0; i < LMIC.dataLen; i++) {
                        serial.print(char(LMIC.frame[LMIC.dataBeg + i]));
                    }
                    serial.println();
                }
            } /* LMIC.dataBeg */
            #endif /* DEBUG && USE_SERIAL */
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
            #if DEBUG && USE_SERIAL
            {
                u1_t sf = getSf(LMIC.rps) + 6; // 1 == SF7
                u1_t bw = getBw(LMIC.rps);
                u1_t cr = getCr(LMIC.rps);
                snprintf(msg, MAX_MSG,
                        "freq=%u.%u, DR%d(SF%d BW%d), CR=4/%d, IH=%d rps=%04x\n",
                        LMIC.freq / 1000000,
                        (LMIC.freq % 1000000) / 100000,
                        LMIC.datarate,
                        sf,
                        bw == BW125 ? 125 : (bw == BW250 ? 250 : 500),
                        cr == CR_4_5 ? 5 : (cr == CR_4_6 ? 6 : (cr == CR_4_7 ? 7 : 8)),
                        getIh(LMIC.rps),
                        LMIC.rps
                );
                serial.write(msg, strlen(msg));
            }
            #endif /* DEBUG && USE_SERIAL */
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
    oledRedraw(ev);
}

void do_send(osjob_t* j){
    // Schedule the next application level uplink so the interval is constant
    // rather than related to the end of an uplink/downlink.
    // Clear any current scheduled copy of the job because there should never be
    // more than one.
    // os_clearCallback(&sendjob);
    // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

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

static bool joined = false;
int lastButtonCState = HIGH;  // the previous state from the input pin

void oledRedraw(ev_t ev) {
    display.clearDisplay();
    display.display();
    display.setCursor(0,0);

    snprintf(msg, MAX_MSG, "%02d:%02d:%02d %c: %d\n",
            rtc.getHours(),
            rtc.getMinutes(),
            rtc.getSeconds(),
            joined ? 'Y' : 'N',
            LMIC.txCnt
    );
    display.write(msg, strlen(msg));

    switch(ev) {
        case EV_SCAN_TIMEOUT:
            display.print("SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            display.print("BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            display.print("BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            display.print("BEACON_TRACKED");
            break;
        case EV_JOINING:
            display.print("JOINING");
            break;
        case EV_JOINED:
            display.print("JOINED");
            joined = true;
            break;
        case EV_JOIN_FAILED:
            display.print("JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            display.print("REJOIN_FAILED");
            joined = false;
            break;
        case EV_TXCOMPLETE:
            display.print("TXCOMPLETE");
            break;
        case EV_LOST_TSYNC:
            display.print("LOST_TSYNC");
            break;
        case EV_RESET:
            display.print("RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            display.print("RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            display.print("LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            display.print("LINK_ALIVE");
            break;
        case EV_TXSTART:
            display.print("TXSTART");
            break;
        case EV_TXCANCELED:
            display.print("TXCANCELED");
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            display.print("NO_JoinAccept");
            break;
        default:
            display.print("UNK_");
            display.print(ev);
            break;
    }
    display.println();

    u1_t sf = getSf(LMIC.rps) + 6; // 1 == SF7
    u1_t bw = getBw(LMIC.rps);
    u1_t cr = getCr(LMIC.rps);
    snprintf(msg, MAX_MSG,
            "%u.%u, DR%d-SF%d BW%d\n",
            LMIC.freq / 1000000,
            (LMIC.freq % 1000000) / 100000,
            LMIC.datarate,
            sf,
            bw == BW125 ? 125 : (bw == BW250 ? 250 : 500)
    );
    display.write(msg, strlen(msg));
    snprintf(msg, MAX_MSG,
            "CR=4/%d, IH=%d rps=%04x\n",
            cr == CR_4_5 ? 5 : (cr == CR_4_6 ? 6 : (cr == CR_4_7 ? 7 : 8)),
            getIh(LMIC.rps),
            LMIC.rps
    );
    display.write(msg, strlen(msg));

    snprintf(msg, MAX_MSG,
            "rssi:%d snr:%d\n",
            LMIC.rssi,
            LMIC.snr
    );
    display.write(msg, strlen(msg));

    display.display();
}

void setup() {
    #ifdef BATTERY_ADC_PIN
        pinMode(BATTERY_ADC_PIN, INPUT);
    #endif
    delay(5000);
    #if USE_SERIAL
        serial.begin(SERIAL_BAUD);
        #if SERIAL_BLOCKING
        while (! serial);
        #endif /* SERIAL_BLOCKING */
        log_msg("Starting");
    #endif /* USE_SERIAL */

    display.begin(0x3C, true); // Address 0x3C default

    // Show image buffer on the display hardware.
    // Since the buffer is intialized with an Adafruit splashscreen
    // internally, this will display the splashscreen.
    display.display();
    delay(1000);

    // Clear the buffer.
    display.clearDisplay();
    display.display();

    // Serial.println("Button test");

    pinMode(BUTTON_A, INPUT_PULLUP); // Also used for Battery Test
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_C, INPUT_PULLUP);

    display.setRotation(3);
    // text display tests
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);

    rtc.begin(false); // Initialize RTC, Preserving clock time

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
    // LMIC_setDrTxpow(DR_SF7,14);
    LMIC_setDrTxpow(LMICbandplan_getInitialDrJoin(), 14);
    // in the AU, with TTN, it saves join time if we start on subband 1 (channels 8-15). This will
    // get overridden after the join by parameters from the network. If working with other
    // networks or in other regions, this will need to be changed.
    LMIC_selectSubBand(1);
    // Define device power source, MCMD_DEVS_EXT_POWER if battery powered
    LMIC_setBatteryLevel(MCMD_DEVS_BATT_NOINFO);


    log_msg("Joining");
    LMIC_startJoining();
    LMIC_requestNetworkTime(user_request_network_time_callback, &userUTCTime);
}

void loop() {
    #if DEBUG && USE_SERIAL
    if (prev_opmode != LMIC.opmode) {
        prev_opmode = LMIC.opmode;
        PrintOpmode(LMIC.opmode);
        serial.print(" ");
        PrintBinaryStrZeroPad(LMIC.opmode, 16, true);
    }
    #endif /* DEBUG && USE_SERIAL */
    os_runloop_once();

    // Let radio do its thing before consider doing anything else. Prioritise TX/RX
    if (LMIC.opmode & (OP_POLL | OP_TXDATA | OP_TXRXPEND)) { return; }

    switch (state) {
        case STATE_LOW_POWER: {
            log_msg("State: LOW POWER");
            if (os_queryTimeCriticalJobs(sec2osticks(TX_INTERVAL))) {
                state = STATE_IDLE;
                break;
            }
            break;
        }
        case STATE_IDLE:
            log_msg("State: Idle");
            if (!os_queryTimeCriticalJobs(sec2osticks(TX_INTERVAL))) {
                log_msg("No scheduled jobs, scheduling do_sent() to run now.");
                #ifdef BATTERY_ADC_PIN
                    uint16_t batteryADC = analogRead(BATTERY_ADC_PIN);
                    long batteryLevelMCMD = map(BATT_ADC_TO_INT_SHIFT(batteryADC),
                            BATTERY_MAP_VAL_MIN, BATTERY_MAP_VAL_MAX,
                            MCMD_DEVS_BATT_MIN, MCMD_DEVS_BATT_MAX);
                    if (batteryLevelMCMD > MCMD_DEVS_BATT_MAX) {
                        batteryLevelMCMD = MCMD_DEVS_BATT_MAX;
                    } else if (batteryLevelMCMD < MCMD_DEVS_BATT_MIN) {
                        batteryLevelMCMD = MCMD_DEVS_BATT_MIN;
                    }
                    #if USE_SERIAL
                        snprintf(msg, MAX_MSG, "Battery: ADC=%d, Volt=%f, LMIC_vBat=%d\n",
                                batteryADC,
                                BATT_ADC_TO_INT_SHIFT(batteryADC),
                                batteryLevelMCMD);
                        serial.write(msg);
                    #endif /* USE_SERIAL */
                    LMIC_setBatteryLevel(batteryLevelMCMD);
                #endif /* BATTERY_ADC_PIN */
                // Start job (sending automatically starts OTAA too if not yet joined)
                int input_button_c = digitalRead(BUTTON_C);
                int input_button_b = digitalRead(BUTTON_B);
                if ( !input_button_c && lastButtonCState) {
                    lastButtonCState = input_button_c;
                    os_setCallback(&sendjob, do_send);
                    display.println("do_send");
                    display.display();
                } else if ( !input_button_b && lastButtonCState) {
                    LMIC_unjoinAndRejoin();
                    display.println("do_rejoin");
                    display.display();
                } else if ( input_button_c && input_button_b && !lastButtonCState) {
                    lastButtonCState = input_button_c;
                }
                // state = STATE_LOW_POWER;
            }
            break;
    } /* Switch(state) */
}
