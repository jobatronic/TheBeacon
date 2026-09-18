/*
 * ═══════════════════════════════════════════════════════════════
 *  ARGUS – Server Rack Controller (Arduino Nano / ATmega328P)
 *  ═══════════════════════════════════════════════════════════════
 *
 *  DESIGN PHILOSOPHY
 *  ─────────────────
 *  This controller is intended to run continuously for many years.
 *
 *  It therefore deliberately favors:
 *
 *    - Fixed-size buffers
 *    - No heap allocation
 *    - No Arduino String objects
 *    - No EEPROM writes during normal operation
 *    - No blocking waits in the main loop
 *    - Overflow-safe millis() timing
 *    - Strict input validation
 *    - Hardware watchdog recovery
 *    - Separate Pi heartbeat watchdog
 *    - Deterministic failsafe behavior
 *    - Conservative handling of malformed commands
 *    - Simple state machines instead of complicated control flow
 *
 *  Role:
 *    Dumb I/O controller. Reads sensors, drives outputs,
 *    reports events to Pi over serial.
 *
 *  It makes NO high-level decisions except the local safety
 *  functions and failsafe behavior described below.
 *
 *
 *  SERIAL PROTOCOL
 *  ───────────────
 *
 *  Pi → Arduino:
 *
 *    R<n> ON
 *    R<n> OFF
 *    R ALL OFF
 *
 *    FAN <0-255>
 *    FAN AUTO
 *    FAN OFF
 *
 *    RESET <n> <secs>
 *
 *    HB
 *
 *
 *  Arduino → Pi:
 *
 *    READY
 *
 *    EVT DOOR OPEN
 *    EVT DOOR CLOSED
 *
 *    EVT FIRE <temp>
 *    EVT FIRE CLEARED
 *    EVT TEMP RISING <rate>
 *    EVT FILTER CLOGGED
 *    EVT FILTER CLEARED
 *    EVT TEMP SENSOR FAULT
 *    EVT TEMP SENSOR OK
 *
 *    EVT WATER DETECTED
 *    EVT WATER CLEARED
 *
 *    EVT RESET STARTED <n>
 *    EVT RESET DONE <n>
 *
 *    EVT WATCHDOG FIRED
 *    EVT WATCHDOG CLEARED
 *
 *    EVT TEMP <t> FAN <s> ...
 *
 *
 * ═══════════════════════════════════════════════════════════════
 *  PIN MAP
 * ═══════════════════════════════════════════════════════════════
 *
 *    D2  – R1 relay (cabinet light — auto-driven by DOOR, see below)
 *    D3  – FAN PWM
 *    D4  – R2 relay
 *    D5  – R3 relay (critical: OpenWRT Pi / router)
 *    D6  – R4 relay (critical: camera DVR)
 *    D7  – R5 relay (critical: Home Automation Pi / ESP8266-32 hub)
 *    D8  – R6 relay
 *    D9  – R7 relay
 *    D10 – R8 relay
 *    D11 – DOOR endstop
 *    D12 – reserved (candidate pin for a future DHT11+ intake
 *          sensor, if/when one is added — see note below)
 *    D13 – reserved / onboard LED
 *
 *    A0  – fan 1 tach
 *    A1  – fan 2 tach
 *    A2  – fan 3 tach
 *    A3  – Keyestudio analog temperature sensor (exhaust)
 *    A4  – water leak sensor (bottom pan)
 *    A5  – reserved (last free analog pin — measureVacRms /
 *          measureIacRms / measureDc12v below are unimplemented
 *          stubs that assumed A1/A2, now claimed by tach; a
 *          future AC/DC sensor would need to move to A5 or a
 *          digital pin instead)
 *
 *    EXHAUST TEMPERATURE SENSOR (A3):
 *      Keyestudio "Analog Temperature Sensor Detection Module"
 *      (NTC thermistor + fixed resistor divider). Mounted at the
 *      cabinet exhaust vent, so it reads the combined output of
 *      everything currently running inside — this drives the fan
 *      control and fire/rise alarms below.
 *
 *    FUTURE INTAKE SENSOR (not yet installed):
 *      If a DHT11 (or better) ever gets added near the fan
 *      intake, wire it to a free digital pin (D12 is reserved for
 *      this) and read it independently via its own library. It is
 *      NOT required for fan control — the exhaust sensor alone is
 *      sufficient for that — but it would let the Pi compute an
 *      intake/exhaust delta for extra diagnostics.
 *
 *
 * ═══════════════════════════════════════════════════════════════
 *  HARDWARE FEATURES
 * ═══════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <limits.h>
#include <math.h>

#if defined(__AVR_ATmega328P__)
  #include <avr/wdt.h>
  #include <avr/io.h>
  #include <avr/interrupt.h>
#endif


// ═══════════════════════════════════════════════════════════════
//  ENABLE / DISABLE FEATURES
// ═══════════════════════════════════════════════════════════════

#define ENABLE_DOOR             true
#define ENABLE_FAN              true
#define ENABLE_AC_SENSORS       false
#define ENABLE_DC_SENSOR        false
#define ENABLE_WATER            true
#define ENABLE_PERIODIC_REPORT  true


// ═══════════════════════════════════════════════════════════════
//  HARDWARE WATCHDOG
// ═══════════════════════════════════════════════════════════════
//
//  This is DIFFERENT from the Pi heartbeat watchdog.
//
//  Pi heartbeat watchdog:
//    "Has the Raspberry Pi stopped talking to me?"
//
//  Hardware watchdog:
//    "Has the Arduino firmware itself stopped executing?"
//
//  If the firmware becomes genuinely stuck, the AVR watchdog
//  resets the microcontroller.
//
//  2 seconds is intentionally much shorter than the Pi heartbeat
//  timeout. The normal loop should never come close to this.
//
//  The watchdog is only enabled on the ATmega328P.
//

#if defined(__AVR_ATmega328P__)
  #define ENABLE_HARDWARE_WATCHDOG true
  #define HARDWARE_WATCHDOG_TIMEOUT WDTO_2S
#endif


// ═══════════════════════════════════════════════════════════════
//  RELAY CONFIGURATION
// ═══════════════════════════════════════════════════════════════
//
//  Wiring:
//
//    relayNC = false
//      NO contact
//      Coil de-energized = load OFF
//
//    relayNC = true
//      NC contact
//      Coil de-energized = load ON
//
//  Board trigger polarity:
//
//    RELAY_COIL_ON / RELAY_COIL_OFF below are the single source
//    of truth for this. Confirmed ACTIVE LOW by direct hardware
//    test: pulling the control pin LOW energizes the relay.
//
//  IMPORTANT:
//  Hardware wiring should provide the actual safety behavior.
//  Software should be considered the second layer, not the only
//  safety mechanism.
//
//  Index:
//
//    [R1] [R2] [R3] [R4] [R5] [R6] [R7] [R8]
//

#define RELAY_COIL_ON   LOW
#define RELAY_COIL_OFF  HIGH

const uint8_t relayPins[8] = {
  2, 4, 5, 6, 7, 8, 9, 10
};

const bool relayNC[8] = {
  false,  // R1 – cabinet light
  false,  // R2 – cabinet light
  true,   // R3 – OpenWRT Pi (router) — critical
  true,   // R4 – camera DVR — critical
  true,   // R5 – Home Automation Pi (ESP8266/32 hub) — critical
  false,  // R6 – assign
  false,  // R7 – assign
  false   // R8 – assign
};


// ═══════════════════════════════════════════════════════════════
//  FAN CONFIGURATION
// ═══════════════════════════════════════════════════════════════

#define FAN_PIN             3

#define FAN_TARGET_TEMP     28.0f
#define FAN_MIN_SPEED       255   // Locked to 100% — this cabinet
                                   // needs full airflow 24/7
                                   // regardless of temperature, so
                                   // floor == ceiling. The PID
                                   // logic below is untouched and
                                   // harmless either way (every
                                   // branch clamps to this floor),
                                   // so this is a one-line, fully
                                   // reversible decision if that
                                   // ever changes.

#define FAN_KP              4.0f
#define FAN_KI              0.5f

#define FAN_DECAY_RATE      2.0f

#define FAN_UPDATE_INTERVAL 100UL

#define FAN_CLOG_THRESHOLD  60000UL
#define FAN_CLOG_DELTA      2.0f


// ═══════════════════════════════════════════════════════════════
//  FAN TACHOMETERS
// ═══════════════════════════════════════════════════════════════
//
//  These are genuine 3-pin fans (+12V, GND, TACH) — the tach wire
//  is an open-collector output inside the fan that pulls to GND
//  twice per shaft revolution (the standard across virtually all
//  PC fans). INPUT_PULLUP gives it something to pull against; no
//  external resistor needed.
//
//  A0/A1/A2 were chosen deliberately: on the ATmega328P they're
//  PCINT8/9/10, all three inside the SAME pin-change-interrupt
//  bank (PCINT1) — so one shared interrupt handler covers all
//  three tach wires, rather than needing separate wiring per pin.
//  (The two "true" hardware interrupt pins, D2/D3, were already
//  spoken for by R1 and FAN_PIN.)
//
//  TACH_PULSES_PER_REV = 2 is the standard for PC fans generally;
//  it's not a spec this exact fan's datasheet confirms, but it's
//  close to universal across the industry.
//

#define FAN1_TACH_PIN        A0
#define FAN2_TACH_PIN        A1
#define FAN3_TACH_PIN        A2

#define TACH_PULSES_PER_REV  2
#define TACH_CHECK_INTERVAL  2000UL

// Below this, a fan is considered stopped rather than "just slow".
#define TACH_SPINNING_RPM    100

// Pin-change interrupts have no built-in debouncing, and these
// wires run near a MOSFET chopping 12V — real-world testing found
// implausible readings (up to ~14,000 RPM on a fan rated for
// 2,000 max) caused by electrical noise being counted as pulses.
// Reject anything faster than this could possibly be for a real
// fan. At 2 pulses/rev, 8ms still allows correctly reading fans
// up to ~3,750 RPM — comfortably above these fans' real 2,000 RPM
// max, so this only filters out noise, never a genuine reading.
#define TACH_MIN_PULSE_MS    8


// ═══════════════════════════════════════════════════════════════
//  EXHAUST TEMPERATURE SENSOR (Keyestudio analog module)
// ═══════════════════════════════════════════════════════════════
//
//  Simple NTC thermistor + fixed resistor divider:
//
//      5V ---[thermistor]---+---[4.7k]--- GND
//                           |
//                        TEMP_PIN (analog input)
//
//  As temperature rises, thermistor resistance falls and the
//  node voltage rises toward 5V. This matches the vendor's own
//  reference formula (Beta = 3950, R0 = 10k at 25C).
//
//  FAULT DETECTION:
//  A properly connected sensor should never sit at the extreme
//  ends of the ADC range. If it does, that means an open wire, a
//  short, or a disconnected connector — not a real temperature —
//  so readExhaustTemp() reports a fault instead of returning a
//  bogus number.
//

#define TEMP_PIN              A3

#define TEMP_R_FIXED          4700.0f   // ohms, fixed divider resistor
#define TEMP_R0                10000.0f // ohms, thermistor R at 25C
#define TEMP_T0_KELVIN          298.15f // 25C in Kelvin
#define TEMP_BETA               3950.0f // thermistor Beta coefficient

#define TEMP_FAULT_LOW_ADC     5        // near 0V -> open/disconnected
#define TEMP_FAULT_HIGH_ADC    1018     // near 5V -> open/shorted

// If the sensor faults while in automatic mode, fail toward MORE
// cooling rather than less — an unreadable sensor is not evidence
// that the cabinet is safe.
#define TEMP_FAULT_FAN_SPEED   200


// ═══════════════════════════════════════════════════════════════
//  TEMPERATURE ALARMS
// ═══════════════════════════════════════════════════════════════
//
//  NOTE:
//  This sensor is mounted at the cabinet exhaust vent. It reads
//  the combined output of everything currently running inside,
//  which is exactly what you want for fan control and for a rough
//  "something in here is too hot" warning — but it is still a
//  single low-cost analog sensor, not a certified fire detector.
//
//  These values are therefore configuration values for the
//  controller's local warning logic, not certified fire protection.
//

#define FIRE_THRESHOLD      65.0f
#define TEMP_RISE_RATE      3.0f

#define TEMP_CHECK_INTERVAL 5000UL


// ═══════════════════════════════════════════════════════════════
//  PI HEARTBEAT WATCHDOG
// ═══════════════════════════════════════════════════════════════

#define HEARTBEAT_TIMEOUT   5000UL


// ═══════════════════════════════════════════════════════════════
//  DOOR
// ═══════════════════════════════════════════════════════════════

#define DOOR_PIN             11
#define DEBOUNCE_MS          50UL

#define WATER_PIN            A4

// The cabinet light (R1) is driven directly by the door switch,
// not by the Pi. It simply follows door state; the Pi is only
// told about the door itself (EVT DOOR OPEN / EVT DOOR CLOSED)
// and can react to that however it wants on its side.
#define DOOR_LIGHT_RELAY_INDEX 0


// ═══════════════════════════════════════════════════════════════
//  PERIODIC REPORT
// ═══════════════════════════════════════════════════════════════

#define REPORT_INTERVAL      5000UL


// ═══════════════════════════════════════════════════════════════
//  SERIAL INPUT
// ═══════════════════════════════════════════════════════════════
//
//  The parser is deliberately stateful.
//
//  We DO NOT assume that an entire serial command is available
//  in one Serial.available() call.
//
//  Serial input may arrive one byte at a time.
//

#define CMD_BUF_SIZE         32


// ═══════════════════════════════════════════════════════════════
//  RESET COMMAND LIMITS
// ═══════════════════════════════════════════════════════════════
//
//  Do not permit absurd reset durations.
//
//  This prevents arithmetic overflow and protects against
//  accidental/malformed commands.
//

#define RESET_MIN_SECONDS    1UL
#define RESET_MAX_SECONDS    3600UL


// ═══════════════════════════════════════════════════════════════
//  INTERNAL STATE
// ═══════════════════════════════════════════════════════════════

// ─── Pi heartbeat ──────────────────────────────────────────────

unsigned long lastHeartbeat = 0;
bool watchdogFired = false;


// ─── Serial parser ────────────────────────────────────────────

char commandBuffer[CMD_BUF_SIZE];
uint8_t commandLength = 0;
bool commandOverflow = false;


// ─── Fan ──────────────────────────────────────────────────────

float fanIntegral = 0.0f;

uint8_t fanSpeed = 0;

unsigned long lastFanUpdate = 0;

bool fanAutoMode = true;


// ─── Fan tachometers ──────────────────────────────────────────
//
//  The pulse counters are touched inside an ISR, so they must be
//  volatile. Everything else here is only ever touched from the
//  normal loop() side, in checkFanTach().

volatile uint16_t fan1PulseCount = 0;
volatile uint16_t fan2PulseCount = 0;
volatile uint16_t fan3PulseCount = 0;

// Last accepted pulse time per channel, for rejecting anything
// faster than TACH_MIN_PULSE_MS as electrical noise rather than a
// real revolution.
volatile unsigned long fan1LastPulseMs = 0;
volatile unsigned long fan2LastPulseMs = 0;
volatile unsigned long fan3LastPulseMs = 0;

// Snapshot of PINC (A0-A5) from the last interrupt, so the ISR
// can tell which bit(s) specifically fell (pulsed), rather than
// just "something on this bank changed".
volatile uint8_t lastPinCState = 0;

uint16_t fan1Rpm = 0;
uint16_t fan2Rpm = 0;
uint16_t fan3Rpm = 0;

unsigned long lastTachCheck = 0;


// ─── Fan clog detection ──────────────────────────────────────

float fanPeakTemp = 0.0f;

unsigned long fanHighSince = 0;

bool fanWasLow = true;

bool clogReported = false;


// ─── Temperature alarms ──────────────────────────────────────

float tempAtLastCheck = 0.0f;

unsigned long lastTempCheck = 0;

bool fireReported = false;

bool tempSensorFaultReported = false;


// ─── Door debounce ───────────────────────────────────────────

bool doorStable = true;
bool doorLast = true;

unsigned long doorChangeTime = 0;


// ─── Water debounce ──────────────────────────────────────────

bool waterStable = false;
bool waterLast = false;

unsigned long waterChangeTime = 0;


// ─── Relay reset tracking ────────────────────────────────────

struct ResetState {
  bool active;
  unsigned long start;
  unsigned long duration;
};

ResetState resets[8];


// ═══════════════════════════════════════════════════════════════
//  VERY EARLY AVR WATCHDOG INITIALIZATION
// ═══════════════════════════════════════════════════════════════
//
//  On ATmega328P-class AVRs, the watchdog can remain enabled
//  after a watchdog reset.
//
//  Disable it very early before normal Arduino startup code
//  has a chance to get trapped in repeated resets.
//
//  MCUSR is preserved so setup() can report a watchdog reset.
//

#if defined(__AVR_ATmega328P__) && ENABLE_HARDWARE_WATCHDOG

uint8_t resetCause
  __attribute__((section(".noinit")));

void earlyWatchdogInit(void)
  __attribute__((naked))
  __attribute__((used))
  __attribute__((section(".init3")));

void earlyWatchdogInit(void)
{
  resetCause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

#endif


// ═══════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════

void handleSerial();
void processCommand(char *cmd);

void checkWatchdog();
void failsafe();

void relaySet(uint8_t idx, bool loadOn);
void initializeRelays();

void updateFan();

void checkFanTach();

void checkTempAlarms();

void checkDoor();

void checkWater();

void checkResets();

void periodicReport();

bool readExhaustTemp(float *outTempC);

float measureVacRms();
float measureIacRms();
float measureDc12v();

bool parseUnsignedLong(
  const char *text,
  unsigned long *value
);


// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════

void setup()
{
  Serial.begin(115200);


  // ───────────────────────────────────────────────────────────
  // RELAYS
  // ───────────────────────────────────────────────────────────
  //
  // Establish the desired output latch BEFORE changing the pin
  // to OUTPUT.
  //
  // This minimizes the chance of an unwanted relay pulse during
  // boot.
  //

  initializeRelays();


  // ───────────────────────────────────────────────────────────
  // FAN
  // ───────────────────────────────────────────────────────────

  #if ENABLE_FAN

    pinMode(FAN_PIN, OUTPUT);

    fanSpeed = 0;
    analogWrite(FAN_PIN, 0);

  #endif


  // ───────────────────────────────────────────────────────────
  // DOOR
  // ───────────────────────────────────────────────────────────

  #if ENABLE_DOOR

    pinMode(DOOR_PIN, INPUT_PULLUP);

    // Read the actual state at startup.
    //
    // Confirmed wiring: the switch is pressed (pulls to GND, LOW)
    // when the door is CLOSED, and releases (pulled HIGH by
    // INPUT_PULLUP) when the door is OPEN.
    //
    // HIGH = door open
    // LOW  = door closed
    //
    doorStable = (digitalRead(DOOR_PIN) == HIGH);
    doorLast = doorStable;

    // The light relay is driven by the door, so it needs to be
    // synced to whatever the door's actual state is right now —
    // not assumed closed.
    relaySet(DOOR_LIGHT_RELAY_INDEX, doorStable);

  #endif


  // ───────────────────────────────────────────────────────────
  // ANALOG PINS
  // ───────────────────────────────────────────────────────────

  // A0/A1/A2 are now the fan tach inputs (see FAN TACHOMETERS
  // above) — open-collector outputs need the internal pullup to
  // have something to pull against.
  pinMode(FAN1_TACH_PIN, INPUT_PULLUP);
  pinMode(FAN2_TACH_PIN, INPUT_PULLUP);
  pinMode(FAN3_TACH_PIN, INPUT_PULLUP);

  pinMode(A3, INPUT);
  pinMode(WATER_PIN, INPUT);
  pinMode(A5, INPUT);


  #if defined(__AVR_ATmega328P__)

    // Enable pin-change interrupts on PCINT8/9/10 (A0/A1/A2) —
    // one shared bank, one shared handler for all 3 tach wires.
    lastPinCState = PINC;

    PCMSK1 |= _BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10);
    PCICR  |= _BV(PCIE1);

  #endif


  // ───────────────────────────────────────────────────────────
  // WATER
  // ───────────────────────────────────────────────────────────

  #if ENABLE_WATER

    // Read the actual state at startup, same reasoning as DOOR
    // above — don't assume dry if the sensor already sees water
    // the moment this thing boots.
    //
    // Confirmed wiring: this module outputs a voltage that rises
    // toward 5V as more water bridges its sensing traces, so a
    // plain digitalRead() reads it as a simple threshold:
    //
    // HIGH = water detected
    // LOW  = dry
    //
    waterStable = (digitalRead(WATER_PIN) == HIGH);
    waterLast = waterStable;

  #endif


  // ───────────────────────────────────────────────────────────
  // INITIAL TIMERS
  // ───────────────────────────────────────────────────────────

  unsigned long now = millis();

  lastHeartbeat = now;

  lastFanUpdate = now;

  lastTempCheck = now;

  float initialTemp;

  if (readExhaustTemp(&initialTemp)) {
    tempAtLastCheck = initialTemp;
  } else {
    // No valid reading yet — start from the fan's target rather
    // than a value that would look like a real temperature.
    tempAtLastCheck = FAN_TARGET_TEMP;
  }


  // ───────────────────────────────────────────────────────────
  // HARDWARE WATCHDOG
  // ───────────────────────────────────────────────────────────

  #if defined(__AVR_ATmega328P__) && ENABLE_HARDWARE_WATCHDOG

    wdt_enable(HARDWARE_WATCHDOG_TIMEOUT);
    wdt_reset();

    if (resetCause & _BV(WDRF)) {
      Serial.println(F("EVT MCU WATCHDOG RESET"));
    }

  #endif


  Serial.println(F("READY"));

  #if ENABLE_DOOR

    // Let the Pi know the door's actual state right away, instead
    // of only reporting it on the next transition. Without this,
    // a door that's already open at boot is invisible to the Pi
    // until someone closes and reopens it.
    //
    // Report both states here (not just "open") so a fresh MQTT
    // subscriber has something to show immediately, rather than
    // waiting on either a real transition or the next periodic
    // report to get its first value.
    if (doorStable) {
      Serial.println(F("EVT DOOR OPEN"));
    } else {
      Serial.println(F("EVT DOOR CLOSED"));
    }

  #endif

  #if ENABLE_WATER

    if (waterStable) {
      Serial.println(F("EVT WATER DETECTED"));
    } else {
      Serial.println(F("EVT WATER CLEARED"));
    }

  #endif
}


// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════
//
//  Keep this loop boring.
//
//  That is intentional.
//
//  A 20-year controller benefits from simple, predictable,
//  non-blocking tasks.
//

void loop()
{
  #if defined(__AVR_ATmega328P__) && ENABLE_HARDWARE_WATCHDOG
    wdt_reset();
  #endif


  handleSerial();

  checkWatchdog();


  #if ENABLE_FAN
    updateFan();
    checkFanTach();
  #endif


  #if ENABLE_DOOR
    checkDoor();
  #endif


  #if ENABLE_WATER
    checkWater();
  #endif


  checkResets();


  #if ENABLE_PERIODIC_REPORT
    periodicReport();
  #endif


  checkTempAlarms();


  #if defined(__AVR_ATmega328P__) && ENABLE_HARDWARE_WATCHDOG
    wdt_reset();
  #endif
}


// ═══════════════════════════════════════════════════════════════
//  RELAY INITIALIZATION
// ═══════════════════════════════════════════════════════════════

void initializeRelays()
{
  for (uint8_t i = 0; i < 8; i++) {

    // Boot default: de-energize every coil.
    //
    // The NC/NO wiring of each relay then decides what that
    // means for its load:
    //
    //   NO relay -> load OFF  (lights, general loads: safe to
    //                          start off)
    //   NC relay -> load ON   (critical equipment: stays powered
    //                          through a boot, including one
    //                          caused by the hardware watchdog —
    //                          matches failsafe() behavior below)
    //
    // ACTIVE LOW board: de-energized = RELAY_COIL_OFF (HIGH).
    digitalWrite(relayPins[i], RELAY_COIL_OFF);

    pinMode(relayPins[i], OUTPUT);

    // Explicitly establish the same state again after OUTPUT.
    digitalWrite(relayPins[i], RELAY_COIL_OFF);

    resets[i].active = false;
    resets[i].start = 0;
    resets[i].duration = 0;
  }
}


// ═══════════════════════════════════════════════════════════════
//  SERIAL INPUT
// ═══════════════════════════════════════════════════════════════
//
//  Important reliability property:
//
//  This function NEVER assumes that a complete command is already
//  in the UART receive buffer.
//
//  It accumulates characters until newline.
//
//  If a line is too long, the entire line is discarded.
//

void handleSerial()
{
  while (Serial.available() > 0) {

    char c = (char)Serial.read();


    // ─────────────────────────────────────────────────────────
    // End of command
    // ─────────────────────────────────────────────────────────

    if (c == '\n' || c == '\r') {

      if (commandOverflow) {

        // Invalid oversized command.
        commandLength = 0;
        commandOverflow = false;

        continue;
      }


      if (commandLength == 0) {
        continue;
      }


      commandBuffer[commandLength] = '\0';

      processCommand(commandBuffer);

      commandLength = 0;

      continue;
    }


    // ─────────────────────────────────────────────────────────
    // Ignore additional bytes after overflow
    // ─────────────────────────────────────────────────────────

    if (commandOverflow) {
      continue;
    }


    // ─────────────────────────────────────────────────────────
    // Store character
    // ─────────────────────────────────────────────────────────

    if (commandLength < CMD_BUF_SIZE - 1) {

      commandBuffer[commandLength++] = c;

    } else {

      // Command too long.
      //
      // Do not process a truncated command.
      commandOverflow = true;
    }
  }
}


// ═══════════════════════════════════════════════════════════════
//  COMMAND PROCESSOR
// ═══════════════════════════════════════════════════════════════

void processCommand(char *cmd)
{
  // ───────────────────────────────────────────────────────────
  // Trim trailing spaces/tabs
  // ───────────────────────────────────────────────────────────

  uint8_t len = strlen(cmd);

  while (len > 0 &&
         (cmd[len - 1] == ' ' ||
          cmd[len - 1] == '\t')) {

    cmd[--len] = '\0';
  }


  if (len == 0) {
    return;
  }


  // ───────────────────────────────────────────────────────────
  // Heartbeat
  // ───────────────────────────────────────────────────────────

  if (strcmp(cmd, "HB") == 0) {

    lastHeartbeat = millis();

    if (watchdogFired) {

      watchdogFired = false;

      #if ENABLE_FAN
        fanAutoMode = true;
        fanIntegral = 0.0f;
      #endif

      Serial.println(F("EVT WATCHDOG CLEARED"));
    }

    return;
  }


  // ───────────────────────────────────────────────────────────
  // Fan commands
  // ───────────────────────────────────────────────────────────

  if (strncmp(cmd, "FAN ", 4) == 0) {

    const char *value = cmd + 4;


    if (strcmp(value, "AUTO") == 0) {

      fanAutoMode = true;
      fanIntegral = 0.0f;

      return;
    }


    if (strcmp(value, "OFF") == 0) {

      // Do not allow FAN OFF to defeat the local Pi-dead
      // failsafe.
      if (watchdogFired) {
        return;
      }

      fanAutoMode = false;
      fanSpeed = 0;

      analogWrite(FAN_PIN, 0);

      return;
    }


    // Manual numeric fan command.
    unsigned long valueNumber = 0;

    if (!parseUnsignedLong(value, &valueNumber)) {
      return;
    }


    if (valueNumber > 255UL) {
      return;
    }


    // Do not allow Pi commands to defeat local failsafe.
    if (watchdogFired) {
      return;
    }


    fanAutoMode = false;

    fanSpeed = (uint8_t)valueNumber;

    analogWrite(FAN_PIN, fanSpeed);

    return;
  }


  // ───────────────────────────────────────────────────────────
  // All relays off
  // ───────────────────────────────────────────────────────────

  if (strcmp(cmd, "R ALL OFF") == 0) {

    // This command means "turn general-purpose loads off".
    //
    // Critical NC loads (currently R3/R4/R5) are intentionally
    // left alone —
    // see the "OFF" handling below for why. The only way to
    // power-cycle one of them is RESET <n> <secs>.

    for (uint8_t i = 0; i < 8; i++) {

      if (relayNC[i]) {
        continue;
      }

      if (!resets[i].active) {
        relaySet(i, false);
      }
    }

    return;
  }


  // ───────────────────────────────────────────────────────────
  // Individual relay
  //
  // Expected:
  //
  //   R1 ON
  //   R1 OFF
  //
  // ON is always allowed. OFF is rejected for critical NC loads
  // (see below) — those can only be power-cycled with RESET.
  //
  // ───────────────────────────────────────────────────────────

  if (cmd[0] == 'R' &&
      cmd[1] >= '1' &&
      cmd[1] <= '8' &&
      cmd[2] == ' ') {

    uint8_t idx = (uint8_t)(cmd[1] - '1');

    const char *state = cmd + 3;


    if (resets[idx].active) {
      return;
    }


    if (strcmp(state, "ON") == 0) {

      relaySet(idx, true);

      return;
    }


    if (strcmp(state, "OFF") == 0) {

      // Critical NC loads (currently R3/R4/R5) cannot be left off
      // by a plain OFF command — that's exactly the "Pi bug turns off the
      // Pi's own power" scenario this guards against. Use
      // RESET <n> <secs> for a controlled, self-recovering
      // power cycle instead.
      if (relayNC[idx]) {
        return;
      }

      relaySet(idx, false);

      return;
    }


    return;
  }


  // ───────────────────────────────────────────────────────────
  // Reset command
  //
  //   RESET <n> <seconds>
  //
  // Only NC relays may be reset.
  //
  // Reset means:
  //
  //   load OFF
  //   wait N seconds
  //   load ON
  //
  // ───────────────────────────────────────────────────────────

  if (strncmp(cmd, "RESET ", 6) == 0) {

    const char *p = cmd + 6;


    // parseUnsignedLong() requires its ENTIRE input to be digits
    // — that's correct for single-value commands like "FAN 200",
    // but RESET has two numbers ("4 30"), and handing it the
    // whole remainder at once meant it always hit the space in
    // the middle and silently failed, every single time, with no
    // error printed. Isolate each number first by temporarily
    // cutting the string at the separating space.

    char *space = strchr((char *)p, ' ');

    if (!space) {
      Serial.println(F("FAIL: RESET needs <relay> <seconds>"));
      return;
    }

    *space = '\0';

    unsigned long relayNumber = 0;

    if (!parseUnsignedLong(p, &relayNumber)) {
      Serial.println(F("FAIL: bad relay number"));
      *space = ' ';
      return;
    }

    *space = ' ';


    if (relayNumber < 1UL ||
        relayNumber > 8UL) {
      Serial.println(F("FAIL: relay number out of range"));
      return;
    }


    // Seconds — this one genuinely does run to the end of the
    // buffer with nothing trailing, so it's fine as-is.
    const char *secondsText = space + 1;

    unsigned long seconds = 0;

    if (!parseUnsignedLong(secondsText, &seconds)) {
      Serial.println(F("FAIL: bad seconds value"));
      return;
    }


    if (seconds < RESET_MIN_SECONDS ||
        seconds > RESET_MAX_SECONDS) {
      Serial.println(F("FAIL: seconds out of range"));
      return;
    }


    uint8_t idx = (uint8_t)(relayNumber - 1UL);


    // Only NC loads are permitted to use the RESET command.
    if (!relayNC[idx]) {
      Serial.println(F("FAIL: relay is not NC"));
      return;
    }


    if (resets[idx].active) {
      Serial.println(F("FAIL: reset already active"));
      return;
    }


    relaySet(idx, false);


    resets[idx].active = true;
    resets[idx].start = millis();
    resets[idx].duration = seconds * 1000UL;


    Serial.print(F("EVT RESET STARTED "));
    Serial.println(relayNumber);

    return;
  }


  // Unknown command:
  //
  // Silently ignore.
  //
  // Do not generate a serial error for arbitrary garbage because
  // the Pi may be reconnecting or recovering.
}


// ═══════════════════════════════════════════════════════════════
//  STRICT UNSIGNED INTEGER PARSER
// ═══════════════════════════════════════════════════════════════
//
//  Rejects:
//    - empty strings
//    - negative numbers
//    - non-numeric characters
//    - overflow
//
//  Accepts only:
//    "0"
//    "123"
//    etc.
//

bool parseUnsignedLong(
  const char *text,
  unsigned long *value
)
{
  if (text == NULL || *text == '\0') {
    return false;
  }


  unsigned long result = 0;


  while (*text != '\0') {

    if (*text < '0' || *text > '9') {
      return false;
    }


    uint8_t digit = (uint8_t)(*text - '0');


    // Overflow check.
    if (result > (ULONG_MAX - digit) / 10UL) {
      return false;
    }


    result = result * 10UL + digit;

    text++;
  }


  *value = result;

  return true;
}


// ═══════════════════════════════════════════════════════════════
//  RELAY HELPER
// ═══════════════════════════════════════════════════════════════
//
//  relaySet(i, true):
//      load ON
//
//  relaySet(i, false):
//      load OFF
//
//  NO:
//      ON  = coil energized
//      OFF = coil de-energized
//
//  NC:
//      ON  = coil de-energized
//      OFF = coil energized
//

void relaySet(uint8_t idx, bool loadOn)
{
  if (idx >= 8) {
    return;
  }


  bool coilEnergized =
    relayNC[idx] ? !loadOn : loadOn;


  digitalWrite(
    relayPins[idx],
    coilEnergized ? RELAY_COIL_ON : RELAY_COIL_OFF
  );
}


// ═══════════════════════════════════════════════════════════════
//  PI HEARTBEAT WATCHDOG
// ═══════════════════════════════════════════════════════════════
//
//  This does NOT reset the Arduino.
//
//  It puts the controller into local failsafe mode when the Pi
//  disappears.
//
//  The hardware watchdog above is a separate mechanism whose job
//  is to recover a completely stuck MCU.
//

void checkWatchdog()
{
  unsigned long now = millis();


  if ((now - lastHeartbeat) >= HEARTBEAT_TIMEOUT) {

    if (!watchdogFired) {

      watchdogFired = true;

      failsafe();

      Serial.println(F("EVT WATCHDOG FIRED"));
    }
  }
}


// ═══════════════════════════════════════════════════════════════
//  FAILSAFE
// ═══════════════════════════════════════════════════════════════
//
//  Pi disappeared.
//
//  Actions:
//
//    FAN       → 100%
//    NO relay  → OFF
//    NC relay  → ON
//    RESETs    → cancelled
//
//  IMPORTANT:
//  fanSpeed is explicitly set to 255 here.
//
//  This prevents updateFan() from immediately overwriting the
//  100% failsafe output on its next iteration.
//

void failsafe()
{
  #if ENABLE_FAN

    fanAutoMode = false;
    fanIntegral = 0.0f;

    fanSpeed = 255;

    analogWrite(FAN_PIN, 255);

  #endif


  for (uint8_t i = 0; i < 8; i++) {

    resets[i].active = false;


    if (relayNC[i]) {

      relaySet(i, true);

    } else {

      relaySet(i, false);
    }
  }
}


// ═══════════════════════════════════════════════════════════════
//  FAN CONTROL
// ═══════════════════════════════════════════════════════════════
//
//  Local fan control continues to work if the Pi is present.
//
//  If the Pi heartbeat has failed:
//
//      FAN = 100%
//
//  until a heartbeat arrives.
//
//  This makes the local failsafe state explicit rather than
//  depending on the automatic controller's state.
//

void updateFan()
{
  unsigned long now = millis();


  if ((now - lastFanUpdate) < FAN_UPDATE_INTERVAL) {
    return;
  }


  unsigned long dt = now - lastFanUpdate;

  lastFanUpdate = now;


  // Pi is dead.
  //
  // Failsafe overrides all normal fan control.

  if (watchdogFired) {

    fanSpeed = 255;

    analogWrite(FAN_PIN, 255);

    return;
  }


  // Door open: cooling the open room air is pointless, so hold
  // the fan off while auto mode would otherwise be deciding a
  // speed from temperature. This does NOT override watchdogFired
  // above (failsafe still wins — we don't know why the Pi is gone
  // or whether the door will stay open, so max cooling stays the
  // safer default), and does NOT override an explicit manual
  // FAN <n> command — that's a deliberate human choice regardless
  // of door state, same precedent as elsewhere in this file.
  //
  // fanIntegral is deliberately left untouched here rather than
  // reset to 0, so the controller resumes close to where it left
  // off the moment the door closes again, instead of re-ramping
  // from scratch every time.

  #if ENABLE_DOOR

    if (fanAutoMode && doorStable) {

      fanSpeed = 0;

      analogWrite(FAN_PIN, 0);

      return;
    }

  #endif


  // ───────────────────────────────────────────────────────────
  // Automatic mode (proportional + integral, with decay)
  // ───────────────────────────────────────────────────────────

  if (fanAutoMode) {

    float temp;
    bool tempValid = readExhaustTemp(&temp);

    if (!tempValid) {

      // Can't trust the reading — fail toward more cooling,
      // not less. checkTempAlarms() is responsible for telling
      // the Pi about the fault; this just picks a safe speed.
      fanSpeed = TEMP_FAULT_FAN_SPEED;

      analogWrite(FAN_PIN, fanSpeed);

      return;
    }

    float error = FAN_TARGET_TEMP - temp;


    if (error < 0.0f) {

      // Too hot.

      float hotAmount = -error;


      float p =
        FAN_KP * hotAmount;


      fanIntegral +=
        FAN_KI *
        hotAmount *
        ((float)dt / 1000.0f);


      // Integral clamp.
      fanIntegral =
        constrain(
          fanIntegral,
          0.0f,
          100.0f
        );


      float requested =
        FAN_MIN_SPEED +
        p +
        fanIntegral;


      if (requested >= 255.0f) {

        fanSpeed = 255;

      } else if (requested <= FAN_MIN_SPEED) {

        fanSpeed = FAN_MIN_SPEED;

      } else {

        fanSpeed = (uint8_t)requested;
      }

    } else {

      // At/below target.
      //
      // Slowly reduce speed rather than immediately turning the
      // fan off. This reduces cycling.

      float decay =
        FAN_DECAY_RATE *
        ((float)dt / 1000.0f);


      int newSpeed =
        (int)fanSpeed -
        (int)decay;


      if (newSpeed < FAN_MIN_SPEED) {
        newSpeed = FAN_MIN_SPEED;
      }


      if (newSpeed > 255) {
        newSpeed = 255;
      }


      fanSpeed = (uint8_t)newSpeed;


      fanIntegral -=
        0.1f *
        ((float)dt / 1000.0f);


      fanIntegral =
        constrain(
          fanIntegral,
          0.0f,
          100.0f
        );
    }
  }


  analogWrite(FAN_PIN, fanSpeed);
}


// ═══════════════════════════════════════════════════════════════
//  FAN TACHOMETER READING
// ═══════════════════════════════════════════════════════════════
//
//  One shared interrupt handler for all 3 tach wires (see FAN
//  TACHOMETERS config above for why they're grouped this way).
//
//  Pin-change interrupts fire on ANY change (rising or falling),
//  but a "pulse" is specifically a falling edge (the fan pulling
//  the line to GND). So this compares the current pin snapshot
//  against the last one, and only counts bits that went from 1
//  to 0 — a plain "did it change" check would double-count every
//  revolution (once on the way down, once on the way back up).
//
//  Kept deliberately tiny and fast, as any ISR should be.
//

#if defined(__AVR_ATmega328P__)

ISR(PCINT1_vect)
{
  uint8_t current = PINC;

  uint8_t fell = (current ^ lastPinCState) & lastPinCState;

  unsigned long now = millis();

  if (fell & _BV(PINC0)) {  // A0
    if ((now - fan1LastPulseMs) >= TACH_MIN_PULSE_MS) {
      fan1PulseCount++;
      fan1LastPulseMs = now;
    }
  }

  if (fell & _BV(PINC1)) {  // A1
    if ((now - fan2LastPulseMs) >= TACH_MIN_PULSE_MS) {
      fan2PulseCount++;
      fan2LastPulseMs = now;
    }
  }

  if (fell & _BV(PINC2)) {  // A2
    if ((now - fan3LastPulseMs) >= TACH_MIN_PULSE_MS) {
      fan3PulseCount++;
      fan3LastPulseMs = now;
    }
  }

  lastPinCState = current;
}

#endif


//  Turns raw pulse counts into RPM every TACH_CHECK_INTERVAL, and
//  resets the counters for the next window. The brief
//  noInterrupts()/interrupts() pair is just to read+clear each
//  counter atomically — without it, the ISR could increment a
//  counter in the middle of it being read, corrupting the value.
//  It's a handful of instructions, not a real blocking concern.

void checkFanTach()
{
  unsigned long now = millis();

  if ((now - lastTachCheck) < TACH_CHECK_INTERVAL) {
    return;
  }

  unsigned long elapsedMs = now - lastTachCheck;

  lastTachCheck = now;


  uint16_t count1;
  uint16_t count2;
  uint16_t count3;

  noInterrupts();

  count1 = fan1PulseCount;
  fan1PulseCount = 0;

  count2 = fan2PulseCount;
  fan2PulseCount = 0;

  count3 = fan3PulseCount;
  fan3PulseCount = 0;

  interrupts();


  fan1Rpm =
    (uint16_t)(((unsigned long)count1 * 60000UL) /
               (TACH_PULSES_PER_REV * elapsedMs));

  fan2Rpm =
    (uint16_t)(((unsigned long)count2 * 60000UL) /
               (TACH_PULSES_PER_REV * elapsedMs));

  fan3Rpm =
    (uint16_t)(((unsigned long)count3 * 60000UL) /
               (TACH_PULSES_PER_REV * elapsedMs));
}


// ═══════════════════════════════════════════════════════════════
//  TEMPERATURE ALARMS
// ═══════════════════════════════════════════════════════════════
//
//  Three independent checks:
//
//    FIRE
//      Absolute temperature threshold.
//
//    RISING
//      Temperature increasing too quickly.
//
//    CLOG
//      Fan has been running hard but temperature has not fallen.
//
//  These are advisory local alarms. For genuine fire protection,
//  use a dedicated external temperature/smoke/fire system.
//

void checkTempAlarms()
{
  unsigned long now = millis();


  if ((now - lastTempCheck) < TEMP_CHECK_INTERVAL) {
    return;
  }


  float temp;
  bool tempValid = readExhaustTemp(&temp);

  if (!tempValid) {

    if (!tempSensorFaultReported) {

      tempSensorFaultReported = true;

      Serial.println(F("EVT TEMP SENSOR FAULT"));
    }

    // Keep the check interval honest even while faulted, and
    // don't evaluate FIRE/RISING/CLOG against a number that
    // isn't a real temperature.
    lastTempCheck = now;

    return;
  }

  if (tempSensorFaultReported) {

    tempSensorFaultReported = false;

    Serial.println(F("EVT TEMP SENSOR OK"));
  }


  // ───────────────────────────────────────────────────────────
  // FIRE
  // ───────────────────────────────────────────────────────────

  if (temp >= FIRE_THRESHOLD) {

    if (!fireReported) {

      fireReported = true;

      Serial.print(F("EVT FIRE "));
      Serial.println(temp, 1);
    }

  } else if (temp < FIRE_THRESHOLD - 5.0f) {

    // Hysteresis prevents alarm chatter.

    if (fireReported) {
      Serial.println(F("EVT FIRE CLEARED"));
    }

    fireReported = false;
  }


  // ───────────────────────────────────────────────────────────
  // RISING RATE
  // ───────────────────────────────────────────────────────────

  unsigned long elapsed =
    now - lastTempCheck;


  if (elapsed > 0) {

    float delta =
      temp - tempAtLastCheck;


    float minutes =
      (float)elapsed / 60000.0f;


    float ratePerMinute =
      delta / minutes;


    if (ratePerMinute > TEMP_RISE_RATE) {

      Serial.print(F("EVT TEMP RISING "));
      Serial.println(ratePerMinute, 1);
    }
  }


  tempAtLastCheck = temp;
  lastTempCheck = now;


  // ───────────────────────────────────────────────────────────
  // FAN CLOG DETECTION
  // ───────────────────────────────────────────────────────────

  if (fanSpeed > 128) {

    if (fanWasLow) {

      fanPeakTemp = temp;
      fanHighSince = now;

      fanWasLow = false;
      clogReported = false;
    }


    if (temp > fanPeakTemp) {
      fanPeakTemp = temp;
    }


    if ((now - fanHighSince) >= FAN_CLOG_THRESHOLD) {

      float dropped =
        fanPeakTemp - temp;


      if (dropped < FAN_CLOG_DELTA &&
          !clogReported) {

        clogReported = true;

        Serial.println(F("EVT FILTER CLOGGED"));
      }
    }

  } else {

    if (clogReported) {
      Serial.println(F("EVT FILTER CLEARED"));
    }

    fanWasLow = true;
    clogReported = false;
  }
}


// ═══════════════════════════════════════════════════════════════
//  DOOR DETECTION
// ═══════════════════════════════════════════════════════════════
//
//  D11 → endstop → GND
//
//  INPUT_PULLUP, confirmed by direct hardware test:
//
//    HIGH = door open    (switch released)
//    LOW  = door closed  (switch pressed, pulls to GND)
//
//  The startup state is read from the actual pin, so the system
//  does not falsely assume the door is closed.
//
//  The cabinet light (DOOR_LIGHT_RELAY_INDEX) is driven directly
//  from doorStable, independent of anything the Pi does. The Pi
//  is only informed via EVT DOOR OPEN / EVT DOOR CLOSED — what it
//  does with that information is up to the Pi side.
//

void checkDoor()
{
  bool doorRaw =
    (digitalRead(DOOR_PIN) == HIGH);


  if (doorRaw != doorLast) {
    doorChangeTime = millis();
  }

  // Updated every single pass, unconditionally — this is the
  // detail the earlier version got wrong. Without it, once doorRaw
  // differs from doorLast, that comparison stays true forever
  // (doorLast never catches up), doorChangeTime keeps getting
  // reset to "now" every loop, and the debounce timer can never
  // actually finish counting. That's a permanent deadlock, not a
  // flaky one — which is exactly "never responds, no matter how
  // long you wait."
  doorLast = doorRaw;


  if ((millis() - doorChangeTime) >= DEBOUNCE_MS &&
      doorRaw != doorStable) {

    doorStable = doorRaw;


    relaySet(DOOR_LIGHT_RELAY_INDEX, doorStable);

    if (doorStable) {

      Serial.println(F("EVT DOOR OPEN"));

    } else {

      Serial.println(F("EVT DOOR CLOSED"));
    }
  }
}


// ═══════════════════════════════════════════════════════════════
//  WATER DETECTION
// ═══════════════════════════════════════════════════════════════
//
//  WATER_PIN (A4) → Gikfun-style analog leak sensor, read here as
//  a simple threshold rather than a proportional value.
//
//  Confirmed by direct test: bridging the sensor pin to 5V (fully
//  "wet") reads HIGH; releasing it (dry) settles LOW.
//
//    HIGH = water detected
//    LOW  = dry
//
//  Same debounce shape as checkDoor() above — this is a plain
//  edge-triggered detector, not a continuous reading. It only
//  ever prints on an actual transition, so it can't spam the log
//  or the dashboard.
//

void checkWater()
{
  bool waterRaw =
    (digitalRead(WATER_PIN) == HIGH);


  if (waterRaw != waterLast) {
    waterChangeTime = millis();
  }

  // Same fix as checkDoor(): update unconditionally every pass,
  // or this deadlocks and never commits a change.
  waterLast = waterRaw;


  if ((millis() - waterChangeTime) >= DEBOUNCE_MS &&
      waterRaw != waterStable) {

    waterStable = waterRaw;


    if (waterStable) {

      Serial.println(F("EVT WATER DETECTED"));

    } else {

      Serial.println(F("EVT WATER CLEARED"));
    }
  }
}


// ═══════════════════════════════════════════════════════════════
//  RESET COMPLETION
// ═══════════════════════════════════════════════════════════════
//
//  Uses:
//
//      millis() - start >= duration
//
//  which remains correct across millis() rollover.
//

void checkResets()
{
  unsigned long now = millis();


  for (uint8_t i = 0; i < 8; i++) {

    if (!resets[i].active) {
      continue;
    }


    if ((now - resets[i].start) >=
        resets[i].duration) {

      relaySet(i, true);

      resets[i].active = false;

      Serial.print(F("EVT RESET DONE "));
      Serial.println(i + 1);
    }
  }
}


// ═══════════════════════════════════════════════════════════════
//  PERIODIC SENSOR REPORT
// ═══════════════════════════════════════════════════════════════
//
//  No dynamic String construction.
//
//  snprintf() is intentionally kept out of the normal control
//  path. This reporting function is optional and relatively
//  infrequent.
//
// ═══════════════════════════════════════════════════════════════

void periodicReport()
{
  static unsigned long lastReport = 0;


  unsigned long now = millis();


  if ((now - lastReport) < REPORT_INTERVAL) {
    return;
  }


  lastReport = now;


  float temp;
  bool tempValid = readExhaustTemp(&temp);


  Serial.print(F("EVT TEMP "));

  if (tempValid) {
    Serial.print(temp, 1);
  } else {
    Serial.print(F("FAULT"));
  }

  Serial.print(F(" FAN "));
  Serial.print(fanSpeed);


  #if ENABLE_FAN

    Serial.print(F(" TACH1 "));
    Serial.print(fan1Rpm);

    Serial.print(F(" TACH2 "));
    Serial.print(fan2Rpm);

    Serial.print(F(" TACH3 "));
    Serial.print(fan3Rpm);

  #endif


  // Same reasoning as DOOR/WATER: without this, a fire tile that
  // never received a single message just sits at "unknown"
  // forever, which defeats the entire point of a safety status
  // indicator — it should say "still fine" continuously, not stay
  // silent until the one day it isn't.
  Serial.print(F(" FIRE "));
  Serial.print(fireReported ? F("ALARM") : F("SAFE"));


  #if ENABLE_AC_SENSORS

    Serial.print(F(" VAC "));
    Serial.print(measureVacRms(), 1);

    Serial.print(F(" IAC "));
    Serial.print(measureIacRms(), 2);

  #endif


  #if ENABLE_DC_SENSOR

    Serial.print(F(" DCV "));
    Serial.print(measureDc12v(), 2);

  #endif


  #if ENABLE_DOOR

    Serial.print(F(" DOOR "));
    Serial.print(doorStable ? F("OPEN") : F("CLOSED"));

  #endif


  #if ENABLE_WATER

    Serial.print(F(" WATER "));
    Serial.print(waterStable ? F("DETECTED") : F("CLEARED"));

  #endif


  Serial.println();
}


// ═══════════════════════════════════════════════════════════════
//  EXHAUST TEMPERATURE SENSOR (Keyestudio analog module)
// ═══════════════════════════════════════════════════════════════
//
//  Standard NTC-thermistor analog module (Keyestudio "Analog
//  Temperature Sensor Detection Module" and equivalents). Uses
//  the vendor's own reference formula: a fixed resistor divider
//  read through the ADC, converted via the Beta parametric
//  equation.
//
//      5V ---[thermistor]---+---[TEMP_R_FIXED]--- GND
//                            |
//                         TEMP_PIN
//
//  Returns false (instead of a fabricated number) if the reading
//  sits at either rail, which means the sensor is disconnected,
//  shorted, or otherwise not actually reporting a temperature.
//

bool readExhaustTemp(float *outTempC)
{
  int adc = analogRead(TEMP_PIN);


  if (adc <= TEMP_FAULT_LOW_ADC ||
      adc >= TEMP_FAULT_HIGH_ADC) {

    return false;
  }


  float voltage = ((float)adc / 1023.0f) * 5.0f;

  float rt =
    TEMP_R_FIXED *
    (5.0f - voltage) /
    voltage;

  float tempK =
    1.0f /
    ( (1.0f / TEMP_T0_KELVIN) +
      ((float)log(rt / TEMP_R0) / TEMP_BETA) );


  *outTempC = tempK - 273.15f;

  return true;
}


// ═══════════════════════════════════════════════════════════════
//  AC VOLTAGE
// ═══════════════════════════════════════════════════════════════
//
//  ZMPT101B stub until hardware is finalized.
//
//  IMPORTANT:
//  AC measurement should eventually be designed and calibrated
//  as a proper isolated measurement subsystem.
//
//  Do not connect mains directly to an Arduino analog input.
//

float measureVacRms()
{
  // TODO:
  //
  // Sample the conditioned ZMPT101B waveform.
  //
  // Calculate RMS after removing DC bias.
  //
  // Calibrate against a trusted multimeter.
  //
  // Do not assume the nominal 2.5 V midpoint is exact.

  return 0.0f;
}


// ═══════════════════════════════════════════════════════════════
//  AC CURRENT
// ═══════════════════════════════════════════════════════════════

float measureIacRms()
{
  // TODO:
  //
  // Sample the CT waveform.
  //
  // Remove measured DC bias.
  //
  // Calculate RMS.
  //
  // Calibrate using a known load.

  return 0.0f;
}


// ═══════════════════════════════════════════════════════════════
//  DC VOLTAGE
// ═══════════════════════════════════════════════════════════════

float measureDc12v()
{
  // TODO:
  //
  // Example divider:
  //
  //   Rtop    = 10.0k
  //   Rbottom = 4.7k
  //
  //   Vinput =
  //     ADC voltage *
  //     (10.0 + 4.7) / 4.7
  //
  // IMPORTANT:
  // Verify resistor tolerances and maximum input voltage
  // before enabling this function.

  return 0.0f;
}


// ═══════════════════════════════════════════════════════════════
//  NOTES – LONG-TERM RELIABILITY
// ═══════════════════════════════════════════════════════════════
//
//  1. SRAM
//  ───────
//
//  SRAM does not wear out from being read or written.
//
//  The important thing is avoiding unnecessary dynamic allocation.
//
//  This program therefore uses:
//
//    - Fixed-size buffers
//    - Static/global state
//    - No String
//    - No malloc()
//    - No free()
//    - No new/delete
//
//  The command buffer is deliberately fixed at 32 bytes.
//
//
//  2. EEPROM
//  ─────────
//
//  This program intentionally performs NO EEPROM writes.
//
//  If configuration storage is added later, do NOT write settings
//  every loop or every sensor update.
//
//  EEPROM has finite write endurance.
//
//  Configuration changes should be rare and explicit.
//
//
//  3. millis() OVERFLOW
//  ────────────────────
//
//  All timers use:
//
//      millis() - start >= duration
//
//  rather than:
//
//      millis() >= start + duration
//
//  This makes the timing logic safe across the normal unsigned
//  millis() rollover.
//
//
//  4. SERIAL INPUT
//  ──────────────
//
//  Serial data is asynchronous.
//
//  Never assume that one Serial.available() call contains a whole
//  command.
//
//  The parser therefore collects a complete newline-terminated
//  command before processing it.
//
//  Overlong commands are discarded rather than truncated.
//
//
//  5. MALFORMED COMMANDS
//  ─────────────────────
//
//  Invalid commands are ignored.
//
//  This is intentional.
//
//  A communication glitch should not accidentally turn into:
//
//      RESET 3 4294967295
//
//  or another dangerous interpretation.
//
//
//  6. HARDWARE WATCHDOG
//  ───────────────────
//
//  The AVR hardware watchdog exists to recover from a genuinely
//  stuck firmware state.
//
//  It is NOT the same as the Pi heartbeat watchdog.
//
//  Pi watchdog:
//      Pi disappears → local failsafe.
//
//  AVR watchdog:
//      Firmware stops executing → MCU reset.
//
//
//  7. PI HEARTBEAT
//  ───────────────
//
//  The Pi must periodically send:
//
//      HB
//
//  If no HB arrives within HEARTBEAT_TIMEOUT:
//
//      - Fan goes to 100%
//      - NC loads are restored ON
//      - NO loads are turned OFF
//      - Active relay resets are cancelled
//
//  The failsafe remains active until another HB arrives.
//
//
//  8. RELAY SAFETY
//  ───────────────
//
//  Software cannot guarantee relay behavior during every possible
//  hardware failure.
//
//  For critical equipment:
//
//      - Use NC relay logic where appropriate.
//      - Consider normally-powered loads.
//      - Consider independent hardware thermal protection.
//      - Consider independent over-temperature protection.
//      - Consider what happens if the Arduino loses power.
//      - Consider what happens if the relay contacts weld.
//
//  The hardware should fail into the desired safe state.
//
//
//  9. FAN FAILSAFE
//  ───────────────
//
//  A Pi heartbeat failure forces:
//
//      fanSpeed = 255
//
//  and updateFan() continues enforcing 255 while watchdogFired
//  is true.
//
//  This avoids the previous failure mode where the failsafe could
//  briefly command 100% and then the automatic controller could
//  overwrite it.
//
//  Separately, if the exhaust sensor itself faults (open/short)
//  while in automatic mode, updateFan() picks TEMP_FAULT_FAN_SPEED
//  rather than guessing — a sensor that can't be read is not
//  evidence that the cabinet is safe.
//
//
//  10. TEMPERATURE SENSOR
//  ─────────────────────
//
//  The Keyestudio analog module at the exhaust vent is a simple,
//  low-cost NTC thermistor — convenient, but not a precision or
//  certified instrument. readExhaustTemp() treats a reading at
//  either ADC rail as a fault rather than trusting it, and both
//  the fan controller and the alarm logic have explicit fallback
//  behavior for that case (see notes 9 and 11).
//
//  If a DHT11 (or better) intake sensor gets added later, keep it
//  independent rather than blending it into this exhaust reading —
//  intake and exhaust answer different questions, and merging them
//  would make failures harder to diagnose.
//
//
//  11. FIRE DETECTION
//  ─────────────────
//
//  Do not treat this Arduino's temperature threshold as a
//  certified fire alarm.
//
//  A real server room should use appropriate independent fire
//  detection/protection equipment.
//
//
//  12. ANALOG MEASUREMENTS
//  ───────────────────────
//
//  ADC readings depend on:
//
//      - Reference voltage
//      - Ground quality
//      - Noise
//      - Sensor conditioning
//      - Component tolerances
//      - Temperature
//      - Calibration
//
//  Do not enable the AC/DC sensor features until the complete
//  analog front ends have been designed and tested.
//
//
//  13. NO BLOCKING DELAYS
//  ─────────────────────
//
//  Normal operation does not use delay().
//
//  Timing is performed using millis().
//
//  This allows:
//
//      - Watchdog servicing
//      - Serial processing
//      - Door monitoring
//      - Relay timers
//      - Fan control
//
//  to continue independently.
//
//
//  14. LONG-TERM POWER SUPPLY
//  ──────────────────────────
//
//  For a genuine 20-year installation, the Arduino is only one
//  part of the reliability chain.
//
//  Pay equal attention to:
//
//      - Power supply quality
//      - Surge protection
//      - Brownout behavior
//      - Grounding
//      - Connector quality
//      - Relay contact ratings
//      - Relay coil suppression
//      - Fan failure
//      - Fan bearing life
//      - Cable strain relief
//      - Moisture
//      - Dust
//      - Heat
//      - EMI
//
//  A good firmware design cannot compensate for poor hardware.
//
//
//  15. BROWNOUT
//  ────────────
//
//  For a long-life installation, configure the ATmega328P brownout
//  detection appropriately in the bootloader/fuse configuration.
//
//  The exact setting depends on the actual supply voltage and
//  regulator design.
//
//  The goal is to prevent the MCU from executing unpredictably
//  during a marginal supply condition.
//
//
//  16. BOOT BEHAVIOR
//  ────────────────
//
//  The hardware should be wired so that the desired safe state
//  exists even while the MCU is:
//
//      - powered off
//      - resetting
//      - booting
//      - crashed
//
//  Software should reinforce the hardware behavior, not be the
//  sole thing making it safe.
//
//
//  17. FUTURE SENSOR DESIGN
//  ───────────────────────
//
//  When adding a sensor:
//
//      1. Choose the hardware input.
//      2. Define its failure behavior.
//      3. Decide what "sensor disconnected" means.
//      4. Add validation/range checking.
//      5. Add hysteresis where appropriate.
//      6. Avoid EEPROM writes.
//      7. Avoid dynamic memory.
//      8. Keep processing bounded and non-blocking.
//      9. Test sensor failure explicitly.
//     10. Test power cycling explicitly.
//
//
//  18. RELAY WIRING CHANGES
//  ────────────────────────
//
//  If changing NO ↔ NC:
//
//      1. Rewire the physical load.
//      2. Change relayNC[].
//      3. Reflash.
//      4. Power-cycle.
//      5. Verify the dead-Arduino state physically.
//      6. Verify the Pi-dead failsafe physically.
//
//
//  19. TESTING FOR "20 YEARS"
//  ──────────────────────────
//
//  Before deployment, test:
//
//      - Power loss
//      - Power restoration
//      - Pi unplugged
//      - Pi crashed
//      - Serial cable unplugged
//      - Corrupt serial data
//      - Very long serial lines
//      - Relay reset
//      - Reset interrupted by Pi failure
//      - Door bouncing
//      - Door stuck open
//      - Fan disconnected
//      - Fan stalled
//      - Temperature sensor failure
//      - MCU watchdog reset
//      - Brownout
//      - millis() rollover in simulation
//
//  Do not merely test the happy path.
//
//
//  20. MOST IMPORTANT PRINCIPLE
//  ─────────────────────────────
//
//  "20 years" does not mean:
//
//      "The Arduino code never crashes."
//
//  It means:
//
//      "Every foreseeable failure has a defined behavior."
//
//  The best architecture is therefore:
//
//      Hardware safety
//          ↓
//      Local Arduino failsafe
//          ↓
//      AVR hardware watchdog
//          ↓
//      Pi supervision
//          ↓
//      Logging / alerting
//
//  No single component should be responsible for the entire
//  safety chain.
//
// ═══════════════════════════════════════════════════════════════

