/*
   Record two IR patterns and transmit them with twice a day.

   Instructions:

     1) Plug in power.
     2) Press button.
     3) Red LED lights. Teach first pattern.
     4) Red+Green LED light. Teach second pattern.
     5) Green LED lights. Press button to start timer and send first pattern.
     6) The second pattern will be sent after a fixed time (kDelay1Secs below).
     7) The first pattern will be sent again after a fixed time (kCycleSecs below).

   LED patterns:

   Red | Green | What it means
   ----+-------+---------------
    -  |   -   | Uninitialized.
    L  |   -   | Waiting for first pattern.
    L  |   L   | Waiting for second pattern.
    -  |   L   | Patterns learned.
    F  |   L   | Pattern sent.

    Copyright (c) 2017 Tommie Gannert
    Licensed under the MIT license.
*/

/// How long to wait after a remote control stops sending patterns while learning.
/// See the sig variable for rationale.
static const uint32_t kRxTimeoutMicros = 1ul << 16;

/// How long to wait between sending the first and second pattern.
static const uint32_t kDelay1Millis = 12ul * 60 * 60 * 1000;

/// How long to wait between sending the first pattern (total cycle time).
static const uint32_t kCycleMillis = 24ul * 60 * 60 * 1000;

/// Carrier frequency for transmission.
static const uint32_t kCarrierFreq = 38000ul;

// Pin definitions.
static const uint8_t kRedLedPin = 3;
static const uint8_t kButtonPin = 4;
static const uint8_t kGreenLedPin = 5;
static const uint8_t kTxPin = 6;  // Must support the tone() PWM functionality.
static const uint8_t kRxPin = 8;

enum State {
  /// Uninitialized. Just waiting for user input.
  STATE_UNINITED = 0,

  /// Waiting for IR data, or receiving IR data.
  STATE_LEARN_0,
  STATE_LEARN_1,

  /// Initialized. Waiting for user to start.
  STATE_INITED,

  /// Send sig[0] pattern.
  STATE_SEND_0,

  /// Wait for time to pass.
  STATE_WAIT_0,

  /// Send sig[1] pattern.
  STATE_SEND_1,

  /// Wait for time to pass.
  STATE_WAIT_1,
};

/// The current overall program state. See State.
static uint8_t state;

/// The previous value of the button pin.
static uint8_t prevButton;

/// The pattern, as microseconds to keep the level running.
/// Note: Only 2 kB RAM in ATMega328. The data type must be large
/// enough to track the largest pattern. kRxTimeoutMicros should be
/// small enough to protect against wraps.
static uint16_t sig[2][400];

/// Pointer to where to write the next received time. Points into one of the sig buffers.
static uint16_t *rxBuf;

/// The number of elements remaining in rxBuf.
static uint16_t rxSize;

/// The previous value of the IR receive pin.
static uint8_t prevRx;

/// When the state started.
static uint32_t startMicros;

/// Pointer to the current timeout for sending. Points into one of the sig buffers.
static uint16_t *txBuf;

/// The number of elements remaining in txBuf.
static uint16_t txLen;

/// True if the current data piece is a HIGH.
static bool txHigh;

/// When the wait state was entered.
static uint32_t waitStartMillis;

/// How many milliseconds to wait.
static uint32_t waitDelayMillis;

static void startRx(uint8_t i) {
  rxBuf = sig[i];
  rxSize = sizeof(sig[i]) / sizeof(*sig[i]);
  memset(rxBuf, 0, rxSize * sizeof(rxBuf));
  startMicros = 0;
  prevRx = HIGH;
}

static void startTx(uint8_t i) {
  digitalWrite(kRedLedPin, HIGH);
  txBuf = sig[i];
  txLen = sizeof(sig[i]) / sizeof(*sig[i]);
  txHigh = false;

  // Ignore the unused tail.
  while (txLen && !txBuf[txLen - 1])
    --txLen;
}

static void startWait(uint32_t delayMillis) {
  digitalWrite(kRedLedPin, LOW);
  waitStartMillis = millis();
  waitDelayMillis = delayMillis;
}

static void setState(uint8_t newState) {
  if (newState == state) {
    return;
  }

  state = newState;

  switch (state) {
    case STATE_UNINITED:
      digitalWrite(kRedLedPin, LOW);
      digitalWrite(kGreenLedPin, LOW);
      break;

    case STATE_LEARN_0:
      digitalWrite(kRedLedPin, HIGH);
      digitalWrite(kGreenLedPin, LOW);
      startRx(0);
      break;

    case STATE_LEARN_1:
      digitalWrite(kRedLedPin, HIGH);
      digitalWrite(kGreenLedPin, HIGH);
      startRx(1);
      break;

    case STATE_INITED:
      digitalWrite(kRedLedPin, LOW);
      digitalWrite(kGreenLedPin, HIGH);
      break;

    case STATE_SEND_0:
      startTx(0);
      break;

    case STATE_WAIT_0:
      digitalWrite(LED_BUILTIN, HIGH);
      startWait(kDelay1Millis);
      break;

    case STATE_SEND_1:
      startTx(1);
      break;

    case STATE_WAIT_1:
      digitalWrite(LED_BUILTIN, LOW);
      startWait(kCycleMillis - kDelay1Millis);
      break;
  }
}

static bool isButtonEvent() {
  uint8_t btn = digitalRead(kButtonPin);

  if (btn == prevButton) {
    return false;
  }

  prevButton = btn;

  return btn == HIGH;
}

static void loopUninitialized() {
  if (isButtonEvent()) {
    setState(STATE_LEARN_0);
  }
}

static void loopRx() {
  uint8_t rx = digitalRead(kRxPin);
  uint32_t now = micros();

  if (startMicros != 0 && now - startMicros >= kRxTimeoutMicros) {
    setState(state + 1);
    return;
  }

  if (rx == prevRx) {
    // No change.
    return;
  }

  prevRx = rx;

  *rxBuf = now - startMicros;
  ++rxBuf;
  --rxSize;
  startMicros = now;

  if (rxSize == 0) {
    // Overflow
    setState(STATE_UNINITED);
    return;
  }
}

static void loopInitialized() {
  if (isButtonEvent()) {
    setState(STATE_SEND_0);
  }
}

/// Send the next data.
static void loopTx() {
  if (*txBuf) {
    if (txHigh) {
      tone(kTxPin, kCarrierFreq);
    } else {
      noTone(kTxPin);
    }

    // TODO: Make non-blocking.
    delayMicroseconds(*txBuf);
  }

  ++txBuf;
  --txLen;
  txHigh = !txHigh;

  if (txLen == 0) {
    // No more data.
    noTone(kTxPin);
    txBuf = nullptr;
    setState(state + 1);
    return;
  }
}

/// Wait for time to pass.
static void loopWait() {
  if (isButtonEvent()) {
    // User requested to start over.
    setState(STATE_SEND_0);
    return;
  }

  uint32_t now = millis();
  if (now - waitStartMillis > waitDelayMillis) {
    waitStartMillis = 0;
    setState(state + 1);
  }
}

/// Called by Arduino once when the device starts up.
void setup() {
  // Initialize non-zero variables. For some reason, global variables are not initialized otherwise.
  prevButton = HIGH;
  prevRx = HIGH;

  // Set up input pins.
  pinMode(kRxPin, INPUT_PULLUP);
  pinMode(kButtonPin, INPUT_PULLUP);

  // Set up output pins.
  pinMode(kTxPin, OUTPUT);
  digitalWrite(kTxPin, LOW);

  pinMode(kRedLedPin, OUTPUT);
  digitalWrite(kRedLedPin, LOW);

  pinMode(kGreenLedPin, OUTPUT);
  digitalWrite(kGreenLedPin, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

/// Called by Arduino repeatedly while the device is on.
void loop() {
  switch (state) {
    case STATE_UNINITED:
      loopUninitialized();
      break;

    case STATE_LEARN_0:
    case STATE_LEARN_1:
      loopRx();
      break;

    case STATE_INITED:  // Initialized
      loopInitialized();
      break;

    case STATE_SEND_0:
    case STATE_SEND_1:
      loopTx();
      break;

    case STATE_WAIT_0:
    case STATE_WAIT_1:
      loopWait();
      break;

    default:
      setState(STATE_SEND_0);
  }
}

