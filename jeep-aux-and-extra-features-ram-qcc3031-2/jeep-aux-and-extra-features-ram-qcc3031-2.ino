/*****************************************************************************************
 *
 * Hack for my Dodge RAM 2006 MEX + RAQ radio head unit
 *
 * My hardware:
 *  1. Arduino Nano re-flashed with 8MHz bootloader
 *  2. Mcp2515_can SPI module (8MHz)
 *  3. QCC3031 bluetooth module controlled by digital pins
 *
 * Features:
 *  1. Bench mode to enable radio functioning while removed from the car
 *  2. Emulating VES presence to enable AUX IN in head unit
 *  2.1. Turn on/off BT A2DP module only when AUX IN is selected and car ignition is on
 *  3. Control BT audio source with buttons
 *
 * Copyright (C) 2015-2017 Anton Viktorov <latonita@yandex.ru> https://github.com/latonita/jeep-canbus,
 * 2025 Tengz https://github.com/tengzl33t/jeep-canbus
 *
 * Modified https://github.com/autowp/arduino-mcp2515 library is used for MCP2515
 *
 * This is free software. You may use/redistribute it under The MIT License terms.
 *
 *****************************************************************************************/
#include <SoftwareSerial.h>
#include <SPI.h>
#include "mcp2515.h"

#include <avr/sleep.h>

// === Constants ===

constexpr uint8_t PLAY_BTN = 9;
constexpr uint8_t SEEK_BCK = 8;
constexpr uint8_t SEEK_FWD = 7;
constexpr uint8_t VOL_UP = 6;
constexpr uint8_t VOL_DWN = 5;
constexpr uint8_t BT_POWER = 4;

constexpr uint16_t CAN_POWER = 0x000;
constexpr uint16_t CAN_RADIO_MODE = 0x09f;
constexpr uint16_t CAN_VES_UNIT = 0x3dd;

constexpr unsigned long BT_POWERUP_DELAY_MS = 1000;
constexpr unsigned long BT_POWEROFF_DELAY_MS = 1000;
constexpr unsigned long ANNOUNCE_PERIOD_MS = 1000;
constexpr unsigned long BUTTON_PRESS_DEBOUNCE_MS = 350;
constexpr unsigned long MIN_PLAY_INTERVAL_MS = 1000;
constexpr unsigned long CAN_ACTIVITY_TIMEOUT_MS = 30000;
unsigned long lastCanActivity = 0;
unsigned long lastPlayPress = 0;

bool initialModeChecked = false;
volatile bool canInterrupt = false;

enum RadioMode : uint8_t {
    OTHER = 0,
    AUX = 1
};

enum MuteState : uint8_t {
  MUTE_OFF = 0,
  MUTE_ON = 1
};


constexpr uint8_t CAN_MODULE_CS_PIN = 10;
constexpr uint8_t CAN_MODULE_INT_PIN = 2;

// = Debug switchers =
constexpr bool debugMode = true;
constexpr bool benchMode = true;

// === Other ===

void debugPrint(const char* msg) {
  if (debugMode) {
    Serial.println(msg);
  }
}

void debugPrint(const __FlashStringHelper* msg) {
  if (debugMode) {
    Serial.println(msg);
  }
}

MCP2515 mcp2515(CAN_MODULE_CS_PIN);

unsigned long lastAnnounce = 0;
unsigned long lastButtonPress = 0;

constexpr size_t msgVesAuxModeLen = 8;
const unsigned char msgVesAuxMode[msgVesAuxModeLen] = { 3, 0, 0, 0, 0, 0, 0, 0 };

//0x63 OLD VALUE
// new values for mygig:
//0x00	Off
//0x01	Key in
//0x41	Accessory/ACC
//0x81	Run

constexpr size_t msgPowerOnLen = 6;
const unsigned char msgPowerOn[msgPowerOnLen] = { 0x41, 0, 0, 0, 0, 0 };

RadioMode radioMode = OTHER;
MuteState muteState = MUTE_OFF;

const char compileDate[] = __DATE__ " " __TIME__;

bool btIsOn = false;
bool carIsOn = false;
unsigned long btPowerChangeTime = 0;

bool QCCPlaying = false;

struct ButtonPress {
  uint8_t pin;
  unsigned long startTime;
  bool active;
};
ButtonPress currentPress = {0,0,false};

void canISR() {
  canInterrupt = true;
}

void pressButton(int pin) {
  if (!btIsOn) {
    return;
  }
  unsigned long now = millis();
  if (now - lastButtonPress < BUTTON_PRESS_DEBOUNCE_MS) {
    return;
  }

  lastButtonPress = now;
  currentPress.pin = pin;
  currentPress.startTime = millis();
  currentPress.active = true;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void handleButtonRelease() {
  if (currentPress.active && millis() - currentPress.startTime >= 200) {
    pinMode(currentPress.pin, INPUT);
    currentPress.active = false;
  }
}

void pressPlayButton() {
  unsigned long now = millis();
  if (now - lastPlayPress < MIN_PLAY_INTERVAL_MS) {
    return;
  }
  lastPlayPress = now;
  QCCPlaying = !QCCPlaying;
  pressButton(PLAY_BTN);
}

void btSmoothOn() {
  if (btIsOn) {
    return;
  }
  btPowerChangeTime = millis();
  while (millis() - btPowerChangeTime < BT_POWERUP_DELAY_MS) {
    handleButtonRelease();
  }

  digitalWrite(BT_POWER, HIGH);
  btIsOn = true;
  lastPlayPress = millis() - MIN_PLAY_INTERVAL_MS + 500;
}

void btSmoothOff() {
  if (!btIsOn) {
    return;
  }
  if (currentPress.active) {
    pinMode(currentPress.pin, INPUT);
    currentPress.active = false;
  }
  digitalWrite(BT_POWER, LOW);
  btPowerChangeTime = millis();
  while (millis() - btPowerChangeTime < BT_POWEROFF_DELAY_MS) {
    handleButtonRelease();
  }
  btIsOn = false;
  QCCPlaying = false;
}

void turnOn() {
  if (carIsOn) {
    return;
  }

  mcp2515.setNormalMode();
  btSmoothOn();
  carIsOn = true;
  lastCanActivity = millis();

  debugPrint(F("Power ON"));
}

void turnOff() {
  if (!carIsOn) {
    return;
  }
  btSmoothOff();
  carIsOn = false;

  if (!benchMode) {
    mcp2515.setListenOnlyMode();
  }

  debugPrint(F("Power OFF"));
}

void setupBTOutputs() {
  constexpr uint8_t buttonPins[] = {PLAY_BTN, SEEK_BCK, SEEK_FWD, VOL_UP, VOL_DWN};
  for (uint8_t p : buttonPins) {
    pinMode(p, INPUT);
  }

  pinMode(BT_POWER, OUTPUT);
  digitalWrite(BT_POWER, LOW);
}

// === Work code ===

void setup() {
  ADCSRA &= ~_BV(ADEN);
  ACSR |= _BV(ACD);

  if (debugMode) {
    Serial.begin(115200);
  }
  debugPrint(F("RAM RAQ VES Control by latonita & tengz v1.0"));
  debugPrint(compileDate);

  setupBTOutputs();

  mcp2515.reset();

  while (mcp2515.setBitrate(CAN_83K3BPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    debugPrint(F("CAN init fail, retrying..."));
    delay(1000);
    mcp2515.reset();
  }

  if (benchMode) {
    mcp2515.setNormalMode();
  } else {
    mcp2515.setListenOnlyMode();
  }

  pinMode(CAN_MODULE_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAN_MODULE_INT_PIN), canISR, FALLING);

  debugPrint(F("CAN init OK"));
  delay(1000);
}

void sendAnnouncements() {
  struct can_frame frame;

  if (benchMode) {
    frame.can_id = CAN_POWER;
    frame.can_dlc = msgPowerOnLen;
    memcpy(frame.data, msgPowerOn, msgPowerOnLen);
    mcp2515.sendMessage(&frame);
  }

  frame.can_id = CAN_VES_UNIT;
  frame.can_dlc = msgVesAuxModeLen;
  memcpy(frame.data, msgVesAuxMode, msgVesAuxModeLen);
  mcp2515.sendMessage(&frame);

  debugPrint(F("CAN announcement sent"));
}

RadioMode newMode = OTHER;

void checkIncomingMessages(MCP2515::RXBn rxBuffer = MCP2515::RXB0) {
  struct can_frame frame;

  if (benchMode && !carIsOn) {
    debugPrint(F("Fake car ON signal"));
    frame.can_id = CAN_POWER;
    frame.can_dlc = 1;
    frame.data[0] = 0x81; // ACC/Run
  } else {
    if (mcp2515.readMessage(rxBuffer, &frame) != MCP2515::ERROR_OK) {
      return;
    }
  }

  if (debugMode) {
    Serial.print("CAN ID: 0x");
    Serial.print(frame.can_id, HEX);
    for (int i = 0; i < frame.can_dlc; i++) {
      Serial.print(",0x");
      Serial.print(frame.data[i], HEX);
    }
    Serial.println();
  }

  // may worth checking for all messages without if statement?
  if (frame.can_id == CAN_POWER || frame.can_id == CAN_RADIO_MODE) {
    lastCanActivity = millis();
  }

  if (frame.can_id == CAN_POWER && frame.can_dlc > 0) {
    switch (frame.data[0]) {
      case 0x41:  // ACC
      case 0x81:  // RUN
        turnOn();
        debugPrint(F("Turned ON by power signal"));
        break;
      case 0x00:  // OFF
      case 0x01:  // KEY IN
      default:    // unknown / any other
        turnOff();
      debugPrint(F("Turned OFF by power signal"));
        break;
    }
    return;
  }

  if (!carIsOn || frame.can_id != CAN_RADIO_MODE) {
    return;
  }

  // 0xEF for bench, 0xE1 in car
  MuteState newMute = (frame.data[7] == 0xE1) ? MUTE_ON : MUTE_OFF;

  newMode = ((frame.data[0] & 0xF) == 6) ? AUX : OTHER;

  if (!initialModeChecked) {
    initialModeChecked = true;
    radioMode = newMode;
    muteState = newMute;

    if (radioMode == AUX && muteState == MUTE_OFF) {
      pressPlayButton();
    }
    return;
  }

  if (radioMode != newMode) {
    radioMode = newMode;
    muteState = newMute;
    if (radioMode == AUX) {
      if (muteState == MUTE_OFF && !QCCPlaying) {
        pressPlayButton();
      }
      debugPrint(F("Radio mode changed to AUX"));
    } else {
      if (QCCPlaying) {
        pressPlayButton();
      }
      debugPrint(F("Radio mode changed to something else"));
    }
    return;
  }

  if (radioMode == AUX && newMute != muteState) {
    muteState = newMute;
    if (muteState == MUTE_ON) {
      if (QCCPlaying) {
        pressPlayButton();
      }
      debugPrint(F("Muted"));
    } else {
      if (!QCCPlaying) {
        pressPlayButton();
      }
      debugPrint(F("Unmuted"));
    }
    return;
  }
}

void loop() {
  unsigned long now = millis();

  if (carIsOn && (now - lastCanActivity > CAN_ACTIVITY_TIMEOUT_MS)) {
    if (!benchMode) {
      debugPrint(F("No CAN activity - forcing power OFF"));
      turnOff();
    } else {
      debugPrint(F("No CAN activity but in bench mode - skipping forced power OFF"));
      lastCanActivity = millis();
    }
  }

  if (carIsOn && now - lastAnnounce >= ANNOUNCE_PERIOD_MS) {
    lastAnnounce = now;
    sendAnnouncements();
  }

  if (canInterrupt || (benchMode && !carIsOn)) {
    canInterrupt = false;

    if (benchMode && !carIsOn) {
      checkIncomingMessages();
    } else {
      uint8_t irq = mcp2515.getInterrupts();

      if (irq & MCP2515::CANINTF_RX0IF) {
        checkIncomingMessages(MCP2515::RXB0);
      }

      if (irq & MCP2515::CANINTF_RX1IF) {
        checkIncomingMessages(MCP2515::RXB1);
      }
    }
  }

  handleButtonRelease();

  if (!carIsOn && !benchMode) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  } else {
    set_sleep_mode(SLEEP_MODE_IDLE);
  }

  sleep_enable();
  sleep_cpu();
  sleep_disable();
}