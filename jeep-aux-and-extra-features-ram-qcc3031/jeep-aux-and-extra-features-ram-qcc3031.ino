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
 * This is free software. You may use/redistribute it under The MIT License terms.
 *
 *****************************************************************************************/
#include <SoftwareSerial.h>
#include <SPI.h>
#include "mcp_can.h"
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
constexpr unsigned long BT_POWEROFF_DELAY_MS = 2000;
constexpr unsigned long CHECK_PERIOD_MS = 200;
constexpr unsigned long ANNOUNCE_PERIOD_MS = 1000;
constexpr unsigned long BUTTON_PRESS_DEBOUNCE_MS = 350;
constexpr unsigned long MIN_PLAY_INTERVAL_MS = 2000;
unsigned long lastPlayPress = 0;

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

// volatile bool canIntFlag = false;

uint8_t lastMuteValue = 0x00;

MCP_CAN CAN(CAN_MODULE_CS_PIN);

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
  btSmoothOn();
  carIsOn = true;
  if (debugMode) {
    Serial.println("Power ON");
  }
}

void turnOff() {
  if (!carIsOn) {
    return;
  }
  btSmoothOff();
  carIsOn = false;
  if (debugMode) {
    Serial.println("Power OFF");
  }
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
    Serial.println("RAM RAQ VES Control by latonita & tengz v1.0");
    Serial.println(compileDate);
  }
  setupBTOutputs();

  while (CAN_OK != CAN.begin(CAN_83K3BPS, MCP_8MHz)) {
    if (debugMode) {
      Serial.println("CAN init fail");
    }
    delay(1000);
  }
  if (debugMode) {
    Serial.println("CAN init ok");
  }
}

void sendAnnouncements() {
  if (benchMode) {
    CAN.sendMsgBuf(CAN_POWER, 0, msgPowerOnLen, msgPowerOn);
  }
  CAN.sendMsgBuf(CAN_VES_UNIT, 0, msgVesAuxModeLen, msgVesAuxMode);
}

unsigned int canId = 0;
unsigned char len = 0;
unsigned char buf[8];
RadioMode newMode = OTHER;

void checkIncomingMessages() {
  if (benchMode && !carIsOn) {
    Serial.println("Faking car ON");
    unsigned char fakeBuf[1] = {0x81}; // ACC/Run
    unsigned int fakeCanId = CAN_POWER;
    len = 1;
    memcpy(buf, fakeBuf, 1);
    canId = fakeCanId;
  } else {
    if (CAN_MSGAVAIL != CAN.checkReceive()) {
      return;
    }
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
  }

  if (canId == CAN_POWER && len > 0) {
    switch (buf[0]) {
      case 0x41:  // ACC
      case 0x81:  // RUN
        turnOn();
        break;
      case 0x00:  // OFF
      case 0x01:  // KEY IN
      default:    // unknown / any other
        turnOff();
        break;
    }
    return;
  }

  if (!carIsOn || canId != CAN_RADIO_MODE) {
    return;
  }

  if (debugMode) {
    Serial.println("CAN ID: ");
    Serial.print(canId, HEX);
    for (int i = 0; i < len; i++) {
      Serial.print(",");
      Serial.print(buf[i], HEX);
    }
    Serial.println();
  }

  MuteState newMute = (buf[7] == 0xEF) ? MUTE_ON : MUTE_OFF;
  newMode = ((buf[0] & 0xF) == 6) ? AUX : OTHER;

  if (radioMode != newMode) {
    radioMode = newMode;
    muteState = newMute;
    if (radioMode == AUX) {
      if (muteState == MUTE_OFF && !QCCPlaying) {
        pressPlayButton();
      }
      if (debugMode) {
        Serial.println("Radio Mode changed to AUX");
      }
    } else {
      if (QCCPlaying) {
        pressPlayButton();
      }
      if (debugMode) {
        Serial.println("Radio Mode changed to something else");
      }
    }
    return;
  }

  if (radioMode == AUX && newMute != muteState) {
    muteState = newMute;
    if (muteState == MUTE_ON) {
      if (debugMode) {
        Serial.println("Mute enabled");
      }
      if (QCCPlaying) {
        pressPlayButton();
      }
    } else {
      if (debugMode) {
        Serial.println("Mute disabled");
      }
      if (!QCCPlaying) {
        pressPlayButton();
      }
    }
    return;
  }
}
void loop() {
  unsigned long now = millis();

  if (carIsOn && now > lastAnnounce + ANNOUNCE_PERIOD_MS) {
    lastAnnounce = now;
    sendAnnouncements();
  }
  checkIncomingMessages();
  handleButtonRelease();

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_cpu();
  sleep_disable();
}
