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
constexpr uint16_t CAN_RADIO_CONTROLS = 0x394;
constexpr uint16_t CAN_VES_UNIT = 0x3dd;

constexpr unsigned long BT_POWERUP_DELAY_MS = 1000;
constexpr unsigned long BT_POWEROFF_DELAY_MS = 2000;
constexpr unsigned long CHECK_PERIOD_MS = 200;
constexpr unsigned long ANNOUNCE_PERIOD_MS = 1000;
constexpr unsigned long BUTTON_PRESS_DEBOUNCE_MS = 350;

enum RadioMode : uint8_t {
    OTHER = 0,
    AUX = 1
};

constexpr uint8_t CAN_MODULE_CS_PIN = 10;
constexpr uint8_t CAN_MODULE_INT_PIN = 2;

// = Debug switchers =
constexpr bool debugMode = false;
constexpr bool benchMode = false;

// === Other ===

volatile bool canIntFlag = false;

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

// === Helper functions ===
void canISR() {
  canIntFlag = true;
}

void pressButton(int pin) {
  currentPress.pin = pin;
  currentPress.startTime = millis();
  currentPress.active = true;
  digitalWrite(pin, HIGH);
}

void handleButtonRelease() {
  if(currentPress.active && millis() - currentPress.startTime >= 200){
    digitalWrite(currentPress.pin,LOW);
    currentPress.active=false;
  }
}

void pressPlayButton() {
  QCCPlaying = !QCCPlaying;
  pressButton(PLAY_BTN);
}

void btSmoothOn() {
  if (!btIsOn) {
    // soft-start / "smooth" power up
    btPowerChangeTime = millis();
    while (millis() - btPowerChangeTime < BT_POWERUP_DELAY_MS) {
      // idle while allowing CAN interrupts
      handleButtonRelease();
    }

    digitalWrite(BT_POWER, HIGH);
    btIsOn = true;
  }
}

void btSmoothOff() {
  if (btIsOn) {
    digitalWrite(BT_POWER, LOW);
    btPowerChangeTime = millis();
    while (millis() - btPowerChangeTime < BT_POWEROFF_DELAY_MS) {
      handleButtonRelease();
    }
    btIsOn = false;
  }
}

void setupFilters() {
  CAN.init_Mask(0, 0, 0x3ff);
  CAN.init_Mask(1, 0, 0x3ff);

  CAN.init_Filt(0, 0, CAN_POWER);
  CAN.init_Filt(1, 0, CAN_RADIO_MODE);
  CAN.init_Filt(2, 0, CAN_RADIO_CONTROLS);
  CAN.init_Filt(4, 0, 0x08);
  CAN.init_Filt(5, 0, 0x09);
}

// === Work code ===

void setup() {
  // Power reduction
  ADCSRA &= ~_BV(ADEN);
  ACSR |= _BV(ACD);

  if (debugMode) {
    Serial.begin(115200);
    Serial.print("RAM RAQ VES Control by latonita & tengz v1.0");
    Serial.println(compileDate);
  } else {
    //setupFilters();
  }

  while (CAN_OK != CAN.begin(CAN_83K3BPS, MCP_8MHz)) {
    if (debugMode) {
      Serial.println("CAN init fail");
    }
    delay(250);
  }
  if (debugMode) {
    Serial.println("CAN init ok");
  }

  // interrupts for can int
  attachInterrupt(digitalPinToInterrupt(CAN_MODULE_INT_PIN), canISR, FALLING);

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

  if (CAN_MSGAVAIL != CAN.checkReceive()) {
    return;
  }

  CAN.readMsgBuf(&len, buf);
  canId = CAN.getCanId();

  if (canId == CAN_POWER && len > 0) {
      if (buf[0] == 0x00 && !benchMode) {
            if (debugMode) {
              Serial.println("Power OFF");
            }
          carIsOn = false;
          btSmoothOff();
      } else {
          if (debugMode) {
            Serial.println("Power ON");
          }
          carIsOn = true;
      }
      return;
  }

  if (!carIsOn) return;

  switch (canId) {
    case CAN_RADIO_MODE:

      if (debugMode) {
        Serial.print("CAN ID: ");
        Serial.print(canId, HEX);

        for (int i = 0; i < len; i++) {
          Serial.print(",");
          Serial.print(buf[i], HEX);
        }
        Serial.println();
        Serial.print("Radio mode: ");
        Serial.print(buf[0] & 0xF, HEX);
        Serial.print(":");
        Serial.print(radioMode);
        Serial.print(">");
        Serial.println(newMode);
      }

      newMode = ((buf[0] & 0xF) == 6) ? AUX : OTHER;
      //newMode = ((buf[0] & 0x0F) == 0x06);

      if (radioMode != newMode) {
        radioMode = newMode;
        if (radioMode == AUX) {
          if (!QCCPlaying) {
            pressPlayButton();
          }
          if (debugMode) {
            Serial.print("Radio Mode changed to AUX");
          }
        } else {
          if (QCCPlaying) {
            pressPlayButton();
          }
          if (debugMode) {
            Serial.print("Radio Mode changed to something else");
          }
        }
      }
      break;
    case CAN_RADIO_CONTROLS:
      // radio mode + buttons
      unsigned long now = millis();
      if (buf[3] > 0 && now > lastButtonPress + BUTTON_PRESS_DEBOUNCE_MS) {  // something pressed
        lastButtonPress = now;
        if (buf[3] & 1) {  // seek right
          pressButton(SEEK_FWD);

          if (debugMode) {
            Serial.println("-- Seek >>");
          }
        } else if (buf[3] & 2) {  // seek left
          pressButton(SEEK_BCK);
          if (debugMode) {
            Serial.println("<< Seek --");
          }
        } else if (buf[3] & 4) {  // rw/ff right
          if (debugMode) {
            Serial.println("-- RW FF >>");
          }
        } else if (buf[3] & 8) {  // rw/ff left
          if (debugMode) {
            Serial.println("<< RW FF --");
          }
        } else if (buf[3] & 0x20) {  // RND/PTY
          pressPlayButton();
          if (debugMode) {
            Serial.println("[RND/PTY] - use as play/pause");
          }
        }
      }

    default:
      break;
  }
}
void loop() {
  unsigned long now = millis();

  if ((carIsOn || benchMode) && now > lastAnnounce + ANNOUNCE_PERIOD_MS) {
    lastAnnounce = now;
    sendAnnouncements();
  }
  if (canIntFlag) {
      canIntFlag = false;
      checkIncomingMessages();
  }
  handleButtonRelease();

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_cpu();
  sleep_disable();
}
