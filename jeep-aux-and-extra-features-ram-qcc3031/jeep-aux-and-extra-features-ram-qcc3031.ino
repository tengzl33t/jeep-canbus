/*****************************************************************************************
 *
 * Hack for my Jeep Grand Cherokee WH 2006 (european) + RAR radio head unit
 *
 * My hardware:
 *  1. Arduino Pro Mini 5v/16Mhz
 *  2. Mcp2515_can SPI module (8Mhz)
 *  3. Generic (H166) bluetooth A2DP module based on OVC3860, UART connected to Arduino
 *
 * Features:
 *  1. Bench mode to enable radio functioning while removed from the car
 *  2. Emulating VES presense to enable AUX IN in head unit
 *  2.1. Turn on/off BT A2DP module only when AUX IN is selected
 *  3. Control BT audio source with seek buttons
 *  4. [todo] Signal comfort module - blinkers blink three times instead of one.
 *  5. [todo] Headlights - always on
 *  6. [todo] Auto wipers - require installation of rain sensor (? maybe it will work alone, w/o my software :)
 *  7. [todo] Auto parameters display
 *  7.1. gasoline cost for a trip
 *
 * Copyright (C) 2015-2017 Anton Viktorov <latonita@yandex.ru>
 *                                    https://github.com/latonita/jeep-canbus
 *
 * This is free software. You may use/redistribute it under The MIT License terms.
 *
 *****************************************************************************************/
#include <SoftwareSerial.h>
#include <SPI.h>
#include "mcp_can.h"
#include <avr/sleep.h>

volatile bool canIntFlag = false;
void canISR() {
  canIntFlag = true;
}

#define CAN_MODULE_CS_PIN 10  // PB2

#define CHECK_PERIOD_MS 200
#define ANNOUNCE_PERIOD_MS 1000
#define BUTTON_PRESS_DEBOUNCE_MS 350

// Debug switchers
//#define DEBUG
//#define BENCH_MODE_ON

MCP_CAN CAN(CAN_MODULE_CS_PIN);

unsigned long lastAnnounce = 0;
unsigned long lastButtonPress = 0;

#define msgVesAuxModeLen 8
unsigned char msgVesAuxMode[8] = { 3, 0, 0, 0, 0, 0, 0, 0 };

#ifdef BENCH_MODE_ON
#define msgPowerOnLen 6
unsigned char msgPowerOn[6] = { 0x63, 0, 0, 0, 0, 0 };
#endif

#define RADIOMODE_OTHER 0
#define RADIOMODE_AUX 1
unsigned char radioMode = RADIOMODE_OTHER;

const char compileDate[] = __DATE__ " " __TIME__;

#define PLAY_BTN 9
#define SEEK_BCK 8
#define SEEK_FWD 7
#define VOL_UP 6
#define VOL_DWN 5

bool QCCPlaying = false;

struct ButtonPress {
  int pin;
  unsigned long startTime;
  bool active;
};
ButtonPress currentPress = {0,0,false};

void pinDown(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void pinsSetup() {
  pinDown(PLAY_BTN);
  pinDown(SEEK_BCK);
  pinDown(SEEK_FWD);
  pinDown(VOL_UP);
  pinDown(VOL_DWN);

  // disable unused pins
  for (int p = 0; p <= 13; p++) {
    if (p!=PLAY_BTN && p!=SEEK_BCK && p!=SEEK_FWD && p!=VOL_UP && p!=VOL_DWN)
      pinMode(p, OUTPUT), digitalWrite(p, LOW);
  }
}

void pressButton(int pin) {
  currentPress.pin = pin;
  currentPress.startTime = millis();
  currentPress.active = true;
  digitalWrite(pin,HIGH);
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

#define CAN_POWER 0x000
#define CAN_BLINKERS 0x006
#define CAN_RADIO_MODE 0x09f
#define CAN_RADIO_CONTROLS 0x394
#define CAN_VES_UNIT 0x3dd

//#define CAN_MULTI_SWITCH   0x11d
//#define CAN_HEADLIGHTS 0x1c8

void setupFilters() {
  CAN.init_Mask(0, 0, 0x3ff);
  CAN.init_Mask(1, 0, 0x3ff);

  CAN.init_Filt(0, 0, CAN_POWER);
  CAN.init_Filt(1, 0, CAN_RADIO_MODE);
  CAN.init_Filt(2, 0, CAN_RADIO_CONTROLS);
  CAN.init_Filt(3, 0, CAN_BLINKERS);
  CAN.init_Filt(4, 0, 0x08);
  CAN.init_Filt(5, 0, 0x09);
}

void setup() {
  // Power reduction
  ADCSRA &= ~_BV(ADEN);
  ACSR |= _BV(ACD);
  PRR |= _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2);

  setupFilters();

#ifdef DEBUG
  Serial.begin(115200);
  Serial.print("Jeep VES Enabler + Extras by latonita v.1.1, ");
  Serial.println(compileDate);
#endif

  while (CAN_OK != CAN.begin(CAN_83K3BPS, MCP_8MHz)) {
#ifdef DEBUG
    Serial.println("CAN init fail");
#endif
    delay(250);
  }
#ifdef DEBUG
  Serial.println("CAN init ok");
#endif

  // interrupts for can int
  attachInterrupt(digitalPinToInterrupt(2), canISR, FALLING);

}

void sendAnnouncements() {
#ifdef BENCH_MODE_ON
  // when on bench - send power on command to radio to enable it
  CAN.sendMsgBuf(CAN_POWER, 0, msgPowerOnLen, msgPowerOn);
#endif
  // tell them VES AUX is here
  CAN.sendMsgBuf(CAN_VES_UNIT, 0, msgVesAuxModeLen, msgVesAuxMode);
}

unsigned int canId = 0;
unsigned char len = 0;
unsigned char buf[8];
unsigned char newMode = 0;

void checkIncomingMessages() {

  if (CAN_MSGAVAIL != CAN.checkReceive())
    return;

  CAN.readMsgBuf(&len, buf);
  canId = CAN.getCanId();

  switch (canId) {
    case CAN_RADIO_MODE:

#ifdef DEBUG
      // some debug output
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
#endif

      //newMode = ((buf[0] & 0xF) == 6) ? RADIOMODE_AUX : RADIOMODE_OTHER;
      newMode = ((buf[0] & 0x0F) == 0x06);

      if (radioMode != newMode) {
        radioMode = newMode;
        if (radioMode == RADIOMODE_AUX && !QCCPlaying) {
          pressPlayButton();
#ifdef DEBUG
          Serial.print("Radio Mode changed to AUX");
#endif
        } else if(radioMode!=RADIOMODE_AUX && QCCPlaying) {
          pressPlayButton();
#ifdef DEBUG
          Serial.print("Radio Mode changed to something else");
#endif
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

#ifdef DEBUG
          Serial.println("-- Seek >>");
#endif
        } else if (buf[3] & 2) {  // seek left
          pressButton(SEEK_BCK);

#ifdef DEBUG
          Serial.println("<< Seek --");
#endif
        } else if (buf[3] & 4) {  // rw/ff right
#ifdef DEBUG
          Serial.println("-- RW FF >>");
#endif
        } else if (buf[3] & 8) {  // rw/ff left
#ifdef DEBUG
          Serial.println("<< RW FF --");
#endif
        } else if (buf[3] & 0x20) {  // RND/PTY
          pressPlayButton();

#ifdef DEBUG
          Serial.println("[RND/PTY] - use as play/pause");
#endif
        }
      }

    default:
      break;
  }
}
void loop() {
#ifdef DEBUG
  Serial.println("loop started");
#endif

  if (millis() > lastAnnounce + ANNOUNCE_PERIOD_MS) {
    lastAnnounce = millis();
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
