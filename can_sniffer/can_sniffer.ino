#include <SPI.h>
#include "mcp_can.h"

#define CAN_MODULE_CS_PIN 10
MCP_CAN CAN(CAN_MODULE_CS_PIN);

void setup() {
  Serial.begin(115200);
  while (CAN_OK != CAN.begin(CAN_83K3BPS, MCP_8MHz)) {
    Serial.println("CAN init fail");
    delay(1000);
  }
  Serial.println("CAN OK");
}

void loop() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned char len = 0;
    unsigned char buf[8];
    CAN.readMsgBuf(&len, buf);
    unsigned long id = CAN.getCanId();

    Serial.print("ID: 0x"); Serial.print(id, HEX); Serial.print("  =>  unsigned char msg[");
    Serial.print(len);
    Serial.print("] = {");

    for (int i = 0; i < len; i++) {
      Serial.print("0x");
      if (buf[i] < 0x10) Serial.print("0");
      Serial.print(buf[i], HEX);
      if (i < len - 1) Serial.print(", ");
    }
    Serial.println("};");
  }
}