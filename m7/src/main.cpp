#include <Arduino.h>

#include "RPC.h"

void setup() {
  RPC.begin();
}

int i = 0;
void loop() {
  String buffer = "";
  while (RPC.available()) {
    buffer += (char)RPC.read();
  }
  if (buffer.length() > 0) {
    Serial.print(buffer);
  }
  while (Serial.available()) {
    RPC.write(Serial.read());
  }
}
