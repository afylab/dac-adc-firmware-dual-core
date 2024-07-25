#include <Arduino.h>

#include "RPC.h"

void setup() {
  Serial.begin(115200);
  RPC.begin();
}

void loop() {
  String buffer = "";
  while (RPC.available()) {
    buffer += (char)RPC.read();  // Fill the buffer with characters
  }
  if (buffer.length() > 0) {
    Serial.print(buffer);
  }
  while (Serial.available()) {
    RPC.write(Serial.read());
  }
}
