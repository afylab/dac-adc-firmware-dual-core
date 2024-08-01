#include <Arduino.h>

#include "RPC.h"
// #include "GIGA_digitalWriteFast.h"

void setup() {
  Serial.begin(115200);
  RPC.begin();
  // pinMode(24, OUTPUT);
}

int i = 0;
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
  // if (i % 10 == 0) {
  //   digitalWrite(24, HIGH);
  //   digitalWrite(24, LOW);
  // }
  // i++;
}
