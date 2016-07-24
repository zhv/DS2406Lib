#include <OneWire.h>
#include "DS2406.h"

OneWire net(9);
DS2406 s(&net);
const uint8_t address[] = { 0x12, 0x49, 0x0A, 0x6D, 0, 0, 0, 0xD2 };

void setup() {
  Serial.begin(9600);
}

/* readPortState example */
void loop() {
  bool pioA = s.readPortState(address, DS2406_PIO_A);
  bool pioB = s.readPortState(address, DS2406_PIO_B);

  Serial.print(F("PIO A state="));
  Serial.print(pioA);
  Serial.print(F(", PIO B state="));
  Serial.print(pioB);
  Serial.print(F("\n"));

  delay(2000);
}
