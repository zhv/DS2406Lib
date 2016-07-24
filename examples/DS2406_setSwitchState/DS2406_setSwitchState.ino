#include <OneWire.h>
#include "DS2406.h"

OneWire net(9);
DS2406 s(&net);
const uint8_t address[] = { 0x12, 0x49, 0x0A, 0x6D, 0, 0, 0, 0xD2 };

/* setSwitchState */
void loop() {
	static bool channelState = HIGH;
	s.setSwitchState(address, channelState, !channelState);
	digitalWrite(PIN_LED, channelState);
	delay(2000);
	channelState = !channelState;
}
