#include <OneWire.h>
#include "DS2406.h"

OneWire net(9);
DS2406 s(&net);
const uint8_t address[] = { 0x12, 0x49, 0x0A, 0x6D, 0, 0, 0, 0xD2 };

void setup() {
	Serial.begin(9600);
}

/* channelAccessWrite */
void loop() {
	uint8_t out = B10101010;
	DS2406ChannelAccessTransaction tx = s.channelAccessStart(address,
			DS2406_CHANNEL_2CH_WRITE4X_WRITE4X | DS2406_CHANNEL_CRC_DISABLED);
	for (byte i = 0; i < 255; i++) {
		s.channelAccessWrite(tx, out);
		digitalWrite(PIN_LED, out & 1);

		Serial.print(F("tx.errorCode="));
		Serial.print(tx.errorCode);
		Serial.print('\n');

		delay(1000);
		out = ~out;
	}
	s.endTransaction();
}
