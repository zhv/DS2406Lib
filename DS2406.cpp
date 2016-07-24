#include "DS2406.h"
#include <stdio.h>

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

//----------------------------------------------------------
DS2406::DS2406(OneWire *bus) {
    this->bus = bus;
    this->crc = 0;
    this->programPulsePin = 0;
}

DS2406::DS2406(OneWire *bus, const uint8_t programPulsePin) {
    this->bus = bus;
    this->crc = 0;
    this->programPulsePin = programPulsePin;
    pinMode(programPulsePin, OUTPUT);
    digitalWrite(programPulsePin, LOW);
}

//----------------------------------------------------------
// Transport protocol API

void DS2406::beginTransaction(const uint8_t *deviceAddress) {
    crc = 0;
    bus->reset();
    bus->select(deviceAddress);
}

void DS2406::endTransaction() {
    bus->reset();
}

void DS2406::send(const uint8_t b0) {
    crc = bus->crc16(&b0, 1, crc);
    bus->write(b0, 1);
}

void DS2406::send(const uint8_t b0, const uint8_t b1, const uint8_t b2) {
    send(b0);
    send(b1);
    send(b2);
}

void DS2406::applyProgramPulse(const bool use12VPulse) {
    if (use12VPulse) {
        noInterrupts();
        digitalWrite(programPulsePin, HIGH);
        delayMicroseconds(480);
        digitalWrite(programPulsePin, LOW);
        interrupts();
    } else {
        send(0xFF);
    }
}

void DS2406::receive(uint8_t *buffer, const uint16_t count, const bool calculateCRC /*= false*/) {
    bus->read_bytes(buffer, count);
    if (calculateCRC) {
        crc = bus->crc16(buffer, count, crc);
    }
}

bool DS2406::receiveAndValidateCRC() {
    uint8_t buffer[2];
    receive(buffer, 2, false);
    return ((crc & 0xFF) == ~buffer[0]) && ((crc >> 8) == ~buffer[1]); // See details in the OneWire::check_crc16()
}

//----------------------------------------------------------
// Low-level API

uint8_t DS2406::writeDataMemory(const uint8_t *deviceAddress, const uint8_t startAddress, const uint8_t *data,
        const uint8_t dataLength) {
    uint8_t errorCode = 0;
    uint8_t buffer[1];
    uint8_t address = startAddress;

    beginTransaction(deviceAddress);
    send(DS2406_CMD_WRITE_MEMORY, startAddress, 0x00);

    for (uint8_t i = 0; i < dataLength; i++, address++) {
        send(data[i]);
        if (!receiveAndValidateCRC()) {
            errorCode = DS2406_ERROR_CRC_ERROR;
            break;
        }
        applyProgramPulse(true);
        receive(buffer, 1);
        if ((buffer[0] | data[i]) != data[i]) {
            errorCode = DS2406_ERROR_MEM_PROGRAM;
            break;
        }
        if (address == 0x7F) {
            break; // last byte (1kbit memory limit)
        }
    }

    endTransaction();
    return errorCode;
}

uint8_t DS2406::readDataMemory(const uint8_t *deviceAddress, const uint8_t startAddress, uint8_t *data, const uint8_t dataLength) {
    return readMemory(DS2406_CMD_READ_MEMORY, deviceAddress, startAddress, data, dataLength, DS2406_DATA_MEM_SIZE);
}

uint8_t DS2406::writeStatusMemory(const uint8_t *deviceAddress, const uint8_t startAddress, const uint8_t *data,
        const uint8_t dataLength) {
    uint8_t errorCode = 0;
    uint8_t buffer[1];
    uint8_t address = startAddress;

    beginTransaction(deviceAddress);
    send(DS2406_CMD_WRITE_STATUS, startAddress, 0x00);

    for (uint8_t i = 0; i < dataLength; i++, address++) {
        send(data[i]);
        if (!receiveAndValidateCRC()) {
            errorCode = DS2406_ERROR_CRC_ERROR;
            break;
        }
        bool isEPROMRange = (address < DS2406_ADDR_STAT_MEM_SRAM);
        applyProgramPulse(isEPROMRange);
        receive(buffer, 1);
        if (isEPROMRange && (buffer[0] | data[i]) != data[i]) {
            errorCode = DS2406_ERROR_MEM_PROGRAM;
            break;
        }
    }

    endTransaction();
    return errorCode;
}

uint8_t DS2406::readStatusMemory(const uint8_t *deviceAddress, const uint8_t startAddress, uint8_t *data, const uint8_t dataLength) {
    return readMemory(DS2406_CMD_READ_STATUS, deviceAddress, startAddress, data, dataLength, DS2406_STATUS_MEM_SIZE);
}

DS2406ChannelAccessTransaction DS2406::channelAccessStart(const uint8_t *deviceAddress, const uint8_t channelControlByte) {
    DS2406ChannelAccessTransaction tx;
    tx.crcMode = channelControlByte & DS2406_CCB_CRC_32BYTE;
    tx.channelAccessCounter = getChannelAccessCounter(tx.crcMode);
    beginTransaction(deviceAddress);
    send(DS2406_CMD_CHANNEL_ACCESS, channelControlByte, DS2406_CCB_BYTE2);
    uint8_t buffer[1];
    receive(buffer, 1, true);
    tx.channelInfoByte = buffer[0];
    return tx;
}

uint8_t DS2406::channelAccessRead(DS2406ChannelAccessTransaction &tx) {
    uint8_t buffer[1];
    receive(buffer, 1, true);
    channelAccessCheckCRC(tx);
    return buffer[0];
}

void DS2406::channelAccessWrite(DS2406ChannelAccessTransaction &tx, const uint8_t data) {
    send(data);
    channelAccessCheckCRC(tx);
}

uint8_t DS2406::readSRAM(const uint8_t *deviceAddress) {
    uint8_t sram;
    readStatusMemory(deviceAddress, DS2406_ADDR_STAT_MEM_SRAM, &sram, 1);
    return sram;
}

uint8_t DS2406::writeSRAM(const uint8_t *deviceAddress, const uint8_t sram) {
    return writeStatusMemory(deviceAddress, DS2406_ADDR_STAT_MEM_SRAM, &sram, 1);
}

//----------------------------------------------------------
// High-level API

bool DS2406::getSwitchState(const uint8_t *deviceAddress, const uint8_t pio) {
    uint8_t sram = readSRAM(deviceAddress);
    return !(sram & (pio << 5));
}

DS2406ChannelAccessTransaction DS2406::setSwitchState(const uint8_t *deviceAddress, const uint8_t pio, bool state) {
    uint8_t channelControlByte = DS2406_CCB_INTERLEAVE_OFF | DS2406_CCB_WRITE8X | DS2406_CCB_CRC_1BYTE | (pio << 2);
    DS2406ChannelAccessTransaction tx = channelAccessStart(deviceAddress, channelControlByte);
    channelAccessWrite(tx, (state) ? 0xFF : 0x00);
    endTransaction();
    return tx;
}

uint8_t DS2406::setSwitchState(const uint8_t *deviceAddress, const bool pioA, const bool pioB /*= false*/) {
    uint8_t sram = 0x1F;
    if (!pioA) sram |= DS2406_SRAM_FLIP_FLOP_A;
    if (!pioB) sram |= DS2406_SRAM_FLIP_FLOP_B;
    return writeSRAM(deviceAddress, sram);
}

bool DS2406::readPortState(const uint8_t *deviceAddress, const uint8_t pio /*= DS2406_PIO_A*/) {
	uint8_t channelControlByte = (pio << 2) | DS2406_CCB_READ8X; // one channel, read 8x
	DS2406ChannelAccessTransaction tx = channelAccessStart(deviceAddress, channelControlByte);
	uint8_t data = channelAccessRead(tx);
	endTransaction();
    return data > 0;
}

uint8_t DS2406::setConditionalSearchParameters(const uint8_t *deviceAddress, const uint8_t pio, const uint8_t sourceSelect, const bool polarity) {
	uint8_t sram = readSRAM(deviceAddress);
	sram &= 0xE0; // clear CSS0..CSS4 bits
	sram |= (pio << 3) | sourceSelect | uint8_t(polarity);
    return writeSRAM(deviceAddress, sram);
}

uint8_t DS2406::conditionalSearch(uint8_t *newAddr) {
	return bus->search(newAddr, false /* conditional search */);
}

//----------------------------------------------------------
// private functions
uint8_t DS2406::readMemory(const uint8_t readCommand, const uint8_t *deviceAddress, const uint8_t startAddress, uint8_t *data, const uint8_t dataLength, const uint8_t memorySize) {
    uint8_t errorCode = 0;

    beginTransaction(deviceAddress);
    send(readCommand, startAddress, 0x00);
    receive(data, dataLength, true);

    if ((startAddress + dataLength) == memorySize) { // all data received
        if (!receiveAndValidateCRC()) {
            errorCode = DS2406_ERROR_CRC_ERROR;
        }
    }

    endTransaction();
    return errorCode;
}

void DS2406::channelAccessCheckCRC(DS2406ChannelAccessTransaction &tx) {
    if (tx.crcMode > 0) { // CRC enabled
        tx.channelAccessCounter--;
        if (tx.channelAccessCounter == 0) { // CRC Due
            tx.errorCode = (!receiveAndValidateCRC()) ? DS2406_ERROR_CRC_ERROR : 0;
            tx.channelAccessCounter = getChannelAccessCounter(tx.crcMode);
            crc = 0;
        }
    }
}

uint8_t DS2406::getChannelAccessCounter(const uint8_t crcMode) {
    switch (crcMode) {
        case DS2406_CCB_CRC_1BYTE: return 1;
        default: return (1 << (crcMode << 1));
    }
}
