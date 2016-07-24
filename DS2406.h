#ifndef DS2406_h
#define DS2406_h

#define ONEWIRE_SEARCH 1

#include <OneWire.h>
#include <inttypes.h>

//-------------------------------------------
#define DS2406_FAMILY                    0x12

// PIO channel selection
#define DS2406_PIO_A                     1
#define DS2406_PIO_B                     2

// useful only for setConditionalSearchParameters
#define DS2406_PIO_LOGICAL_OR            3

// Source select for setConditionalSearchParameters
#define DS2406_SS_ACTIVITY_LATCH         B00000010
#define DS2406_SS_CHANNEL_FLIP_FLOP      B00000100
#define DS2406_SS_PIO_STATUS             B00000110

//-------------------------------------------
#define DS2406_CMD_READ_MEMORY           0xF0
#define DS2406_CMD_WRITE_MEMORY          0x0F
#define DS2406_CMD_READ_STATUS           0xAA
#define DS2406_CMD_WRITE_STATUS          0x55
#define DS2406_CMD_CHANNEL_ACCESS        0xF5
#define DS2406_CMD_CONDITINAL_SEARCH     0xEC

//-------------------------------------------
#define DS2406_ADDR_STAT_MEM_SRAM        0x07

// Memory size in bytes
#define DS2406_DATA_MEM_SIZE             128
#define DS2406_STATUS_MEM_SIZE           8

//-------------------------------------------
// SRAM mask
#define DS2406_SRAM_CSS0_POLARITY        B00000001
#define DS2406_SRAM_CSS1_SOURCE_SELECT   B00000010
#define DS2406_SRAM_CSS2_SOURCE_SELECT   B00000100
#define DS2406_SRAM_CSS3_CH_SELECT       B00001000
#define DS2406_SRAM_CSS4_CH_SELECT       B00010000

#define DS2406_SRAM_FLIP_FLOP_A          B00100000
#define DS2406_SRAM_FLIP_FLOP_B          B01000000

#define DS2406_SRAM_SUPPLY_INDICATION    B10000000

//-------------------------------------------
// Channel Control Byte (CCB, byte 1) mask
#define DS2406_CCB_CRC_DISABLED          B00000000
#define DS2406_CCB_CRC_1BYTE             B00000001
#define DS2406_CCB_CRC_8BYTE             B00000010
#define DS2406_CCB_CRC_32BYTE            B00000011

// bit 2,3 = zero not allowed for CHANNEL_SELECT
#define DS2406_CCB_CHANNEL_SELECT_A      B00000100
#define DS2406_CCB_CHANNEL_SELECT_B      B00001000
#define DS2406_CCB_CHANNEL_SELECT_BOTH   B00001100

#define DS2406_CCB_INTERLEAVE_OFF        B00000000
#define DS2406_CCB_INTERLEAVE_ON         B00010000

#define DS2406_CCB_TOOGLE_MODE_OFF       B00000000
#define DS2406_CCB_TOOGLE_MODE_ON        B00100000

#define DS2406_CCB_INITIAL_MODE_OFF      B00000000
#define DS2406_CCB_INITIAL_MODE_ON       B01000000

#define DS2406_CCB_ACT_LATCH_OFF         B00000000
#define DS2406_CCB_ACT_LATCH_ON          B10000000

// combinations
#define DS2406_CCB_WRITE8X               DS2406_CCB_TOOGLE_MODE_OFF | DS2406_CCB_INITIAL_MODE_OFF
#define DS2406_CCB_READ8X                DS2406_CCB_TOOGLE_MODE_OFF | DS2406_CCB_INITIAL_MODE_ON
#define DS2406_CCB_WRITE8X_READ8X        DS2406_CCB_TOOGLE_MODE_ON  | DS2406_CCB_INITIAL_MODE_OFF
#define DS2406_CCB_READ8X_WRITE8X        DS2406_CCB_TOOGLE_MODE_ON  | DS2406_CCB_INITIAL_MODE_ON

#define DS2406_CCB_CH_A                  DS2406_CCB_CHANNEL_SELECT_A | DS2406_CCB_INTERLEAVE_OFF
#define DS2406_CCB_CH_A_WRITE8X          DS2406_CCB_CH_A | DS2406_CCB_WRITE8X
#define DS2406_CCB_CH_A_READ8X           DS2406_CCB_CH_A | DS2406_CCB_READ8X
#define DS2406_CCB_CH_A_WRITE8X_READ8X   DS2406_CCB_CH_A | DS2406_CCB_WRITE8X_READ8X
#define DS2406_CCB_CH_A_READ8X_WRITE8X   DS2406_CCB_CH_A | DS2406_CCB_READ8X_WRITE8X

#define DS2406_CCB_CH_B                  DS2406_CCB_CHANNEL_SELECT_B | DS2406_CCB_INTERLEAVE_OFF
#define DS2406_CCB_CH_B_WRITE8X          DS2406_CCB_CH_B | DS2406_CCB_WRITE8X
#define DS2406_CCB_CH_B_READ8X           DS2406_CCB_CH_B | DS2406_CCB_READ8X
#define DS2406_CCB_CH_B_WRITE8X_READ8X   DS2406_CCB_CH_B | DS2406_CCB_WRITE8X_READ8X
#define DS2406_CCB_CH_B_READ8X_WRITE8X   DS2406_CCB_CH_B | DS2406_CCB_READ8X_WRITE8X

#define DS2406_CCB_2CH                   DS2406_CCB_CHANNEL_SELECT_BOTH
#define DS2406_CCB_2CH_WRITE4X_WRITE4X   DS2406_CCB_2CH | DS2406_CCB_WRITE8X
#define DS2406_CCB_2CH_READ4X_READ4X     DS2406_CCB_2CH | DS2406_CCB_READ8X
#define DS2406_CCB_2CH_WRITE4X_READ4X    DS2406_CCB_2CH | DS2406_CCB_WRITE8X_READ8X
#define DS2406_CCB_2CH_READ4X_WRITE4X    DS2406_CCB_2CH | DS2406_CCB_READ8X_WRITE8X

//-------------------------------------------
// CCB byte 2, always 0xFF for DS2406
#define DS2406_CCB_BYTE2                 0xFF

//-------------------------------------------
// Channel Info Byte mask
#define DS2406_CIB_FLIP_FLOP_A           B00000001
#define DS2406_CIB_FLIP_FLOP_B           B00000010

#define DS2406_CIB_SENSED_LEVEL_A        B00000100
#define DS2406_CIB_SENSED_LEVEL_B        B00001000

#define DS2406_CIB_ACTIVITY_LATCH_A      B00010000
#define DS2406_CIB_ACTIVITY_LATCH_B      B00100000

#define DS2406_CIB_NUMBER_OF_CHANNELS    B01000000
#define DS2406_CIB_SUPPLY_INDICATION     B10000000

//-------------------------------------------
#define DS2406_ERROR_CRC_ERROR           1
#define DS2406_ERROR_MEM_PROGRAM         2

//-------------------------------------------
/**
 * Transaction information for DS2406 Channel Access command communication session.
 *
 * Don't change values during communication session, it may lead to unexpected results.
 *
 * errorCode            - 0 (no error), DS2406_ERROR_CRC_ERROR or DS2406_ERROR_MEM_PROGRAMMING
 */
struct DS2406ChannelAccessTransaction {
    uint8_t errorCode = 0;
    uint8_t crcMode = 0;
    uint8_t channelAccessCounter = 0;
    uint8_t channelInfoByte;
};

class DS2406 {
  public:
    DS2406(OneWire *bus);
    DS2406(OneWire *bus, const uint8_t programPulsePin);

    // Transport protocol API
    /**
     * Send presence detection request and select device
     */
    void beginTransaction(const uint8_t *deviceAddress);

    /**
     * Release communication bus
     */
    void endTransaction();

    /**
     * Send byte to already selected device
     */
    void send(const uint8_t b0);

    /**
     * Send 3 bytes (common for several DS2406 commands) to already selected device
     */
    void send(const uint8_t b0, const uint8_t b1, const uint8_t b2);

    /**
     * Apply 480 uSec program pulse
     * 
     * @param use12VPulse true to use 12V pulse. Additional circuit elements are required! (datasheet page 25)
     */
    void applyProgramPulse(const bool use12VPulse);

    /**
     * Receive data (with or without CRC calculation)
     */
    void receive(uint8_t *buffer, const uint16_t count, const bool calculateCRC = false);

    /**
     * Receive CRC16 bytes and compare with local CRC value (the value calculated by bus master)
     */
    bool receiveAndValidateCRC();

    // Low-level API
    /**
     * Issue Write Memory (0F) command Write 1-kbit EPROM memory
     * 
     * @param deviceAddress MicroLan(c) 8-byte device address
     * @paarm startAddress memory address to start writing from (in the range of 0x00..0x7F)
     * @paarm data data to write
     * @paarm dataLength length of data (bytes count)
     */
    uint8_t writeDataMemory(const uint8_t *deviceAddress, const uint8_t startAddress, const uint8_t *data, const uint8_t dataLength);

    /**
     * Issue Read Memory (F0) command. Read 1-kbit EPROM memory
     * 
     * @param deviceAddress MicroLan(c) 8-byte device address
     * @paarm startAddress memory address to start reading from (in the range of 0x00..0x7F)
     * @paarm data data array to read into
     * @paarm dataLength length of data (bytes count)
     */
    uint8_t readDataMemory(const uint8_t *deviceAddress, const uint8_t startAddress, uint8_t *data, const uint8_t dataLength);

    /**
     * Issue Write Status (55) command. Write status memory (8 bytes available)
     * 
     * @param deviceAddress MicroLan(c) 8-byte device address
     * @paarm startAddress memory address to start writing from (in the range of 0x00..0x07)
     * @paarm data data to write
     * @paarm dataLength length of data (bytes count)
     */
    uint8_t writeStatusMemory(const uint8_t *deviceAddress, const uint8_t startAddress, const uint8_t *data, const uint8_t dataLength);

    /**
     * Issue Read Status (AA) command. Read status memory (8 bytes available)
     * 
     * @param deviceAddress MicroLan(c) 8-byte device address
     * @paarm startAddress memory address to start reading from (in the range of 0x00..0x07)
     * @paarm data data array to read into
     * @paarm dataLength length of data (bytes count)
     */
    uint8_t readStatusMemory(const uint8_t *deviceAddress, const uint8_t startAddress, uint8_t *data, const uint8_t dataLength);

    /**
     * Issue Channel Access (F5) command, with subsequent read or write iterations (see channelAccessRead, channelAccessWrite)
     * 
     * @param deviceAddress MicroLan(c) 8-byte device address
     * @param channelCtrlByte bit-OR value for Channel Access Byte (see DS2406_CCB_* constants)
     */
    DS2406ChannelAccessTransaction channelAccessStart(const uint8_t *deviceAddress, const uint8_t channelCtrlByte);

    /**
     * Continue reading PIO state as part of Channel Access (F5) command. The command channelAccessStart must be executed first.
     * 
     * @param tx transaction object got from channelAccessStart
     */
    uint8_t channelAccessRead(DS2406ChannelAccessTransaction &tx);

    /**
     * Continue writing PIO state as part of Channel Access (F5) command. The command channelAccessStart must be executed first.
     * 
     * @param tx transaction object got from channelAccessStart
     * @param data data to write into PIO state
     */
    void channelAccessWrite(DS2406ChannelAccessTransaction &tx, const uint8_t data);

    uint8_t readSRAM(const uint8_t *deviceAddress);

    uint8_t writeSRAM(const uint8_t *deviceAddress, const uint8_t sram);

    // High-level API
    /**
     * Get the switch state
     * 
     * @param deviceAddress MicroLan(c) 8-byte device address
     * @param pio port DS2406_PIO_A or DS2406_PIO_B constant
     */
    bool getSwitchState(const uint8_t *deviceAddress, const uint8_t pio = DS2406_PIO_A);

    /**
     * Set the switch state (implemented as channel access command)
     * 
     * @param deviceAddress MicroLan(c) 8-byte device address
     * @param pio port DS2406_PIO_A or DS2406_PIO_B constant
     * @param state logical level to switch to
     */
    DS2406ChannelAccessTransaction setSwitchState(const uint8_t *deviceAddress, const uint8_t pio, bool state);

    /**
     * Set the switch state (implemented as write status memory command)
     * 
     * @param deviceAddress MicroLan(c) 8-byte device address
     * @param pioA logical level for PIO-A port
     * @param pioB logical level for PIO-B port
     */
    uint8_t setSwitchState(const uint8_t *deviceAddress, bool pioA, bool pioB = false);

    /**
     * Get port sensed level
     */
    bool readPortState(const uint8_t *deviceAddress, const uint8_t pio = DS2406_PIO_A);

    /**
     * Set Conditional Search bits.
     *
     * @param deviceAddress MicroLan(c) 8-byte device address
     * @param pio one of the DS2406_PIO_* constants
     * @param sourceSelect one of the DS2406_SS_* constants
     * @param polarity LOW or HIGH
     */
    uint8_t setConditionalSearchParameters(const uint8_t *deviceAddress, const uint8_t pio, const uint8_t sourceSelect, const bool polarity);

    uint8_t conditionalSearch(uint8_t *newAddr);

  private:
    OneWire *bus;
    uint16_t crc;
    uint8_t programPulsePin;

    uint8_t readMemory(const uint8_t readCommand, const uint8_t *deviceAddress, const uint8_t startAddress, uint8_t *data, const uint8_t dataLength, const uint8_t memorySize);
    void channelAccessCheckCRC(DS2406ChannelAccessTransaction &tx);
    uint8_t getChannelAccessCounter(const uint8_t channelControlByte);
};

#endif

