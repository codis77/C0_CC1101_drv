#include "main.h"

//----------------------------------------------------------------------------------
// --- defines
#define FALSE               0
#define TRUE                1
#define CRC_OK              0x80
#define GDO0_PIN            P0_6
#define RSSI                0
#define LQI                 1
#define BYTES_IN_RXFIFO     0x7F

#define code    // ... away

//----------------------------------------------------------------------------------

// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS
{
    BYTE FSCTRL1;   // Frequency synthesizer control.
    BYTE FSCTRL0;   // Frequency synthesizer control.
    BYTE FREQ2;     // Frequency control word, high byte.
    BYTE FREQ1;     // Frequency control word, middle byte.
    BYTE FREQ0;     // Frequency control word, low byte.
    BYTE MDMCFG4;   // Modem configuration.
    BYTE MDMCFG3;   // Modem configuration.
    BYTE MDMCFG2;   // Modem configuration.
    BYTE MDMCFG1;   // Modem configuration.
    BYTE MDMCFG0;   // Modem configuration.
    BYTE CHANNR;    // Channel number.
    BYTE DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
    BYTE FREND1;    // Front end RX configuration.
    BYTE FREND0;    // Front end RX configuration.
    BYTE MCSM0;     // Main Radio Control State Machine configuration.
    BYTE FOCCFG;    // Frequency Offset Compensation Configuration.
    BYTE BSCFG;     // Bit synchronization Configuration.
    BYTE AGCCTRL2;  // AGC control.
    BYTE AGCCTRL1;  // AGC control.
    BYTE AGCCTRL0;  // AGC control.
    BYTE FSCAL3;    // Frequency synthesizer calibration.
    BYTE FSCAL2;    // Frequency synthesizer calibration.
    BYTE FSCAL1;    // Frequency synthesizer calibration.
    BYTE FSCAL0;    // Frequency synthesizer calibration.
    BYTE FSTEST;    // Frequency synthesizer calibration control
    BYTE TEST2;     // Various test settings.
    BYTE TEST1;     // Various test settings.
    BYTE TEST0;     // Various test settings.
    BYTE FIFOTHR;   // RXFIFO and TXFIFO thresholds.
    BYTE IOCFG2;    // GDO2 output pin configuration
    BYTE IOCFG0;    // GDO0 output pin configuration
    BYTE PKTCTRL1;  // Packet automation control.
    BYTE PKTCTRL0;  // Packet automation control.
    BYTE ADDR;      // Device address.
    BYTE PKTLEN;    // Packet length.
} RF_SETTINGS;



//----------------------------------------------------------------------------
// API functions
void     initTransmitter         (void);
void     udelay                  (uint32_t n);

void     puReset_CC1101          (void);
BOOL     halRfReceivePacket      (BYTE *rxBuffer, UINT8 *length);
BOOL     halRfReceiveFixedPacket (BYTE *rxBuffer, UINT8 *length, UINT8 size);
void     rfSendPacket            (uint8_t *txBuffer, uint32_t size);
uint8_t  rfReceivePacket         (uint8_t *rxBuffer, uint8_t *length);
void     halRfWriteRfSettings    (RF_SETTINGS *pRfSettings);
void     rfSetPaTable            (uint8_t paTable);

void     setRxState              (void);
void     setTxState              (void);
void     setIdleState            (void);
void     flushRxFifo             (void);
void     flushTxFifo             (void);
