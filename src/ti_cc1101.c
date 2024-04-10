#include "stm32c0xx.h"
#include "hal_TI_cc1101.h"
#include "ti_cc1101.h"
#ifdef DEBUG
  #include <stdio.h>
#endif

//-------------------------------------------------------------------------------------------------------
// Chipcon
// Product = CC1101
// Chip version = A   (VERSION = 0x04)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 541.666667 kHz
// Deviation = 127 kHz
// Datarate = 249.938965 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 867.999939 MHz _OR_ 432.999817 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync sent / received, de-asserts at end of packet
// GDO2 signal selection = (41) CHIP_RDY
RF_SETTINGS  rfSettings =
{
    0x0C,               // FSCTRL1   Frequency synthesizer control
    0x00,               // FSCTRL0   Frequency synthesizer control
#ifdef _BASEFREQ_867_
    0x21,               // FREQ2     Frequency control word, high byte
    0x62,               // FREQ1     Frequency control word, middle byte
    0x76,               // FREQ0     Frequency control word, low byte
#else
    0x0C,   // FREQ2     Frequency control word, high byte.
    0x1D,   // FREQ1     Frequency control word, middle byte.
    0x89,   // FREQ0     Frequency control word, low byte.
#endif
    0x2D,               // MDMCFG4   Modem configuration
    0x3B,               // MDMCFG3   Modem configuration
    0x13,               // MDMCFG2   Modem configuration
    0x22,               // MDMCFG1   Modem configuration
    0xF8,               // MDMCFG0   Modem configuration
    0x00,               // CHANNR    Channel number
    0x62,               // DEVIATN   Modem deviation setting (when FSK modulation is enabled)
    0xB6,               // FREND1    Front end RX configuration
    0x10,               // FREND0    Front end TX configuration
    0x18,               // MCSM0     Main Radio Control State Machine configuration
    0x1D,               // FOCCFG    Frequency Offset Compensation Configuration
    0x1C,               // BSCFG     Bit synchronization Configuration
    0xC7,               // AGCCTRL2  AGC control
    0x00,               // AGCCTRL1  AGC control
    0xB0,               // AGCCTRL0  AGC control
    0xEA,               // FSCAL3    Frequency synthesizer calibration
    0x2A,               // FSCAL2    Frequency synthesizer calibration
    0x00,               // FSCAL1    Frequency synthesizer calibration
    0x1F,               // FSCAL0    Frequency synthesizer calibration
    0x59,               // FSTEST    Frequency synthesizer calibration
    0x88,               // TEST2     Various test settings
    0x31,               // TEST1     Various test settings
    0x09,               // TEST0     Various test settings
    0x07,               // FIFOTHR   RXFIFO and TXFIFO thresholds
    0x29,               // IOCFG2    GDO2 output pin configuration
    0x06,               // IOCFG0D   GDO0 output pin configuration 
    PRJ_PKTCTRL1,       // PKTCTRL1  Packet automation control; 0x04 = no address check
    PRJ_PKTCTRL0,       // PKTCTRL0  Packet automation control; 0x05 = variable length with CRC_Enable
    PRJ_DEVICE_ADDRESS, // ADDR      Device address
//  PRJ_PACKET_LENGTH*2 // PKTLEN    max. Packet length; 0xFF = max length
    PRJ_PACKET_LENGTH   // PKTLEN    max. Packet length; 0xFF = max length
};
// SYNC0 and SYNC1 are omitted here, thus remain at default (SYNC0=0x91, SYNC1=0xD3)
// PATABLE (0 dBm output power)
BYTE  paTable = 0x81;

///> external symbols for debugging purposes
extern uint8_t           deviceAdr;
extern volatile uint8_t  devStatus;
extern uint8_t           isVarSize;

extern void              dbgputDevStatus (void);


/* -------- internal functions -------- */
static void  wait_GDO0_high (void);
static void  wait_GDO0_low  (void);


/* -----------------------------------------------------
 * initialize the MCU peripheral pins, and the CC1101
 */
void  initTransmitter (void)
{
    cc1101_setupPins ();   ///> setup MCU SPI pins
    puReset_CC1101 ();     ///> power-up reset of the CC1101 transmitter
}



//-------------------------------------------------------------------------------------------------------
//  BOOL  halRfReceivePacket (BYTE *rxBuffer, UINT8 *length)
//
//  DESCRIPTION: 
//   This function can be used to receive a packet of variable packet length (first byte in the packet
//   must be the length byte). The packet length should not exceed the RX FIFO size.
//   To use this function, GD00 must be configured to assert when sync word is sent, and de-assert
//   at the end of the packet => halSpiWriteReg(CC1101_IOCFG0, 0x06);
//   Also, APPEND_STATUS in the PKTCTRL1 register must be enabled.
//   The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//   for it to be cleared.
//   After the GDO0 pin has been de-asserted, the RXBYTES register is read to make sure that there
//   are bytes in the FIFO. This is because the GDO signal will indicate sync received even if the
//   FIFO is flushed due to address filtering, CRC filtering, or packet length filtering. 
//  
//  ARGUMENTS:
//   BYTE *rxBuffer - pointer to the buffer where the incoming data should be stored
//   UINT8 *length  - pointer to a variable containing the size of the buffer where the data
//                    should be stored. At function return, that variable holds the packet length.
//  RETURN VALUE:
//      BOOL  TRUE:   CRC OK
//            FALSE:  CRC NOT OK (or no packet was put in the RX FIFO due to filtering)
//-------------------------------------------------------------------------------------------------------
BOOL  halRfReceivePacket (BYTE *rxBuffer, UINT8 *length)
{
    BYTE   status[2];
    UINT8  packetLength;

    halSpiStrobe (CC1101_SRX);

    // while (!GDO0_PIN);    // Wait for GDO0 to be set -> sync received
    wait_GDO0_high ();

    // while (GDO0_PIN);     // Wait for GDO0 to be cleared -> end of packet
    wait_GDO0_low ();

    // This status register is safe to read since it will not be updated after
    // the packet has been received (See the CC1100 and 2500 Errata Note)
    if ((halSpiReadStatus (CC1101_RXBYTES) & BYTES_IN_RXFIFO))
    {
        packetLength = halSpiReadReg (CC1101_RXFIFO);    // Read length byte

        // Read data from RX FIFO and store in rxBuffer
        if (packetLength <= *length)
        {
            halSpiReadBurstReg (CC1101_RXFIFO, rxBuffer, packetLength); 
            *length = packetLength;

            // Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
            halSpiReadBurstReg (CC1101_RXFIFO, status, 2); 

            return (status[LQI] & CRC_OK);       // MSB of LQI is the CRC_OK bit
        }
        else
        {
            *length = packetLength;

            // Make sure that the radio is in IDLE state before flushing the FIFO
            // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
            halSpiStrobe(CC1101_SIDLE);

            halSpiStrobe(CC1101_SFRX);         // Flush RX FIFO
            return FALSE;
        }
    }
    else
        return FALSE;
}



//-------------------------------------------------------------------------------------------------------
//  BOOL  halRfReceiveFixedPacket (BYTE *rxBuffer, UINT8 *length, UINT8 size)
//
//  DESCRIPTION: 
//   This function can be used to receive a packet of fixed length.
//   To use this function, GD00 must be configured to assert when sync word is sent, and de-assert
//   at the end of the packet => halSpiWriteReg(CC1101_IOCFG0, 0x06);
//   Also, APPEND_STATUS in the PKTCTRL1 register must be enabled.
//   The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//   for it to be cleared.
//   After the GDO0 pin has been de-asserted, the RXBYTES register is read to make sure that there
//   are bytes in the FIFO. This is because the GDO signal will indicate sync received even if the
//   FIFO is flushed due to address filtering, CRC filtering, or packet length filtering. 
//  
//  ARGUMENTS:
//   BYTE  *rxBuffer - pointer to the buffer where the incoming data should be stored;
//   UINT8 *length   - At function return, that variable holds the packet length;
//   UINT8  size     - packet size; should not exceed the RX FIFO size;
//  RETURN VALUE:
//      BOOL  TRUE:   CRC OK
//            FALSE:  CRC NOT OK (or no packet was put in the RX FIFO due to filtering)
//-------------------------------------------------------------------------------------------------------
static uint32_t  dbgCounter = 0;
BOOL  halRfReceiveFixedPacket (BYTE *rxBuffer, UINT8 *length, UINT8 size)
{
    BYTE   status[2];
    UINT8  packetLength;

    halSpiStrobe (CC1101_SRX);
//  dbgputDevStatus ();

    wait_GDO0_high ();    // wait for GDO0 to be set, sync received

    wait_GDO0_low ();     // wait for GDO0 to be cleared, end of packet

    // This status register is safe to read since it will not be updated after
    // the packet has been received (See the CC1100 and 2500 Errata Note)
    if ((halSpiReadStatus (CC1101_RXBYTES) & BYTES_IN_RXFIFO))
    {
        packetLength = size;  // fixed size

        // Read data from RX FIFO and store in rxBuffer
        if (packetLength <= *length)
        {
            halSpiReadBurstReg (CC1101_RXFIFO, rxBuffer, packetLength); 
            *length = packetLength;

#if (PRJ_PKTCTRL1 & 0x04)
            // Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
            halSpiReadBurstReg (CC1101_RXFIFO, status, 2); 

            if (++dbgCounter < 20)
                printf ("L= %u, CRC-OK = %c, LQI = %d\n", (unsigned) packetLength, (status[LQI] & CRC_OK) ? '+':'-', status[LQI] & 0x7F);
            return (status[LQI] & CRC_OK);       // MSB of LQI is the CRC_OK bit
#else
            return (1);
#endif
        }
        else
        {
            *length = packetLength;

            // Make sure that the radio is in IDLE state before flushing the FIFO
            // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
            halSpiStrobe (CC1101_SIDLE);

            halSpiStrobe (CC1101_SFRX);         // Flush RX FIFO
            return FALSE;
        }
    }
    else
        return FALSE;
}


uint8_t  myFixedPckReceive (BYTE *rxBuffer)
{
    UINT8  pcklen, bytes;

    halSpiStrobe (CC1101_SRX);

    // wait for GDO0 to toggle
    wait_GDO0_high ();
    wait_GDO0_low ();

    // this status register is safe to read since it will not be updated after
    // the packet has been received (See the CC1100 and 2500 Errata Note)
    bytes = (halSpiReadStatus (CC1101_RXBYTES) & BYTES_IN_RXFIFO);
    if (bytes == 0)
        return 0;

printf ("\nRXBYTES = %02X\n", bytes);

    // Read data from RX FIFO and store in rxBuffer
    if (bytes <= BUFFER_SIZE)
    {
int i;
        halSpiReadBurstReg (CC1101_RXFIFO, rxBuffer, bytes);
printf ("\npck data:\n  ");
for (i=0; i<bytes; i++)
  printf ("%02X%s", rxBuffer[i], ((i%8)==7) ? "\n  " : " ");

        return (bytes);
    }

    // ...else
    halSpiStrobe (CC1101_SIDLE);
    halSpiStrobe (CC1101_SFRX);  // Flush RX FIFO
    return FALSE;
}



//-------------------------------------------------------------------------------------------------------
//  void halRfSendPacket(BYTE *txBuffer, UINT8 size)
//
//  DESCRIPTION:
//      This function can be used to transmit a packet with up to 63 bytes. To use this function,
//      GD00 must be configured to be asserted when sync word is sent and de-asserted at the end
//      of the packet => halSpiWriteReg(CC1101_IOCFG0, 0x06);
//      The function uses polling of GDO0. First waits for GD00 to be set and then to be cleared.  
//      
//  ARGUMENTS:
//      BYTE *txBuffer  - pointer to send data buffer
//      UINT8 size      - size of the txBuffer
//-------------------------------------------------------------------------------------------------------
void  rfSendPacket (uint8_t *txBuffer, uint32_t size)
{
    // in variable-packet-size mode, a size byte must be written first
    if (isVarSize)
        halSpiWriteReg (CC1101_TXFIFO, size);

    // write the device address first if not 0
    if (deviceAdr != 0)
        halSpiWriteReg (CC1101_TXFIFO, deviceAdr);

    halSpiWriteBurstReg (CC1101_TXFIFO, txBuffer, size);

    halSpiStrobe (CC1101_STX);

    wait_GDO0_high ();   // wait for GDO0 being set -> sync transmitted
    wait_GDO0_low ();    // wait for GDO0 being cleared -> end of packet
}



//-------------------------------------------------------------------------------------------------------
//  BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length)
//
//  DESCRIPTION: 
//      This function can be used to receive a packet of variable packet length (first byte in the packet
//      must be the length byte). The packet length should not exceed the RX FIFO size.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CC1101_IOCFG0, 0x06);
//      Also, APPEND_STATUS in the PKTCTRL1 register must be enabled.
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.
//      After the GDO0 pin has been de-asserted, the RXBYTES register is read to make sure that there
//      are bytes in the FIFO. This is because the GDO signal will indicate sync received even if the
//      FIFO is flushed due to address filtering, CRC filtering, or packet length filtering. 
//  
//  ARGUMENTS:
//      BYTE *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      UINT8 *length
//          Pointer to a variable containing the size of the buffer where the incoming data should be
//          stored. After this function returns, that variable holds the packet length.
//          
//  RETURN VALUE:
//      BOOL
//          TRUE:   CRC OK
//          FALSE:  CRC NOT OK (or no packet was put in the RX FIFO due to filtering)
//-------------------------------------------------------------------------------------------------------
uint8_t  rfReceivePacket (uint8_t *rxBuffer, uint8_t *length)
{
    uint8_t  status[2];
    uint8_t  pckLen;

    halSpiStrobe(CC1101_SRX);

    // while (!GDO0_PIN);    // Wait for GDO0 to be set -> sync received
    wait_GDO0_high ();

    // while (GDO0_PIN);    // Wait for GDO0 to be cleared -> end of packet
    wait_GDO0_low ();

    // This status register is safe to read since it will not be updated after
    // the packet has been received (See the CC1100 and 2500 Errata Note)
    if ((halSpiReadStatus (CC1101_RXBYTES) & BYTES_IN_RXFIFO))
    {
        pckLen = halSpiReadReg (CC1101_RXFIFO);  // Read length byte
        if (pckLen <= *length)                   // Read data from RX FIFO and store in rxBuffer
        {
            halSpiReadBurstReg(CC1101_RXFIFO, rxBuffer, pckLen);
            *length = pckLen;
        
            // Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
            halSpiReadBurstReg(CC1101_RXFIFO, status, 2);
        
            // MSB of LQI is the CRC_OK bit
            return (status[LQI] & CRC_OK);
        }
        else
        {
#ifdef _CC_DEBUG
            // DEBUG : TRY TO READ ANYWAY, AND DUMP TO THE CONSOLE
            cc1101_readBurstReg (rxBuffer, CC1101_RXFIFO, *length - 1);
            rxBuffer[*length - 1] = '\0';
            printf ("::<%s>\n", rxBuffer);
            *length = pckLen;
#endif
            // Make sure that the radio is in IDLE state before flushing the FIFO
            // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
            halSpiStrobe(CC1101_SIDLE);

            halSpiStrobe(CC1101_SFRX);  // Flush RX FIFO
            return 0;
        }
    }
    else
        return 0;
}



//-------------------------------------------------------------------------------------------------------
//  void RfWriteRfSettings (RF_SETTINGS *pRfSettings)
//
//  DESCRIPTION:
//   This function is used to configure the CC1101 based on a given rf setting
//
//  ARGUMENTS:
//   RF_SETTINGS *pRfSettings - pointer to a struct containing rf register settings
//-------------------------------------------------------------------------------------------------------
void  halRfWriteRfSettings (RF_SETTINGS *pRfSettings)
{
    halSpiWriteReg (CC1101_FSCTRL1,  pRfSettings->FSCTRL1);
    halSpiWriteReg (CC1101_FSCTRL0,  pRfSettings->FSCTRL0);
    halSpiWriteReg (CC1101_FREQ2,    pRfSettings->FREQ2);
    halSpiWriteReg (CC1101_FREQ1,    pRfSettings->FREQ1);
    halSpiWriteReg (CC1101_FREQ0,    pRfSettings->FREQ0);
    halSpiWriteReg (CC1101_MDMCFG4,  pRfSettings->MDMCFG4);
    halSpiWriteReg (CC1101_MDMCFG3,  pRfSettings->MDMCFG3);
    halSpiWriteReg (CC1101_MDMCFG2,  pRfSettings->MDMCFG2);
    halSpiWriteReg (CC1101_MDMCFG1,  pRfSettings->MDMCFG1);
    halSpiWriteReg (CC1101_MDMCFG0,  pRfSettings->MDMCFG0);
    halSpiWriteReg (CC1101_CHANNR,   pRfSettings->CHANNR);
    halSpiWriteReg (CC1101_DEVIATN,  pRfSettings->DEVIATN);
    halSpiWriteReg (CC1101_FREND1,   pRfSettings->FREND1);
    halSpiWriteReg (CC1101_FREND0,   pRfSettings->FREND0);
    halSpiWriteReg (CC1101_MCSM0 ,   pRfSettings->MCSM0 );
    halSpiWriteReg (CC1101_FOCCFG,   pRfSettings->FOCCFG);
    halSpiWriteReg (CC1101_BSCFG,    pRfSettings->BSCFG);
    halSpiWriteReg (CC1101_AGCCTRL2, pRfSettings->AGCCTRL2);
    halSpiWriteReg (CC1101_AGCCTRL1, pRfSettings->AGCCTRL1);
    halSpiWriteReg (CC1101_AGCCTRL0, pRfSettings->AGCCTRL0);
    halSpiWriteReg (CC1101_FSCAL3,   pRfSettings->FSCAL3);
    halSpiWriteReg (CC1101_FSCAL2,   pRfSettings->FSCAL2);
    halSpiWriteReg (CC1101_FSCAL1,   pRfSettings->FSCAL1);
    halSpiWriteReg (CC1101_FSCAL0,   pRfSettings->FSCAL0);
    halSpiWriteReg (CC1101_FSTEST,   pRfSettings->FSTEST);
    halSpiWriteReg (CC1101_TEST2,    pRfSettings->TEST2);
    halSpiWriteReg (CC1101_TEST1,    pRfSettings->TEST1);
    halSpiWriteReg (CC1101_TEST0,    pRfSettings->TEST0);
    halSpiWriteReg (CC1101_FIFOTHR,  pRfSettings->FIFOTHR);
    halSpiWriteReg (CC1101_IOCFG2,   pRfSettings->IOCFG2);
    halSpiWriteReg (CC1101_IOCFG0,   pRfSettings->IOCFG0);    
    halSpiWriteReg (CC1101_PKTCTRL1, pRfSettings->PKTCTRL1);
    halSpiWriteReg (CC1101_PKTCTRL0, pRfSettings->PKTCTRL0);
    halSpiWriteReg (CC1101_ADDR,     pRfSettings->ADDR);
    halSpiWriteReg (CC1101_PKTLEN,   pRfSettings->PKTLEN);

    if ((pRfSettings->PKTCTRL0 & 0x03) == 0x01)
        isVarSize = 1;
}


/* set the PA table value; is set separately in the example
 * just takes the PA table setting as parameter
 */
void  rfSetPaTable (uint8_t paTable)
{
    halSpiWriteReg (CC1101_PATABLE, paTable);
}


/* some more convenience functions to set/change
 * some transmitter settings & functionality
 */
void  setRxState(void)
{
    halSpiStrobe (CC1101_SRX);
}

void  setTxState(void)
{
    halSpiStrobe (CC1101_STX);
}

void  setIdleState(void)
{
    halSpiStrobe (CC1101_SIDLE);
}

void  flushRxFifo(void)
{
    halSpiStrobe (CC1101_SFRX);
}

void  flushTxFifo(void)
{
    halSpiStrobe (CC1101_SFTX);
}


//-------------------------------------------------------------------------
// function to reset the CC1101 after power-on, and wait for it to be ready
//
//                 min 40 us
//             <----------------------->
// CSn      |--|  |--------------------|          |-----------
//          |  |  |                    |          |
//              --                      ----------
//
// MISO                                       |---------------
//          - - - - - - - - - - - - - - - -|  |         
//                                          --          
//               Unknown / don't care
//
// MOSI     - - - - - - - - - - - - - - - ---------- - - - - - 
//                                         | SRES |
//          - - - - - - - - - - - - - - - ---------- - - - - -                    
//
void  puReset_CC1101 (void)
{
    setSS (1);

    udelay (5);
    setSS (0);

    // wait for about 10us
    udelay (10);
    setSS (1);

    // wait at least 40us
    udelay (40);
    setSS (0);

    while (getMISO ());         // wait for MISO to go low
    spi_send (CC1101_SRES);     // send reset command strobe

//  while (getMISO ());         // wait for MISO to go low

    setSS (1);
}


/* replicate the orignal function calls ...
 */
static void  wait_GDO0_high (void)
{
    while (!getGDO0 ());
}


static void  wait_GDO0_low (void)
{
    while (getGDO0 ());
}


#ifdef _HW_TEST_
#define HWTX_SIZE      32
static uint8_t            hwtBuf[HWTX_SIZE] = {0};
extern volatile uint32_t  toDelay;
void  testCC1101LoopTx (void)
{
    int  i;

    for (i=0; i<HWTX_SIZE; i++)
        hwtBuf[i] = (uint8_t) i;

    toDelay = 0; // start without delay ...
    do
    {
        if (!toDelay)
        {
            for (i=0; i<10; i++)
            {
                halSpiWriteBurstReg (CC1101_TXFIFO, hwtBuf, HWTX_SIZE);
                halSpiStrobe (CC1101_STX);
                udelay (10);
            }
            toDelay = 80;  // wait 500ms to the next burst
        }
    }
    while (1);
}
#endif
