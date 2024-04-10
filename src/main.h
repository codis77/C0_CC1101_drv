#ifndef _MAIN_H_
 #define _MAIN_H_

/* ---------------- SPI interface ----------------
 * if "_SPI_BITBANG_IF_" is defined, a SW implementation for the SPI interface is used;
 * else, the STM32C03x peripheral unit SPI1 is used;
 * in both cases, the very same GPIO pins are used
 */
//#define _SPI_BITBANG_IF_

#define _FIXED_PCK_LEN

// --- types
typedef unsigned char    BYTE;
typedef unsigned char    UINT8;
typedef unsigned char    BOOL;

#define GPIO_PIN_6                0x00000040  // GPIO mask for P<x>.6
#define PB6_EXTICR2_MSK           0x00010000  // mask to set PB6 as EXTI input in EXTICR2
#define EXTI_PEND_CLR_MASK        0x00FFFF    // mask for clearing the "pending" bit registers
#define EVT_TRANSIT_NONE          0           // "clear" state
#define EVT_TRANSIT_UP            1           // rising edge occured
#define EVT_TRANSIT_DOWN          2           // falling edge occured
#define EVT_TRANSIT_MISSED        3           // both directional flags were set - EVT overflow ?

#define CC1101_TX_CYCLE           50     // Tx cycle timer, in sys ticks
#define BUFFER_SIZE               40     // TX/RX buffer size
#define TX_TRANSMIT_SIZE          32     // actual package size

#if defined (_FIXED_ADR_MODE)
  #define PRJ_PKTCTRL0              0x05   // PKTCTRL0; 0x05 = variable length with CRC_Enable
  #define PRJ_PKTCTRL1              0x04   // PKTCTRL1; 0x04 = no address check
  #define PRJ_DEVICE_ADDRESS        0x43   // an address to filter out crap ...
  #define PRJ_PACKET_LENGTH         0x20   // 32 bytes
#elif defined (_FIXED_PCK_LEN)
  #define PRJ_PKTCTRL0              0x04   // PKTCTRL0; 0x04 = fixed length with CRC_Enable
  #define PRJ_PKTCTRL1              0x00   // PKTCTRL1; 0x0 = no address check, no append, no CRC-autoflush
  #define PRJ_DEVICE_ADDRESS        0x00   // an address
  #define PRJ_PACKET_LENGTH         0x20   // 32 bytes
#else
  #define PRJ_PKTCTRL0              0x05   // PKTCTRL0; 0x05 = variable length with CRC_Enable
  #define PRJ_PKTCTRL1              0x00   // PKTCTRL1; 0x0 = no address check, no append, no CRC-autoflush
  #define PRJ_DEVICE_ADDRESS        0x00   // an address 
  #define PRJ_PACKET_LENGTH         0x20   // 32 bytes
#endif

//#define _HW_TEST_

#endif // _MAIN_H_
