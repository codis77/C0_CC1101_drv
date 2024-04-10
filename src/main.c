/*********************************************************
*             CC1101 test driver application             *
*                for the STM32C031 MCU                   *
**********************************************************
*
* A simple test application to send RF packages between
* two board interfacing a CC1101. On startup, check the
* user button on the board to go into Tx mode (button
* is pressed), or Rx mode (default).
* The sender prepares a packet of 32 bytes every 500ms,
* sending it via the CC1101. The receiver is waiting for
* for packages, and displays them in the debug console.
* The second transmitted byte is used to turn an onboard
* LED on or off. Zero values will turn the receiver LED
* off, other values will turn them on.
* The sender checks the user button before every send
* cycle, and sends either a 1 (button pressed) or 0 for
* this second byte.
*
* the following transmission parameters are used for the
* CC1101 (non-default only):
* - frequency = 868MHz
* - fixed packet length (32 byte)
* - address check enabled, boar address = 0x43
*
* peripheral pins used in the application:
*  -  PB.3  =  SPI SCLK    -->  O
*  -  PB.4  =  SPI MISO    <--  i
*  -  PB.5  =  SPI MOSI    -->  O
*  -  PB.6  =  GDO0        <--  i
*  -  PB.7  =  SPI CS      -->  O
* --------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include "stm32c0xx.h"
#include "ti_cc1101.h"
#include "hal_TI_cc1101.h"
#include "main.h"

#define MODE_RX         0
#define MODE_TX         1
#define BIT_USER_BTN    0x2000  // PC13


volatile uint32_t         toDelay   = 0;          /* Timeout Delay, in ms */
uint32_t                  sysMode   = MODE_RX;    /* tx/rx mode */
volatile uint32_t         txTrigger = 0;
volatile uint32_t         txTimer   = 0;
uint8_t                   deviceAdr = PRJ_DEVICE_ADDRESS;  // currently fixed
uint8_t                   isVarSize = 0;                   // fixed or variable-size packet
volatile uint8_t          gdo0State = 0;

static volatile uint8_t   sendBuffer[BUFFER_SIZE] = { 0 };   // send buffer
static volatile uint8_t   recvBuffer[BUFFER_SIZE] = { 0 };   // receive buffer

///> ============ DEBUG ============
volatile uint8_t     devStatus = 0;

///> ------------ external data ------------
extern uint32_t           SystemCoreClock;
extern RF_SETTINGS        rfSettings;
extern BYTE               paTable;

/* -------- internal prototypes --------
 */
static void        initN44C_LED    (void);
static void        setCoreClock    (void);
static void        initSysData     (uint32_t mode);
static void        sysDelay        (uint16_t delaytime);
static void        setupUserButton (void);
static uint32_t    getUserButton   (void);
static void        setLED          (uint8_t state);
static uint32_t    prepareBuffer   (char *buf, uint32_t count, uint32_t size);
#ifdef _HW_TEST_
void               testCC1101LoopTx (void);
#endif
static void        putccRegsDbg    (void);
static void        dumpRecvBuf     (char *, uint8_t);

extern uint8_t     myFixedPckReceive (BYTE *rxBuffer);

/* ----------------------------------------
 * ----------------- CODE -----------------
 * ----------------------------------------
 */

/* ------------ main() ------------
 */
int  main (void)
{
    uint32_t  i, temp;
    uint8_t   len, id = 0;

    setCoreClock ();
    SystemCoreClockUpdate ();
    i = SystemCoreClock;

    SysTick_Config (i/100);   // 10ms cycle

    // some hardware/system initialisations
    initN44C_LED ();
    setupUserButton ();
    initSysData (sysMode);

    // check user button at startup;
    if (getUserButton())
        sysMode = MODE_TX;

    // init the SPI interface and the transmitter
    initTransmitter ();

    // write the application settings to the transmitter
    halRfWriteRfSettings (&rfSettings);
    rfSetPaTable (paTable);

    // display CC1101 registers in the debug console
    putccRegsDbg ();

#ifdef _HW_TEST_
//  testCC1101LoopTx ();
//#warning "ATTENTION: HW TX TEST MODE !"
#endif

    temp = 0;

    ///> main loop in Tx mode;
    ///> transmit a message every once in a while
    do
    {
        if (txTrigger)
        {
            txTrigger = 0;
            setLED (1);

            // create packet data and send
            len = prepareBuffer ((char *) sendBuffer, i, BUFFER_SIZE);

            // add a LED switch flag derived from the sender's user button
            if (getUserButton())
                sendBuffer[0] = 0xFF;

            rfSendPacket ((uint8_t *) sendBuffer, TX_TRANSMIT_SIZE);

            // go back to RX mode
            setIdleState();           // Enter IDLE state
            flushRxFifo();            // Flush Rx FIFO  
            setRxState ();
            sysDelay (250);
            setLED (0);
            sysDelay (750);
        }

//      else  // check Rx state, wait for data reception
        if (sysMode != MODE_TX)
        {
            halSpiStrobe (CC1101_SFRX);  // Flush RX FIFO
            len = sizeof (recvBuffer);
//          if (halRfReceiveFixedPacket ((BYTE *) recvBuffer, &len, TX_TRANSMIT_SIZE)) //, RX_TIMEOUT))
            if (myFixedPckReceive ((BYTE *) recvBuffer))
            {
                // check the LED switch flag
                if (recvBuffer[1] == 0xFF)
                    setLED (1);
                else setLED (0);
#if 0
                recvBuffer[TX_TRANSMIT_SIZE] = 0x00;   // zero-terminate
                printf ("%s\n", recvBuffer);           // debug output
#else
//              if (temp++ < 20)
//                  dumpRecvBuf ((char *) recvBuffer, PRJ_PACKET_LENGTH);
#endif
                sysDelay(50);
                setLED (0);
            };
        };
    }
    while (1);
}


static void  dumpRecvBuf (char *buffer, uint8_t size)
{
    int  i;

    printf ("\npck data:\n  ");
    for (i=0; i<size; i++)
        printf ("%02X%s", buffer[i], ((i%16)==15) ? "\n  " : " ");
}


static void  setupUserButton (void)
{
    uint32_t  temp;

    // enable power to GPIOC
    RCC->IOPENR |= (1 << RCC_IOPENR_GPIOCEN_Pos);

    //  set the GPIOC MODE register bits for pin 13
    // in this case, just clear the two bits
    temp  = GPIOC->MODER;
    temp &= ~(GPIO_MODER_MODE13_Msk);
    GPIOC->MODER = temp;
}



/* get the user button state;
 * button logic is inverted, reads H if not pressed
 */
static uint32_t  getUserButton (void)
{
    if (GPIOC->IDR & BIT_USER_BTN)
        return 0;
    else 
        return 1;
}



/* set the user LED at PA5 on the Nucleo board */
static void  setLED (uint8_t state)
{
    if (state)
        GPIOA->BSRR = 0x0020;
    else
        GPIOA->BRR  = 0x0020;
}


static uint32_t  counter = 0;

/* SysTick handler; toggle bit to measure clock cycle
 * @@@> measured cycle is with SystemCoreClockUpdate() reporting 12.0 MHz
 *      and a SysTick divider of 1000 is 1.0ms/1kHz, i.e. 12 MHz
 */
void  SysTick_Handler (void)
{
    // in Tx mode, set the transmission timing
    if (sysMode == MODE_TX)
    {
        txTimer++;
        if (txTimer >= CC1101_TX_CYCLE)
        {
            txTrigger = 1;    ///> set trigger
            txTimer   = 0;    ///> reset time counter
        }
    }

    // delay timer
    if (toDelay)
        toDelay--;
}



/* handler for the EXTI interrupt;
 * check the EXTI_pending registers and update the status flag
 */
void  EXTI4_15_IRQHandler (void)
{
    uint8_t  isHL, isLH;

    // get transition status
    isHL = (EXTI->FPR1 & GPIO_PIN_6) ? 1 : 0;
    isLH = (EXTI->RPR1 & GPIO_PIN_6) ? 1 : 0;

    // clear all pending bits; writing '1' clears the bits
    EXTI->FPR1 |= EXTI_PEND_CLR_MASK;
    EXTI->RPR1 |= EXTI_PEND_CLR_MASK;

    // update status flag
    if ((gdo0State == EVT_TRANSIT_NONE) && (isLH))
        gdo0State = EVT_TRANSIT_UP;
    else if ((gdo0State == EVT_TRANSIT_UP) && (isHL))
        gdo0State = EVT_TRANSIT_DOWN;
    else // all other combinations mean we probably missed an event
        gdo0State = EVT_TRANSIT_MISSED;
}



/* configure the EXTI interrupt for pin PB6 (GDO0),
 * interrupt on both transitions
 */
void  configGDO_Exti (void)
{
    EXTI->RTSR1     |= GPIO_PIN_6;
    EXTI->FTSR1     |= GPIO_PIN_6;
    EXTI->EXTICR[1] |= PB6_EXTICR2_MSK;
}


/* enable the EXTI interrupt for PB6;
 */
void  enableGDOint (void)
{
    __NVIC_EnableIRQ (EXTI4_15_IRQn);
}



/* init PA5 as digital out (for an attached LED);
 * the MODER register is set appropriately;
 * OTYPER, OSPEEDR and PUPDR are not touched, no change required;
 */
static void   initN44C_LED (void)
{
    uint32_t  temp;

    // enable power to GPIOA
    RCC->IOPENR |= (1 << RCC_IOPENR_GPIOAEN_Pos);

    //  set the GPIOA MODE register for pin 5
    temp          = GPIOA->MODER;
    temp         &= ~(GPIO_MODER_MODE5_Msk);
    GPIOA->MODER  = temp;
    GPIOA->MODER |= GPIO_MODER_MODE5_0;
}


/* set the core clock to the external quartz frequency;
 * assumes >24MHz and thus increases Flash latency/wait states to 1;
 * the different peripheral clock dividers are not changed !
 */
static void  setCoreClock (void)
{
    FLASH->ACR |= (1 << FLASH_ACR_LATENCY_Pos);    // set latency (Flash) to 1 WS

    RCC->CR |= (1 << RCC_CR_HSEON_Pos);            // enable ext. quartz oscillator
    while (!(RCC->CR & (1 << RCC_CR_HSERDY_Pos))); // wait till ready
    RCC->CFGR |= 0x0001;                           // now, switch over to HSE
}



/* use SysTick and a dedicated counter variable for a delay;
 * parameter is interpreted as ms, but SysTick based upon 10ms cycle;
 * thus it is divided by 10
 */
static void  sysDelay (uint16_t delaytime)
{
    toDelay = delaytime / 10;
    while (toDelay > 0);
}



/* init address arrays depending on system mode
 */
static void  initSysData (uint32_t mode)
{
}


/* prepare a test package for sending
 */
static uint32_t  prepareBuffer (char *buf, uint32_t count, uint32_t size)
{
    uint32_t len, i;
#if 0
    len = snprintf (buf, size, "CC1101 Tx message #<%d> from F3_Disco\n", count);
    buf[len] = '\0';    // zero-terminate
#else
    for (i=0; i<PRJ_PACKET_LENGTH; i++)
        buf[i] = (char) i;
    len = PRJ_PACKET_LENGTH;
#endif
    return (len);
}


extern BYTE  halSpiReadReg (BYTE);
extern BYTE  halSpiReadStatus (BYTE);

// read and dump CC1101 regs to the debug console
static void  putccRegsDbg (void)
{
    uint8_t  reg, val;

    printf (" -- config regs -- \n");
    for (reg=0; reg<0x30; reg++)
    {
        val = halSpiReadReg (reg);
        printf ("%02X%c", val, (reg && ((reg%8)==7)) ? '\n' : ' ');
    }

    ///> WARNING: a single-byte read from register address 0x30 will trigger
    ///> a reset, and cause a revert to default values !!!
    printf ("\n -- status regs -- \n-- ");
    for (reg=0x31; reg<0x40; reg++)
    {
        val = halSpiReadStatus (reg);
        printf ("%02X%c", val, (reg && ((reg%8)==7)) ? '\n' : ' ');
    }
}


void  dbgputDevStatus (void)
{
    printf ("\ndev state = %02X\n", devStatus);
    printf ("%s, state = ", (devStatus & 0x80) ? "NRDY!" : "RDY");  // low-active
    switch (devStatus & 0x70)
    {
        case 0:    printf ("idle");       break;
        case 0x10: printf ("Rx");         break;
        case 0x20: printf ("Tx");         break;
        case 0x40: printf ("FSTXon");     break;
        case 0x50: printf ("Calibrate");  break;
        case 0x60: printf ("FIFO_Rx_OV"); break;
        case 0x70: printf ("FIFO_Tx_OV"); break;
    }
    printf ("\nFIFO Avail = %d\n", (devStatus & 0x0F));
}


/********************* End of file **********************/
