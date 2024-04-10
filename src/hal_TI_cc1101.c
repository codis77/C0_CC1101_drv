
#include "stm32c0xx.h"
#include "main.h"
#include "hal_TI_cc1101.h"
#include <stdio.h>


#define GPIO_RF_TRX_PORT    GPIOB
#define RF_TRX_SCLK         0x0008   // GPIO_Pin_3
#define RF_TRX_MISO         0x0010   // GPIO_Pin_4
#define RF_TRX_MOSI         0x0020   // GPIO_Pin_5
#define RF_TRX_GDO0         0x0040   // GPIO_Pin_6
#define RF_TRX_CS           0x0080   // GPIO_Pin_7


/* ---------------- internal prototypes ----------------
 */
void  udelay (uint32_t time);



/* set the Slave Select signal
 */
void  setSS (uint8_t state)
{
    if (state)
        GPIO_RF_TRX_PORT->BSRR = RF_TRX_CS;
    else
        GPIO_RF_TRX_PORT->BRR  = RF_TRX_CS;
}


/* set the SPI clock signal
 */
void  setSCK (uint8_t state)
{
    if (state)
        GPIO_RF_TRX_PORT->BSRR = RF_TRX_SCLK;
    else
        GPIO_RF_TRX_PORT->BRR  = RF_TRX_SCLK;
}


/* set the MOSI pin signal
 */
void  setMOSI (uint8_t state)
{
    if (state)
        GPIO_RF_TRX_PORT->BSRR = RF_TRX_MOSI;
    else
        GPIO_RF_TRX_PORT->BRR  = RF_TRX_MOSI;
}


/* read the MISO input pin back
 */
uint8_t  getMISO (void)
{
    return (GPIO_RF_TRX_PORT->IDR & RF_TRX_MISO);
}


/* read the GDO0 pin back;
 * configured as CC1101 output, asserts with sync reception
 * and deasserts with completed packet transmission
 */
uint8_t  getGDO0 (void)
{
    return (GPIO_RF_TRX_PORT->IDR & RF_TRX_GDO0);
}

//----------------------------------------------------------------
//  void halSpiStrobe(BYTE strobe)
//      Function for writing a strobe command to the CCxxx0
//  ARGUMENTS:
//      BYTE strobe   Strobe command
//----------------------------------------------------------------
void  halSpiStrobe (BYTE strobe)
{
    setSS (0);
    while (getMISO ());  // wait for MISO to go low
    spi_send (strobe);
    setSS (1);
}


//----------------------------------------------------------------
//  BYTE halSpiReadStatus(BYTE addr)
//      read a CCxxx0 status register
//  ARGUMENTS:
//      BYTE addr  address of the status register to be accessed
//  RETURN VALUE:
//      BYTE       value of the accessed CCxxx0 status register
//----------------------------------------------------------------
BYTE  halSpiReadStatus (BYTE addr)
{
    UINT8  val;

    setSS (0);
    while (getMISO ());  // wait for MISO to go low
    spi_send (addr + READ_BURST);
    val = spi_send (0);
    setSS (1);
    return (val);
}




//----------------------------------------------------------------
//  BYTE halSpiReadReg(BYTE addr)
//      gets the value of a single specified CCxxx0 register
//  ARGUMENTS:
//    BYTE addr  address of the CCxxx0 register to access
//
//  RETURN VALUE:
//    BYTE       value of the accessed CCxxx0 register
//----------------------------------------------------------------
BYTE  halSpiReadReg (BYTE addr)
{
    UINT8  val;

    setSS (0);
    while (getMISO ());  // wait for MISO to go low
    spi_send (addr + READ_SINGLE);
    val = spi_send (0);
    setSS (1);
    return (val);
}



//----------------------------------------------------------------
//  void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//      reads multiple CCxxx0 register, using SPI burst access
//  ARGUMENTS:
//      BYTE addr     address of the first register to access;
//      BYTE *buffer  pointer to a byte array to store the values
//                    read back from the registers
//      BYTE count    number of bytes to transfer
//----------------------------------------------------------------
void  halSpiReadBurstReg (BYTE addr, BYTE *buffer, BYTE count)
{
    UINT8 i;

    setSS (0);
    while (getMISO ());  // wait for MISO to go low
    spi_send (addr + READ_BURST);
    for (i = 0; i < count; i++)
        buffer[i] = spi_send (0);
    setSS (1);
}



//----------------------------------------------------------------
//  void halSpiWriteReg(BYTE addr, BYTE value)
//
//  DESCRIPTION:
//      Function for writing to a single CCxxx0 register
//
//  ARGUMENTS:
//      BYTE addr
//          Address of a specific CCxxx0 register to accessed.
//      BYTE value
//          Value to be written to the specified CCxxx0 register.
//----------------------------------------------------------------
void  halSpiWriteReg (BYTE addr, BYTE value)
{
    setSS (0);
    while (getMISO ());  // wait for MISO to go low
    spi_send (addr);
    spi_send (value);
    setSS (1);
}



//----------------------------------------------------------------
//  void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//      writes to multiple CCxxx0 register, using SPI burst access
//  ARGUMENTS:
//    BYTE addr     address of the first CCxxx0 register to access
//    BYTE *buffer  array of bytes to be written into a corresponding
//                  range of registers, starting 'addr';
//    BYTE count    number of bytes to write to the registers   
//----------------------------------------------------------------
void  halSpiWriteBurstReg (BYTE addr, BYTE *buffer, BYTE count)
{
    UINT8  i;

    setSS (0);
    while (getMISO ());  // wait for MISO to go low
    spi_send (addr + WRITE_BURST);
    for (i = 0; i < count; i++)
        spi_send (buffer[i]);

    setSS (1);
}

// --- debug ---
extern volatile uint8_t     devStatus;

// ------------------------ bit-bang code ------------------------ 

#ifdef _SPI_BITBANG_IF_

/* this function sets up the pins for the bit-banging interface;
 * for easy interchangeability, the same pins as for SPI1 are used:
 *   -  PB.3  =  SPI SCLK    -->  O
 *   -  PB.4  =  SPI MISO    <--  i
 *   -  PB.5  =  SPI MOSI    -->  O
 *   -  PB.6  =  GDO0        <--  i
 *   -  PB.7  =  SPI CS      -->  O
 */
void  cc1101_setupPins (void)
{
    uint32_t  temp, mask;

    // enable power to GPIOB
    RCC->IOPENR |= (1 << RCC_IOPENR_GPIOBEN_Pos);

    //  set the GPIOA MODE register for pin 5
    temp  = GPIOB->MODER;
    mask  = GPIO_MODER_MODE3_Msk + GPIO_MODER_MODE4_Msk + GPIO_MODER_MODE5_Msk + GPIO_MODER_MODE6_Msk + GPIO_MODER_MODE7_Msk;
    temp &= ~(mask);

    // set MODER bits for output; inputs are zero-value, no change for MISO & GDO0 (PB4, PB6) required
    GPIOB->MODER  = temp;
    mask          = GPIO_MODER_MODE3_0 + GPIO_MODER_MODE5_0 + GPIO_MODER_MODE7_0;
    GPIOB->MODER |= mask;
}



/* send uint8_t via SPI;
 * does only the serial clocking, slave select handling
 * must be done in the context of the calling routine !
 * 'value'  value to be sent
 * Return:
 *  Response received from SPI slave
 */
uint8_t  spi_send (uint8_t tx)
{
    uint8_t  i  = 0;
    uint8_t  rx = 0;

    setSCK (0);
    udelay (2);

    for (i=0; i<8; i++)
    {
        if(tx & (1<<(7-i)))
            setMOSI (1);
        else
            setMOSI (0);

        udelay (10);       // wait till signal stabilizes
        setSCK (1);

        rx = rx << 1;
        if (getMISO ())
            rx |= 0x01;

        udelay (10);
        setSCK (0);
    }

    // short delay for a possible follow-up call;
    // the datasheet requires 55/75ns CLK low phase between bytes !
    udelay (10);
    devStatus = rx;
    return rx;
}

#else

/* this function sets up SPI 1 (SPI & GPIO);
 *   -  PB.3  =  SPI SCLK    -->  O
 *   -  PB.4  =  SPI MISO    <--  i
 *   -  PB.5  =  SPI MOSI    -->  O
 *   -  PB.6  =  GDO0        <--  i
 *   -  PB.7  =  SPI CS      -->  O
 *  PB6 & PB7 (GDO0 + CS) remain GPIO, i.e. manual SW control
 */
void  cc1101_setupPins (void)
{
    uint32_t  temp, mask;

    // enable power to GPIOB and the SPI peripheral
    RCC->IOPENR  |= (1 << RCC_IOPENR_GPIOBEN_Pos);
    RCC->APBENR2 |= (1 << RCC_APBENR2_SPI1EN_Pos);

    // set MODE register bits;
    // the SPI pins are set to 0b10 (alt. function), CS and GDO0 remain GPIOs
    //  PB6 (GDO0) is set to DIN
    temp  = GPIOB->MODER;
    mask  = ~(GPIO_MODER_MODE3_Msk + GPIO_MODER_MODE4_Msk + GPIO_MODER_MODE5_Msk + GPIO_MODER_MODE6_Msk + GPIO_MODER_MODE7_Msk);
    temp &= mask;
    temp |= (GPIO_MODER_MODE3_1 + GPIO_MODER_MODE4_1 + GPIO_MODER_MODE5_1 + GPIO_MODER_MODE7_0); // set 3, 4 and 5 to AF, 7 t output
    GPIOB->MODER = temp;

    // AF mode for SPI pins; all pins (PB3, PB4, PB5) are AF0,
    // i.e. the register default value; this means no need to change anything ...

    // setup the SPI unit; CPOL & CPHA = 0, manual SS, MSB first
    // BR is set to PCLK / 32 (with PCLK = CoreClk / 1 as default value !) = 0b100
    // this equals 48MHz / 32 = 1.5 MHz (max would be 10MHz at most, 6.5MHz for 
    mask  = (SPI_CR1_BIDIOE_Msk + SPI_CR1_MSTR_Msk + SPI_CR1_SSM_Msk + SPI_CR1_SSI_Msk);
//  mask  = (SPI_CR1_MSTR_Msk + SPI_CR1_SSM_Msk + SPI_CR1_SSI_Msk);
    mask |= (SPI_CR1_BR_2);
    SPI1->CR1 = mask;

    // set 8-bit transfers; 8 bits = 0b0111
    mask = (SPI_CR2_DS_0 + SPI_CR2_DS_1 + SPI_CR2_DS_2); 
    SPI1->CR2 = mask;

    // enable the unit
    SPI1->CR1 |= SPI_CR1_SPE_Msk;
}

/* send uint8_t via HW - SPI;
 * does only the SPI byte transfer, slave select handling
 * must be done in the context of the calling routine !
 * 'value'  value to be sent
 * Return:
 *  response received from SPI slave
 */
uint8_t  spi_send (uint8_t tx)
{
    uint8_t   rx  = 0;
    uint8_t  *drb = (uint8_t *) &(SPI1->DR);

    *drb = tx;                           // write Tx byte
//  while (!(SPI1->SR & SPI_SR_RXNE));   // wait till finished (sync'ed. Rx in)
    while ((SPI1->SR & SPI_SR_BSY));     // wait till finished
    rx = *drb;                           // read MISO value

    return (rx);
}

#endif


#define UDELAY_LOOP_CNTR      64
void  udelay (uint32_t n)
{
    volatile uint32_t  cnt;

    do
    {
        for (cnt=UDELAY_LOOP_CNTR; cnt>0; cnt--);
        n--;
    }
    while (n);
}


#if 0
void  delay (uint32_t time)
{
    volatile uint32_t  i;

    do
    {
        for (i=0; i<16; )
            i++;
    }
    while (--time);
}
#endif
