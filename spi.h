// SPI setup for ATmega (JeeNode) and ATtiny (JeeNode Micro)
// ATtiny thx to @woelfs, see http://jeelabs.net/boards/11/topics/6493

/*
 SPCR: SPI Control Register
 Bit 7: SPIE - Enables the SPI interrupt when 1
 Bit 6: SPE  - Enables the SPI when 1
 Bit 5: DORD - Sends data least Significant Bit First when 1, most Significant Bit first when 0
 Bit 4: MSTR - Sets the Arduino in master mode when 1, slave mode when 0
 Bit 3: CPOL - Sets the data clock to be idle when high if set to 1, idle when low if set to 0
 Bit 2: CPHA - Samples data on the falling edge of the data clock when 1, rising edge when 0
 Bits 1-0: SPR1 and SPR0 - Sets the SPI speed, 00 is fastest (4MHz) 11 is slowest (250KHz)

 SPSR: SPI Status register
 SPIF: SPI Interrupt Flag
 
 */

template< int N>
class SpiDev {
  static uint8_t spiTransferByte (uint8_t data) {
    asm volatile("nop");

    SPI0.DATA = data;
    while ((SPI0.INTFLAGS & SPI_RXCIF_bm) == 0);  // wait for complete send
    return SPI0.DATA;                             // read data back
  }

public:
  static void master (int div) {
    digitalWriteFast(PIN_PA7, 1);
    pinMode(PIN_PA7, OUTPUT);

    // SPI.pins(PIN_PA4, PIN_PA5, PIN_PA6);
    
    PORTMUX.SPIROUTEA = SPI_MUX | (PORTMUX.SPIROUTEA & (~PORTMUX_SPI0_gm));

    pinMode(PIN_PA6, OUTPUT); // SCK
    pinMode(PIN_PA4, OUTPUT); // MOSI

    SPI0.CTRLB |= (SPI_SSD_bm);
    SPI0.CTRLA |= (SPI_ENABLE_bm | SPI_MASTER_bm);
  }

  static uint8_t rwReg (uint8_t cmd, uint8_t val) {
    digitalWriteFast(PIN_PA7, 0);
    spiTransferByte(cmd);
    uint8_t in = spiTransferByte(val);
    digitalWriteFast(PIN_PA7, 1);
    return in;
  }
};

typedef SpiDev<10> SpiDev10;

