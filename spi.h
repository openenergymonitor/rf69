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
  static uint8_t spiTransferByte (uint8_t out) {
#ifdef SPCR
    SPDR = out;
    while ((SPSR & (1<<SPIF)) == 0)
      ;
    return SPDR;
#else
    // ATtiny
    USIDR = out;
    byte v1 = bit(USIWM0) | bit(USITC);
    byte v2 = bit(USIWM0) | bit(USITC) | bit(USICLK);
    for (uint8_t i = 0; i < 8; ++i) {
      USICR = v1;
      USICR = v2;
    }
    return USIDR;
#endif
  }

public:
  static void master (int div) {
    digitalWrite(N, 1);
    pinMode(N, OUTPUT);

#ifdef SPCR
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, INPUT);
    pinMode(13, OUTPUT);

    SPCR = _BV(SPE) | _BV(MSTR);
    SPSR |= _BV(SPI2X);
#else
    // ATtiny
    pinMode(1, OUTPUT); // SS
    pinMode(4, INPUT);  // MISO 7
    pinMode(5, OUTPUT); // MOSI 8
    pinMode(6, OUTPUT); // SCK 9

    USICR = bit(USIWM0);
#endif
  }

  static uint8_t rwReg (uint8_t cmd, uint8_t val) {
    digitalWrite(N, 0);
    spiTransferByte(cmd);
    uint8_t in = spiTransferByte(val);
    digitalWrite(N, 1);
    return in;
  }
};

#ifdef SPCR
typedef SpiDev<10> SpiDev10;
#else
// ATtiny
typedef SpiDev<1> SpiDev1;
#endif
