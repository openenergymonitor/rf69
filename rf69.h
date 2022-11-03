/* 
 * Native mode RF69 driver.
 *
 * Derived from the JeeLabs file rf69.h, with some small changes (RSSI)
 *
 * OpenEnergyMonitor version V1.0.0 
 *
 * Note 1: Default transmit power reduced to +7 dBm to avoid damage to RFM if no effective antenna is present.
 * Note 2: This is not related to the JeeLib file RF69.h [RF in capital letters]
 */
#include <util/crc16.h>

#ifndef chThdYield
#define chThdYield() // FIXME should be renamed, ChibiOS leftover
#endif

#ifdef EMONPI2
  #define SSpin PIN_PA7
  #define MOSIpin PIN_PA4
  #define MISOpin PIN_PA5
  #define SCKpin PIN_PA6
#endif
     
#ifdef EMONTX4
  #define SSpin PIN_PB5
  #define MOSIpin PIN_PC0
  #define MISOpin PIN_PC1
  #define SCKpin PIN_PC2
#endif

class RF69 {
  public:
    void init (uint8_t id, uint8_t group, int freq);
    void encrypt (const char* key);
    void txPower (uint8_t level);   // 0 - 31 min -18 dBm, steps of 1 dBm, max = +13 dBm

    int receive (void* ptr, int len);

    void wait_clear();
    void send (uint8_t header, const void* ptr, int len);
    void send_v1 (uint8_t header, const void* ptr, int len);
    void sleep ();

    int16_t afc;
    uint8_t rssi;
    uint8_t lna;
    uint8_t myId;
    uint8_t parity;
    
    void select() {
      digitalWriteFast(SSpin, 0);
    }
    
    void unselect() {
      digitalWriteFast(SSpin, 1);
    }
    
    void spi_init() {
      pinMode(SSpin, OUTPUT);
      pinMode(MOSIpin, OUTPUT);
      pinMode(SCKpin, OUTPUT);
         
      unselect();
      
      #ifdef EMONPI2
      PORTMUX.SPIROUTEA = SPI_MUX | (PORTMUX.SPIROUTEA & (~PORTMUX_SPI0_gm));
      SPI0.CTRLB |= (SPI_SSD_bm);
      SPI0.CTRLA |= (SPI_ENABLE_bm | SPI_MASTER_bm);
      #endif
           
      #ifdef EMONTX4
      PORTMUX.SPIROUTEA = SPI_MUX | (PORTMUX.SPIROUTEA & (~PORTMUX_SPI1_gm));
      SPI1.CTRLB |= (SPI_SSD_bm);
      SPI1.CTRLA |= (SPI_ENABLE_bm | SPI_MASTER_bm);
      #endif
    }
    
    uint8_t spi_transfer (uint8_t data) {
      asm volatile("nop");
      #ifdef EMONPI2
      SPI0.DATA = data;
      while ((SPI0.INTFLAGS & SPI_RXCIF_bm) == 0);  // wait for complete send
      return SPI0.DATA;                             // read data back
      #endif
           
      #ifdef EMONTX4
      SPI1.DATA = data;
      while ((SPI1.INTFLAGS & SPI_RXCIF_bm) == 0);  // wait for complete send
      return SPI1.DATA;                             // read data back
      #endif
    }

    uint8_t readReg (uint8_t addr) {
      select();
      spi_transfer(addr & 0x7F);
      uint8_t regval = spi_transfer(0);
      unselect();
      return regval;
    }
    void writeReg (uint8_t addr, uint8_t value) {
      select();
      spi_transfer(addr | 0x80);
      spi_transfer(value);
      unselect();
    }

  protected:
    enum {
      REG_FIFO          = 0x00,
      REG_OPMODE        = 0x01,
      REG_FRFMSB        = 0x07,
      REG_PALEVEL       = 0x11,
      REG_LNAVALUE      = 0x18,
      REG_AFCMSB        = 0x1F,
      REG_AFCLSB        = 0x20,
      REG_FEIMSB        = 0x21,
      REG_FEILSB        = 0x22,
      REG_RSSIVALUE     = 0x24,
      REG_IRQFLAGS1     = 0x27,
      REG_IRQFLAGS2     = 0x28,
      REG_SYNCVALUE1    = 0x2F,
      REG_SYNCVALUE2    = 0x30,
      REG_NODEADDR      = 0x39,
      REG_BCASTADDR     = 0x3A,
      REG_FIFOTHRESH    = 0x3C,
      REG_PKTCONFIG2    = 0x3D,
      REG_AESKEYMSB     = 0x3E,

      MODE_SLEEP        = 0<<2,
      MODE_STANDBY      = 1<<2,
      MODE_TRANSMIT     = 3<<2,
      MODE_RECEIVE      = 4<<2,

      START_TX          = 0xC2,
      STOP_TX           = 0x42,

      RCCALSTART        = 0x80,
      IRQ1_MODEREADY    = 1<<7,
      IRQ1_RXREADY      = 1<<6,
      IRQ1_SYNADDRMATCH = 1<<0,

      IRQ2_FIFONOTEMPTY = 1<<6,
      IRQ2_PACKETSENT   = 1<<3,
      IRQ2_PAYLOADREADY = 1<<2,
      
      REG_RSSI_CONFIG   = 0x23,
      RSSI_START        = 0x01,
      RSSI_DONE         = 0x02,
      RESTART_RX        = 0x04
      
    };

    void setMode (uint8_t newMode);
    void configure (const uint8_t* p);
    void setFrequency (uint32_t freq);
    
    volatile uint8_t mode;
};

void RF69::setMode (uint8_t newMode) {
  mode = newMode;
  writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | newMode);
  while ((readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0)
    ;
}

void RF69::setFrequency (uint32_t hz) {
  // accept any frequency scale as input, including kHz and MHz
  // multiply by 10 until freq >= 100 MHz (don't specify 0 as input!)
  while (hz < 100000000)
    hz *= 10;

  // Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
  // use multiples of 64 to avoid multi-precision arithmetic, i.e. 3906.25 Hz
  // due to this, the lower 6 bits of the calculated factor will always be 0
  // this is still 4 ppm, i.e. well below the radio's 32 MHz crystal accuracy
  // 868.0 MHz = 0xD90000, 868.3 MHz = 0xD91300, 915.0 MHz = 0xE4C000
  // 434.0 MHz = 0x6C8000.
  uint32_t frf = (hz << 2) / (32000000L >> 11);
  writeReg(REG_FRFMSB, frf >> 10);
  writeReg(REG_FRFMSB+1, frf >> 2);
  writeReg(REG_FRFMSB+2, frf << 6);
}

void RF69::configure (const uint8_t* p) {
  while (true) {
    uint8_t cmd = p[0];
    if (cmd == 0)
      break;
    writeReg(cmd, p[1]);
    p += 2;
  }
}

static const uint8_t configRegs [] = {
// POR value is better for first rf_sleep  0x01, 0x00, // OpMode = sleep
  0x01, 0x04, // OpMode = standby
  0x02, 0x00, // DataModul = packet mode, fsk
  0x03, 0x02, // BitRateMsb, data rate = 49,261 bits/s
  0x04, 0x8A, // BitRateLsb, divider = 32 MHz / 650
  0x05, 0x05, // FdevMsb 90 kHz                                                     rfm69nTxLib: 0x02
  0x06, 0xC3, // FdevLsb 90 kHz                                                     rfm69nTxLib: 0xE1
  0x0B, 0x20, // Low M
  0x11, 0x99, // OutputPower = +7 dBm - was default = max = +13 dBm
  0x19, 0x42, // RxBw 125 kHz                                                       rfm69nTxLib: 0x4A (100kHz)
  //0x1A, 0x42, // AfcBw 125 kHz                                                    rfm69nTxLib: 0x42 (125kHz)
  0x1E, 0x2C, // AfcAutoclearOn, AfcAutoOn                                                       0x80
  //0x25, 0x40, //0x80, // DioMapping1 = SyncAddress (Rx)
  0x26, 0x07, // disable clkout                                                                  
  0x29, 0xA0, // RssiThresh -80 dB                                                               0xC8 (-100 dB)
  0x2D, 0x05, // PreambleSize = 5
  0x2E, 0x88, // SyncConfig = sync on, sync size = 2
  0x2F, 0x2D, // SyncValue1 = 0x2D
  0x37, 0xD0, // PacketConfig1 = variable, white, no filtering
  0x38, 0x42, // PayloadLength = 0, unlimited
  0x3C, 0x8F, // FifoThresh, not empty, level 15
  0x3D, 0x12, // 0x10, // PacketConfig2, interpkt = 1, autorxrestart off
  0x6F, 0x20, // TestDagc ...
  0x71, 0x02, // RegTestAfc
  0
};

void RF69::init (uint8_t id, uint8_t group, int freq) {
  myId = id;

  // b7 = group b7^b5^b3^b1, b6 = group b6^b4^b2^b0
  parity = group ^ (group << 4);
  parity = (parity ^ (parity << 2)) & 0xC0;

  // 10 MHz, i.e. 30 MHz / 3 (or 4 MHz if clock is still at 12 MHz)
  spi_init();
  
  uint32_t start = millis();
  uint8_t timeout = 50;
  do
    writeReg(REG_SYNCVALUE1, 0xAA);
  while (readReg(REG_SYNCVALUE1) != 0xAA && millis()-start < timeout);
  do
    writeReg(REG_SYNCVALUE1, 0x55);
  while (readReg(REG_SYNCVALUE1) != 0x55 && millis()-start < timeout);

  configure(configRegs);
  configure(configRegs);
  
  setFrequency(freq);

  writeReg(REG_SYNCVALUE2, group);
}

void RF69::encrypt (const char* key) {
  uint8_t cfg = readReg(REG_PKTCONFIG2) & ~0x01;
  if (key) {
    for (int i = 0; i < 16; ++i) {
      writeReg(REG_AESKEYMSB + i, *key);
      if (*key != 0)
        ++key;
    }
    cfg |= 0x01;
  }
  writeReg(REG_PKTCONFIG2, cfg);
}

void RF69::txPower (uint8_t level) {
  writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & ~0x1F) | level);
}

void RF69::sleep () {
  setMode(MODE_SLEEP);
}

int RF69::receive (void* ptr, int len) {
  if (mode != MODE_RECEIVE)
    setMode(MODE_RECEIVE);
  else {
    /*
    // Commented out as not being used
    static uint8_t lastFlag;
    if ((readReg(REG_IRQFLAGS1) & IRQ1_RXREADY) != lastFlag) {
      lastFlag ^= IRQ1_RXREADY;
      if (lastFlag) { // flag just went from 0 to 1
        lna = (readReg(REG_LNAVALUE) >> 3) & 0x7;
        rssi = readReg(REG_RSSIVALUE);  // It appears this can report RSSI of the previous packet.
        afc = readReg(REG_AFCMSB) << 8;
        afc |= readReg(REG_AFCLSB);
      }
    }*/

    if (readReg(REG_IRQFLAGS2) & IRQ2_PAYLOADREADY) {
      
      // FIFO access
      select();
      spi_transfer(REG_FIFO & 0x7F);
      int count = spi_transfer(0);
      for (int i = 0; i < count; ++i) {
        uint8_t v = spi_transfer(0);
        if (i < len)
          ((uint8_t*) ptr)[i] = v;
      }
      unselect();

      // only accept packets intended for us, or broadcasts
      // ... or any packet if we're the special catch-all node
      rssi = readReg(REG_RSSIVALUE);   // Duplicated here - RW
      uint8_t dest = *(uint8_t*) ptr;
      if ((dest & 0xC0) == parity) {
        uint8_t destId = dest & 0x3F;
        if (destId == myId || destId == 0 || myId == 63)
          return count;
      }
    }
  }
  return -1;
}

void RF69::wait_clear() {

  unsigned long t_start = millis();
  bool success = true;                                                // return false if timed out, else true
  
  int threshold = -97;                                                // Signal level below which the radio channel is clear to transmit
  uint8_t timeout = 25;                                                // Time in ms to wait for the channel to become clear, before transmitting anyway
  
  if (timeout) {
    success = false;
    setMode(MODE_RECEIVE);
    while ((millis()-t_start)<(unsigned long)timeout)
    {
      while((readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0);
      writeReg(REG_RSSI_CONFIG, RSSI_START);
      while((readReg(REG_RSSI_CONFIG) & RSSI_DONE) == 0x00); 
      
      if (readReg(REG_RSSIVALUE) > (threshold * -2)) {                          // because REG_RSSI_VALUE is upside down!
        success = true;
        break;                                                                  // Nothing heard - go ahead and transmit
      }
      writeReg(REG_PKTCONFIG2, (readReg(REG_PKTCONFIG2) & 0xFB) | RESTART_RX);  // Restart the receiver
    }                                                                           // We have waited long enough - go ahead and transmit anyway
  }
}

void RF69::send (uint8_t header, const void* ptr, int len) {

  wait_clear();

  setMode(MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0x00); // wait for mode ready
  
  // write to FIFO
  select();
  spi_transfer(REG_FIFO | 0x80);
  spi_transfer(len + 2);
  spi_transfer((header & 0x3F) | parity);
  spi_transfer((header & 0xC0) | myId);
  
  for (uint8_t i = 0; i < len; i++)
    spi_transfer(((uint8_t*) ptr)[i]);
  unselect();
  
  setMode(MODE_TRANSMIT);
  while ((readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT) == 0x00); // wait for packet sent
  setMode(MODE_STANDBY);
}

void RF69::send_v1 (uint8_t header, const void* ptr, int len) {

  setMode(MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  
  uint8_t group = 210;
  
  writeReg(0x30, group);
  uint16_t crc = _crc16_update(~0, group);
  
  // write to FIFO
  select();
  spi_transfer(REG_FIFO | 0x80);
  
  spi_transfer(myId & 0x1F);
  crc = _crc16_update(crc, myId & 0x1F);
  
  spi_transfer(len);
  crc = _crc16_update(crc, len);
  
  for (uint8_t i = 0; i < len; i++) {
    spi_transfer(((uint8_t*) ptr)[i]);
    crc = _crc16_update(crc, ((uint8_t*) ptr)[i]);
  }
  spi_transfer((byte)crc);
  spi_transfer((byte)(crc>>8));
  spi_transfer((byte)(crc>>8));
  spi_transfer(0xAA);
  
  unselect();
  
  setMode(MODE_TRANSMIT);
  while ((readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT) == 0x00); // wait for packet sent
  setMode(MODE_STANDBY);
}
