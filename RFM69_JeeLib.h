/* 
 * Native mode RFM69 driver.
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

#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

class RFM69 {
  public:
    void format(uint8_t radioFormat = 2);
    bool initialize (uint8_t freq, uint8_t id, uint8_t group);
    void send (uint8_t header, const void* buffer, uint8_t bufferSize);
    bool receiveDone();
    void encrypt (const char* key);
    int16_t readRSSI(bool forceTrigger=false);

    uint8_t DATA[62];
    uint8_t RSSI;
    uint8_t DATALEN;
    uint8_t PAYLOADLEN;
    uint8_t TARGETID;
    uint8_t SENDERID;
    
    uint8_t _address;
    uint8_t _parity;
    uint8_t _radioFormat = 2;

  protected:
    enum {
      REG_FIFO                  = 0x00,
      REG_OPMODE                = 0x01,
      REG_FRFMSB                = 0x07,
      REG_PALEVEL               = 0x11,
      REG_RSSIVALUE             = 0x24,
      REG_IRQFLAGS1             = 0x27,
      REG_IRQFLAGS2             = 0x28,
      REG_SYNCVALUE1            = 0x2F,
      REG_SYNCVALUE2            = 0x30,
      REG_PACKETCONFIG2         = 0x3D,
      REG_AESKEY1               = 0x3E,

      RF69_MODE_SLEEP           = 0<<2,
      RF69_MODE_STANDBY         = 1<<2,
      RF69_MODE_TX              = 3<<2,
      RF69_MODE_RX              = 4<<2,
      
      RF_IRQFLAGS1_MODEREADY    = 0x80,
      RF_IRQFLAGS1_RXREADY      = 0x40,

      RF_IRQFLAGS2_PACKETSENT   = 0x08,
      RF_IRQFLAGS2_PAYLOADREADY = 0x04,
      
      REG_RSSICONFIG            = 0x23,
      RF_RSSI_START             = 0x01,
      RF_RSSI_DONE              = 0x02,
      RF_PACKET2_RXRESTART      = 0x04,
      
      RF_PALEVEL_PA0_ON         = 0x80,
      
      RF69_MAX_DATA_LEN         = 61,
      CSMA_LIMIT                = -97,
      RF69_CSMA_LIMIT_MS        = 25
    };

    void spi_init();
    uint8_t spi_transfer (uint8_t data);
    void select();
    void unselect();
    uint8_t readReg (uint8_t addr);
    void writeReg (uint8_t addr, uint8_t value);
            
    void setMode (uint8_t newMode);
    void configure (const uint8_t* p);
    void setFrequency (uint32_t freq);
    void interruptHandler();
    void receiveBegin();

    void wait_clear();
    void sendFrame_v2 (uint8_t header, const void* buffer, uint8_t bufferSize);
    void sendFrame_v1 (uint8_t header, const void* buffer, uint8_t bufferSize);
    
        
    volatile uint8_t _mode;
};

void RFM69::setFrequency (uint32_t hz) {
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

static const uint8_t configRegs_v2 [] = {
// POR value is better for first rf_sleep  0x01, 0x00, // OpMode = sleep
  0x01, 0x04, // RegOpMode: Sequencer on, listen off, standby mode
  0x02, 0x00, // RegDataModul: Packet mode, FSK modulation, no modulation shaping
  0x03, 0x02, // BitRateMsb, 0x02 << 8 = 512
  0x04, 0x8A, // BitRateLsb, 0x8A      = 138, (512+138)=650, 32 MHz / 650 = 49,230 bits/s
  0x05, 0x05, // RegFdevMsb, 0x05 << 8 = 1280
  0x06, 0xC3, // RegFdevLsb, 0xC3 << 0 = 195, (1280+195)=1475, 1475 x FSTEP (61Hz) = 90 kHz (should probably be half this!?)
  
  0x0B, 0x20, // RegAfcCtrl, Improved AFC routine (default is standard 0x00)
  0x11, 0x99, // OutputPower = +7 dBm - was default = max = +13 dBm
  0x19, 0x42, // RegRxBw 32000000÷(16×2^(2+2)) = 125 kHz  but could be set to 100 kHz 0x4a                                                  rfm69nTxLib: 0x4A (100kHz)
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

static const uint8_t configRegs_v1 [] = {
  0x01, 0x04, // RegOpMode: Sequencer on, listen off, standby mode
  0x02, 0x00, // RegDataModul: Packet mode, FSK modulation, no modulation shaping
  0x03, 0x02, // BitRateMsb, 0x02 << 8 = 512
  0x04, 0x8A, // BitRateLsb, 0x8A      = 138, (512+138)=650, 32 MHz / 650 = 49,230 bits/s
  0x05, 0x05, // RegFdevMsb, 0x05 << 8 = 1280
  0x06, 0xC3, // RegFdevLsb, 0xC3 << 0 = 195, (1280+195)=1475, 1475 x FSTEP (61Hz) = 90 kHz (should probably be half this!?)
  
  0x11, 0x99, // OutputPower = +7 dBm - was default = max = +13 dBm
  0x1E, 0x2C, // AfcAutoclearOn, AfcAutoOn
  0x25, 0x80,
  0x26, 0x03, // disable clkout  
  0x28, 0x00,                                                                
  0x2E, 0x88, // SyncConfig = sync on, sync size = 2
  0x2F, 0x2D, // SyncValue1 = 0x2D
  0x37, 0x00, // PacketConfig1 = variable, white, no filtering
  0
};

void RFM69::format (uint8_t radioFormat) {
  _radioFormat = radioFormat;
}

bool RFM69::initialize (uint8_t freq, uint8_t ID, uint8_t group) {
  _address = ID;

  // b7 = group b7^b5^b3^b1, b6 = group b6^b4^b2^b0
  _parity = group ^ (group << 4);
  _parity = (_parity ^ (_parity << 2)) & 0xC0;

  // 10 MHz, i.e. 30 MHz / 3 (or 4 MHz if clock is still at 12 MHz)
  spi_init();
  
  uint32_t start = millis();
  uint8_t timeout = 50;
  do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1) != 0xaa && millis()-start < timeout);
  start = millis();
  do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55 && millis()-start < timeout);

  if (_radioFormat==1) {
    configure(configRegs_v1);
    configure(configRegs_v1);
  } else if (_radioFormat==2) {
    configure(configRegs_v2);
    configure(configRegs_v2); 
  }
  
  setFrequency(434);
  writeReg(REG_SYNCVALUE2, group);
  
  uint8_t powerLevel = 25; // could be up to 31... -18 + dBm with PA0 = 7dBm
  if (powerLevel>31) powerLevel = 31;
  writeReg(0x11,RF_PALEVEL_PA0_ON | powerLevel); 

  setMode(RF69_MODE_STANDBY);
  start = millis();
  while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis()-start < timeout); // wait for ModeReady
  if (millis()-start >= timeout)
    return false;
  
  return true;
}

void RFM69::setMode (uint8_t newMode) {

  if (newMode == _mode)
    return;
  
  writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | newMode);

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
    
  _mode = newMode;
}

void RFM69::interruptHandler() {
  if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    setMode(RF69_MODE_STANDBY);
    select();
    spi_transfer(REG_FIFO & 0x7F);
    PAYLOADLEN = spi_transfer(0);
    PAYLOADLEN = PAYLOADLEN > 66 ? 66 : PAYLOADLEN; // precaution
    uint8_t dest = spi_transfer(0);
    if ((dest & 0xC0) != _parity) {
      unselect();
      receiveBegin();
      return;
    }
    TARGETID = dest & 0x3F;
    if (TARGETID != _address && TARGETID != 0 && _address != 63) {
      unselect();
      receiveBegin();
      return;
    }
    SENDERID = spi_transfer(0);
    DATALEN = PAYLOADLEN - 3;
    for (uint8_t i = 0; i < DATALEN; i++) DATA[i] = spi_transfer(0);
    DATA[DATALEN] = 0; // add null at end of string // add null at end of string
    unselect();
    setMode(RF69_MODE_RX);
    RSSI = readRSSI();
  }
}

void RFM69::receiveBegin() {
  DATALEN = 0;
  PAYLOADLEN = 0;
  RSSI = 0;
  SENDERID = 0;
  TARGETID = 0;
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  // writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
  setMode(RF69_MODE_RX);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
bool RFM69::receiveDone() {
  interruptHandler();
  
  if (_mode == RF69_MODE_RX && PAYLOADLEN > 0)
  {
    setMode(RF69_MODE_STANDBY); // enables interrupts
    return true;
  }
  else if (_mode == RF69_MODE_RX) // already in RX no payload yet
  {
    return false;
  }
  receiveBegin();
  return false;
}

void RFM69::wait_clear() {
  if (RF69_CSMA_LIMIT_MS > 0){
    setMode(RF69_MODE_RX);
    uint32_t now = millis();
    while ((millis()-now) < RF69_CSMA_LIMIT_MS) {
      while((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0);
      if (readRSSI() < CSMA_LIMIT) break;
      writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART);  // Restart the receiver
    }
  }
}

void RFM69::send (uint8_t header, const void* buffer, uint8_t bufferSize) {
  
  wait_clear();
  
  if (_radioFormat==1) {
    sendFrame_v1(header,buffer,bufferSize);
  } else if (_radioFormat==2) {
    sendFrame_v2(header,buffer,bufferSize);
  }
}

void RFM69::sendFrame_v2 (uint8_t header, const void* buffer, uint8_t bufferSize) {
   
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;
  
  // write to FIFO
  select();
  spi_transfer(REG_FIFO | 0x80);
  spi_transfer(bufferSize + 2);
  spi_transfer((header & 0x3F) | _parity);
  spi_transfer((header & 0xC0) | _address);
  
  for (uint8_t i = 0; i < bufferSize; i++)
    spi_transfer(((uint8_t*) buffer)[i]);
  unselect();
  
  setMode(RF69_MODE_TX);
  while ((readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0x00); // wait for packet sent
  setMode(RF69_MODE_STANDBY);
}

void RFM69::sendFrame_v1 (uint8_t header, const void* buffer, uint8_t bufferSize) {
      
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

  uint8_t group = 210;  
  writeReg(0x30, group);
  uint16_t crc = _crc16_update(~0, group);
  
  // write to FIFO
  select();
  spi_transfer(REG_FIFO | 0x80);
  
  spi_transfer(_address & 0x1F);
  crc = _crc16_update(crc, _address & 0x1F);
  
  spi_transfer(bufferSize);
  crc = _crc16_update(crc, bufferSize);
  
  for (uint8_t i = 0; i < bufferSize; i++) {
    spi_transfer(((uint8_t*) buffer)[i]);
    crc = _crc16_update(crc, ((uint8_t*) buffer)[i]);
  }
  spi_transfer((byte)crc);
  spi_transfer((byte)(crc>>8));
  spi_transfer((byte)(crc>>8));
  spi_transfer(0xAA);
  
  unselect();
  
  setMode(RF69_MODE_TX);
  while ((readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0x00); // wait for packet sent
  setMode(RF69_MODE_STANDBY);
}

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void RFM69::encrypt (const char* key) {
  if (_radioFormat==1) return;

  setMode(RF69_MODE_STANDBY);
  uint8_t validKey = key != 0 && strlen(key)!=0;
  if (validKey)
  {
    select();
    spi_transfer(REG_AESKEY1 | 0x80);
    for (uint8_t i = 0; i < 16; i++)
      spi_transfer(key[i]);
    unselect();
  }
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFE) | (validKey ? 1 : 0));
}

// get the received signal strength indicator (RSSI)
int16_t RFM69::readRSSI(bool forceTrigger) {
  int16_t rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
  }
  rssi = -readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

uint8_t RFM69::readReg(uint8_t addr)
{
  select();
  spi_transfer(addr & 0x7F);
  uint8_t regval = spi_transfer(0);
  unselect();
  return regval;
}

void RFM69::writeReg(uint8_t addr, uint8_t value)
{
  select();
  spi_transfer(addr | 0x80);
  spi_transfer(value);
  unselect();
}

void RFM69::select() {
  digitalWriteFast(SSpin, 0);
}

void RFM69::unselect() {
  digitalWriteFast(SSpin, 1);
}

void RFM69::spi_init() {
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

uint8_t RFM69::spi_transfer (uint8_t data) {
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
  
  return 0;
}

void RFM69::configure (const uint8_t* p) {
  while (true) {
    uint8_t cmd = p[0];
    if (cmd == 0)
      break;
    writeReg(cmd, p[1]);
    p += 2;
  }
}
