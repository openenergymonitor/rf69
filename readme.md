# RF69 Minimal JeeLib library (AVR-DB)

1. Supports AVD-DB microcontrollers only

2. Supports transmitting data using the classic JeeLib packet format used by OpenEnergyMonitor hardware up to Nov 2022.

3. Supports transmitting and receiving data using the native JeeLabs packet format. This makes use of the in-built capabilities of the RFM69 module rather than emulating the RFM12 module via the compatibility mode provided by the standard JeeLib packet format. 

Note on history: This library was originally based on a patched version of the JeeLib native rf69.h driver, adapted from original by Robert Wall. The latest version is based more closely on the LowPowerLabs RFM69 library implementation, while keeping the packet format the same.

## Transmitter example

    #define Serial Serial3
    #define EMONTX4
    #define RFM69_JEELIB_CLASSIC 1
    #define RFM69_JEELIB_NATIVE 2

    #include <RFM69_JeeLib.h>

    RFM69 radio;

    typedef struct {
        unsigned long Msg;
        int Vrms,P1,P2,P3,P4,P5,P6; 
        long E1,E2,E3,E4,E5,E6; 
        int T1,T2,T3;
        unsigned long pulse;
    } PayloadTX;
    PayloadTX emontx;

    void setup() {
      Serial.begin(115200);
      Serial.println("JeeLib Native TX");

      radio.format(RFM69_JEELIB_NATIVE);
      radio.initialize(RF69_433MHZ,15,210);
      radio.encrypt("89txbe4p8aik5kt3");

      emontx.Msg = 0;
    }

    void loop() {
      emontx.Msg++;
      radio.send(0, (byte *)&emontx, sizeof(emontx));
      delay(2000);
    }

## Receiver example

    #define Serial Serial3
    #define EMONTX4
    #include <RFM69_JeeLib.h>

    RFM69 radio;

    void setup() {
      Serial.begin(115200);
      Serial.println("JeeLib Native RX");
      radio.initialize(RF69_433MHZ,5,210);
      radio.encrypt("89txbe4p8aik5kt3");
    }

    void loop() {
      if (radio.receiveDone())
      {
        Serial.print(F("OK")); 
        Serial.print(F(" "));
        Serial.print(radio.SENDERID, DEC);
        Serial.print(F(" "));
        for (byte i = 0; i < radio.DATALEN; i++) {
          Serial.print((word)radio.DATA[i]);
          Serial.print(F(" "));
        }
        Serial.print(F("("));
        Serial.print(radio.readRSSI());
        Serial.print(F(")"));
        Serial.println();
      }
    }

## API

**void format (uint8_t format);**<br>
This sets the transmit format to either JeeLib classic = 1 or JeeLib native = 2. Note that this library can only receive data sent using the JeeLib native format. This must be called before initialize. There is no return value.

**void initialize (int freq, uint8_t id, uint8_t group);**<br>
Initialises the driver with the NodeID, Group & Frequency. Node ID range is 1 – 60, 61 is
send-only, 62 is reserved, 63 is receive-all. The OEM default group is 210. Frequency is in
MHz e.g 434 = 434 MHz. There is no return value.

**void encrypt (const char\* key);**<br>
Encrypts the data with a 16 character (max) / 128-bit key. If encryption is used, it must be
used with the same key in all nodes in the same group. Encryption can be disabled with a
null key:

    rf.encrypt(0);

**void send (uint8_t header, const void\* buffer, uint8_t bufferSize);**<br>
Transmit the data of length len in the buffer pointed to by ptr. The data must not exceed
62 bytes. Header is not normally required and should be zero. There is no return value.

**int receive (void\* ptr, int len);**<br>
Returns the actual size of the received packet (0 – 62 bytes). The caller-supplied receive
buffer pointed to by ptr (of size len) must be large enough to hold the entire packet.



**void txPower (uint8_t level);**<br>
Sets the transmit power level. Level ranges from 0 – 31 representing -18 dBm to + 13
dBm in steps of 1 dBm. The default is 25 (+7 dBm) to avoid damage to the RFM69 if no
antenna is fitted. There is no return value.

**uint8_t rssi;**<br>
The received signal strength of the latest message, in steps of 0.5 dB below the reference
level. i.e. to convert to the normal representation:

    rfInfo.rssi = -rf.rssi/2;



**void sleep ();**<br>
Puts the radio module into sleep mode. There is no return value.

Some other variables are available – refer to the source code for details.




This driver is a patched version of the JeeLabs rf69.h driver. The changes are:

1. The default transmit power has been lowered to +7 dBm in order to avoid damage
to the radio module in the event that the transmitter ip operated without an efficient
antenna.

2. The point where the RSSI value is extracted has been moved because in the
original position, the RSSI value of the previous message could occasionally be
reported.
