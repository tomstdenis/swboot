/*

UART to 1-wire translator for swboot programming

This sketch sits between the host(PC) and target.  It receives byte commands over serial
and then relays them over the 1-wire protocol to the target being programmed.  It uses
a scheme similar to dalas 1-wire where:

0-bit: 15uS LOW, 5uS HIGH
1-bit: 5uS LOW, 15uS HIGH

loop:
  wait for LOW
  sample at 10uS
  wait for HIGH

Commands begin with a PAGE byte

H = host (your PC)
A = This adapter sketch
T = The target (programmed with the boot sketch at 0x1E00)

PAGE:
  - 0..119 ==> program page (top 512-bytes are reserved), H transmits 65 byte page payload, A relays to T, T responds with ACK byte
  - 120 ==> means to stop programming
  - 121 ==> Power control (mostly for mosfet based designs)
  - 127 ==> A should send "HELLO" to H
  - 128..255 ==> read page (PAGE-128) (T sends 64 byte page + ACK to A, A relays to H)

To support USI/bitbang devices the protocol assumes half-duplex meaning if the UART is busy the 1-wire is not
and vice versa.  This allows us to do either (but especially the 1-wire) in a cli/sei wrapped tight loop.

When using an ATTiny84 as the adapter:

  - PA1 - UART TX
  - PA2 - UART RX (using ATTinyCore's Serial class)
  - PA4 - Target reset
  - PA5 - 1-wire

Ideal wiring is:

  - 470 to 1000 ohm pullup attached to 1-wire

Right now in the breadboard I'm using the reset pin to get the target into the bootloader.
Eventually I plan actually use a P-CH mosfet to just turn the target off, this gives me far more
control over what's going on since every programming cycle will result in the device coming into a cold power up.

I also plan to put a (source-source) pair of P-CH on the wire line so A can turn off the wire when programming is done to
remove it much as possible from the targets circuit.

Finally I also plan to put a ~200 Ohm resistor in series A's data wire so that if T shorts to Vcc (outputs high on it) it won't fry A's pin

*/
#include "swboot.h"

#define PAGE_LIMIT 119

// MOSFET based support (not yet done)
//#define USE_MOSFETS

// pins ...
#if 0
// tiny84
#define PIN_PORT PORTA
#define PIN_DDR  DDRA
#define PIN_PIN  PINA
#define PIN_RESET 4           // The reset pin
#define PIN_WIRE  5           // The data wire
#else
// mega32u4 uses MISO/MOSI pins
#define PIN_PORT PORTB
#define PIN_DDR  DDRB
#define PIN_PIN  PINB
#ifndef USE_MOSFETS
#define PIN_RESET 3           // The reset pin
#endif
#define PIN_WIRE  2 // MOSI
#ifdef USE_MOSFETS
#define PIN_PWREN 14 // TBD (MISO)
#define PIN_DATAEN 15 // TBD (SCLK)
#endif
#endif

#ifndef USE_MOSFETS
#define BRESET (1<<PIN_RESET)
#endif
#define BWIRE (1<<PIN_WIRE)
#define SERIAL_BAUD  115200UL     // baud rate for serial comms, lower values are more friendly for USI/bitbang targets
#define DELAY_US(us) __builtin_avr_delay_cycles((unsigned long)(us) * (F_CPU / 1000000UL))

unsigned char is_reset = 0;

// read one byte
unsigned char ow_readbyte()
{
  unsigned char x, y;
  unsigned long z;
  for (x = y = 0; x < 8; x++) {
    // wait for low
    z = 0;
    while (PIN_PIN & BWIRE) if (!(++z & 0x3FFFFF)) return 1; // eventually timeout...
    // sample at the mid point
    DELAY_US(PULSE_MID);
    y <<= 1;
    y |= (PIN_PIN >> PIN_WIRE) & 1;
    // wait for high
    z = 0;
    while (!(PIN_PIN & BWIRE)) if (!(++z & 0x3FFFFF)) return 2;
  }
  return y;
}

void ow_readbytes(unsigned char *dst, unsigned char len)
{
  while (len--) {
    *dst++ = ow_readbyte();
  }
}

void ow_writebyte(unsigned char y)
{
  unsigned char x;
  DELAY_US(PULSE_A);
  for (x = 0; x < 8; x++) {
    unsigned char wa, wb;

    // determine pulse timing
    if (y & 0x80) {
      wa = SIMP_US_TO_LOOPS(PULSE_A);
      wb = SIMP_US_TO_LOOPS(PULSE_B);
    } else {
      wa = SIMP_US_TO_LOOPS(PULSE_B);
      wb = SIMP_US_TO_LOOPS(PULSE_A);
    }

    // set output direction and lowUSING_X5
    PIN_DDR |= BWIRE;
    SIMP_DELAY_US(wa);

    // set as input (high)
    PIN_DDR &= ~BWIRE;
    SIMP_DELAY_US(wb);

    y <<= 1;
  }
}

void ow_writebytes(unsigned char *src, unsigned char len)
{
  while (len--) {
    ow_writebyte(*src++);
  }
}


void setup() {
  Serial.begin(SERIAL_BAUD);
#ifndef USE_MOSFETS
  // setup WIRE and RESET pins
  PIN_PORT |= BRESET; // output HIGH on RESET (don't reset target), use pullup on WIRE
  PIN_DDR |= BRESET; // RESET defaults to output
  PIN_PORT &= ~BWIRE; // ensure BWIRE will be LOW when changed to an output
  PIN_DDR &= ~BWIRE; // WIRE defaults to input
#else
  // setup WIRE and RESET pins
  PIN_PORT &= ~BWIRE; // ensure BWIRE will be LOW when changed to an output
  PIN_DDR &= ~BWIRE; // WIRE defaults to input
  pinMode(PIN_PWREN, OUTPUT);
  digitalWrite(PIN_PWREN, HIGH);
  pinMode(PIN_DATAEN, OUTPUT);
  digitalWrite(PIN_DATAEN, HIGH);
#endif
}

#ifndef USE_MOSFETS
void target_power(int on)
{
  if (on) {
    // disable reset
    PIN_PORT |= BRESET;
  } else {
    // set reset LOW which puts the target into reset
    PIN_PORT &= ~BRESET;
  }
}

void target_data_ch(int on)
{
  // nop in the mosfet-less design
}
#else
void target_power(int on)
{
  if (on) {
    // enable P-CH
    digitalWrite(PIN_PWREN, LOW);
  } else {
    // disable P-CH
    digitalWrite(PIN_PWREN, HIGH);
  }
}

void target_data_ch(int on)
{
  if (on) {
    // enable P-CH
    digitalWrite(PIN_DATAEN, LOW);
  } else {
    // disable P-CH
    digitalWrite(PIN_DATAEN, HIGH);
  }
}
#endif

void do_reset()
{
  unsigned char x, y;
  target_data_ch(1); // turn on data channel
  do {
top:
    // reset the target
    PIN_DDR |= BWIRE;       // set WIRE as output should put line low
    target_power(0);        // turn off/reset
    DELAY_US(1000UL * 350);  // hold reset for 250ms
    target_power(1);        // turn on/out of reset
    DELAY_US(1000UL * 150);  // wait 150ms for it to power up
    PIN_DDR &= ~BWIRE;      // reset wire to input, line should go high

    // now sample 100us for low
    for (x = y = 0; x < 100; x++) {
      if (!(PIN_PIN & BWIRE)) {
        ++y;
      }
      DELAY_US(1);
    }

    // we expect at least 25% of the cycle
    if (y < 25) {
      continue;
    }

    // wait for line to go high with timeout (10ms)
    // the previous task from the target was 0.1ms long so a 10ms timeout is more than generous
    x = 0;
    while (!(PIN_PIN & BWIRE)) {
      DELAY_US(1000);
      if (++x == 11) {
        goto top;
      }
    }

    // try to send the page == 127 to recv 'H' back
    DELAY_US(PULSE_LENGTH);
    ow_writebyte(127);
    x = ow_readbyte();
  } while (x != 0x54);
  is_reset = 1;
}

void loop() {
  static unsigned char page_size = 64;
  unsigned char payload[66];

  // block while no input
  if (Serial.available() == 0) {
    return;
  }

  // read PAGE byte
  payload[0] = Serial.read();

  if (payload[0] <= PAGE_LIMIT) { // write a page
    // programming a page, read PAGE+1 bytes from UART and then relay
    Serial.readBytes(payload+1, page_size+1);

    // reset if needed AFTER reading from UART so we don't lose payload
    if (!is_reset) {
      do_reset();
    }
    ow_writebytes(payload, page_size+2); // PAGE command, payload, checksum == page_size + 2 bytes
    DELAY_US(PULSE_A); // pause the short pulse waiting for the other side to get ready to send
    Serial.write(ow_readbyte());
    DELAY_US(10000); // pause 10ms between page writes
  } else if (payload[0] >= 128) { // read a page
    // reading a page, transmit page and then read
    // reset if needed AFTER reading from UART so we don't lose payload
    if (!is_reset) {
      do_reset();
    }
    ow_writebytes(payload, page_size+2);
    DELAY_US(PULSE_A); // pause the short pulse waiting for the other side to get ready to send
    ow_readbytes(payload, page_size + 1); // payload + ACK byte
    Serial.write(payload, page_size + 1);
  } else if (payload[0] == 121) { // power and channel control
    // power control
    payload[0] = Serial.read();
    if (payload[0] & 2) {
      target_power(payload[0] & 1);
    }
    if (payload[0] & 8) {
      target_data_ch(payload[0] & 4);
    }
    Serial.write(0x54); // ACK
  } else if (payload[0] == 122) { // write/read 1-wire back to back
    unsigned char x = Serial.read() & (page_size - 1);
    unsigned char y = Serial.read() & (page_size - 1);
    if (x) {
      Serial.readBytes(payload, x);
      ow_writebytes(payload, x);
      DELAY_US(PULSE_A);
    }
    if (y) {
      ow_readbytes(payload, y);
      Serial.write(payload, y);
    }
  } else if (payload[0] == 124) { // EEPROM READ
    if (!is_reset) {
      do_reset();
    }
    Serial.readBytes(payload+1, 2); // EEPROM ADDRH, ADDRL
    ow_writebytes(payload, page_size + 2);
    DELAY_US(PULSE_A);
    ow_readbytes(payload, 2); // ack + value
    Serial.write(payload, 2);
  } else if (payload[0] == 125) { // EEPROM WRITE
    if (!is_reset) {
      do_reset();
    }
    Serial.readBytes(payload+1, 3); // 3 bytes ADDRH, ADDRL,   VAL
    ow_writebytes(payload, page_size + 2);
    DELAY_US(PULSE_A);
    Serial.write(ow_readbyte()); // echo back ACK byte
    DELAY_US(5000); // wait for EEPROM write to finish
  } else if (payload[0] == 126) {
    unsigned char x;
    if (!is_reset) {
      do_reset();
    }
    ow_writebyte(payload[0]);
    DELAY_US(PULSE_A);
    x = ow_readbyte();
    switch (x) {
      case 0x84:
      case 0x85: page_size = 64; break;
      case 0x44:
      case 0x45: page_size = 64; break;
      case 0x24:
      case 0x25: page_size = 32; break;
    }
    Serial.write(x);
  } else if (payload[0] == 127) {
    // Init packet to check H - A connection
    Serial.print(F("HELLO"));
    is_reset = 0; // go back to assuming the device needs a reset
    target_data_ch(0); // turn off data channel
  } else {
    // we're done programming
    if (!is_reset) {
      do_reset();
    }
    payload[0] = 120;
    ow_writebytes(payload, page_size + 2); // PAGE=120 means to launch user app
    is_reset = 0;
    target_data_ch(0); // turn off data channel
  }
}