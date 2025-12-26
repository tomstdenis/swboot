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
  - 120..126 ==> means to stop programming
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

  - 4.7k - 10k pullup attached to 1-wire
  - 100nF cap in line with reset pin (- leg attached to PA4)

*/

#define PAGE_LIMIT 119

// pins must be on PORTA
#define PIN_RESET 4           // The reset pin
#define PIN_WIRE  5           // The data wire
#define BRESET (1<<PIN_RESET)
#define BWIRE (1<<PIN_WIRE)

// "flexible" timing
#define OWIRE_SAMPLE 10       // how many uS to wait before sampling, try 12 if your 1-wire line is higher capacitance (not you'd have to also update the boot sketch)
#define SERIAL_BAUD  9600     // baud rate for serial comms, lower values are more friendly for USI/bitbang targets

#define DELAY_US(us) __builtin_avr_delay_cycles((unsigned long)(us) * (F_CPU / 1000000UL))

unsigned char is_reset = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);

  // setup WIRE and RESET pins
  PORTA |= BRESET; // output HIGH on RESET (don't reset target), use pullup on WIRE
  PORTA &= ~BWIRE; // ensure BWIRE will be LOW when changed to an output
  DDRA &= ~BWIRE; // WIRE defaults to input
  DDRA |= BRESET; // RESET defaults to output
}

// read one byte
unsigned char ow_readbyte()
{
  unsigned char x, y;
  cli();
  for (x = y = 0; x < 8; x++) {
    // wait while high
    while (PINA & BWIRE);
    // sample at the mid point
    DELAY_US(OWIRE_SAMPLE);
    y <<= 1;
    y |= (PINA >> PIN_WIRE) & 1;
    // wait for high
    while (!(PINA & BWIRE));
  }
  sei();
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
  cli();
  for (x = 0; x < 8; x++) {
    DDRA |= BWIRE; // toggle to output 
    if (y & 0x80) {
      DELAY_US(5);
      DDRA &= ~BWIRE; // toggle back to input
      DELAY_US(15);
    } else {
      DELAY_US(15);
      DDRA &= ~BWIRE; // toggle back to input
      DELAY_US(5);
    }
    y <<= 1;
  }
  sei();
}

void ow_writebytes(unsigned char *src, unsigned char len)
{
  while (len--) {
    ow_writebyte(*src++);
  }
}

void loop() {
  unsigned char payload[66];

  // block while no input
  while (Serial.available() == 0);

  // read PAGE byte
  payload[0] = Serial.read();

  if (payload[0] <= PAGE_LIMIT) {
    // programming a page, read 65 bytes from UART and then relay
    Serial.readBytes(payload+1, 65);

    // reset if needed AFTER reading from UART so we don't lose payload
    if (!is_reset) {
      // reset the target, blip the RESET pin and output low for 100uS then high
      DDRA |= BWIRE;
      PORTA &= ~BWIRE;  // set WIRE low
      PORTA &= ~BRESET; // set RESET low
      DELAY_US(1000);   // hold low for 1000uS
      PORTA |= BRESET;  // set RESET high
      DELAY_US(1500);   // 1500uS should be long enough for the target to power up and wait for the low pulse
      DDRA &= ~BWIRE;   // reset wire to input HIGH pullup
      PORTA |= BWIRE;
      is_reset = 1;
    }
    ow_writebytes(payload, 66);
    DELAY_US(20); // wait 20uS before reading ACK
    Serial.write(ow_readbyte());
  } else if (payload[0] >= 128) {
    // reading a page, transmit page and then read
    ow_writebyte(payload[0]);
    DELAY_US(20); // wait 20uS before switching roles on 1-wire
    ow_readbytes(payload, 65); // payload + ACK byte
    Serial.write(payload, 65);
  } else if (payload[0] >= 120 && payload[0] <= 126) {
    // we're done programming
    ow_writebyte(payload[0]);
    is_reset = 0;
    DELAY_US(20);
  } else if (payload[0] == 127) {
    Serial.print(F("HELLO"));
  }
}