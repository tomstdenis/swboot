/*

UART to 1-wire translator for swboot programming

This sketch receives byte commands over serial and then relays them
over the 1-wire protocol to the target being programmed.  It uses
a scheme similar to dalas 1-wire where:

0-bit: 15uS LOW, 5uS HIGH
1-bit: 5uS LOW, 15uS HIGH

loop:
  wait for LOW
  sample at 10uS
  wait for HIGH

The serial commands are basically of the format

  - A -> T PAGE address (1 byte)
  - A -> T PAYLOAD (0 or 65 bytes)
  - T -> A PAYLOAD (0 or 64 bytes)
  - T -> A ACK byte (0x54 good, 0x83 bad)

PAGE:
  - 0..119 ==> program page (top 512-bytes are reserved), A must send 65 byte payload, T must return ACK
  - 120..127 ==> means to stop programming
  - 128..255 ==> read page (PAGE-128) (T must return 64 bytes + ACK) (can read bootload back if wanted...)

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


#define PIN_RESET 4
#define PIN_WIRE  5
#define BRESET (1<<PIN_RESET)
#define BWIRE (1<<PIN_WIRE)
#define DELAY_US(us) __builtin_avr_delay_cycles((unsigned long)(us) * (F_CPU / 1000000UL))

unsigned char is_reset = 0;

void setup() {
  Serial.begin(9600); // simple low baud to handle (handy if you're bitbanging)

  // setup PA4/PA5 
  PORTA |= BRESET | BWIRE; // output HIGH on RESET (don't reset target), use pullup on WIRE
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
    DELAY_US(10);
    y |= (PINA >> PIN_WIRE) & 1;
    y <<= 1;
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
    PORTB &= ~BWIRE;
    DDRB |= BWIRE; // low pulse
    if (y & 0x80) {
      DELAY_US(5);
      DDRB &= ~BWIRE;
      PORTB |= BWIRE; // set high input
      DELAY_US(15);
    } else {
      DELAY_US(15);
      DDRB &= ~BWIRE;
      PORTB |= BWIRE; // set high input
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
      PORTA &= ~BWIRE; // set WIRE low
      PORTA &= ~BRESET; // set RESET low
      DELAY_US(1);
      PORTA |= BRESET; // set RESET high
      DELAY_US(100);
      DDRA &= ~BWIRE;  // reset wire to input HIGH pullup
      PORTA |= BWIRE;
      is_reset = 1;
    }
    ow_writebytes(payload+1, 65);
  } else if (payload[0] >= 128) {
    // reading a page, transmit page and then read
    ow_writebyte(payload[0]);
    DELAY_US(20);
    ow_readbytes(payload+1, 64);
  } else if (payload[0] >= 120 && payload[0] <= 127) {
    // we're done programming
    ow_writebyte(payload[0]);
    is_reset = 0;
    DELAY_US(20);
  }

  // if is_reset != 0 we should have an ACK byte
  if (!is_reset) {
    DELAY_US(20);
    Serial.write(ow_readbyte());
  }
}