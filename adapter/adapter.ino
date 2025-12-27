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

Right now in the breadboard I'm using the reset pin to get the target into the bootloader.
Eventually I plan actually use a P-CH mosfet to just turn the target off, this gives me far more
control over what's going on since every programming cycle will result in the device coming into a cold power up.

I also plan to put a (source-source) pair of P-CH on the wire line so A can turn off the wire when programming is done to
remove it much as possible from the targets circuit.

Finally I also plan to put a ~200 Ohm resistor in series A's data wire so that if T shorts to Vcc (outputs high on it) it won't fry A's pin

*/

#define PAGE_LIMIT 119

// REALLY slow...(use this if your line has high capacitance or a weak pullup, uses a 80uS period)
#define REALLY_SLOW_PULSE

// use SLOW_PULSE (40uS) if your target doesn't have an external clock
//#define SLOW_PULSE

#ifdef REALLY_SLOW_PULSE
// 80uS timebase
#define PULSE_A 20
#define PULSE_B 60
#elif defined(SLOW_PULSE)
// 40uS timebase
#define PULSE_A 10
#define PULSE_B 30
#else
// 20us timebase
#define PULSE_A 5
#define PULSE_B 15
#endif

#define PULSE_MID ((PULSE_A+PULSE_B)/2)

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
#define PIN_RESET 3 // MISO
#define PIN_WIRE  2 // MOSI
#endif

#define BRESET (1<<PIN_RESET)
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
    while (PIN_PIN & BWIRE) if (!(++z & 0xFFFFFFF)) return 1; // 0x1000000 loops at 1 cycle every 1/16th of a uS is a bit over 1 second
    // sample at the mid point
    DELAY_US(PULSE_MID);
    y <<= 1;
    y |= (PIN_PIN >> PIN_WIRE) & 1;
    // wait for high
    z = 0;
    while (!(PIN_PIN & BWIRE)) if (!(++z & 0xFFFFFFF)) return 2;
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
    PIN_DDR |= BWIRE; // toggle to output 
    if (y & 0x80) {
      DELAY_US(PULSE_A);
      PIN_DDR &= ~BWIRE; // toggle back to input
      DELAY_US(PULSE_B);
    } else {
      DELAY_US(PULSE_B);
      PIN_DDR &= ~BWIRE; // toggle back to input
      DELAY_US(PULSE_A);
    }
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

  // setup WIRE and RESET pins
  PIN_PORT |= BRESET; // output HIGH on RESET (don't reset target), use pullup on WIRE
  PIN_PORT &= ~BWIRE; // ensure BWIRE will be LOW when changed to an output
  PIN_DDR &= ~BWIRE; // WIRE defaults to input
  PIN_DDR |= BRESET; // RESET defaults to output
}

void do_reset()
{
  unsigned char x, y;
  do {
top:
    // reset the target
    PIN_DDR |= BWIRE;       // set WIRE as output should put line low
    PIN_PORT &= ~BRESET;    // set RESET low (halts the target)
    DELAY_US(1000UL * 250);  // hold reset for 250ms
    PIN_PORT |= BRESET;     // set RESET high (reboots the target)
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
    DELAY_US(PULSE_A+PULSE_B);
    ow_writebyte(127);
    //DELAY_US(PULSE_A); // pause the short pulse waiting for the other side to get ready to send
    x = ow_readbyte();
//    Serial.write(x);
  } while (x != 'H');
  is_reset = 1;
}

void loop() {
  unsigned char payload[66];

#if 0
  payload[0] = ow_readbyte();
  Serial.write(payload[0]);
  return;
#endif

  // block while no input
  if (Serial.available() == 0) {
    return;
  }

  // read PAGE byte
  payload[0] = Serial.read();

  if (payload[0] <= PAGE_LIMIT) {
    // programming a page, read 65 bytes from UART and then relay
    Serial.readBytes(payload+1, 65);

    // reset if needed AFTER reading from UART so we don't lose payload
    if (!is_reset) {
      do_reset();
    }
    ow_writebytes(payload, 66);
    DELAY_US(PULSE_A); // pause the short pulse waiting for the other side to get ready to send
    Serial.write(ow_readbyte());
  } else if (payload[0] >= 128) {
    // reading a page, transmit page and then read
    // reset if needed AFTER reading from UART so we don't lose payload
    if (!is_reset) {
      do_reset();
    }
    ow_writebyte(payload[0]);
    DELAY_US(PULSE_A); // pause the short pulse waiting for the other side to get ready to send
    ow_readbytes(payload, 65); // payload + ACK byte
    Serial.write(payload, 65);
    DELAY_US(5000); // pause 1ms between page writes
  } else if (payload[0] >= 120 && payload[0] <= 126) {
    // we're done programming
    ow_writebyte(payload[0]);
    is_reset = 0;
  } else if (payload[0] == 127) {
    // Init packet to check H - A connection
    Serial.print(F("HELLO"));
    is_reset = 0; // go back to assuming the device needs a reset
  }
}