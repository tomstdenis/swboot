// demo app that can make use of OW support...
// (untested)
// app assumes host sends 1 byte then expects 1 back
// we increment the input by 1 to send it back so the host can see "something happened"

#include <avr/io.h>
#include <avr/interrupt.h>
#include "swboot.h"

// how many uS does entering the ISR cost us?
#define ISR_PULSE_COST 6 // tested with 64uS timing on an 8MHz attiny84 and attiny85

// what port is this on, note that changing from PORTB will change which interrupt/ISR you use
#define PIN_PORT PORTB
#define PIN_DDR  DDRB
#define PIN_PIN  PINB
#define PIN_WIRE  2 // PB2
#define BWIRE (1<<PIN_WIRE)
#define DELAY_US(us) __builtin_avr_delay_cycles((unsigned long)(us) * (F_CPU / 1000000UL))

// Volatile variables for ISR communication
#define RB_SIZE 16       // 1-wire ring buffer, must be a power of 2
#define RB_MASK (RB_SIZE - 1)
volatile uint8_t ring_buffer[RB_SIZE];
volatile uint8_t rb_head = 0;
volatile uint8_t rb_tail = 0;

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
#define USING_X5
#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny24__)
#define USING_X4
#endif


void init_1wire_isr() {
    // init data port as input without pull up.
    PIN_DDR &= ~BWIRE;
    PIN_PORT &= ~BWIRE;

    // PB2 (data pin) is on different interrupt pins for the 85/84
#if defined(USING_X5)
    GIMSK |= (1 << PCIE);   // Enable Pin Change Interrupts
    PCMSK |= (1 << PCINT2); // Enable specifically for PB2
#elif defined(USING_X4)
    GIMSK |= (1 << PCIE1);   // Enable Pin Change Interrupts
    PCMSK1 |= (1 << PCINT10); // Enable specifically for PB2
#endif
    sei(); // Enable global interrupts
}

#if defined(USING_X5)
ISR(PCINT0_vect) {
#else
ISR(PCINT1_vect) {
#endif
    // 1. Check if this is the start of a byte (Falling edge)
    if (PINB & BWIRE) return; 

    // 2. Immediately disable this interrupt so we don't trigger on subsequent edges
    #if defined(USING_X5)
      PCMSK &= ~(1 << PCINT2); // Disable PC interrupt so we don't listen to our own transmission
    #elif defined(USING_X4)
      PCMSK1 &= ~(1 << PCINT10); // Disable PC interrupt so we don't listen to our own transmission
    #endif

    uint8_t incoming = 0;

    // 3. We are currently at the falling edge of Bit 7.
    // The bit period is 80us. We want to sample at the 40us mark.
    for (uint8_t i = 0; i < 8; i++) {
        // 1. Sample at the midpoint
        DELAY_US(PULSE_MID - ISR_PULSE_COST);  // we delay a little less on the first pass to account for ISR overhead.

        incoming <<= 1;
        if (PINB & BWIRE) {
            incoming |= 1;
        }

        // 2. ALWAYS wait for the current bit's pulse to finish (go HIGH)
        // This ensures we are back in the "idle/recovery" state.
        uint16_t timeout = 5000;
        while (!(PINB & BWIRE) && --timeout); 

        // 3. If there are more bits to come, wait for the NEXT falling edge
        if (i < 7) {
            timeout = 5000;
            while ((PINB & BWIRE) && --timeout);
            DELAY_US(ISR_PULSE_COST); // add a delay for bits 0..6 so that the delay at the top is a proper alignment
        }
    }

    // 4. Store the byte
    uint8_t next = (rb_head + 1) & RB_MASK;
    if (next != rb_tail) {
        ring_buffer[rb_head] = incoming;
        rb_head = next;
    }

    // 5. Clear flag and re-enable interrupt for the NEXT byte
    #if defined(USING_X5)
      GIFR = (1 << PCIF);
      PCMSK |= (1 << PCINT2);
    #elif defined(USING_X4)
      GIFR = (1 << PCIF1);
      PCMSK1 |= (1 << PCINT10);
    #endif
}

void ow_writebyte(unsigned char y)
{
  unsigned char x;
#if defined(USING_X5)
  PCMSK &= ~(1 << PCINT2); // Disable PC interrupt so we don't listen to our own transmission
#elif defined(USING_X4)
  PCMSK1 &= ~(1 << PCINT10); // Disable PC interrupt so we don't listen to our own transmission
#endif
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

    // set output direction and low
    PIN_DDR |= BWIRE;
    SIMP_DELAY_US(wa);

    // set as input (high)
    PIN_DDR &= ~BWIRE;
    SIMP_DELAY_US(wb);

    y <<= 1;
  }
#if defined(USING_X5)
  PCMSK |= (1 << PCINT2); // Re-enable PC interrupt
#elif defined(USING_X4)
  PCMSK1 |= (1 << PCINT10); // Re-enable PC interrupt
#endif
}

void ow_writebytes(unsigned char *src, unsigned char len)
{
  while (len--) {
    ow_writebyte(*src++);
  }
}

// Helper to get data in main loop
int rb_get_byte(uint8_t *data) {
    if (rb_head == rb_tail) return 0;
    *data = ring_buffer[rb_tail];
    rb_tail = (rb_tail + 1) & RB_MASK;
    return 1;
}

void setup() {
  // put your setup code here, to run once:
  init_1wire_isr();
}

void loop() {
  unsigned char x, y, buf[8];
  // if there's any data echo it back, this assume the host only sends 1 byte at a time
  // if your protocol requires more inputs then you should wait till you receive them all before replying
  if (rb_get_byte(&x)) {
    // first byte is how many bytes (1..8)
    y = 0;
    while (y < x) {
      // loop until we have a byte
      // obviously there are better ways to do this in a real app (for instance you could have your ISR fire a handler once your ring buffer
      // has a "packet" length of data) but this is just a simple test app so here we are.
      while (rb_get_byte(buf+y) == 0);
      ++y;
    }
    DELAY_US(PULSE_REVERSAL_LENGTH); // <-- safe delay before writing.  If you have multiple bytes to write only delay this long on the first one.
    ow_writebytes(buf, y);
  }
}