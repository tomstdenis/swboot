
// attiny84/attiny85 bootloader that uses single wire dallas style comms from programmer
/* 
To deploy:
  - With ISP (Arduino as ISP) program the fuses and this sketch to the device.
    - In this config the bootloader runs as a "setup()" function so there's a tiny startup delay

To Program
  - Flash the swadapter to another ATTiny84/Arduino-like (using ISP or Optiboot) with an FTDI or similar connection to it's TX/RX pins (see ATTinycode specs for pins)
    - Ideally configure it per the apps notes (use an external clock, put a 1kOhm resistor on the data line, put a 100nF cap in series with the RESET of the target)
  - Use swclient program on your Linux host to send an Intel HEX file through swadapter to swboot.
    - swclient formats IHEX into completed pages, patches the reset vector, and then streams commands over serial to swadapter who in turn streams it over single wire to swboot.

Limitations:
  - Your program cannot be larger than 8192 - 514 == 7678 bytes.
  - The data pin connected to high impedence on the target's circuit during programming (e.g. nothing to drive it high or low)
  - If you enable SLOW_PULSE make sure you do so on the adapter too.  It should only be used if clock skew is too much for 20uS pulses.
*/
#include "swboot.h"

// PER arch config

// data pin (PB2) suitable for both ATTiny84 and ATTiny85
#define DATA_PIN PINB
#define DATA_DDR DDRB
#define DATA_PORT PORTB
#define BOOT_PIN 2
#define BB ((unsigned char)(1U<<BOOT_PIN))
// layout and page size varies per part
#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny84__)
  #define BOOT_ADDR 0x1E00
  // page size
  #define PAGE_SHIFT 6
#elif defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny44__)
  #define BOOT_ADDR 0x0E00
  // page size
  #define PAGE_SHIFT 6
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny24__)
  #define BOOT_ADDR 0x0600
  // page size
  #define PAGE_SHIFT 5
#else
  #error Unsupported micro
#endif
// these are computed from arch specific values
#define PAGE_SIZE (1U<<PAGE_SHIFT)

#define DELAY_US(us) __builtin_avr_delay_cycles((unsigned long)(us) * (F_CPU / 1000000UL))


#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny84__)
void __attribute__((section(".bootloader8k"), naked, used, noinline, noreturn)) setup()
#elif defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny44__)
void __attribute__((section(".bootloader4k"), naked, used, noinline, noreturn)) setup()
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny24__)
void __attribute__((section(".bootloader2k"), naked, used, noinline, noreturn)) setup()
#endif
{
    // initialize stack pointer to RAMEND-80 bytes (or -48 for 32-byte pages)
  asm volatile (
      // disable interrupts
      "cli \n\t"
      // ensure r1 is zero
      "eor r1,r1 \n\t" 
      "out %[sreg], r1 \n\t"
      // reset and clear Watchdog
      "out %[mcusr], r1 \n\t"
      "ldi  r16, 0x18    \n\t" // (1<<WDCE) | (1<<WDE)
      "out  %[wdtcsr], r16 \n\t"
      "out  %[wdtcsr], r1   \n\t"
      // setup stack
      "ldi r28, lo8(%0) \n\t"
      "ldi r29, hi8(%0) \n\t"
      "out __SP_L__, r28 \n\t"
#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny84__)
      "out __SP_H__, r29 \n\t"
#endif
      :
      : "i" (RAMEND - PAGE_SIZE - 16), [sreg] "I"(_SFR_IO_ADDR(SREG)), [mcusr] "I" (_SFR_IO_ADDR(MCUSR)),[wdtcsr] "I" (0x21): "r16"
  );
    unsigned char buf, x, y, z, page[2 + PAGE_SIZE];

    // config pin as input (and PORT will be low when toggled to an output)
    DATA_DDR &= ~BB;
    DATA_PORT |= BB; // enable internal pullup so if the target is fielded with the pin floating it won't randomly enter the bootloader.

    // test for low
    if (DATA_PIN & BB) {
      goto done;
    }

    // We don't know where in the 250mS post-RESET window we're in so syncup to the adapter going high.
    while (!(DATA_PIN & BB));

    // now we hold the line low for 100uS
    // set LOW so we don't accidentally output HIGH while a sink is present.
    DATA_PORT &= ~BB;
    // enable output
    DATA_DDR |= BB;
    DELAY_US(100);
    // at this point we don't need the internal pullup since we're entering the bootloader so one is assumed to be present.
    DATA_DDR &= ~BB;
 
    for (;;) {
      // now read the command byte
      DELAY_US(PULSE_A);
      z = PAGE_SIZE + 2; // default to reading the max we would ever read PAGE_SIZE + 2 bytes
      y = 0; // output offset
      while (z--) {
        for (x = 0; x < 8; x++) {
          // wait for low
          while ((DATA_PIN & BB));
          // wait till middle
          DELAY_US(PULSE_MID);
          buf <<= 1;
          buf |= (DATA_PIN & BB) ? 1 : 0;
          // wait for it to go high
          while (!(DATA_PIN & BB));
        }
        page[y] = buf;
        // hello and ident can't read PAGE_SIZE+2 since the adapter and host don't know what PAGE_SIZE is at this point
        if (!y && (buf == 127 || buf == 126)) break;
        ++y;
      }
      // both EEPROM opcodes put the address as page[1], page[2] so just load
      // the registers here.  For other opcodes it technically puts garbage
      // into the registers but that's ok
      asm volatile (
              "out %[eearh], %[addrh] \n\t" // Load address
              "out %[eearl], %[addrl] \n\t" // Load address
              ::        // result
                [eearl] "I" (_SFR_IO_ADDR(EEARL)),
                [eearh] "I" (_SFR_IO_ADDR(EEARH)),
                [addrh]  "r" (page[1]),
                [addrl]  "r" (page[2]));
      z = 1;

      // prcompute  shifted page address because this is used by multiple
      // opcodes and saves space to do once 
      uint16_t addr = (uint16_t)(page[0] & 0x7F) << PAGE_SHIFT;

      // process commands
      if (page[0] == 127) {  // page 127 == send 0x54 back
          goto good;
      } else if (page[0] == 126) {  // page 126 == ident
        #if defined(__AVR_ATtiny85__)
          page[0] = 0x85;
        #elif defined(__AVR_ATtiny84__)
          page[0] = 0x84;
        #elif defined(__AVR_ATtiny45__)
          page[0] = 0x45;
        #elif defined(__AVR_ATtiny44__)
          page[0] = 0x44;
        #elif defined(__AVR_ATtiny25__)
          page[0] = 0x25;
        #elif defined(__AVR_ATtiny24__)
          page[0] = 0x24;
        #endif
        goto sendcode;  // don't send ack back
      } else if (page[0] == 125) { // write EEPROM
        asm volatile (
            "out  %[eecr], r1          \n\t" // Force Atomic mode using zeroed r1            
            "out  %[eedr], %[data]     \n\t" // Set Data
            "sbi  %[eecr], %[eempe]    \n\t" // Set Master Enable
            "sbi  %[eecr], %[eepe]     \n\t" // Set Write Enable (Strike!)
            : 
            : [eecr]  "I" (_SFR_IO_ADDR(EECR)),
              [eedr]  "I" (_SFR_IO_ADDR(EEDR)),
              [eepe]  "I" (EEPE),
              [eempe] "I" (EEMPE),
              [data]  "r" (page[3]));
      } else if (page[0] == 124) { // read EEPROM
        asm volatile (
                "sbi %[eecr],  %[eere]  \n\t" // Trigger Read Enable
                "in  %[res],   %[eedr]  \n\t" // Read the byte immediately
                : [res] "=r" (page[1])        // result
                : [eecr]  "I" (_SFR_IO_ADDR(EECR)),
                  [eedr]  "I" (_SFR_IO_ADDR(EEDR)),
                  [eere]  "I" (EERE),
                  [eepe]  "I" (EEPE));
         ++z;
      // leaves code 123 unused...
      } else if (page[0] & 0x80) { // read back page
          // addr is already precomputed in your code as (page[0] & 0x7F) << PAGE_SHIFT
          uint16_t p = (uint16_t)&page[1];
          asm volatile (
              "ldi r24, %[cnt] \n\t"
              "1: lpm r0, Z+   \n\t"   // Z is automatically r31:r30
              "st  X+, r0      \n\t"   // X is automatically r27:r26
              "dec r24         \n\t"
              "brne 1b         \n\t"
              : "+z" (addr), "+x" (p)  // Let GCC pick the regs and load them
              : [cnt] "M" (PAGE_SIZE)
              : "r0", "r24"
          );
          z = 1 + PAGE_SIZE;
      } else if (page[0] < (BOOT_ADDR >> PAGE_SHIFT)) { // pages 0..BOOT_ADDR>>PAGE_SHIFT means program the page (you can't program over the bootloader)
        // simple CRC-8 with polynomial 0x8C
        unsigned char *crc_ptr = &page[0]; // Need a pointer for the 'X' register
        asm volatile (
          "ldi r16, %[cnt] \n\t"    // number of bytes
          "ldi r17, 0x8C   \n\t"    // CRC polynomial
          "eor %[buf], %[buf] \n\t" // buf == 0
          "1: \n\t"
            "ld  r18, X+     \n\t"    // Get byte from page
            "eor %[buf], r18 \n\t"    // XOR into CRC
            "ldi r19, 8      \n\t"    // 8 bits
              "2: \n\t"
              "lsr %[buf]      \n\t"    // Shift LSB into Carry
              "brcc 3f         \n\t"    // Jump FORWARD if Carry is 0
              "eor %[buf], r17 \n\t"    // XOR polynomial
              "3: \n\t"
              "dec r19         \n\t"
              "brne 2b         \n\t"
            "dec r16         \n\t"
            "brne 1b         \n\t"
          "ld r18, X+ \n\t"      // fetch page[PAGE_SIZE+1]
          "eor %[buf], r18 \n\t" // XOR into buf
          : [buf] "+r" (buf), "+x" (crc_ptr)
          : [cnt] "M" (PAGE_SIZE + 1)
          : "r16", "r17", "r18", "r19"
        );
        // checksum byte doesn't match send 0x83 back to host
        if (buf) {
          page[0] = 0x83;
          goto sendcode;
        } else {
          // Note  we have no waits on SPM here.  We assume the adapter will pause
          // at least 5ms after every page write command (after receiving the reply from the bootloader that is)
          // failure to wait could result in corrupt uploads.
          uint16_t buffer_ptr = (uint16_t)&page[1];
          asm volatile (
              // --- 1. WAIT & ERASE ---
              "ldi r16, 0x03 \n\t"      // Page Erase command
              "out %[spmcsr], r16 \n\t"
              "spm \n\t"
              // --- 2. FILL BUFFER ---
              "ldi r17, %[cnt] \n\t"
              "ldi r16, 0x01   \n\t" // Initialize SPMCSR value (Fill Buffer command)
              "1: \n\t"
              "ld r0, X+       \n\t"
              "ld r1, X+       \n\t"
              "out %[spmcsr], r16 \n\t"
              "spm             \n\t"
              "adiw r30, 2     \n\t"
              "dec r17         \n\t"
              "brne 1b         \n\t"
              "subi r30, lo8(%[cnt2]) \n\t " // rewind 'addr' back PAGE_SIZE bytes
              "sbci r31, hi8(%[cnt2]) \n\t "
              // --- 3. WAIT & WRITE PAGE ---
              "ldi r16, 0x05 \n\t"      // Page Write command
              "out %[spmcsr], r16 \n\t"
              "spm \n\t"
              "eor r1, r1      \n\t" // Clean up r1 for C
              : "+z" (addr), "+x" (buffer_ptr)
              : [spmcsr] "I" (_SFR_IO_ADDR(SPMCSR)),
                [cnt] "M" (PAGE_SIZE / 2),
                [cnt2] "M" (PAGE_SIZE)
              : "r17", "r16", "r0"
          );
        }
      } else {
        // invalid PAGE, launch user app
        goto done;
      }
good:
      page[0] = 0x54;
sendcode:      
      DELAY_US(PULSE_REVERSAL_LENGTH); // <--- we're switching from reading to writing so we need to wait for the bus
      for (y = 0; y < z; y++) {
        buf = page[y];
        for (x = 0; x < 8; x++) {
          unsigned char wa, wb;

          // determine pulse timing
          if (buf & 0x80) {
            wa = SIMP_US_TO_LOOPS(PULSE_A);
            wb = SIMP_US_TO_LOOPS(PULSE_B);
          } else {
            wa = SIMP_US_TO_LOOPS(PULSE_B);
            wb = SIMP_US_TO_LOOPS(PULSE_A);
          }

          // set output direction and low
          DATA_DDR |= BB;
          SIMP_DELAY_US(wa);

          // set as input (high)
          DATA_DDR &= ~BB;
          SIMP_DELAY_US(wb);

          buf <<= 1;
        }
      }
    }
done:
    // Load the last 2 bytes of EEPROM (e.g., 0x1FE and 0x1FF on ATtiny85)
    // assumes E2END and (E2END-1) share an upper byte
    // into the Z register (r31:r30)
    asm volatile (
      "ldi r30, lo8(%[addr]) \n\t"
      "ldi r31, hi8(%[addr]) \n\t"

      // Read Low Byte
      "out %[eearh], r31     \n\t"
      "out %[eearl], r30     \n\t"
      "sbi %[eecr], %[eere]  \n\t"
      "in r24, %[eedr]       \n\t"

      // Read High Byte
      "adiw r30, 1           \n\t"
#if ((E2END>>8) != ((E2END-1)>>8))
      "out %[eearh], r31     \n\t"
#endif      
      "out %[eearl], r30     \n\t"
      // Note: we don't need to write EEARH again if it didn't change (likely 0 or 1)
      "sbi %[eecr], %[eere]  \n\t"
      "in r31, %[eedr]       \n\t"

      "mov r30, r24          \n\t"
      "ijmp                  \n\t"
      :: [addr] "i" (E2END - 1),
         [eearl] "I" (_SFR_IO_ADDR(EEARL)),
         [eearh] "I" (_SFR_IO_ADDR(EEARH)),
         [eere]  "I" (EERE),
         [eedr]  "I" (_SFR_IO_ADDR(EEDR)),
         [eecr]  "I" (_SFR_IO_ADDR(EECR))
      : "r24", "r30", "r31"
    );
}
