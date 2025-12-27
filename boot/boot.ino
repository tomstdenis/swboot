
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

// use SLOW_PULSE if your target doesn't have an external clock
#define SLOW_PULSE

#ifdef SLOW_PULSE
// 40uS timebase
#define PULSE_A 10
#define PULSE_B 30
#else
// 20us timebase
#define PULSE_A 5
#define PULSE_B 15
#endif

//#define DEBUG

#define PULSE_MID ((PULSE_A+PULSE_B)/2)


  // to read a bit sync to LOW, wait 10uS then sample pin
  // assumes 8MHz or 16MHz clock
  // use PORTB always (leaves port A's 8-bits untouched on tiny84's )
  #define BOOT_PIN 2
  #define BB ((unsigned char)(1U<<BOOT_PIN))
  #define DELAY_US(us) __builtin_avr_delay_cycles((unsigned long)(us) * (F_CPU / 1000000UL))
  void __attribute__((section(".bootloader"), naked, used, noinline, noreturn)) setup()
  {
    // initialize stack pointer to RAMEND-80 bytes 
  asm volatile (
      // disable interrupts
      "cli \n\t"
      // ensure r1 is zero
      "eor r1,r1 \n\t" 
      // clear the SREG in case we're in a warm reset
      "out %[sreg], r1 \n\t"
      // reset and clear Watchdog
      "out %[mcusr], r1 \n\t"
      "ldi  r16, 0x18    \n\t" // (1<<WDCE) | (1<<WDE)
      "out  %[wdtcsr], r16 \n\t"
      "out  %[wdtcsr], r1   \n\t"
      // setup stack
      "ldi r28, lo8(%0) \n\t"
      "ldi r29, hi8(%0) \n\t"
      "sbiw r28, 40      \n\t" // Reserve ~80 bytes for local variables
      "sbiw r28, 40      \n\t"
      "out __SP_L__, r28 \n\t"
      "out __SP_H__, r29 \n\t"
      :
      : "i" (RAMEND), [sreg] "I"(_SFR_IO_ADDR(SREG)), [mcusr] "I" (_SFR_IO_ADDR(MCUSR)),[wdtcsr] "I" (0x21): "r16"
  );
    unsigned char buf, x, y, z, page[65];

    // config pin as input (and PORT will be low when toggled to an output)
    DDRB &= ~BB;
    PORTB |= BB; // enable internal pullup so if the target is fielded with the pin floating it won't randomly enter the bootloader.

    // use PB0 as a debug port
#ifdef DEBUG
    DDRB |= 1;
    PORTB &= ~1;
#endif

    // sample the pin for 1000us
    for (y = x = 0; x < 100; x++) {
      if (!(PINB & BB)) {
        ++y;
#ifdef DEBUG
        PINB = 1;
#endif        
      }
      DELAY_US(10);
    }
   
    // if it was LOW for less than 10% of the time it's likely a regular boot.
    if (y < 10) {
      goto done;
    }

    // wait for it to go back high
    while (!(PINB & BB));

    // now we hold the line low for 100uS
    // set LOW so we don't accidentally output HIGH while a sink is present.
    PORTB &= ~BB;
    // enable output
    DDRB |= BB;
    DELAY_US(100);
    // at this point we don't need the internal pullup since we're entering the bootloader so one is assumed to be present.
    DDRB &= ~BB;

    for (;;) {
      // now read the command byte
      for (x = 0; x < 8; x++) {
        // wait for low
        while ((PINB & BB));
        // wait till middle
        DELAY_US(PULSE_MID);
        buf <<= 1;
        buf |= (((unsigned char)PINB) >> BOOT_PIN) & 1;
        // wait for it to go high
        while (!(PINB & BB));
      }

      z = 1;
      if (buf == 120) {         // page 120 == done
        goto done;
      } else if (buf == 127) {  // page 127 == send 'H' back
        page[0] = 'H';
        goto sendcode;
      } else if (buf >= 128) {  // page 128+ == read back page-128 to the adapter
          // buf was the command, let's assume bits 0-6 are the page address
          uint16_t addr = (uint16_t)(buf & 0x7F) << 6; 

          for (y = 0; y < 64; y++) {
              // Read byte from Flash
              asm volatile(
                  "movw r30, %1 \n\t"
                  "lpm %0, Z \n\t"
                  : "=r" (buf)
                  : "r" (addr + y)
                  : "r30", "r31"
              );
              page[y+1] = buf;
          }
          page[0] = 0x54; // ACK byte
          z = 65;
          goto sendcode;
      } else if (buf < (0x1E00U >> 6)) { // pages 0..0x1E00>>6 mean program the page
        unsigned char pageaddr = buf;
        // receive a 64-byte page + checksum
        for (y = 0; y < 65; y++) {
          for (x = 0; x < 8; x++) {
            // wait for low
            while (PINB & BB);
            // wait till middle
            DELAY_US(PULSE_MID);
            buf <<= 1;
            buf |= (((unsigned char)PINB) >> BOOT_PIN) & 1;
            // wait for high
            while (!(PINB & BB));
          }
          page[y] = buf;
        }
        // compute checksum
        for (x = 0, buf = 0xAA; x < 64; x++) {
          buf += x ^ page[x];
        }
        // checksum byte doesn't match send 0x83 back to host
        if (buf != page[64]) {
          page[0] = 0x83;
          goto sendcode;
        }
        // ensure we're not writing to the boot loader
        if (pageaddr < (0x1E00U >> 6)) {
          uint16_t addr = (uint16_t)pageaddr << 6; 

          // --- 1. WAIT & ERASE ---
          asm volatile (
              "movw r30, %0 \n\t"       
              "ldi r16, 0x03 \n\t"      // Page Erase command
              "out %1, r16 \n\t"
              "spm \n\t"
              :
              : "r" (addr), "I" (_SFR_IO_ADDR(SPMCSR))
              : "r16", "r30", "r31"
          );

          // --- 2. FILL BUFFER ---
          for (x = 0; x < 64; x += 2) {
              uint16_t word = (uint16_t)page[x] | ((uint16_t)page[x + 1] << 8);
              
              asm volatile (
                  "movw r30, %0 \n\t"   
                  "movw r0, %1 \n\t"    // r1:r0 = word data
                  "ldi r16, 0x01 \n\t"  // Fill Page Buffer command
                  "out %2, r16 \n\t"
                  "spm \n\t"
                  :
                  : "r" (addr + x), "r" (word), "I" (_SFR_IO_ADDR(SPMCSR))
                  : "r16", "r0", "r1", "r30", "r31"
              );
          }

          // --- 3. WAIT & WRITE PAGE ---
          asm volatile (
              "movw r30, %0 \n\t"
              "ldi r16, 0x05 \n\t"      // Page Write command
              "out %1, r16 \n\t"
              "spm \n\t"
              :
              : "r" (addr), "I" (_SFR_IO_ADDR(SPMCSR))
              : "r16", "r30", "r31"
          );
        }
      }
      // write back an ACK byte of 0x54
      page[0] = 0x54;
sendcode:
      for (y = 0; y < z; y++) {
        buf = page[y];
        for (x = 0; x < 8; x++) {
          // set output direction and low
          DDRB |= BB;
          if (buf & 0x80) {
            // high bit == delay 5uS, set high, then delay 15uS
            DELAY_US(PULSE_A); 
            DDRB &= ~BB;
            DELAY_US(PULSE_B);
          } else {
            // low bit == delay 15uS, then high then delay 5uS
            DELAY_US(PULSE_B); 
            DDRB &= ~BB;
            DELAY_US(PULSE_A);
          }
          buf <<= 1;
        }
      }
    }

    // our uploader program hijacks the sketches reset vector and stores it at 0x1DFE
  done:
    // reset stack pointer then jump to app's reset vector
    asm volatile (
      "ldi r28, lo8(%0) \n\t"
      "ldi r29, hi8(%0) \n\t"
      "out __SP_L__, r28 \n\t"
      "out __SP_H__, r29 \n\t"
      "eor r1, r1 \n\t"
      "ijmp              \n\t"
      : 
      : "i" (RAMEND), "z" ((uint16_t)(0x1DFE >> 1))
      :
    );
  }
