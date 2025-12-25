
  // attiny84/attiny85 bootloader that uses single wire dallas style comms from programmer
  // uses 20uS bit width
  // 1 bit == 5uS LOW, 15us HIGH
  // 0 bit == 15us LOW, 5us LOW
  // uses output LOW, and input HIGH (with external pullup)
/* 
The overall protocol

- Host toggles the reset line of the target
- Host sets data as output low for 100uS
- Host sets data as high impedence (external pullup pulls it high) for at least 20uS
- Host then sends 1 byte using above protocol
- If the MSB is set then
    - wait 20uS
    - target transmits the page indicate by the lower 7 bits of the byte read using the above protocol
- If the MSB were cleared
    - if the Page address is below 0x1E00
      - Host transmits 65 bytes using the above protocol
      - target sums up bytes 0..63 and see if it matches byte 64
      - If not, wait 20uS, send byte 0x83
      - If matches, program bytes, wait 20uS, send byte 0x54
    - If the page address is >= 0x1E00, then we're done, jump to user code

To deploy:
  - With ISP (Arduino as ISP) program the fuses and this sketch to the device.
    - In this config the bootloader runs as a "setup()" function so there's a tiny startup delay

To Program
  - Flash the swadapter to another ATTiny84 (using ISP or Optiboot) with an FTDI or similar connection to it's TX/RX pins (see ATTinycode specs for pins)
    - Ideally configure it per the apps notes (use an external clock, put a 1kOhm resistor on the data line, put a 100nF cap in series with the RESET of the target)
  - Use swclient program on your Linux host to send an Intel HEX file through swadapter to swboot.
    - swclient formats IHEX into completed pages, patches the reset vector, and then streams commands over serial to swadapter who in turn streams it over single wire to swboot.
*/

  // to read a bit sync to LOW, wait 10uS then sample pin
  // assumes 8MHz or 16MHz clock
  // use PORTB always
  #define BOOT_PIN 0
  #define BB ((unsigned char)(1U<<BOOT_PIN))
  #define DELAY_US(us) __builtin_avr_delay_cycles((unsigned long)(us) * (F_CPU / 1000000UL))
  void __attribute__((section(".bootloader"), naked, used, noinline, noreturn)) setup()
  {
    // initialize stack pointer to RAMEND-80 bytes
  asm volatile (
      "cli \n\t"
      "ldi r28, lo8(%0) \n\t"
      "ldi r29, hi8(%0) \n\t"
      "sbiw r28, 40      \n\t" // Reserve ~80 bytes for local variables
      "sbiw r28, 40      \n\t"
      "out __SP_L__, r28 \n\t"
      "out __SP_H__, r29 \n\t"
      : : "i" (RAMEND) :
  );
    unsigned char buf, x, y, page[65];

    // config pin as input with pullup 
    DDRB = ~BB;
    PORTB |= BB;
    DELAY_US(2); // settle line

    // the programmer must hold the data wire low for 100uS after resetting the target.
    // check for low
    if (PINB & BB) {
      goto done;
    }

    // delay 50us to make sure it's still low, recall we're here right out of reset
    DELAY_US(50);
    if (PINB & BB) {
      goto done;
    }

    // wait for it to go back high
    while (!(PINB & BB));

    for (;;) {
      // now read the command byte
      for (x = 0; x < 8; x++) {
        // wait for low
        while ((PINB & BB));
        // wait 10uS
        DELAY_US(10);
        buf <<= 1;
        buf |= (((unsigned char)PINB) >> BOOT_PIN) & 1;
        // wait for it to go high
        while (!(PINB & BB));
      }

      // if high bit is set we're sending a page back
      if (buf & 0x80) {
          // buf was the command, let's assume bits 0-6 are the page address
          uint16_t addr = (uint16_t)(buf & 0x7F) << 6; 

          DELAY_US(20); // wait one cycle 
          for (y = 0; y < 64; y++) {
              // Read byte from Flash
              uint8_t val;
              asm volatile(
                  "movw r30, %1 \n\t"
                  "lpm %0, Z \n\t"
                  : "=r" (val)
                  : "r" (addr + y)
                  : "r30", "r31"
              );

              // Send 'val' over the wire using your existing bit-bang logic
              for (x = 0; x < 8; x++) {
                  PORTB &= ~BB;
                  DDRB |= BB;
                  if (val & 0x80) {
                      DELAY_US(5);
                      DDRB &= ~BB;
                      PORTB |= BB;
                      DELAY_US(15);
                  } else {
                      DELAY_US(15);
                      DDRB &= ~BB;
                      PORTB |= BB;
                      DELAY_US(5);
                  }
                  val <<= 1;
              }
          }
      } else {
        unsigned char pageaddr = buf;
        // receive a 64-byte page + checksum
        for (y = 0; y < 65; y++) {
          for (x = 0; x < 8; x++) {
            // wait for low
            while (PINB & BB);
            // wait 10uS
            DELAY_US(10);
            buf <<= 1;
            buf |= (((unsigned char)PINB) >> BOOT_PIN) & 1;
            // wait for high
            while (!(PINB & BB));
          }
          page[y] = buf;
        }
        // compute checksum
        for (x = buf = 0; x < 64; x++) {
          buf += page[x];
        }
        // checksum byte doesn't match send 0x83 back to host
        if (buf != page[64]) {
          buf = 0x83;
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
        } else {
          // trying to write to bootloader signals we're done
          break;
        }
        // write back an ACK byte of 0x54
        buf = 0x54;
  sendcode:
        DELAY_US(20); // min spacing before switching roles to transmit
        for (x = 0; x < 8; x++) {
          // set output direction and low
          PORTB &= ~BB;
          DDRB |= BB;
          if (buf & 0x80) {
            // high bit == delay 5uS, set high, then delay 15uS
            DELAY_US(5); 
            DDRB &= ~BB;
            PORTB |= BB;
            DELAY_US(15);
          } else {
            // low bit == delay 15uS, then high then delay 5uS
            DELAY_US(15); 
            DDRB &= ~BB;
            PORTB |= BB;
            DELAY_US(5);          
          }
          buf <<= 1;
        }
      }
    }

    // our uploader program hijacks the sketches reset vector and stores it at 0x1DFE
  done:
    // reset stack pointer then jump to app's resect vector
    asm volatile (
      "ldi r28, lo8(%0) \n\t"
      "ldi r29, hi8(%0) \n\t"
      "out __SP_L__, r28 \n\t"
      "out __SP_H__, r29 \n\t"
      "ijmp              \n\t"
      : 
      : "i" (RAMEND), "z" ((uint16_t)(0x1DFE >> 1))
      :
    );
  }
