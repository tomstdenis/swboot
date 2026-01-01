/**
 * Data Bit Timing (T_bit)
 * At 8MHz RC, 64uS (512 cycles) is the aggressive "sweet spot."
 * Lower values (e.g., 40uS) may work on crystal-stabilized parts, 
 * but RC drift and breadboard capacitance make 64uS the reliable floor.
 * 
 * If you find stability issues try bumping it up by steps of 4 using
 * the demo_ow target app and "client /dev/tty... -q" you can test
 * the link quality.
 * 
 * In my breadboard setup I found 60uS worked 99.9% of the time but that's
 * not really good enough.  So far 64uS is working 100% of the time
 */
#define PULSE_LENGTH 64

// define our own delay that allows variable counts upto about 94uS
// on either 8 or 16 MHz parts
#if ((((PULSE_LENGTH * 3) / 4) * (F_CPU/1000000UL)) / 3) >= 256

// each faster clock loop will take 6 cycles
#define DELAY_DIV 6

// our delay macro
#define SIMP_DELAY_US(wa) \
    asm volatile ( \
      "dec %[cnt] \n\t" \
      "nop\n\t"         \
      "brne .+0  \n\t"  \
      "brne .-8 \n\t"   \
      "rjmp .+0 \n\t"   \
      : [cnt] "+r" (wa));

#else

// each slower clock loop will take 3 cycles
#define DELAY_DIV 3

#define SIMP_DELAY_US(wa) \
    asm volatile ( \
      "dec %[cnt] \n\t" \
      "brne .-4 \n\t"   \
      "nop \n\t"        \
      : [cnt] "+r" (wa));
#endif

// convert uS to the number of loops required for the clock
#define SIMP_US_TO_LOOPS(x) (((x) * (F_CPU/1000000)) / DELAY_DIV)


/**
 * Role Reversal / Bus Recovery (T_recovery)
 * The guard time required when switching from Talk to Listen (or vice versa).
 * This must account for:
 * 1. Software state-machine transition time on the target.
 * 2. RC Rise Time: The time required for the pull-up resistor to bring 
 * the line back to VCC after a driven LOW.
 */
#define PULSE_REVERSAL_LENGTH 80

/**
 * Sampling Sub-intervals (The "A-B" Strategy)
 * Used for edge-synced bit-banging:
 * - PULSE_A: Initial delay after the falling edge to clear signal ringing.
 * - PULSE_MID: The ideal "eye" of the data window for stable sampling.
 * - PULSE_B: The remainder of the pulse to ensure frame alignment.
 */
#define PULSE_A   (PULSE_LENGTH / 4)
#define PULSE_B   (3 * PULSE_A)
#define PULSE_MID (PULSE_LENGTH / 2)
