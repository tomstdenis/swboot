# swboot

Single-Wire bootloader for ATTiny85/84 devices.

When all is working you can program a device using a single wire from outside the target
circuit.  The bootloader takes 512 bytes (well less but I reserve that block) and 2 bytes just before.

Your app has from 0x0000 to 0x1EFD (about 7.5kb) to use.

The booter tries to sense if the programmer is present and if not jumps into your target application relatively 
quickly.

# Hardware limitations

- Requires a strong pullup on the data wire.  You can limit this to the adapter side of things though if you want.  I used a 680 Ohm resistor
- Other things attached to your target's data PIN should be high impedence otherwise it could skew the pulses.  So best to
  leave this pin as an input target (e.g. buttons, etc), or make provisions to jumper it out of circuit while you program it.
- Only supports 8x series parts right now (8KB).  In theory this should work on 2K and 4K parts with the appropriate changes to defines and bootloader section  

# Setup

This project uses the ATTinyCore board provider.  You need to update it to support this project.

First add the extended fuse changes to ~/.arduino15/packages/ATTinyCore/hardware/avr/1.5.2/boards.txt:

- attinyx4.bootloader.extended_fuses=0xFE
- attinyx5.bootloader.extended_fuses=0xFE

(and others as you need them).  This enables the SPM instruction for self-programming.

Next, add the following line to a new file "platform.local.txt" in the same directory as boards.txt:

- compiler.c.elf.extra_flags=-Wl,--section-start=.bootloader=0x1E00

This tells the compiler where to locate the ".bootloader" section

You will need to re-load the IDE after making these changes as Arduino (as of 2.3.7) caches them on init.

There are REFERENCE copies of the files in the root of this repo.  Note they may be out of date by time you go to use them
so use them as guidance only.

# boot

This is the bootloader application.  Without an external clock I suggest defaulting to REALLY_SLOW_PULSE
for timing (make sure adapter uses the same setting!).  Even with "REALLY_SLOW_PULSE" it's not terribly slow.
My 384(+app vector) byte blinking LED demo uploads from scratch (so including target resetting, handshake, etc) in 1.7 seconds.
Of which about 0.5 seconds was simply it waiting for the adapter to get ready.  So in 1.2 seconds it wrote and read back 8
64-byte pages.  At 0.15 seconds per page a full device (about 7.5KB of code) would take about 18 seconds to program.
Not too shabby.

It defaults to using PB2 as the data pin you can change this by changing BOOT_PIN.

To get off the ground use an "Arduino as ISP" to burn the fuses ("burn bootloader") and program the boot loader.

At this point the device is ready to go.

NOTE:  If you want to install the bootloader on a difference mcu (e.g. going from an 85 to 84) make sure you rebuild the loader as there
may (and likely are) register offset deltas between the two.

# adapter

The adapter application receives serial from your PC and translates it to 1-wire for the device.  Make sure you use
the same PULSE timing as booter or it won't work.

On my "pro micro" clone I'm using pin 16 and 14 for data and RESET signalling respectively.  The app is configured for these
pins out of the box.

If you're using a different mcu then make sure you set the PIN_* macros correctly for your pins.  Keep in
mind because I'm lazy both the data and RESET pins have to be on the same port.

There should be a good pullup on the data wire (I'm using a 680Ohm resistor) so there's nice sharp edges.

# client

The client application is very very very bare bones at the moment.  You give it a tty device and a HEX file and it will
try to write-then-readback every page.  It also patches the RESET vector (address 0) to jump to the bootloader.  It places
the applications jump instruction at 0x1DFE.

The client really needs a lot of work to be honest I vibe coded a lot of it because I was simultaneously working in three spaces
at once... I'll fix it up properly later.

You can program devices via

./client ${TTY} ${HEXFILE}

e.g.

./client /dev/ttyACM0 myapplication.ino.hex

Tip: You can use CTRL+ALT+S to build and export your app as a hex file (look for build/${core}/${appname}.ino.hex under your sketches directory)

# One-wire spec

The comms is essentially like dallas 1-wire.  There should be an external pullup on the line and every device only outputs a LOW (sink) never a HIGH (source).
The way you read a bit is define as 

- wait for line to go LOW
- delay for (PULSE_A+PULSE_B)/2 microseconds
- sample line for bit
- wait for line to go HIGH
- repeat as required

Basically we're sampling in the middle of where it would change.  To write a bit you do

- set line output LOW
- if the bit is a 0, delay PULSE_B uS, switch the line to an input (make it high), delay PULSE_A uS
- if the bit is a 1, delay PULSE_A uS, switch the line to an input (make it high), delay PULSE_B uS

The real trick to the protocol is writing is blind.  Reading is synchronized (wait for low).  So always make sure
the consumer is ready BEFORE the producer, and for fun, these roles change often enough...

At the default 80uS pulse timing (REALLY_SLOW_PULSE) that's about 12.5kbit/sec (minus overhead).  Not winning any awards but gets the job done and more
importantly works off the RC internal clock when fed from the Vcc pin of a pro-micro (so like there's voltage drop, etc).

# Troubleshooting the bootloader

- Usual suspects, ensure the wiring is correct, that you have a pullup, etc (it will fail without the pullup)

- Ensure you have the configuration changes required (reload the IDE after changing these)
- Ensure you burn the extended fuse correctly 
- Ensure that the PULSE timing is the same on the adapter and boot (as programmed on the devices)
- Ensure there is a good pullup on the data wire

If all that is going next check the data wire.  It should pulse after the adapter resets the device.  If you don't see this maybe you're on the wrong pin
of the adapter's MCU?

If you see that, you should see the target hold the line low for around 100uS after the adapter releases it high.

If you see that, you should see the adapter write 127 (01111111) and the target respond with 'H'

If you see that, chances are it's noise on the line.

# Troubleshooting the Client

If you don't see the "HELLO" reply try rebooting the MCU.  It can get stuck (I didn't add timeouts for everything).  Failing that be patient that app
is going to be re-written...

# TODO

- Tidy up code (there's stale comments, debug logic here and there)
- Re-write client.c to be an actual app that you can use....
- Maybe write an "install" script that tries to patch your boards.txt and install platform.local.txt for you

- I will likely make a PCB that caddy's the pro micro with and produces a standard 1x4 pin header (GND, VCC, DATA, RESET) making it easier to connect
  also sport the required pullup on the data line
