# swboot

Single-Wire bootloader for ATTinyx5/x4 devices.

This repository features a bootloader you can program on ATTinyx5/x4 devices (like the ATTiny85) that allows a client
programmer to flash new software, dump the flash memory, and read and write EEPROM.  The project is split into three
applications: "boot" which is the bootloader, "adapter" which is a program that sits between the host PC and target,
and "client" which runs on the host PC to give commands to the target.

This project makes use of the ATTinyCore board provider.

When all is working you can program a device using a single wire from outside the target
circuit.  The bootloader takes a 512 bytes block flash code space (with little to spare).  It stores the app reset
vector in EEPROM as the last 2 bytes during programming.  This also allows you to alter the boot target in program and
from the command line.

Your app has:

- 8KB parts: code from 0x0000 to 0x1EFF or 7680 bytes to use, EEPROM from 0x000 to 0x1FD
- 4KB parts: code from 0x0000 to 0x0DFF or 3584 bytes to use, EEPROM from 0x000 to 0x0FD
- 2KB parts: code from 0x0000 to 0x05FF or 1536 bytes to use, EEPROM from 0x000 to 0x07D

The booter tries to sense if the programmer is present and if not jumps into your target application relatively 
quickly.  This is done by sensing the data pin for being driven low.  If it is then it tries to enter programming mode.
The sensing takes very little time so with the pin high (or floating) it will launch your app very quickly.

## Quickguide

Target configuration:

1.  Install ATTinyCore (https://github.com/SpenceKonde/ATTinyCore)
2.  Modify the boards.txt and copy platform.local.txt over (re-load your IDE)
3.  Load up the boot sketch, configure for your target (e.g. "board: ATTiny25/45/85 (No Bootloader)") and programmer 
	(e.g., Arduino as ISP)
4.  Run "burn bootloader" to set fuses and clock fuses
5.  Upload bootloader sketch

At this point your target will boot into the bootloader when powered on.

Adapter Configuration (e.g. pro-micro ATMega32u4)

1.  Using Arduino Leonardo as board, upload the adapter sketch to it.
2.  Wire up the pins to the BOOT and RESET pins of the target (e.g. PB2 and RESET)
	(and common the Vcc and GND of the adapter and target)
3.  Add a 500-1000Ohm pullup from Vcc to the BOOT pin.

Programming your sketch on the target device

1.  Pick your board/chip for your sketch
2.  Export the build CTRL+ALT+S
3.  Using the client tool upload the sketch found in the buiild directory, e.g. ${sketch}/build/ATTinyCore.avr.attinyx5/${sketch}.ino.hex
	e.x., client /dev/ttyACM0 -p demo_ow/build/ATTinyCore.avr.attinyx5/demo_ow.ino.hex

If it uploads successfully your sketch should be now running on the target.  You can use the code found in client to make a host
app that talks to your sketch over 1-wire via the adapter (this is what the demo_ow does!).

# Hardware limitations

- Requires a strong pullup on the data wire.  You can limit this to the adapter side of things though if you want.  I used a 680 Ohm resistor
- Other things attached to your target's data PIN should be high impedence otherwise it could skew the pulses.  So best to
  leave this pin as an input target (e.g. buttons, etc), or make provisions to jumper it out of circuit while you program it.
- Only tested on 8x series parts right now (8KB).  Compile tested for 2x and 4x.
- Only tested at 8MHz RC.  >= 8MHz should be fine (with a suitable external clock signal).  Below 8MHz might work but I'd strongly
  encourage you to have a stable clock since drift at lower frequencies can accumulate more error.

# Setup

This project uses the ATTinyCore board provider.  You need to update it to support this project.

First add the extended fuse changes to ~/.arduino15/packages/ATTinyCore/hardware/avr/1.5.2/boards.txt:

- attinyx4.bootloader.extended_fuses=0xFE
- attinyx5.bootloader.extended_fuses=0xFE

(and others as you need them).  This enables the SPM instruction for self-programming.

Next, add the following line to a new file "platform.local.txt" in the same directory as boards.txt:

```
compiler.c.elf.extra_flags=-Wl,--section-start=.bootloader8k=0x1E00 -Wl,--section-start=.bootloader4k=0x0E00 -Wl,--section-start=.bootloader2k=0x0600
```

This tells the compiler where to locate the ".bootloader*" sections for 8k, 4k, and 2k bootloaders.  You will need to re-load the IDE
after making these changes as Arduino (as of 2.3.7) caches them on init.

There are REFERENCE copies of the files in the root of this repo.  Note they may be out of date by time you go to use them
so use them as guidance only.

# boot

This is the bootloader application.  Without an external clock I suggest defaulting to at least for 64uS pulse timing
for timing (make sure adapter uses the same setting!).  My 384(+app vector) byte blinking LED demo uploads from scratch 
(so including target resetting, handshake, etc) in 1.7 seconds. Of which about 0.5 seconds was simply it waiting for the
adapter to get ready.  So in 1.2 seconds it wrote and read back 8 64-byte pages.  At 0.15 seconds per page a full 
device (about 7.5KB of code) would take about 18 seconds to program. Not too shabby.

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

PROGRESS:  I've ordered prints of my rev1 programmer boards.  These will use P-CH mosfets to control
the target Vcc as well as a pair used to make a digital relay.  This eliminates the need for a RESET pin on the target
and will isolate the programmer from the target when not programming meaning you can leave it connected physically.

The adapter works on a simple payload format from the host.  The first byte is the PAGE byte and falls into one of
several cartegories

- PAGE <= 119      ==> program a page (*****)
- PAGE == 120      ==> done, launch user app (56..120 on 4K, 48..120 on 2K)
- PAGE == 121      ==> power/channel mosfet control (*, ****)
- PAGE == 122      ==> write 'x' bytes to 1-wire, read 'y' bytes from 1-wire (*, ***)
- PAGE == 123      ==> **UNUSED** undefined don't use
- PAGE == 124      ==> EEPROM read (pass it 2 bytes ADDRH and ADDRL, returns the ACK + EEPROM value)
- PAGE == 125      ==> EEPROM write (pass it 3 bytes ADDRH, ADDRL, VAL, returns ACK)
- PAGE == 126      ==> Ident target (e.g. 0x85 == ATtiny85, 0x84 == ATtiny84, etc)
- PAGE == 127      ==> Adapter replies with 5 bytes "HELLO", (**)
- PAGE >= 128      ==> read back the page PAGE - 128

Note except for PAGE's 127 and 126 the adapter always transmits PAGE_SIZE + 2 bytes to the target.  This is a trivial
code size optimization.  The client only transmits as many bytes as required to the adapter though.

(*) These are not supported by the target bootloader itself, just the adapter responds to these.  They're accessible
at any stage of the target operation.

(**) The target boot loader supports this but replies with a single byte 0x54 instead of "HELLO".

(***) PAGE 122 requires the Serial to specify how many bytes to process.  It requires both a send and receive count.
e.g. "0x7A 0x03 0x05" would say to read 3 bytes from serial and write those out to 1-wire, then turn around and read 5
bytes from 1-wire and write then back to serial.  If you try to read from 1-wire and the adapter times out it will return 0x01 
or 0x02 bytes depending on which pulse level it timed out on.

Note:  If your user application implements the same 1-wire protocol you can use the serial protocol to access the device
from your host PC.  The typical flow would be to issue a PAGE 122 command with 'x' and/or 'y' > 0.  The adapter waits
PULSE_A (16uS at default timing), then proceeds to do a read that can timeout.  You can read/write 
upto 64 bytes at a time in either direction.

The target app should have a delay of at least PULSE_LENGTH uS between receiving the last byte and and transmitting the first
byte.  A bit longer is fine as the adapter has a generous timeout on reads.

The demo_ow/demo_ow.ino sketch shows off how to setup an ISR to receive 1-wire bytes and how to write back.

(****) Page 121 power control (requires my PCB or equivalent...) uses the following format:

- bit 1 is on: set power on status indicated in bit 0
- bit 3 is on: set the data connection enabled status indicated in bit 2

So for instance sending 0x02 would turn the power off to the target, sending 0x03 would turn it on.  Similarly,
sending 0x08 would turn off the data channel and sending 0x0C would turn the data channel on.  When the data
channel is disabled the target and host data pins are effectively (but not galvanically) isolated from each other.
You can mix and match too so for instance sending 0x0F will turn the power and channel on.

Without the MOSFET based PCB the "power" control is simply the reset pin.  Sending 0x02 for instance will put the
target into reset until you send 0x03.  Eitherway the target is still powered.  In this configuration bits 2 and 3
don't do anything.

(*****) On all parts trying to write into the bootloader (except for specific special purpose PAGEs) results in the target
app being launched.  On 8K parts you send a PAGE=120 for instance, on a 4K part PAGE=56..120 would cause it to start, on
a 2K part PAGE=48..120 would cause it to start (note on 2K parts a page is 32 bytes not 64 bytes).

# client

The client application is very very very bare bones at the moment.  You give it a tty device and a HEX file and it will
try to write-then-readback every page.  It also patches the RESET vector (address 0) to jump to the bootloader.  It places
the applications jump instruction at BOOTLOADER_ADDR-2.

The client really needs a lot of work to be honest I vibe coded a lot of it because I was simultaneously working in three spaces
at once... I'll fix it up properly later.

## programming a sketch
You can program devices via

./client ${TTY} -p ${HEXFILE}

e.g.

./client /dev/ttyACM0 -p myapplication.ino.hex

Tip: You can use CTRL+ALT+S to build and export your app as a hex file (look for build/${core}/${appname}.ino.hex under your sketches directory)

## dumping the flash
The client tool can also dump the entire flash memory

./client ${TTY} -d ${OUTPUTNAME}

The dump will include the ALREADY patched vectors.  So if you want to re-program a dump use the "-f" programming option.  This will program everything
shy of the bootloader from the file without patching the reset vector.

Note:  If you use "-f" with a raw sketch output (e.g. ${sketch}.ino.hex) you will program it's reset vector  and the bootloader though present will
not be called unless the new sketch jumps to the bootloader.  So default to using "-p" to program sketches as it will patch the reset vector.  If you happen
to mix this up just put it back in your Arduino as ISP (or equiv) and reflash the boot loader sketch.

## 1-wire line quality testing
If you upload the demo_ow program on your target you can use the client's "-q" option to test the link quality:

./client ${TTY} -q

This performs rounds of tests where it sends and then reads back packets of 1 through 8 bytes long.  It does each size
5000 times, the first 8*256 passes are sending monotonically increasing bytes.  After that, it sends packets of random contents
to test overall signal integrity.  In total 8 * 5000 = 40000 tests are performed.  You should get 0 errors for a qualified link.

If you get failures first make sure you have a decent pull up, low capacitance traces, good voltage (e.g. close to 5V).  After that
try increasing the PULSE_LENGTH by 4uS at a time.  If you need to go over 92uS or so your setup is faulty (bad power rail,
noisy environment, etc) fix that first.

## reading/writing EEPROM
You can read the EEPROM one byte at a time via:

./client ${TTY} -re ${ADDR}

or write via:

./client ${TTY} -we ${ADDR} ${VAL}

All values are assumed to be in hex. e.g.,

./client /dev/ttyACM0 -we 0x20 0x40

Will write the value 64 into the 32'nd EEPROM byte.

# One-wire spec

The comms is essentially like dallas 1-wire.  There should be an external pullup on the line and every device only outputs a LOW (sink) never a HIGH (source).
The way you read a bit is define as 

- wait for line to go LOW
- delay for (PULSE_LENGTH)/2 microseconds
- sample line for bit
- wait for line to go HIGH
- repeat as required

Basically we're sampling in the middle of where it would change.  By using "wait for ..." we're essentially syncronizing with the transmitter
which means if there is a bit of clock drift it shouldn't accumulate bit-to bit.

To write a bit you do:

- set line output LOW
- if the bit is a 0, delay PULSE_B uS, switch the line to an input (make it high), delay PULSE_A uS
- if the bit is a 1, delay PULSE_A uS, switch the line to an input (make it high), delay PULSE_B uS

Some things of note

- Inter-byte (the bits in a byte, and bytes in a stream) timing is very lax as the receiving side just waits (till a large timeout) for the line to go LOW
- Inter-bit (for each bit) timing is more sensitive and should be as close to spec as possible

What this means is in your transmit loop do any math you need BEFORE taking the line low.

The real trick to the protocol is writing is blind.  Reading is synchronized (wait for low).  So always make sure
the consumer is ready BEFORE the producer, and for fun, these roles change often enough...As a general rule if you are switching from reading
to then writing put a delay of PULSE_LENGTH uS between the two.  The other side should be ready to read as soon as possible.

At the default 64uS pulse timing that's about 15kbit/sec (minus overhead).  Not winning any awards but gets the job done and more
importantly works off the RC internal clock when fed from the Vcc pin of a pro-micro (so like there's voltage drop, etc).  If you have a cleaner
circuit and more crucially a proper external clock you might be able to go lower than 64uS (especially if the target has 16MHz clock) but
honestly ... it's fast enough to be practical at 64uS.

You can test the link by uploading the demo_ow program to the target and then using the client's "-q" command.  If you are getting errors
try nudging PULSE_LENGTH up by increments of 4 until it's 100% stable.  Make sure you reflash the adapter (and bootloader if you're using that) after
every change to PULSE_LENGTH otherwise they won't be able to talk to each other.

# Troubleshooting the bootloader

- Usual suspects, ensure the wiring is correct, that you have a pullup, etc (it will fail without the pullup)

- Ensure you have the configuration changes required (reload the IDE after changing these)
- Ensure you burn the extended fuse correctly for instance if you don't you might see something like

```
Connecting to /dev/ttyACM0 attempt 1... SUCCESS
Target Ident: 0x85
Page 000: Reading...writing...verifying...failed!
Differing bytes...
00: 0e ff f1
01: c0 ce 0e
04: 1c 67 7b
34: a9 ab 02
35: 36 37 01
3a: 4c 97 db
3c: 5b 0b 50
3d: c0 c1 01
```

Which indicates it connected to the device, accepted a valid checksummed page but the read back failed.

- Ensure that the PULSE timing is the same on the adapter and boot (as programmed on the devices)
- Ensure there is a good pullup on the data wire
- If you have the gnd/reset/data pin still wired to your adapter even if the adapter is unplugged from usb you might find it always boots into the bootloader.  

If all that is going next check the data wire.  It should pulse after the adapter resets the device.  If you don't see this maybe you're on the wrong pin
of the adapter's MCU?

If you see that, you should see the target hold the line low for around 100uS after the adapter releases it high.

If you see that, you should see the adapter write 127 (01111111) and the target respond with 'H'

If you see that, chances are it's noise on the line.

# Troubleshooting the Client

If you don't see the "HELLO" reply try rebooting the MCU.  It can get stuck (I didn't add timeouts for everything).  Failing that be patient that app
is going to be re-written...
