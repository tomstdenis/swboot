#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

// Flash config (defaults to 8KB globally, reset by ident_target()
static unsigned EEPROM_SIZE = 512;
static unsigned FLASH_SIZE = 8192;
static unsigned BOOT_START = 0x1E00;
static unsigned PAGE_SIZE = 64;
#define VEC_PATCH_ADDR (BOOT_START - 2)
#define MAX_PAGES (BOOT_START / PAGE_SIZE)

// support a max of 8KB total 
static uint8_t flash_buffer[8192];
static uint8_t dirty_pages[8192/64];
static uint16_t app_start;

static int set_interface_attribs(int fd, int speed) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;

    // cfmakeraw sets the terminal to a state where bytes are
    // passed exactly as received: no echo, no translations, no signals.
    cfmakeraw(&tty);

    // Set baud rate
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8-bit chars, enable receiver, ignore modem control lines
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8 | CREAD | CLOCAL;

    // Setup timing: non-blocking read with 0.5s timeout
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    return 0;
}

// Vector patching logic
static uint16_t encode_rjmp(uint16_t current_pc, uint16_t target_pc) {
    int16_t offset = (target_pc / 2) - (current_pc / 2 + 1);
    return 0xC000 | (offset & 0x0FFF);
}

static uint16_t decode_rjmp(uint16_t current_pc, uint16_t instr) {
    int16_t offset = instr & 0x0FFF;
    if (offset & 0x0800) offset |= 0xF000;
    return (current_pc / 2 + 1 + offset) * 2;
}

static uint8_t calc_chk(uint8_t *data)
{
	uint8_t buf, x, y;
	for (buf = x = 0; x < PAGE_SIZE + 1; x++) { // CRC-8 the PAGE and payload bytes
		buf ^= data[x];
		for (y = 8; y > 0; y--) {
			if (buf & 0x01) {
				buf = (buf >> 1) ^ 0x8C;
			} else {
				buf >>= 1;
			}
		}
	}
	return buf;
}

static int target_ow_packet(int fd, unsigned char *out, int outlen, unsigned char *in, int inlen)
{
	unsigned char buf[96];

	outlen &= 0x3F;
	inlen &= 0x3F;

	buf[0] = 122;
	buf[1] = outlen;
	buf[2] = inlen;
	memcpy(buf+3, out, outlen);
	if (write(fd, buf, 3+outlen) != (outlen+3)) {
		return -1;
	}
	tcdrain(fd);

	if (inlen) {
		return read(fd, in, inlen) == inlen ? 0 : -1;
	}
	return 0;
}

static int target_read_page(int fd, unsigned page_addr, unsigned char *dst)
{
	uint8_t read_cmd = 128 + page_addr;
	int n = 0, total = 0;

	if (write(fd, &read_cmd, 1) != 1) {
		return -1;
	}
	tcdrain(fd);

	while(total < (PAGE_SIZE+1) && (n = read(fd, dst + total, (PAGE_SIZE+1) - total)) > 0) {
		total += n;
	}
	return dst[0] == 0x54 ? 0 : -1;
}

static int target_write_page(int fd, unsigned page_addr, unsigned char *src)
{
	uint8_t write_pkt[66];
	write_pkt[0] = page_addr;
	memcpy(write_pkt + 1, &flash_buffer[page_addr * PAGE_SIZE], PAGE_SIZE);
	write_pkt[65] = calc_chk(write_pkt);

	if (write(fd, write_pkt, (2+PAGE_SIZE)) != (2+PAGE_SIZE)) {
		return -1;
	}
	tcdrain(fd);

    if (read(fd, write_pkt, 1) != 1) {
		return -1;
	}

	return write_pkt[0] == 0x54 ? 0 : -1;
}

static int target_read_eeprom(int fd, unsigned addr)
{
	unsigned char buf[8];

	if (addr >= EEPROM_SIZE) {
		printf("Warning: Ignoring read from invalid EEPROM address 0x%03x\n", addr);
		return -1;
	}

	buf[0] = 124; // EEPROM read
	buf[1] = addr >> 8;
	buf[2] = addr & 0xFF;
	if (write(fd, buf, 3) != 3) {
		return -1;
	}
	tcdrain(fd);

    if (read(fd, buf, 2) != 2) {
		return -1;
	}
	return buf[0] == 0x54 ? buf[1] : -1;
}

static int target_write_eeprom(int fd, unsigned addr, unsigned val)
{
	unsigned char buf[8];

	if (addr >= EEPROM_SIZE) {
		printf("Warning: Ignoring write to invalid EEPROM address 0x%03x\n", addr);
		return -1;
	}

	buf[0] = 125; // EEPROM write
	buf[1] = addr >> 8;
	buf[2] = addr & 0xFF;
	buf[3] = val;

	if (write(fd, buf, 4) != 4) {
		return -1;
	}
	tcdrain(fd);

    if (read(fd, buf, 1) != 1) {
		return -1;
	}
	return buf[0] == 0x54 ? 0 : -1;
}

static int target_reboot(int fd)
{
    uint8_t exit_cmd = 120;
    if (write(fd, &exit_cmd, 1) != 1) {
		printf("Could not write reset command to target...\n");
		return -1;
	}
	tcdrain(fd);
    printf("Target resetting...\n");
    return 0;
}

static int target_ident(int fd)
{
    uint8_t ident_cmd = 126;
    if (write(fd, &ident_cmd, 1) != 1) {
		printf("Could not query ident...\n");
		return -1;
	}
	tcdrain(fd);

    if (read(fd, &ident_cmd, 1) != 1) {
		printf("Could not read  ident response...\n");
		return -1;
	}

	// configure flash size/etc
	switch (ident_cmd) {
		case 0x84:
		case 0x85: // 8KB parts
			FLASH_SIZE = 8192;
			BOOT_START = 0x1E00;
			PAGE_SIZE = 64;
			EEPROM_SIZE = 512;
			break;
		case 0x44:
		case 0x45:
			FLASH_SIZE = 4096;
			BOOT_START = 0x0E00;
			PAGE_SIZE = 64;
			EEPROM_SIZE = 256;
			break;
		case 0x24:
		case 0x25:
			FLASH_SIZE = 2048;
			BOOT_START = 0x0600;
			PAGE_SIZE = 32;
			EEPROM_SIZE = 128;
			break;
		default:
			printf("ERROR:  Unknown ident code from target: 0x%02x\n", ident_cmd);
			target_reboot(fd);
			exit(-1);
	}

    return ident_cmd;
}

static void parse_hex(const char *filename, int patch_vector)
{
    memset(flash_buffer, 0xFF, sizeof(flash_buffer));
    memset(dirty_pages, 0, sizeof(dirty_pages));
    
    FILE *f = fopen(filename, "r");
    if (!f) { perror("Hex file"); exit(1); }
    
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] != ':') continue;
        uint8_t len, type;
        uint16_t addr;
        sscanf(line + 1, "%2hhx%4hx%2hhx", &len, &addr, &type);
        if (type == 0 && (addr + len) < BOOT_START) {
            for (int i = 0; i < len; i++) {
                sscanf(line + 9 + (i * 2), "%2hhx", &flash_buffer[addr + i]);
                dirty_pages[(addr + i) / PAGE_SIZE] = 1;
            }
        }
    }
    fclose(f);

	if (patch_vector) {
		// if the RESET vector of the code to upload is an RJMP then we patch it
		// to jump to BOOT_START instead,
		uint16_t orig = flash_buffer[0] | (flash_buffer[1] << 8);
		if ((orig & 0xF000) == 0xC000) {
			// decode the jump from 0x0000 to orig 
			app_start = decode_rjmp(0, orig);
			// store the jump to our boot loader at RESET vector
			uint16_t boot_j = encode_rjmp(0, BOOT_START);
			flash_buffer[0] = boot_j & 0xFF; flash_buffer[1] = boot_j >> 8;
		}
	}
}

static int dump_hex(int fd, char *fname)
{
	unsigned char allff[PAGE_SIZE], input[1+PAGE_SIZE];
	FILE *f;
	unsigned x, y, z;
	
	f = fopen(fname, "w");
	if (!f) {
		printf("Could not open output file: %s\n", fname);
		return -1;
	}
	memset(allff, 0xFF, PAGE_SIZE);

	printf("Dumping to %s...\n", fname);
	for (z = x = 0; x < FLASH_SIZE; x += PAGE_SIZE) {
		unsigned p = x / PAGE_SIZE;
		printf("Reading page: %3u of %3u\r", p, (FLASH_SIZE / PAGE_SIZE));
		fflush(stdout);
		if (target_read_page(fd, p, input)) {
			fclose(f);
			return -1;
		}
		if (memcmp(input+1, allff, PAGE_SIZE)) {
			++z;
			// output hex line
			fprintf(f, ":%02x%04x%02x", PAGE_SIZE, x, 0);
			for (y = 0; y < PAGE_SIZE; y++) {
				fprintf(f, "%02x", input[1+y]);
			}
			fprintf(f, "\n");
		}
	}
	printf("\nDone dumping %u pages to %s.\n", z, fname);
	fclose(f);
	return target_reboot(fd);
}

static int program_file(int fd, char *hexfname, int patch_vector)
{
	if (!patch_vector && strstr(hexfname, ".ino.hex")) {
		printf("WARNING: You elected to not patch the reset vector and are programming what is likely an Arduino sketch... hit CTRL+C to cancel in the next 10 seconds...\n");
		printf("WARNING: If you are programming an Arduino sketch use the '-p' option instead.\n");
		sleep(10);
	}

    parse_hex(hexfname, patch_vector);

    // 2. Upload and Verify
    for (int p = 0; p < MAX_PAGES; p++) {
		unsigned char page_buf[128];
        if (!dirty_pages[p]) continue;

        printf("Page %03d: Reading...", p);
		fflush(stdout);
        
        // read page and compare first 
        if (target_read_page(fd, p, page_buf)) {
			printf("Could not read page...\n");
			return -1;
		}
		
		// compare
		if (!memcmp(page_buf+1, flash_buffer + p * PAGE_SIZE, PAGE_SIZE)) {
			printf("skipping!\n");
			continue;
		}
		
		// write
		printf("writing...");
		fflush(stdout);
		if (target_write_page(fd, p, flash_buffer + p * PAGE_SIZE)) {
			printf("Could not write page...\n");
			return -1;
		}
		
		// read back
		printf("verifying...");
		fflush(stdout);
        if (target_read_page(fd, p, page_buf)) {
			printf("Could not read page...\n");
			return -1;
		}
		
		if (memcmp(page_buf + 1, flash_buffer + p * PAGE_SIZE, PAGE_SIZE)) {
			unsigned x;
			printf("failed!\nDiffering bytes...\n");
			for (x = 0; x < PAGE_SIZE; x++) {
				if (page_buf[1+x] ^ flash_buffer[p * PAGE_SIZE + x]) {
					printf("%02x: %02x %02x %02x\n", x, page_buf[1+x], flash_buffer[p * PAGE_SIZE + x], page_buf[1+x] ^ flash_buffer[p * PAGE_SIZE + x]);
				}
			}
			return -1;
		}
		printf("done.\n");
    }
    printf("Writing app start address (0x%04x) to EEPROM...", app_start);
    fflush(stdout);
    // write app_start >> 1 since IJMP uses a WORD address not byte
    if (target_write_eeprom(fd, EEPROM_SIZE - 2, (app_start>>1) & 0xFF) ||
		target_write_eeprom(fd, EEPROM_SIZE - 1, (app_start>>1) >> 8)) {
		printf("ERROR writing to EEPROM\n");
	} else {
		printf("done.\n");
	}
	return target_reboot(fd);
}

#include "../swboot.h"

static int link_test(int fd)
{
	unsigned long stride, x, y, miss, totaltests, totalmisses;
	unsigned char outbuf[9], inbuf[8];
	int rng = open("/dev/urandom", O_RDWR);
	if (rng < 0) {
		printf("Couldn't open /dev/urandom...\n");
		return -1;
	}

	printf("Testing link quality (%d uS pulses, %d uS turnaround)...\n", PULSE_LENGTH, PULSE_REVERSAL_LENGTH);
	totaltests = totalmisses = 0;
	for (stride = 1; stride < 9; stride++) {
		for (miss = x = 0; x < 5000; x++) {
			printf("Stride %2lu bytes - Test #%6lu...(%6lu)\r", stride, x, miss);
			fflush(stdout);
			outbuf[0] = stride;
			if (x < 256 * 8) {
				// ensure all 256 codes are valid
				for (y = 0; y < stride; y++) {
					outbuf[1 + y] = x & 255;
				}
			} else {
				// send bytes in random order
				if (read(rng, &outbuf[1], stride) != stride) {
					printf("Could not read from /dev/urandom\n");
					break;
				}
			}
			if (target_ow_packet(fd, outbuf, 1+stride, inbuf, stride)) {
				break;
			}
			if (memcmp(outbuf+1, inbuf, stride)) {
				++miss;
			}
		}
		printf("Stride %lu bytes with %lu samples we missed %lu times\n", stride, x, miss);
		totaltests += x;
		totalmisses += miss;
	}
	printf("Test results: %lu tests, %lu misses, link is likely %s\n", totaltests, totalmisses, totalmisses ? "not good" : "good");
	close(rng);
	return 0;
}

int main(int argc, char *argv[])
{
	int i;

    if (argc < 2) {
		printf("Usage: %s <port> [-q -r] [-f or -p or -d <file.hex>] [-re addr] [-we addr val]\n", argv[0]);
		printf(
			"\n\t-r\tReset target"
			"\n\t-q\tPerform OW link test (requires demo_ow to be loaded on target)"
			"\n\t-p\tProgram hex file and patch reset vector at 0000"
			"\n\t-f\tProgram hex file without patching reset vector (*)"
			"\n\t-d\tDump entire flash (skipping all FF's pages) to file"
			"\n\t-we\tWrite a byte to EEPROM"
			"\n\t-re\tRead a byte from EEPROM"
			"\n\n*\tIf you skip patching the reset vector and you program something"
			"\n\tyou didn't dump with -d you will disable the bootloader!!! Normally"
			"\n\tyou will want to use -p to program Arduino sketches using this system.\n"
			);
		return 1;
	}

    int fd = open(argv[1], O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("Open port"); return 1; }
    set_interface_attribs(fd, B115200);
    usleep(1000000);
	tcflush(fd, TCIOFLUSH);

    // 1. Handshake with 3 retries
    int connected = 0;
    for (int i = 0; i < 3; i++) {
        printf("Connecting to %s attempt %d... ", argv[1], i + 1);
        uint8_t hello_cmd = 127;
        if (write(fd, &hello_cmd, 1) != 1) {
			printf("Could not write hello command to adapter...\n");
			return -1;
		}
		tcdrain(fd);
        char response[6] = {0};
        if (read(fd, response, 5) > 0 && strcmp(response, "HELLO") == 0) {
            printf("SUCCESS\n");
            connected = 1;
            break;
        }
        printf("Timed out (or failed to read... [%s]).\n", response);
        tcflush(fd, TCIOFLUSH);
        sleep(1);
    }
    if (!connected) { printf("Could not find swadapter.\n"); return 1; }

    // perform ident to get flash/eeprom size
    i = target_ident(fd);
    if (i < 0) {
		printf("Could not identify target, aborting...\n");
		return -1;
	}
    printf("Target Ident: 0x%02x\n", i);

	for (i = 2; i < argc; i++) {
		if (!strcmp(argv[i], "-r")) {
			target_reboot(fd);
		} else if (!strcmp(argv[i], "-q")) {
			target_reboot(fd);
			link_test(fd);
		} else if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "-f")) {
			if (i + 1 < argc) {
				program_file(fd, argv[i+1], argv[i][1] == 'p' ? 1 : 0);
				++i;
			} else {
				printf("-p/-f requires one parameter\n");
				return -1;
			}
		} else if (!strcmp(argv[i], "-d")) {
			if (i + 1 < argc) {
				dump_hex(fd, argv[i+1]);
				++i;
			} else {
				printf("-d requires one parameter\n");
				return -1;
			}
		} else if (!strcmp(argv[i], "-re")) {
			if (i + 1 < argc) {
				unsigned addr;
				int val;
				sscanf(argv[i+1], "%x", &addr);
				val = target_read_eeprom(fd, addr);
				if (val < 0) {
					printf("Could not read from EEPROM.\n");
					return -1;
				} else {
					printf("0x%02x\n", val);
				}
				++i;
			} else {
				printf("-re requires one parameter\n");
				return -1;
			}
		} else if (!strcmp(argv[i], "-we")) {
			if (i + 2 < argc) {
				unsigned addr, val;
				sscanf(argv[i+1], "%x", &addr);
				sscanf(argv[i+2], "%x", &val);
				if (target_write_eeprom(fd, addr, val)) {
					printf("Could not write to EEPROM.\n");
					return -1;
				}
				i += 2;
			}
		}
	}
    return 0;
}
