#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define FLASH_SIZE 8192
#define BOOT_START 0x1E00
#define VEC_PATCH_ADDR 0x1DFE
#define PAGE_SIZE 64
#define MAX_PAGES (BOOT_START / PAGE_SIZE)

static uint8_t flash_buffer[FLASH_SIZE];
static uint8_t dirty_pages[MAX_PAGES];

static int set_interface_attribs(int fd, int speed) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 10;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

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

static int read_page(int fd, unsigned page_addr, unsigned char *dst)
{
	uint8_t read_cmd = 128 + page_addr;
	int n = 0, total = 0;

	if (write(fd, &read_cmd, 1) != 1) {
		return -1;
	}
	tcdrain(fd);

	while(total < 65 && (n = read(fd, dst + total, 65 - total)) > 0) {
		total += n;
	}
	return dst[0] == 0x54 ? 0 : -1;
}

static uint8_t calc_chk(uint8_t *data) {
    uint8_t sum = 0xAA;
    for (int i = 0; i < 64; i++) sum += (uint8_t)i ^ data[i];
    return sum;
}

static int write_page(int fd, unsigned page_addr, unsigned char *src)
{
	uint8_t write_pkt[66];
	write_pkt[0] = page_addr;
	memcpy(write_pkt + 1, &flash_buffer[page_addr * PAGE_SIZE], PAGE_SIZE);
	write_pkt[65] = calc_chk(write_pkt + 1);

	if (write(fd, write_pkt, 66) != 66) {
		return -1;
	}
	tcdrain(fd);

    if (read(fd, write_pkt, 1) != 1) {
		return -1;
	}

	return write_pkt[0] == 0x54 ? 0 : -1;
}

static int reboot_target(int fd)
{
    uint8_t exit_cmd = 120;
    if (write(fd, &exit_cmd, 1) != 1) {
		printf("Could not write reset command to target...\n");
		return -1;
	}
    printf("Target resetting...\n");
    return 0;
}

static void parse_hex(const char *filename, int patch_vector)
{
    memset(flash_buffer, 0xFF, FLASH_SIZE);
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
		// to jump to 0x1E00 instead, we put a re-encoded jump at 0x1DFE and
		// jump there if the bootloader isn't being used.
		uint16_t orig = flash_buffer[0] | (flash_buffer[1] << 8);
		if ((orig & 0xF000) == 0xC000) {
			// decode the jump from 0x0000 to orig 
			uint16_t app_start = decode_rjmp(0, orig);
			// the patched RJMP relative to 0x1DFE
			uint16_t patched = encode_rjmp(VEC_PATCH_ADDR, app_start);
			// store the jump to the application
			flash_buffer[VEC_PATCH_ADDR] = patched & 0xFF;
			flash_buffer[VEC_PATCH_ADDR+1] = patched >> 8;
			dirty_pages[VEC_PATCH_ADDR / PAGE_SIZE] = 1;
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
		printf("Reading page: %3u...\r", p);
		fflush(stdout);
		if (read_page(fd, p, input)) {
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
	return reboot_target(fd);
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
        if (read_page(fd, p, page_buf)) {
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
		if (write_page(fd, p, flash_buffer + p * PAGE_SIZE)) {
			printf("Could not write page...\n");
			return -1;
		}
		
		// read back
		printf("verifying...");
		fflush(stdout);
        if (read_page(fd, p, page_buf)) {
			printf("Could not read page...\n");
			return -1;
		}
		
		if (memcmp(page_buf + 1, flash_buffer + p * PAGE_SIZE, PAGE_SIZE)) {
			printf("failed!\n");
			return -1;
		}
		printf("done.\n");
    }
	return reboot_target(fd);
}	

int main(int argc, char *argv[])
{
    if (argc < 4) { 
		printf("Usage: %s [-f or -p or -d] <port> <file.hex>\n", argv[0]); 
		printf(
			"\n\t-p\tProgram hex file and patch reset vector at 0000"
			"\n\t-f\tProgram hex file without patching reset vector (*)"
			"\n\t-d\tDump entire flash (skipping all FF's pages) to file\n"
			"\n*\tIf you skip patching the reset vector and you program something"
			"\n\tyou didn't dump with -d you will disable the bootloader!!! Normally"
			"\n\tyou will want to use -p to program Arduino sketches using this system.\n"
			);
		return 1;
	}

    int fd = open(argv[2], O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("Open port"); return 1; }
    set_interface_attribs(fd, B115200);
    usleep(250000); 
	tcflush(fd, TCIOFLUSH);

    // 1. Handshake with 3 retries
    int connected = 0;
    for (int i = 0; i < 3; i++) {
        printf("Connection attempt %d... ", i + 1);
        uint8_t hello_cmd = 127;
        if (write(fd, &hello_cmd, 1) != 1) {
			printf("Could not write hello command to adapter...\n");
			return -1;
		}
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

	if (!strcmp(argv[1], "-p") || !strcmp(argv[1], "-f")) {
		return program_file(fd, argv[3], argv[1][1] == 'p' ? 1 : 0);
	}
	if (!strcmp(argv[1], "-d")) {
		return dump_hex(fd, argv[3]);
	}
    return 0;
}
