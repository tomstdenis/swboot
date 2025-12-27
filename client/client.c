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

uint8_t flash_buffer[FLASH_SIZE];
uint8_t dirty_pages[MAX_PAGES];

int set_interface_attribs(int fd, int speed) {
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
uint16_t encode_rjmp(uint16_t current_pc, uint16_t target_pc) {
    int16_t offset = (target_pc / 2) - (current_pc / 2 + 1);
    return 0xC000 | (offset & 0x0FFF);
}

uint16_t decode_rjmp(uint16_t current_pc, uint16_t instr) {
    int16_t offset = instr & 0x0FFF;
    if (offset & 0x0800) offset |= 0xF000;
    return (current_pc / 2 + 1 + offset) * 2;
}

void parse_hex(const char *filename) {
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
    
    unsigned x, y;
    for (x = 0; x < MAX_PAGES; x++) {
		if (dirty_pages[x]) {
			printf("Page %3u dirty (0x%04x): ", x, x << 6);
			for (y = 0; y < PAGE_SIZE; y++) {
				printf("%02x ", flash_buffer[x*64 + y]);
			}
			printf("\n");
		}
	}
}

uint8_t calc_chk(uint8_t *data) {
    uint8_t sum = 0xAA;
    for (int i = 0; i < 64; i++) sum += (uint8_t)i ^ data[i];
    return sum;
}

int main(int argc, char *argv[]) {
    if (argc < 3) { printf("Usage: %s <port> <file.hex>\n", argv[0]); return 1; }

    parse_hex(argv[2]);
    
    int fd = open(argv[1], O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("Open port"); return 1; }
    set_interface_attribs(fd, B115200);
    usleep(500000); 
	tcflush(fd, TCIOFLUSH);

#if 0
	for (;;) {
		unsigned char c;
		if (read(fd, &c, 1) == 1) {
			printf("Read: %u\n", c);
		}
	}
#endif

    // 1. Handshake with 3 retries
    int connected = 0;
    for (int i = 0; i < 3; i++) {
        printf("Connection attempt %d... ", i + 1);
        uint8_t hello_cmd = 127;
        write(fd, &hello_cmd, 1);
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
    
#if 0
    // test try to read page at 0x1E00
    unsigned char buf[128];
    buf[0] = 0x80 | (0x1E00 >> 6);
    write(fd, buf, 1);
    tcdrain(fd);
        int n = 0, total = 0;
        while(total < 65 && (n = read(fd, buf + total, 65 - total)) >= 0) {
			total += n;
			printf("Read %d bytes...\n", total);
		}
    printf("Read %d bytes...\n", total);
    for (int x = 0; x < total; x++) printf("%02x ", buf[x]);
    printf("\n");
//	return 0;
#endif    
	
    // 2. Upload and Verify
    for (int p = 0; p < MAX_PAGES; p++) {
        if (!dirty_pages[p]) continue;

        uint8_t write_pkt[66];
        write_pkt[0] = p;
        memcpy(write_pkt + 1, &flash_buffer[p * PAGE_SIZE], PAGE_SIZE);
        write_pkt[65] = calc_chk(write_pkt + 1);

        printf("Page %02d: Writing...", p);
        fflush(stdout);
        write(fd, write_pkt, 66);
        tcdrain(fd);
        
        uint8_t ack = 0;
        read(fd, &ack, 1);
        if (ack != 0x54) { 
			printf("FAILED Write (0x%02X)\n", ack); 
			return 1;
		} else {
			printf("OK!...");
			fflush(stdout);
		}

        printf("Verifying... ");
		fflush(stdout);
        uint8_t read_cmd = 128 + p;
        write(fd, &read_cmd, 1);
        
        uint8_t rx_buf[65]; // 1 ack + 64 data
        int n = 0, total = 0;
        while(total < 65 && (n = read(fd, rx_buf + total, 65 - total)) > 0) total += n;

        if (rx_buf[0] == 0x54 && memcmp(rx_buf+1, &flash_buffer[p * PAGE_SIZE], PAGE_SIZE) == 0) {
            printf("OK\n");
        } else {
            printf("MISMATCH!\n");
            printf("ACK byte: %02x\n", rx_buf[0]);
            for (int x = 0; x < 64; x++) {
				printf("byte %2d: %02x %02x (%02x)\n", x, flash_buffer[p * PAGE_SIZE + x], rx_buf[1+x], flash_buffer[p * PAGE_SIZE + x] ^ rx_buf[1+x]);
			}
            return 1;
        }
    }

    uint8_t exit_cmd = 120;
    write(fd, &exit_cmd, 1);
    printf("Flash Complete. Target jumping to app.\n");
    close(fd);
    return 0;
}
