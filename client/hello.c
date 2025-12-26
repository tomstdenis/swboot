#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

int set_interface_attribs(int fd, int speed) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 64;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    return 0;
}

int main(int argc, char *argv[]) {
    char *portname = "/dev/ttyACM0"; // Default, or use argv[1]
    if (argc > 1) portname = argv[1];

    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "Error opening %s: %s\n", portname, strerror(errno));
        return 1;
    }

    set_interface_attribs(fd, B9600);
    
    // Most Arduino-like devices reset when the port opens.
    // Give the ATtiny84 a second to boot up before we talk to it.
    printf("Connecting to adapter on %s...\n", portname);
    usleep(2000000); 
	tcflush(fd, TCIOFLUSH);

    unsigned char cmd = 127;
    write(fd, &cmd, 1);
    printf("Sent HELLO (127), waiting for response...\n");

    char buf[10];
    memset(buf, 0, sizeof(buf));
    int n = read(fd, buf, sizeof(buf) - 1);

    if (n > 0) {
        printf("Adapter Response: (%d bytes) %s: ", n, buf);
        for (int x = 0; x < n; x++) printf("%02x ", buf[x]);
        printf("\n");
        if (strcmp(buf, "HELLO") == 0) {
            printf("Handshake SUCCESS!\n");
        } else {
            printf("Handshake FAILED (Unexpected response).\n");
        }
    } else {
        printf("Error: No response from adapter. Check wiring/baud rate.\n");
    }

    close(fd);
    return 0;
}
