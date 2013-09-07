#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <termio.h>
#include <fcntl.h>
#include <err.h>
#include <unistd.h>

#include <linux/serial.h>

#include "mspsa430_lld.h"

static int rate_to_constant(int baudrate) {
#define B(x) case x: return B##x
    switch(baudrate) {
        B(50);     B(75);     B(110);    B(134);    B(150);
        B(200);    B(300);    B(600);    B(1200);   B(1800);
        B(2400);   B(4800);   B(9600);   B(19200);  B(38400);
        B(57600);  B(115200); B(230400); B(460800); B(500000);
        B(576000); B(921600); B(1000000);B(1152000);B(1500000);
    default: return 0;
    }
#undef B
}

int8_t mspsa430_lld_open(mspsa430_lld_t *m, const char *device, int speed) {
    struct termios options;
    struct serial_struct serinfo;
    int fd;

    /* Open and configure serial port */
    if ((fd = open(device,O_RDWR|O_NOCTTY)) == -1)
        return -1;

    speed = rate_to_constant(speed);

    if (speed == 0) {
        /* Custom divisor */
        serinfo.reserved_char[0] = 0;
        if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
            return -1;
        serinfo.flags &= ~ASYNC_SPD_MASK;
        serinfo.flags |= ASYNC_SPD_CUST;
        serinfo.custom_divisor = (serinfo.baud_base + (speed / 2)) / speed;
        if (serinfo.custom_divisor < 1)
            serinfo.custom_divisor = 1;
        if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0)
            return -1;
        if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
            return -1;
        if (serinfo.custom_divisor * speed != serinfo.baud_base) {
            warnx("actual baudrate is %d / %d = %f",
                serinfo.baud_base, serinfo.custom_divisor,
                (float)serinfo.baud_base / serinfo.custom_divisor);
        }
    }

    fcntl(fd, F_SETFL, 0);
    tcgetattr(fd, &options);
    cfsetispeed(&options, speed ?: B38400);
    cfsetospeed(&options, speed ?: B38400);
    cfmakeraw(&options);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ECHO | ECHONL);

    if (tcsetattr(fd, TCSANOW, &options) != 0)
        return -1;
    if (tcsetattr(fd, TCSAFLUSH, &options) != 0)
        return -1;

    m->device = strdup(device);
    m->speed = speed;
    m->fd = fd;

    return 0;
}

void mspsa430_lld_close(mspsa430_lld_t *m) {
    // TODO: we might want to restore original tty settings

    if (m) {
        close(m->fd);

        m->speed = 0;

        if (m->device) {
            free(m->device);
        }
    }
}

ssize_t mspsa430_lld_read(mspsa430_lld_t *m, uint8_t *buffer, size_t buflen) {
    return read(m->fd, buffer, buflen);
}

ssize_t mspsa430_lld_write(mspsa430_lld_t *m, uint8_t *buffer, size_t buflen) {
    int i=0;
    printf("\nbuflen: %d\n", buflen);
    for (i=0; i<buflen; i++) {
        printf("%02X", buffer[i]);
    }
    printf("\n");
    return write(m->fd, buffer, buflen);
}


