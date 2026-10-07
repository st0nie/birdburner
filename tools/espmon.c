// espmon: reset ESP32 via RTS pulse, then dump serial output for N seconds.
// Usage: espmon <port> <seconds>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: espmon <port> <seconds>\n"); return 2; }
    int secs = atoi(argv[2]);

    int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return 1; }

    struct termios tio;
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= CLOCAL | CREAD;
    tcsetattr(fd, TCSANOW, &tio);

    // Classic reset: DTR=0, RTS=1 (EN low), 100ms, RTS=0
    int m;
    ioctl(fd, TIOCMGET, &m);
    m &= ~TIOCM_DTR; m |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &m);
    usleep(100 * 1000);
    m &= ~TIOCM_RTS;
    ioctl(fd, TIOCMSET, &m);

    time_t end = time(NULL) + secs;
    char buf[512];
    while (time(NULL) < end) {
        ssize_t n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            fwrite(buf, 1, n, stdout);
            fflush(stdout);
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("read");
            break;
        } else {
            usleep(20 * 1000);
        }
    }
    close(fd);
    return 0;
}
