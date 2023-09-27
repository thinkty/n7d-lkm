#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#define N7D_CLR _IOW('a', 1, int32_t)

int main(int argc, char * argv[])
{
    int fd = open("/dev/n7d0", O_WRONLY);
    if (fd < 0) {
        printf("Unable to open device\n");
        return 1;
    }

    int32_t buf;
    ioctl(fd, N7D_CLR, (int32_t *) &buf);
    close(fd);
    return 0;
}