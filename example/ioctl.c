#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#include "../include/n7d_ioctl.h"

int main(int argc, char * argv[])
{
    int fd = open("/dev/n7d0", O_WRONLY);
    if (fd < 0) {
        printf("Unable to open device\n");
        return 1;
    }

    ioctl(fd, N7D_CLR);
    close(fd);
    return 0;
}