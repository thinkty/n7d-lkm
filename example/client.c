#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#include "../n7d_ioctl.h"

int main(int argc, char * argv[])
{
    int fd = open("/dev/n7d", O_WRONLY);
    if (fd < 0) {
        printf("Unable to open device\n");
        return 1;
    }
    printf("Opened device\n");
    printf("Type 'quit' to quit\n");

    char buffer[64];
    while (1) {
        printf("> ");
        scanf("%63s", buffer);
        if (strncmp(buffer, "quit", 4) == 0) {
            break;
        }
        printf("[%u]\n", (unsigned int) write(fd, buffer, strlen(buffer)));
    }

    printf("Closing device\n");
    close(fd);
    return 0;
}