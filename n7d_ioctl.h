/**
 * 
 * Contains N7D device ioctl configurations
 * 
 */

#include <linux/ioctl.h>

/* see include/asm/ioctl.h and Documentation/ioctl-number.txt to avoid conflicting ioctl numbers */
#define N7D_IOC_MAGIC_NUMBER (0x07)

/* IO None command to clear display */
#define N7D_CLR _IO(N7D_IOC_MAGIC_NUMBER, 1)

/* Command characters corresponding to each command. These will be inserted to kfifo */
#define N7D_CLR_CHAR 'C'