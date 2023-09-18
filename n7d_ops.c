/**
 * 
 * Defines the device operations
 * 
 */

#include "include/n7d_ops.h"

int n7d_open(struct inode * inode, struct file * file)
{
    printk(KERN_INFO "n7d: open");
    return 0;
}

int n7d_release(struct inode * inode, struct file * file)
{
    printk(KERN_INFO "n7d: release");
    return 0;
}

ssize_t n7d_write(struct file * file, const char __user * buf, size_t count, loff_t * offset)
{
    char data[DATA_LEN_MAX+1];
    size_t tocopy = count < DATA_LEN_MAX ? count : DATA_LEN_MAX;
    size_t copied = copy_from_user(data, buf, tocopy);

    if (copied > 0) {
        tocopy -= copied;
    }
    data[tocopy] = 0;
    printk(KERN_INFO "n7d: write %u bytes, %s\n", tocopy, data);
    return tocopy;
}
