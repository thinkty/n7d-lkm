/**
 *
 * Contains N7D device initialization, cleanup, and device operations.
 * 
 */

#ifndef N7D_DEV_H
#define N7D_DEV_H

#include <linux/cdev.h>
#include <linux/printk.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/minmax.h>
#include <linux/uaccess.h>

#include "n7d_buffer.h"

#define N7D_DEVICE_NAME  "n7d"
#define N7D_DEVICE_COUNT (1) /* Single device for now */

/* Defined in n7d_main.c */
extern unsigned int n7d_major;
extern struct n7d_dev * n7d_devices;

/**
 * struct n7d_dev - device specific data
 * 
 * @param buffer Device buffer for storing bytes temporarily
 * @param cdev 
 */
struct n7d_dev {
    struct n7d_dev_buffer buffer;
    struct cdev cdev;
};

/**
 * Device Operations
 */

int n7d_open(struct inode *, struct file *);

int n7d_release(struct inode *, struct file *);

ssize_t n7d_write(struct file *, const char __user *, size_t, loff_t *);

// TODO: iocntl to send commands like CLEAR to clear the display

/**
 * Device Initialization & Exit
 */

void n7d_device_destroy(struct class *, struct n7d_dev *, int, int);

int n7d_device_init(struct class *, struct n7d_dev *, int, int);

#endif