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
#include <linux/mutex.h>
#include <linux/ioctl.h>
#include <linux/kfifo.h>

#include "n7d_buffer.h"

#define N7D_DEVICE_NAME  "n7d"
#define N7D_DEVICE_COUNT (1) /* Single device for now */
#define N7D_DEVICE_FIFO_SIZE (256) /* Size of the kfifo circular buffer */
// #define N7D_IRQ_NO (77) /* 32 ~ 127 can be used @see irq_vectors.h */

/* IOCTL command - IO Write command to clear display */
#define N7D_CLR _IOW('a', 1, int32_t)

/* Defined in n7d_main.c */
extern unsigned int n7d_major;
extern struct n7d_dev * n7d_devices;

/**
 * struct n7d_dev - device specific data
 * 
 * @param fifo Circular buffer for storing bytes
 * @param buf_mutex Mutex for the buffer
 * @param cdev Character device
 */
struct n7d_dev {
    struct kfifo fifo;
    struct mutex buf_mutex;
    struct cdev cdev;
};

/**
 * Device Operations
 */

int n7d_open(struct inode *, struct file *);

int n7d_release(struct inode *, struct file *);

ssize_t n7d_ioctl(struct file *, unsigned int, unsigned long);

ssize_t n7d_write(struct file *, const char __user *, size_t, loff_t *);

/**
 * Device Initialization & Exit
 */

void n7d_device_destroy(struct class *, struct n7d_dev *, int, int);

int n7d_device_init(struct class *, struct n7d_dev *, int, int);

#endif