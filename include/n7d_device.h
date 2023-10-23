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
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "n7d_ioctl.h"

#define N7D_DEVICE_NAME  "n7d"
#define N7D_DEVICE_COUNT (1) /* Single device for now */
#define N7D_DEVICE_WQ "n7d-workqueue" /* Name for the device's workqueue */
#define N7D_DEVICE_FIFO_SIZE (256) /* Size of the kfifo circular buffer */
#define N7D_DEVICE_TIMEOUT (100) /* Time (ms) to wait until buffer has space */

/**
 * Configuration for the lower half of the device
 * 
 * @see https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#peripheral-addresses
 * 
 */
#define N7D_PERF_BASE (0x3f000000) 
#define N7D_GPIO_BASE (N7D_PERF_BASE + 0x200000)


/* Defined in n7d_main.c */
extern unsigned int n7d_major;
extern struct n7d_dev * n7d_devices;

/**
 * struct n7d_dev - device specific data
 * 
 * @param n7d_workqueue Queue to handle work to write the bytes to UART memory
 * @param n7d_work Work to handle sending each available byte to UART memory
 * @param writer_waitq Queue to block writers when buffer is full
 * @param work_waitq Queue to block work functions until there is data in buffer
 * @param fifo Circular buffer for storing bytes
 * @param buf_mutex Mutex for the buffer
 * @param cdev Character device
 */
struct n7d_dev {
    struct workqueue_struct * n7d_workqueue;
    struct work_struct n7d_work;
    wait_queue_head_t writer_waitq;
    wait_queue_head_t work_waitq;
    struct kfifo fifo;
    struct mutex buf_mutex;
    struct cdev cdev;
    bool closing;
};

/**
 * Device Operations
 */

int n7d_open(struct inode *, struct file *);

int n7d_release(struct inode *, struct file *);

ssize_t n7d_ioctl(struct file *, unsigned int, unsigned long);

ssize_t n7d_write(struct file *, const char __user *, size_t, loff_t *);

/**
 * Device Bottom-half Handler
 */

void n7d_work_func(struct work_struct *);

/**
 * Device Initialization & Exit
 */

void n7d_device_destroy(struct class *, struct n7d_dev *, int, int);

int n7d_device_init(struct class *, struct n7d_dev *, int, int);

#endif