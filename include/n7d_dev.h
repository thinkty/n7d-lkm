/**
 *
 * Contains N7D device initialization and cleanup.
 * 
 */

#ifndef N7D_DEV_H
#define N7D_DEV_H

#include <linux/cdev.h>
#include <linux/printk.h>
#include <linux/err.h>

#include "n7d_ops.h"

#define DEVICE_NAME "n7d"

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

void n7d_device_destroy(struct class *, struct n7d_dev *, int, int);

int n7d_device_init(struct class *, struct n7d_dev *, int, int);

#endif