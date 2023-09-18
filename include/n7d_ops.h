/**
 * 
 * Contains device operations (open, release, write)
 * 
 */

#ifndef N7D_OPS_H
#define N7D_OPS_H

#include <linux/kernel.h>
#include <linux/fs.h>

#define DATA_LEN_MAX (10)  /* Size of the device buffer TODO: change.  */

/**
 * @brief Open 
 */
int n7d_open(struct inode *, struct file *);

/**
 * @brief Release
 */
int n7d_release(struct inode *, struct file *);

/**
 * @brief Filter and write the data to the connected hardware.
 * 
 * @return Number of bytes successfully written.
 */
ssize_t n7d_write(struct file *, const char __user *, size_t, loff_t *);

#endif