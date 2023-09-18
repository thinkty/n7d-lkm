/**
 *
 * Contains device specific data and its related functions
 * 
 */

#ifndef N7D_DEV_H
#define N7D_DEV_H

#include <linux/cdev.h>

/**
 * struct n7d_dev - device specific data
 * 
 * @param data buffer to store bytes before transmitting
 * @param data_len buffer size
 * @param cdev 
 */
struct n7d_dev {
    char * data;
    unsigned int data_len;
    struct cdev cdev;
};

#endif