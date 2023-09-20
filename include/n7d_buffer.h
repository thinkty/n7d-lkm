/**
 *
 * Contains device buffer related functions
 * 
 */

#ifndef N7D_DATA_H
#define N7D_DATA_H

#include <linux/printk.h>
#include <linux/slab.h>

#define BUFFER_LEN (10) /* Fixed length for the device buffer */

/**
 * struct n7d_dev_buffer - bounded buffer to temporarily store bytes
 * 
 * @param data buffer to store bytes before transmitting
 * @param size number of bytes in buffer
 * @param in insertion point
 * @param out extraction point
 * 
 * TODO: will probably need a mutex to handle multiple producers
 */
struct n7d_dev_buffer {
    char * data;
    unsigned int size;
    unsigned int in;
    unsigned int out;
};

void buffer_del(struct n7d_dev_buffer *);

int buffer_init(struct n7d_dev_buffer *);

int buffer_putc(struct n7d_dev_buffer *, char);

#endif