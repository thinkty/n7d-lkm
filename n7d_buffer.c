/**
 *
 * Defines device buffer related functions
 * 
 */

#include "include/n7d_buffer.h"

/**
 * @brief Cleanup device buffer
 * 
 * @param buffer Pointer to current device's buffer
 */
void buffer_del(struct n7d_dev_buffer * buffer)
{
    kfree(buffer->data);
    buffer->data = NULL;
    buffer->in = 0;
    buffer->out = 0;
    buffer->size = 0;

    // TODO: flush? wait for all bytes to be transferred?

    printk(KERN_INFO "n7d: clearing buffer\n");
    return;
}

/**
 * @brief Initialize device buffer
 * 
 * @param buffer Pointer to current device's buffer
 * 
 * @returns 0 on success, less than 0 on error.
 */
int buffer_init(struct n7d_dev_buffer * buffer)
{
    int err = 0;

    buffer->data = (char *) kzalloc(BUFFER_LEN * sizeof(char), GFP_KERNEL);
    if (buffer->data == NULL) {
        err = -ENOMEM;
        return err;
    }

    buffer->in = 0;
    buffer->out = 0;
    buffer->size = 0;

    printk(KERN_INFO "n7d: buffer initialized\n");
    return 0;
}

/**
 * @brief Put the given character into buffer
 * 
 * @param buffer Pointer to current device's buffer
 * @param c Character to put to buffer
 * 
 * @returns 0 on success, less than 0 on error.
 */
int buffer_putc(struct n7d_dev_buffer * buffer, char c)
{
    buffer->data[buffer->in] = c;
    
    /* Increment size */
    buffer->size += 1;
    if (buffer->size >= BUFFER_LEN) {
        // TODO: signal the consumer when buffer is full
        buffer->size = BUFFER_LEN;
    }

    /* Move the insertion point */
    buffer->in += 1;
    if (buffer->in >= BUFFER_LEN) {
        buffer->in = 0;
    }

    // TODO: print contents of buffer to kernel for now
    printk(KERN_INFO "n7d: putc() \"%c%c%c%c%c%c%c%c%c%c\"\n", buffer->data[0], buffer->data[1], buffer->data[2],buffer->data[3],buffer->data[4],buffer->data[5],buffer->data[6],buffer->data[7],buffer->data[8],buffer->data[9]);
    
    return 0;
}
