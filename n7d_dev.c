/**
 * 
 * Defines N7D device initialization and cleanup.
 * 
 */

#include "include/n7d_dev.h"

/**
 * N7D device operations given as function pointers 
 */
static const struct file_operations n7d_fops = {
    .owner = THIS_MODULE,
    .open = n7d_open,
    .release = n7d_release,
    .write = n7d_write,
};

/**
 * @brief Free the data, cdev, and device for the given device.
 * 
 * @param class Pointer to the device class
 * @param dev Pointer to current device data
 * @param major Major device number
 * @param minor Minor device number
 */
void n7d_device_destroy(struct class * class, struct n7d_dev * dev, int major, int minor)
{
    dev_t number = MKDEV(major, minor);

    if (dev == NULL || class == NULL) {
        printk(KERN_ERR "n7d: n7d_device_init() failed\n");
        return;
    }

    /* Cleanup device data */
    buffer_del(&dev->buffer);
    cdev_del(&dev->cdev);

    device_destroy(class, number);
    return;
}

/**
 * @brief Initialize the device data, add to kernel and user-space
 * 
 * @param class Pointer to the device class
 * @param dev Pointer to current device data
 * @param major Major device number
 * @param minor Minor device number
 * 
 * @returns 0 on success, less than 0 on error.
 */
int n7d_device_init(struct class * class, struct n7d_dev * dev, int major, int minor)
{
    int err = 0;
    dev_t number = MKDEV(major, minor);
    struct device * device = NULL;

    if (dev == NULL || class == NULL) {
        printk(KERN_ERR "n7d: n7d_device_init() failed\n");
        return -ENODEV;
    }

    /* Initialize device buffer */
    err = buffer_init(&dev->buffer);
    if (err < 0) {
        printk(KERN_ERR "n7d: buffer_init() failed\n");
        return err;
    }

    /* Initialize cdev structure */
    cdev_init(&dev->cdev, &n7d_fops);
    dev->cdev.owner = THIS_MODULE;

    /* Add the device available operations to the kernel */
    err = cdev_add(&dev->cdev, number, 1);
    if (err < 0) {
        printk(KERN_ERR "n7d: cdev_add() failed(%d) to add %s%d", err, DEVICE_NAME, minor);
        return err;
    }

    /* Create the device to be visible to user-space */
    device = device_create(class, NULL, number, NULL, "%s%d", DEVICE_NAME, minor);
    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        printk(KERN_ERR "n7d: device_create() failed(%d) to add %s%d", err, DEVICE_NAME, minor);
        cdev_del(&dev->cdev);
        return err;
    }

    return 0;
}