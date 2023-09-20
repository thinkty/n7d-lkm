/**
 * 
 * Defines N7D device initialization, cleanup, and device operations.
 * 
 */

#include "include/n7d_device.h"

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
 * @brief Open TODO:
 * 
 * @param inode Pointer to the file containing device metadata
 * @param filp Pointer to the open file for the device
 * 
 * @returns 0 on success, less than 0 on error.
 */
int n7d_open(struct inode * inode, struct file * filp)
{
    unsigned int major = imajor(inode);
    unsigned int minor = iminor(inode);
    struct n7d_dev * dev = NULL;

    /* Check device numbers */
    if (major != n7d_major || minor >= N7D_DEVICE_COUNT) {
        printk(KERN_WARNING "n7d: Device %s%u with major %u not found\n", N7D_DEVICE_NAME, minor, major);
        return -ENODEV;
    }

    /* Assign the device to the file so that other fops can use it */
    dev = &n7d_devices[minor];
    filp->private_data = dev;

    /* Check device */
    if (inode->i_cdev != &dev->cdev) {
        printk(KERN_WARNING "n7d: cdev does not match (internal error)\n");
        return -ENODEV;
    }

    printk(KERN_INFO "n7d: opened %s%u\n", N7D_DEVICE_NAME, minor);
    return 0;
}

/**
 * @brief Release
 * 
 * @param inode Pointer to the file containing device metadata
 * @param filp Pointer to the open file for the device
 * 
 * @returns 0
 */
int n7d_release(struct inode * inode, struct file * filp)
{
	unsigned int mn = iminor(inode);

    printk(KERN_INFO "n7d: released %s%u\n", N7D_DEVICE_NAME, mn);
    return 0;
}

/**
 * @brief Check user input and write to the device buffer.
 * 
 * @param filp Pointer to the open file for the device
 * @param buf User-space buffer
 * @param count Number of bytes to write
 * @param f_pos Not used
 * 
 * @returns Number of bytes successfully written or a negative errno value.
 */
ssize_t n7d_write(struct file * filp, const char __user * buf, size_t count, loff_t * f_pos)
{
    char tbuf[BUFFER_LEN] = {0};
    size_t to_copy = 0;
    size_t not_copied = 0;
    int i = 0;
    int err = 0;
    struct n7d_dev * dev = (struct n7d_dev *) filp->private_data;

    /* No matter the given number of bytes, only take by the size of the buffer */
    to_copy = count < BUFFER_LEN ? count : BUFFER_LEN;
    not_copied = copy_from_user(tbuf, buf, to_copy);
    to_copy -= not_copied;
    
    /* Check that it's all numerical */
    for (i = 0; i < to_copy; i++) {

        /* Allow space for testing purposes */
        if (tbuf[i] == '\0') {
            continue;
        }

        if (tbuf[i] < '0' || tbuf[i] > '9') {
            err = -EINVAL;
            return err;
        }
    }

    /* Write to device buffer */
    for (i = 0; i < to_copy; i++) {
        err = buffer_putc(&dev->buffer, tbuf[i]);
        if (err < 0) {
            return err;
        }
    }

    return to_copy;
}

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
        printk(KERN_ERR "n7d: cdev_add() failed(%d) to add %s%d", err, N7D_DEVICE_NAME, minor);
        return err;
    }

    /* Create the device to be visible to user-space */
    device = device_create(class, NULL, number, NULL, "%s%d", N7D_DEVICE_NAME, minor);
    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        printk(KERN_ERR "n7d: device_create() failed(%d) to add %s%d", err, N7D_DEVICE_NAME, minor);
        cdev_del(&dev->cdev);
        return err;
    }

    return 0;
}