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
    .unlocked_ioctl = n7d_ioctl,
};

/**
 * @brief Open device and set the appropriate device data
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
 * @brief Release is called when the device file descriptor is closed. However,
 * the deallocation of buffer and device related data structures is done in exit
 * module since the display is persistent. 
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
 * @brief Handle ioctl requests (clear screen) to the driver
 * 
 * @param filp Pointer to the open file for the device
 * @param cmd Ioctl command
 * @param arg Arguments for the command
 * 
 * @returns 0 on success, less than 0 on error.
 */
ssize_t n7d_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
    switch (cmd) {
        case N7D_CLR:
            printk(KERN_INFO "n7d: Clear display\n");
            // TODO: send a special (non-numerical) character through UART 
            break;
        
        default:
            return -ENOTTY;
    }

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
    char tbuf[N7D_DEVICE_FIFO_SIZE] = {0};
    size_t to_copy = 0;
    size_t not_copied = 0;
    int i = 0;
    int err = 0;
    struct n7d_dev * dev = (struct n7d_dev *) filp->private_data;

    /* At maximum, the size of buffer */
    to_copy = count < N7D_DEVICE_FIFO_SIZE ? count : N7D_DEVICE_FIFO_SIZE;
    not_copied = copy_from_user(tbuf, buf, to_copy);
    to_copy -= not_copied;

    /* Check that it's all numerical */
    for (i = 0; i < to_copy; i++) {
        if (tbuf[i] < '0' || tbuf[i] > '9') {
            err = -EINVAL;
            return err;
        }
    }

    /* By being interruptible, when given any signal, the process will just
    give up on acquiring the lock and return -EINTR. */
    err = mutex_lock_interruptible(&dev->buf_mutex);
    if (err < 0) {
        return err;
    }

    /* Depending on the buffer vacancy, it might write less than specified */
    to_copy = kfifo_in(&dev->fifo, tbuf, to_copy);
    mutex_unlock(&dev->buf_mutex);

    mutex_unlock(&dev->buf_mutex);

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
    kfifo_free(&dev->fifo);
    mutex_destroy(&dev->buf_mutex);
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
    err = kfifo_alloc(&dev->fifo, N7D_DEVICE_FIFO_SIZE, GFP_KERNEL);
    if (err != 0) {
        printk(KERN_ERR "n7d: kfifo_alloc() failed\n");
        return err;
    }

    /* Initialize buffer mutex */
    mutex_init(&dev->buf_mutex);

    /* Initialize cdev structure */
    cdev_init(&dev->cdev, &n7d_fops);
    dev->cdev.owner = THIS_MODULE;

    /* Add the device available operations to the kernel */
    err = cdev_add(&dev->cdev, number, 1);
    if (err < 0) {
        printk(KERN_ERR "n7d: cdev_add() failed(%d) to add %s%d", err, N7D_DEVICE_NAME, minor);
        mutex_destroy(&dev->buf_mutex);
        kfifo_free(&dev->fifo);
        return err;
    }

    /* Create the device to be visible to user-space */
    device = device_create(class, NULL, number, NULL, "%s%d", N7D_DEVICE_NAME, minor);
    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        printk(KERN_ERR "n7d: device_create() failed(%d) to add %s%d", err, N7D_DEVICE_NAME, minor);
        mutex_destroy(&dev->buf_mutex);
        kfifo_free(&dev->fifo);
        cdev_del(&dev->cdev);
        return err;
    }

    return 0;
}