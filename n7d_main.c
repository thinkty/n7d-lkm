/*
 * 
 * Numerical 7-segment display device driver
 * 
 */

#include <linux/cdev.h>
#include <linux/device/class.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "n7d_ioctl.h"

#define N7D_DEVICE_NAME        "n7d"
#define N7D_DEVICE_WORKQUEUE   "n7d-workqueue"
#define N7D_DEVICE_FIFO_SIZE   (8)
#define N7D_DEVICE_TIMEOUT     (100)


static int n7d_baudrate = 38400;
module_param(n7d_baudrate, int, 0);
MODULE_PARM_DESC(n7d_baudrate, "\tBaudrate of the device (default=38400)");

/**
 * Function prototypes for the char driver
 */
static int n7d_open(struct inode * inode, struct file * filp);
static int n7d_release(struct inode * inode, struct file * filp);
static ssize_t n7d_write(struct file * filp, const char __user * buf, size_t count, loff_t * f_pos);
static long n7d_ioctl(struct file * filp, unsigned int cmd, unsigned long arg);

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
 * n7d_device_data - device specific data
 * 
 * @param workqueue Queue to handle work to write the bytes to UART memory
 * @param work Work to handle sending each available byte to UART memory
 * @param writer_waitq Queue to block writers when buffer is full
 * @param work_waitq Queue to block work functions until there is data in buffer
 * @param fifo Circular buffer for storing bytes
 * @param buf_mutex Mutex for the buffer
 * @param cdev Character device
 * @param closing Used to stop works from self-queueing
 * @param rx GPIO descriptor for receival
 * @param tx GPIO descriptor for transmission
 */
struct n7d_dd {
    struct workqueue_struct * workqueue;
    struct work_struct work;
    wait_queue_head_t writer_waitq;
    wait_queue_head_t work_waitq;
    struct kfifo fifo;
    struct mutex buf_mutex;
    struct cdev cdev;
    bool closing;
    struct gpio_desc * tx;
};

static struct n7d_dd n7d_device_data; // TODO: instead of global var, allocate
static unsigned int n7d_major = 0;
static struct class * n7d_class = NULL;
static struct device * n7d_device = NULL;

/**
 * @brief Open device and set the appropriate device data
 * 
 * @param inode Pointer to the file containing device metadata
 * @param filp Pointer to the open file for the device
 * 
 * @returns 0 on success, less than 0 on error.
 */
static int n7d_open(struct inode * inode, struct file * filp)
{
    unsigned int major = imajor(inode);
    unsigned int minor = iminor(inode);
    struct n7d_dd * dev_data = NULL;

    /* Check device numbers */
    if (major != n7d_major) {
        printk(KERN_WARNING "n7d: Device %s%u with major %u not found\n", N7D_DEVICE_NAME, minor, major);
        return -ENODEV;
    }

    /* Assign the device to the file so that other fops can use it */
    dev_data = &n7d_device_data;
    filp->private_data = dev_data;

    /* Check device */
    if (inode->i_cdev != &dev_data->cdev) {
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
static int n7d_release(struct inode * inode, struct file * filp)
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
static ssize_t n7d_write(struct file * filp, const char __user * buf, size_t count, loff_t * f_pos)
{
    char tbuf[N7D_DEVICE_FIFO_SIZE] = {0};
    size_t to_copy = 0;
    size_t not_copied = 0;
    int i = 0;
    int err = 0;
    struct n7d_dd * dev_data = (struct n7d_dd *) filp->private_data;

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
    err = mutex_lock_interruptible(&dev_data->buf_mutex);
    if (err < 0) {
        return err;
    }

    /* Check in a loop since another writer may get to kfifo first. This could
    potentially be a thundering herd problem */
    while (kfifo_is_full(&dev_data->fifo) && !dev_data->closing) {
        mutex_unlock(&dev_data->buf_mutex);

        /* If non-blocking, return immediately */
        if (filp->f_flags & O_NDELAY || filp->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }

        /* Sleep until space available, timeout, interrupt, or closing device */
        err = wait_event_interruptible_hrtimeout(dev_data->writer_waitq,
                                                 dev_data->closing || !kfifo_is_full(&dev_data->fifo),
                                                 ms_to_ktime(N7D_DEVICE_TIMEOUT));
        if (err < 0) {
            return err;
        }

        err = mutex_lock_interruptible(&dev_data->buf_mutex);
        if (err < 0) {
            return err;
        }
    }

    /* If closing device, discard the given bytes and return error */
    if (dev_data->closing) {
        mutex_unlock(&dev_data->buf_mutex);
        return -ECANCELED;
    }

    /* Depending on the buffer vacancy, it might write less than specified */
    to_copy = kfifo_in(&dev_data->fifo, tbuf, to_copy);
    mutex_unlock(&dev_data->buf_mutex);

    /* Wake up work waitqueue since there are bytes available */
    wake_up_interruptible(&dev_data->work_waitq);

    return to_copy;
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
static long n7d_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
    // struct n7d_dd * dev_data = (struct n7d_dd *) filp->private_data;

    switch (cmd) {
        case N7D_CLR:
            printk(KERN_INFO "n7d: Clear display\n");
            // TODO: send a special (non-numerical) character
            break;
        
        default:
            return -ENOTTY;
    }

    return 0;
}

/**
 * @brief Bottom half of the device driver to process available bytes in the
 * device buffer to send via UART TX
 * 
 * @param work Pointer to the work structure
 */
static void n7d_work_func(struct work_struct * work)
{
    int err, byte;
    struct n7d_dd * dev_data = container_of(work, struct n7d_dd, work);

    /* Sleep until there is something in fifo or the device is unloading */
    err = wait_event_interruptible(dev_data->work_waitq,
                                   dev_data->closing || !kfifo_is_empty(&dev_data->fifo));
    if (err < 0 || dev_data->closing) {
        /* If interrupted or woke up to close the device, stop the work */
        return;
    }

    /* Since there is only 1 actual HW per device to write to, no locks */
    err = kfifo_get(&dev_data->fifo, &byte);
    if (err == 0) {
        printk(KERN_WARNING "n7d: kfifo_get() says empty although it shouldn't be\n");
        queue_work(dev_data->workqueue, &dev_data->work);
        return;
    }

    // TODO: process the byte (just toggling for now)
    printk(KERN_INFO "n7d: processing '%c'\n", byte);
    gpiod_set_value(dev_data->tx, !gpiod_get_value(dev_data->tx));

    /* Wake up writer_waitq since new space is available */
    wake_up_interruptible(&dev_data->writer_waitq);

    // Self-requeueing work
    queue_work(dev_data->workqueue, &dev_data->work);
}

/**
 * @brief Callback function for managing environment variables for the device.
 * Here, it is used to add the permissions for the device.
 * 
 * @param dev Pointer to the device
 * @param env Device's environment buffer structure
 * 
 * @returns 0 on success, -ENOMEM if no memory available.
 */
static int n7d_uevent(struct device * dev, struct kobj_uevent_env * env)
{
    int err = 0;

    /* Set the permissions as read-write for all */
    err = add_uevent_var(env, "DEVMODE=%#o", 0666);
    if (err < 0) {
        printk(KERN_ERR "n7d: n7d_uevent() failed to set permission\n");
        return err;
    }

    return 0;
}

/**
 * @brief Initialize the device by allocating its numbers (major, minor), and
 * creating a character device, and filling the device data with relevant data
 * structures.
 * 
 * @param pdev Platform device
 * 
 * @returns 0 on success, less than 0 on error.
 */
static int n7d_dt_probe(struct platform_device *pdev)
{
    dev_t dev_num;
    int err = 0;

    /* Check that the device exists before attaching device driver */
    if (!device_property_present(&pdev->dev, "exists")) {
        err = -ENODEV;
        printk(KERN_ERR "n7d: device does not exist\n");
        goto DT_PROBE_DEV_EXISTS;
    }

    /* Allocate the appropriate device major number and a set of minor numbers */
    err = alloc_chrdev_region(&dev_num, 0, 1, N7D_DEVICE_NAME);
    if (err < 0) {
        printk(KERN_ERR "n7d: alloc_chrdev_region() failed\n");
        goto DT_PROBE_REG_CHRDEV;
    }

    n7d_major = MAJOR(dev_num);

    /* Create the device class in sysfs to add the device node visible in /dev */
    n7d_class = class_create(THIS_MODULE, N7D_DEVICE_NAME);
    if (IS_ERR(n7d_class)) {
        err = PTR_ERR(n7d_class);
        printk(KERN_ERR "n7d: class_create() failed\n");
        goto DT_PROBE_CLS_CRT;
    }
    n7d_class->dev_uevent = n7d_uevent;

    /* Initialize device buffer */
    err = kfifo_alloc(&n7d_device_data.fifo, N7D_DEVICE_FIFO_SIZE, GFP_KERNEL);
    if (err != 0) {
        printk(KERN_ERR "n7d: kfifo_alloc() failed\n");
        goto DT_PROBE_KFIFO_ALLOC;
    }

    /* Create workqueue and waitqueues for the device to handle writing bytes */
    n7d_device_data.workqueue = alloc_ordered_workqueue(N7D_DEVICE_WORKQUEUE, 0);
    if (n7d_device_data.workqueue == NULL) {
        printk(KERN_ERR "n7d: alloc_ordered_workqueue() failed to allocate workqueue");
        err = -ENOMEM; /* However, there are multiple reasons to fail */
        goto DT_PROBE_ALLOC_WQ;
    }

    init_waitqueue_head(&n7d_device_data.work_waitq);
    init_waitqueue_head(&n7d_device_data.writer_waitq);
    mutex_init(&n7d_device_data.buf_mutex);

    /* Make the device available to the kernel */
    cdev_init(&n7d_device_data.cdev, &n7d_fops);
    n7d_device_data.cdev.owner = THIS_MODULE;
    err = cdev_add(&n7d_device_data.cdev, dev_num, 1);
    if (err < 0) {
        printk(KERN_ERR "n7d: cdev_add() failed(%d) to add %s0\n", err, N7D_DEVICE_NAME);
        goto DT_PROBE_CDEV_ADD;
    }

    /* Make the device available to user space (/dev) */
    n7d_device = device_create(n7d_class, NULL, dev_num, NULL, "%s%d", N7D_DEVICE_NAME, 0);
    if (IS_ERR(n7d_device)) {
        err = PTR_ERR(n7d_device);
        printk(KERN_ERR "n7d: device_create() failed(%d) to add %s%d", err, N7D_DEVICE_NAME, 0);
        goto DT_PROBE_DEVICE_CREATE;
    }

    /* Get the GPIO descriptors from the pin numbers */
    n7d_device_data.tx = devm_gpiod_get(&pdev->dev, "serial", GPIOD_OUT_HIGH);
    if (IS_ERR(n7d_device_data.tx)) {
        err = PTR_ERR(n7d_device_data.tx);
        printk(KERN_ERR "n7d: devm_gpiod_get() failed\n");
        goto DT_PROBE_GET_GPIO_TX;
    }

    /* As the device is ready, queue work to start handling data if available */
    n7d_device_data.closing = false;
    INIT_WORK(&n7d_device_data.work, n7d_work_func);
    queue_work(n7d_device_data.workqueue, &n7d_device_data.work);

    printk(KERN_INFO "n7d: successful init with Baudrate=%d", n7d_baudrate);
    return 0;

DT_PROBE_GET_GPIO_TX:
    device_destroy(n7d_class, dev_num);
DT_PROBE_DEVICE_CREATE:
    cdev_del(&n7d_device_data.cdev);
DT_PROBE_CDEV_ADD:
    destroy_workqueue(n7d_device_data.workqueue);
DT_PROBE_ALLOC_WQ:
    kfifo_free(&n7d_device_data.fifo);
DT_PROBE_KFIFO_ALLOC:
    class_destroy(n7d_class);
DT_PROBE_CLS_CRT:
    unregister_chrdev_region(dev_num, 1);
DT_PROBE_REG_CHRDEV:
DT_PROBE_DEV_EXISTS:
    return err;
}

/**
 * @brief Free the device data, cdev, and the allocated device numbers.
 * 
 * @param pdev Platform device
 */
static int n7d_dt_remove(struct platform_device *pdev)
{
    /* Mark as closing so new work will not be added */
    n7d_device_data.closing = true;
    wake_up_interruptible(&n7d_device_data.writer_waitq);
    wake_up_interruptible(&n7d_device_data.work_waitq); 

    /* Cancel the remaining work, and wait for any remaining work */
    cancel_work_sync(&n7d_device_data.work);
    flush_workqueue(n7d_device_data.workqueue); /* Just in case */
    destroy_workqueue(n7d_device_data.workqueue);

    /* Cleanup device data */
    kfifo_free(&n7d_device_data.fifo);
    mutex_destroy(&n7d_device_data.buf_mutex);

    /* Free the character device number */
    unregister_chrdev_region(MKDEV(n7d_major, 0), 1);

    /* Remove from user space */
    device_destroy(n7d_class, MKDEV(n7d_major, 0));

    /* Remove char device from kernel */
    cdev_del(&n7d_device_data.cdev);

    printk(KERN_INFO "n7d: exit\n");
    return 0;
}

/**
 * Specifying the name of the device in device tree using the overlay
 */
static struct of_device_id n7d_dt_ids[] = {
    { .compatible = "thinkty,n7d", },
    { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, n7d_dt_ids);

/**
 * N7D platform device operations given as function pointers
 */
static struct platform_driver n7d_platform_driver = {
    .probe = n7d_dt_probe,
    .remove = n7d_dt_remove,
    .driver = {
        .name = N7D_DEVICE_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(n7d_dt_ids),
    },
};

/* Macro for module init and exit */
module_platform_driver(n7d_platform_driver);

MODULE_AUTHOR("Tae Yoon Kim");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Numerical 7-segment display driver");
MODULE_VERSION("0:0.1");