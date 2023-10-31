/*
 * 
 * Numerical 7-segment display device driver
 * 
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
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

/**
 * n7d_drvdata - driver data
 * 
 * @param workqueue Queue for work(s) to write the bytes
 * @param work Work to handle sending each available byte to display
 * @param writer_waitq Queue to block writers when buffer is full
 * @param work_waitq Queue to block work functions until there is data in buffer
 * @param fifo Circular buffer for storing bytes
 * @param buf_mutex Mutex for the buffer
 * @param closing Used to stop works from self-queueing
 * @param misc Used for getting drvdata using container_of
 * @param tx GPIO descriptor for transmission
 */
struct n7d_drvdata {
    struct workqueue_struct * workqueue;
    struct work_struct transmit_work;
    wait_queue_head_t writer_waitq;
    wait_queue_head_t work_waitq;
    struct kfifo fifo;
    struct mutex buf_mutex;
    bool closing;
    int somevalue; // TODO: just to check container_of is working
    struct miscdevice misc;
    struct gpio_desc * tx;
};

static int n7d_baudrate = 38400;
module_param(n7d_baudrate, int, 0);
MODULE_PARM_DESC(n7d_baudrate, "\tBaudrate of the device (default=38400)");

/**
 * Function prototypes for the device file
 */
static int n7d_release(struct inode * inode, struct file * filp);
static ssize_t n7d_write(struct file * filp, const char __user * buf, size_t count, loff_t * f_pos);
static long n7d_ioctl(struct file * filp, unsigned int cmd, unsigned long arg);

/**
 * N7D file (device) operations given as function pointers. .open is handled by
 * default and sets file->private_data to point to the structure.
 */
static struct file_operations n7d_fops = {
    .owner = THIS_MODULE,
    .release = n7d_release,
    .write = n7d_write,
    .unlocked_ioctl = n7d_ioctl,
};

/**
 * N7D misc. device description
 */
static struct miscdevice n7d_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = N7D_DEVICE_NAME,
    .fops = &n7d_fops,
};

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

    pr_info("n7d: released %s%u\n", N7D_DEVICE_NAME, mn); // TODO:
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
    struct n7d_drvdata * drvdata = container_of(filp->private_data, struct n7d_drvdata, misc);
    // TODO: is this working?
    pr_info("n7d: checking if container_of is working = %d\n", drvdata->somevalue);

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
    err = mutex_lock_interruptible(&drvdata->buf_mutex);
    if (err < 0) {
        return err;
    }

    /* Check in a loop since another writer may get to kfifo first. This could
    potentially be a thundering herd problem */
    while (kfifo_is_full(&drvdata->fifo) && !drvdata->closing) {
        mutex_unlock(&drvdata->buf_mutex);

        /* If non-blocking, return immediately */
        if (filp->f_flags & O_NDELAY || filp->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }

        /* Sleep until space available, timeout, interrupt, or closing device */
        err = wait_event_interruptible_hrtimeout(drvdata->writer_waitq,
                                                 drvdata->closing || !kfifo_is_full(&drvdata->fifo),
                                                 ms_to_ktime(N7D_DEVICE_TIMEOUT));
        if (err < 0) {
            return err;
        }

        err = mutex_lock_interruptible(&drvdata->buf_mutex);
        if (err < 0) {
            return err;
        }
    }

    /* If closing device, discard the given bytes and return error */
    if (drvdata->closing) {
        mutex_unlock(&drvdata->buf_mutex);
        return -ECANCELED;
    }

    /* Depending on the buffer vacancy, it might write less than specified */
    to_copy = kfifo_in(&drvdata->fifo, tbuf, to_copy);
    mutex_unlock(&drvdata->buf_mutex);

    /* Wake up work waitqueue since there are bytes available */
    wake_up_interruptible(&drvdata->work_waitq);

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
    // struct n7d_drvdata * drvdata = container_of(filp->private_data, struct n7d_drvdata, misc);

    switch (cmd) {
        case N7D_CLR:
            pr_info("n7d: Clear display\n");
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
    struct n7d_drvdata * drvdata = container_of(work, struct n7d_drvdata, transmit_work);
    static int tmp = 0; // TODO: just to toggle

    /* Sleep until there is something in fifo or the device is unloading */
    err = wait_event_interruptible(drvdata->work_waitq,
                                   drvdata->closing || !kfifo_is_empty(&drvdata->fifo));
    if (err < 0 || drvdata->closing) {
        /* If interrupted or woke up to close the device, stop the work */
        return;
    }

    /* Since there is only 1 actual HW per device to write to, no locks */
    err = kfifo_get(&drvdata->fifo, &byte);
    if (err == 0) {
        printk(KERN_WARNING "n7d: kfifo_get() says empty although it shouldn't be\n");
        queue_work(drvdata->workqueue, &drvdata->transmit_work);
        return;
    }

    // TODO: process the byte (just toggling for now)
    pr_info("n7d-work: '%c', tmp=%d\n", byte, tmp);
    tmp = tmp == 0 ? 1 : 0;
    gpiod_set_value(drvdata->tx, tmp);

    /* Wake up writer_waitq since new space is available */
    wake_up_interruptible(&drvdata->writer_waitq);

    /* Self-requeueing work */
    queue_work(drvdata->workqueue, &drvdata->transmit_work);
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
    int err = 0;
    struct n7d_drvdata * drvdata = NULL;

    /* Check that the device exists before attaching device driver */
    if (!device_property_present(&pdev->dev, "exists")) {
        pr_err("n7d: device does not exist\n");
        return -ENODEV;
    }
    pr_info("n7d: device exists\n");

    /* Allocate driver data */
    // TODO: move to device managed devm_kzalloc
    drvdata = (struct n7d_drvdata *) kzalloc(sizeof(struct n7d_drvdata), GFP_KERNEL);
    if (!drvdata) {
        pr_err("n7d: kzalloc() failed\n");
        return -ENOMEM;
    }
    pr_info("n7d: allocated device driver data\n");

    /* Allocate buffer */
    err = kfifo_alloc(&drvdata->fifo, N7D_DEVICE_FIFO_SIZE, GFP_KERNEL);
    if (err != 0) {
        pr_err("n7d: kfifo_alloc() failed\n");
        goto DT_PROBE_KFIFO;
    }
    pr_info("n7d: allocated buffer\n");

    /* Get the GPIO descriptors from the pin numbers */
    // TODO: move to device managed devm_gpiod_get
    drvdata->tx = gpiod_get(&pdev->dev, "serial", GPIOD_OUT_HIGH);
    if (IS_ERR(drvdata->tx)) {
        err = PTR_ERR(drvdata->tx);
        pr_err("n7d: devm_gpiod_get() failed\n");
        goto DT_PROBE_GET_GPIO_TX;
    }
    pr_info("n7d: requested gpio for serial\n");

    /* Initialize various queues and buffer mutex */
    init_waitqueue_head(&drvdata->work_waitq);
    init_waitqueue_head(&drvdata->writer_waitq);
    mutex_init(&drvdata->buf_mutex);

    /* Create workqueue and waitqueues for the device to handle writing bytes */
    drvdata->workqueue = alloc_ordered_workqueue(N7D_DEVICE_WORKQUEUE, 0);
    if (!drvdata->workqueue) {
        pr_err("n7d: alloc_ordered_workqueue() failed to allocate workqueue");
        err = -ENOMEM; /* However, there are multiple reasons to fail */
        goto DT_PROBE_ALLOC_WQ;
    }
    pr_info("n7d: allocated workqueue\n");

    /* Make the device available to the kernel & user */
    err = misc_register(&n7d_misc_device);
    if (err < 0) {
        pr_err("n7d: misc_register() failed\n");
        goto DT_PROBE_MISC_REG;
    }
    drvdata->misc = n7d_misc_device;
    pr_info("n7d: registered misc device for file IO\n");

    /* As the device is ready, queue work to start handling data if available */
    drvdata->closing = false;
    INIT_WORK(&drvdata->transmit_work, n7d_work_func);
    queue_work(drvdata->workqueue, &drvdata->transmit_work);
    pr_info("n7d: initialized and queued transmission work\n");

    /* Set the driver data to the platform device and misc device */
    dev_set_drvdata(&pdev->dev, drvdata);

    // TODO: just to check container_of is working
    drvdata->somevalue = 594728; // some specific meaningless value

    pr_info("n7d: successful init with Baudrate=%d", n7d_baudrate);
    return 0;

DT_PROBE_MISC_REG:
    destroy_workqueue(drvdata->workqueue);
DT_PROBE_ALLOC_WQ:
    mutex_destroy(&drvdata->buf_mutex);
    gpiod_put(drvdata->tx);
DT_PROBE_GET_GPIO_TX:
    kfifo_free(&drvdata->fifo);
DT_PROBE_KFIFO:
    kfree(drvdata);
    return err;
}

/**
 * @brief Free the device data, cdev, and the allocated device numbers.
 * 
 * @param pdev Platform device
 */
static int n7d_dt_remove(struct platform_device *pdev)
{
    struct n7d_drvdata * drvdata = dev_get_drvdata(&pdev->dev);
    if (!drvdata) {
        pr_err("n7d: driver data does not exist\n");
        return -ENODATA;
    }

    /* Unregister misc device */
    misc_deregister(&n7d_misc_device);

    /* Mark as closing so new work will not be added. */
    drvdata->closing = true;
    wake_up_interruptible(&drvdata->writer_waitq);
    wake_up_interruptible(&drvdata->work_waitq);

    /* Cancel the remaining work, and wait for any remaining work */
    cancel_work_sync(&drvdata->transmit_work);
    flush_workqueue(drvdata->workqueue); /* Just in case */
    destroy_workqueue(drvdata->workqueue);

    /* Cleanup driver data */
    mutex_destroy(&drvdata->buf_mutex);
    gpiod_put(drvdata->tx);
    kfifo_free(&drvdata->fifo);
    kfree(drvdata);

    pr_info("n7d: exit\n");
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