/**
 * 
 * Handles initialization and cleanup of the N7D device
 * 
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kobject.h>

#include "include/n7d_dev.h"
#include "include/n7d_ops.h"

MODULE_AUTHOR("Tae Yoon Kim");
MODULE_LICENSE("GPL");

#define DEVICE_NAME "n7d"
#define DEVICE_COUNT (1) /* Single device for now */

static int n7d_major = 0;
static struct n7d_dev * n7d_devices = NULL;
static struct class * n7d_class = NULL;

const struct file_operations n7d_fops = {
    .owner = THIS_MODULE,
    .open = n7d_open,
    .release = n7d_release,
    .write = n7d_write,
};

/**
 * @brief Free the data, cdev, and device for the given device.
 * 
 * @param dev Pointer to current device
 * @param minor Current device's minor number
 */
static void n7d_device_destroy(struct n7d_dev * dev, int minor)
{
    dev_t number = MKDEV(n7d_major, minor);

    if (dev == NULL || n7d_class == NULL) {
        printk(KERN_ERR "n7d: n7d_device_init() failed\n");
        return;
    }

    kfree(dev->data);
    cdev_del(&dev->cdev);
    device_destroy(n7d_class, number);
    return;
}

/**
 * @brief Free the individual device data, device class, and the allocated device
 * minor numbers.
 * 
 * @param devices_count Number of devices to destoy 
 */
static void n7d_cleanup(int devices_count)
{
    int i = 0;

    /* Free the array of device data */
    if (n7d_devices) {

        /* Destroy individual devices */
        for (i = 0; i < devices_count; i++) {
            n7d_device_destroy(&n7d_devices[i], i);
        }

        kfree(n7d_devices);
        n7d_devices = NULL;
    }

    if (n7d_class) {
        class_destroy(n7d_class);
        n7d_class = NULL;
    }

    /* Unregister all the minor devices */
    unregister_chrdev_region(MKDEV(n7d_major, 0), DEVICE_COUNT);
    return;
}

/**
 * @brief Initialize the device data, add to kernel and user-space
 * 
 * @param dev Pointer to current device data
 * @param minor Minor device number
 * 
 * @returns 0 on success, less than 0 on error.
 */

static int n7d_device_init(struct n7d_dev * dev, int minor)
{
    int err = 0;
    dev_t number = MKDEV(n7d_major, minor);
    struct device * device = NULL;

    if (dev == NULL || n7d_class == NULL) {
        printk(KERN_ERR "n7d: n7d_device_init() failed\n");
        return -ENODEV;
    }

    /* Initialize the fields to their appropriate value */
    dev->data = NULL;
    dev->data_len = DATA_LEN_MAX;
    cdev_init(&dev->cdev, &n7d_fops);
    dev->cdev.owner = THIS_MODULE;

    /* Add the device available operations to the kernel */
    err = cdev_add(&dev->cdev, number, 1);
    if (err < 0) {
        printk(KERN_ERR "n7d: cdev_add() failed(%d) to add %s%d", err, DEVICE_NAME, minor);
        return err;
    }

    /* Create the device to be visible to user-space */
    device = device_create(n7d_class, NULL, number, NULL, "%s%d", DEVICE_NAME, minor);
    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        printk(KERN_ERR "n7d: device_create() failed(%d) to add %s%d", err, DEVICE_NAME, minor);
        cdev_del(&dev->cdev);
        return err;
    }

    return 0;
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
 * creating a device class and its individual devices with its own data.
 * 
 * @returns 0 on success, less than 0 on error.
 */
static int __init n7d_init(void)
{
    dev_t dev;
    int i = 0;
    int err = 0;

    /* Allocate the appropriate device major number and a set of minor numbers */
    err = alloc_chrdev_region(&dev, 0, DEVICE_COUNT, DEVICE_NAME);
    if (err < 0) {
        printk(KERN_ERR "n7d: alloc_chrdev_region() failed\n");
        return err;
    }

    /* Save the device's major number */
    n7d_major = MAJOR(dev);

    /* Create device class for sysfs */
    n7d_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(n7d_class)) {
        err = PTR_ERR(n7d_class);
        n7d_cleanup(0);
        return err;
    }
    n7d_class->dev_uevent = n7d_uevent;

    /* Allocate the array of devices from kernel memory */
    n7d_devices = (struct n7d_dev *) kzalloc(DEVICE_COUNT * sizeof(struct n7d_dev), GFP_KERNEL);
    if (n7d_devices == NULL) {
        err = -ENOMEM;
        n7d_cleanup(0);
        return err;
    }

    /* Initialize each device instances */
    for (i = 0; i < DEVICE_COUNT; i++) {
        err = n7d_device_init(&n7d_devices[i], i);
        if (err < 0) {
            n7d_cleanup(i);
            return err;
        }
    }

    printk(KERN_INFO "n7d: successful init\n");
    return 0;
}

/**
 * @brief Cleanup the individual devices and class.
 */
static void __exit n7d_exit(void)
{
    n7d_cleanup(DEVICE_COUNT);

    printk(KERN_INFO "n7d: exit\n");
    return;
}

module_init(n7d_init);
module_exit(n7d_exit);
