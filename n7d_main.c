/**
 * 
 * Handles initialization and cleanup of the N7D device driver.
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

#include "include/n7d_device.h"

MODULE_AUTHOR("Tae Yoon Kim");
MODULE_LICENSE("GPL");

unsigned int n7d_major = 0;
struct n7d_dev * n7d_devices = NULL;
static struct class * n7d_class = NULL;

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
            n7d_device_destroy(n7d_class, &n7d_devices[i], n7d_major, i);
        }

        kfree(n7d_devices);
        n7d_devices = NULL;
    }

    if (n7d_class) {
        class_destroy(n7d_class);
        n7d_class = NULL;
    }

    /* Unregister all the minor devices */
    unregister_chrdev_region(MKDEV(n7d_major, 0), N7D_DEVICE_COUNT);
    return;
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
    err = alloc_chrdev_region(&dev, 0, N7D_DEVICE_COUNT, N7D_DEVICE_NAME);
    if (err < 0) {
        printk(KERN_ERR "n7d: alloc_chrdev_region() failed\n");
        return err;
    }

    /* Save the device's major number */
    n7d_major = MAJOR(dev);

    /* Create device class for sysfs */
    n7d_class = class_create(THIS_MODULE, N7D_DEVICE_NAME);
    if (IS_ERR(n7d_class)) {
        err = PTR_ERR(n7d_class);
        n7d_cleanup(0);
        return err;
    }
    n7d_class->dev_uevent = n7d_uevent;

    /* Allocate the array of devices from kernel memory */
    n7d_devices = (struct n7d_dev *) kzalloc(N7D_DEVICE_COUNT * sizeof(struct n7d_dev), GFP_KERNEL);
    if (n7d_devices == NULL) {
        err = -ENOMEM;
        n7d_cleanup(0);
        return err;
    }

    /* Initialize each device instances */
    for (i = 0; i < N7D_DEVICE_COUNT; i++) {
        err = n7d_device_init(n7d_class, &n7d_devices[i], n7d_major, i);
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
    n7d_cleanup(N7D_DEVICE_COUNT);

    printk(KERN_INFO "n7d: exit\n");
    return;
}

module_init(n7d_init);
module_exit(n7d_exit);
