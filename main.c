
#include <linux/cdev.h>    /*                           */
#include <linux/fs.h>      /* struct file_operations    */
#include <linux/init.h>    /* module_init,              */
#include <linux/kernel.h>  /* printk                    */
#include <linux/module.h>  /*                           */

#define DEV_MAX (2)        /* Maximum number of devices */
#define BUF_SIZE (10)      /* Size of the device buffer */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tae Yoon Kim");

/* Device major number */
static int dev_major = 0;

/* Device data for each device */
struct n7d_dev_data {
    struct cdev cdev;
} n7d_data[DEV_MAX];

/* Sysfs class structure */
// static struct class * nd7_class = NULL;

/**
 * @brief Open 
 */
static int n7d_open(struct inode * inode, struct file * file)
{
    printk("n7d: open");
    return 0;
}

/**
 * @brief Release
 */
static int n7d_release(struct inode * inode, struct file * file)
{
    printk("n7d: release");
    return 0;
}

/**
 * @brief Filter and write the data to the connected hardware.
 * 
 * @return Number of bytes successfully written.
 */
static ssize_t n7d_write(struct file * file, const char __user *buf, size_t count, loff_t *offset)
{
    char data[BUF_SIZE+1];
    size_t tocopy = count < BUF_SIZE ? count : BUF_SIZE;
    size_t copied = copy_from_user(data, buf, tocopy);

    if (copied > 0) {
        tocopy -= copied;
    }
    data[tocopy] = 0;
    printk("n7d: write %lu bytes, %s\n", tocopy, data);
    return tocopy;
}

static const struct file_operations n7d_fops = {
    .owner = THIS_MODULE,
    .open = n7d_open,
    .release = n7d_release,
    .write = n7d_write,
};

/**
 * @brief
 */
static int __init n7d_init(void)
{
    dev_t dev;
    int i = 0;

    int err = alloc_chrdev_region(&dev, 0, DEV_MAX, "n7d");
    if (err != 0) {
        printk("n7d: failed to init");
        return err;
    }

    /* Get the major device number */
    dev_major = MAJOR(dev);

    /* TODO: Create class for sysfs */
    // nd7_class = class_create(THIS_MODULE, "n7d");
    // nd7_class->dev_uevent = NULL; // TODO:

    /* Initialize & add character devices */
    for (i = 0; i < DEV_MAX; i++) {
        cdev_init(&n7d_data[i].cdev, &n7d_fops);
        n7d_data[i].cdev.owner = THIS_MODULE;
        cdev_add(&n7d_data[i].cdev, MKDEV(dev_major, i), 1);

        // device_create();
    }

    printk("n7d: successful init");
    return 0;
}

/**
 * @brief
 */
static void __exit n7d_exit(void)
{
    // for (int i = 0; i < DEV_MAX; i++) {
    //     device_destroy(, MKDEV(dev_major, i));
    // }

    // class_unregister(mychardev_class);
    // class_destroy(mychardev_class);

    // TODO:
    unregister_chrdev_region(MKDEV(dev_major, 0), MINORMASK);

    printk("n7d: exit");
    return;
}

module_init(n7d_init);
module_exit(n7d_exit);
