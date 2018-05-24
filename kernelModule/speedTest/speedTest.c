/**
 *
  ___             _
 | _ \___ __ _ __| |  _ __  ___
 |   / -_) _` / _` | | '  \/ -_)
 |_|_\___\__,_\__,_| |_|_|_\___|

 * Function : Write on a gpio pin as fast as possible
 * function triggered when the user write to the pseudo file
 */


#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <linux/uaccess.h>        // Required for the copy to user function
#include <linux/gpio.h>           // Required for the GPIO functions

#define  DEVICE_NAME "speedTest" ///< The device will appear at /dev/ebbchar using this value
#define  CLASS_NAME  "speedTester"    ///< The device class -- this is a character device driver
#define  GPIO_OFFSET 54
#define  LED_GPIO_ADDR (GPIO_OFFSET + 24)      ///< GPIO Address + Offset (54).
#define  MAX_ITER 1000000000

MODULE_LICENSE("GPL");                         ///< The license type -- this affects available functionality
MODULE_AUTHOR("Mathieu Hannoun");              ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("Test the max frequency reachable with gpio.h");   ///< The description -- see modinfo
MODULE_VERSION("0.22");                        ///< A version number to inform users

static int    majorNumber;                  ///< Stores the device number -- determined automatically
static struct class*  ebbcharClass  = NULL; ///< The device-driver class struct pointer
static struct device* ebbcharDevice = NULL; ///< The device-driver device struct pointer

// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t device_write(struct file *file, const char __user * buffer, size_t length, loff_t * offset);




/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
        {
                .open = dev_open,
                .write = device_write,
                .release = dev_release,
        };

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point.
 *  @return returns 0 if successful
 */
static int __init ebbchar_init(void){
    printk(KERN_INFO "SpeedTest: Initializing the Speedtest LKM\n");

    // Try to dynamically allocate a major number for the device -- more difficult but worth it
    majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
    if (majorNumber<0){
        printk(KERN_ALERT "SpeedTest failed to register a major number\n");
        return majorNumber;
    }
    printk(KERN_INFO "SpeedTest: registered correctly with major number %d\n", majorNumber);

    // Register the device class
    ebbcharClass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(ebbcharClass)){                // Check for error and clean up if there is
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to register device class\n");
        return PTR_ERR(ebbcharClass);          // Correct way to return an error on a pointer
    }
    printk(KERN_INFO "SpeedTest: device class registered correctly\n");

    // Register the device driver
    ebbcharDevice = device_create(ebbcharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
    if (IS_ERR(ebbcharDevice)){               // Clean up if there is an error
        class_destroy(ebbcharClass);           // Repeated code but the alternative is goto statements
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to create the device\n");
        return PTR_ERR(ebbcharDevice);
    }
    printk(KERN_INFO "SpeedTest: device class created correctly\n"); // Made it! device was initialized

    printk(KERN_INFO "GPIO_TEST: Initializing the GPIO_TEST LKM\n");
    // Is the GPIO a valid GPIO number (e.g., the BBB has 4x32 but not all available)
    if (!gpio_is_valid(LED_GPIO_ADDR)){
        printk(KERN_INFO "GPIO_TEST: invalid LED GPIO\n");
        return -ENODEV;
    }
    // Going to set up the LED. It is a GPIO in output mode and will be on by default
    gpio_request(LED_GPIO_ADDR, "sysfs");       // gpioLED is hardcoded to 49, request it
    gpio_direction_output(LED_GPIO_ADDR, 0);    // Set the gpio to be in output mode and off
    // gpio_set_value(gpioLED, ledOn);          // Not required as set by line above (here for reference)
    //gpio_export(gpioLED, false);              // Causes gpio49 to appear in /sys/class/gpio


    return 0;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit ebbchar_exit(void){

    gpio_set_value(LED_GPIO_ADDR, 0);              // Turn the LED off, makes it clear the device was unloaded
//    gpio_unexport(LED_GPIO_ADDR);                  // Unexport the LED GPIO
    gpio_free(LED_GPIO_ADDR);                      // Free the LED GPIO

    device_destroy(ebbcharClass, MKDEV(majorNumber, 0));     // remove the device
    class_unregister(ebbcharClass);                          // unregister the device class
    class_destroy(ebbcharClass);                             // remove the device class
    unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
    printk(KERN_INFO "SpeedTest: Goodbye from the LKM!\n");
}

/** @brief The device open function that is called each time the device is opened
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){
    printk(KERN_INFO "SpeedTest: Device has been opened \n");
    return 0;
}

/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
    printk(KERN_INFO "SpeedTest: Device successfully closed\n");
    return 0;
}

/**
 * @brief Trigger a square wave generation from the registered GPIO
 * @param file
 * @param __user
 * @return Return the length of the argument buffer to prevent possible error and notify that the message has been taken in account
 */
static ssize_t device_write(struct file *file, const char __user * buffer, size_t length, loff_t * offset){
    int i;
    for(i = 0; i < MAX_ITER; ++i){
        gpio_set_value(LED_GPIO_ADDR, 1);
        gpio_set_value(LED_GPIO_ADDR, 0);
    }
    return length;
}

/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
 *  identify the initialization function at insertion time and the cleanup function (as
 *  listed above)
 */
module_init(ebbchar_init);
module_exit(ebbchar_exit);
