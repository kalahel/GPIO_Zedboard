/**
 * @file   ebbchar.c
 * @author Derek Molloy
 * @date   7 April 2015
 * @version 0.1
 * @brief   An introductory character driver to support the second article of my series on
 * Linux loadable kernel module (LKM) development. This module maps to /dev/ebbchar and
 * comes with a helper C program that can be run in Linux user space to communicate with
 * this the LKM.
 * @see http://www.derekmolloy.ie/ for a full description and follow-up descriptions.
 */

#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <linux/uaccess.h>          // Required for the copy to user function
#define  DEVICE_NAME "gpioTeller"    ///< The device will appear at /dev/ebbchar using this value
#define  CLASS_NAME  "teller"        ///< The device class -- this is a character device driver
#define  BUF_LEN 32

MODULE_LICENSE("GPL");            ///< The license type -- this affects available functionality
MODULE_AUTHOR("Mathieu Hannoun");    ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("Store the number you entered");  ///< The description -- see modinfo
MODULE_VERSION("0.1");            ///< A version number to inform users

static int    majorNumber;                  ///< Stores the device number -- determined automatically
static char   message[256] = {0};           ///< Memory for the string that will be sent to userspace
//static char   receivedMessage[128] = {0};   ///< Memory of the string that will be passed from userspace
static short  size_of_message;              ///< Used to remember the size of the string stored
static int    numberOpens = 0;              ///< Counts the number of times the device is opened
static struct class*  ebbcharClass  = NULL; ///< The device-driver class struct pointer
static struct device* ebbcharDevice = NULL; ///< The device-driver device struct pointer

// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
static ssize_t device_file_printGpio(struct file *file_ptr, char __user *user_buffer, size_t count, loff_t *position);
static ssize_t device_write(struct file *file, const char __user * buffer, size_t length, loff_t * offset);

// Used for read function
static const char    g_s_Hello_World_string[] = "Hello world from kernel mode!\n\0";
static const ssize_t g_s_Hello_World_size = sizeof(g_s_Hello_World_string);

/*
 * The message the device will give when asked
 */
static char messageReceived[BUF_LEN];
/*
 * How far did the process reading the message get?
 * Useful if the message is larger than the size of the
 * buffer we get to fill in device_read.
 */
static char *Message_Ptr;

static long gpioNumber;

/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
        {
                .open = dev_open,
                .read = device_file_printGpio,
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
    printk(KERN_INFO "EBBChar: Initializing the EBBChar LKM\n");

    // Try to dynamically allocate a major number for the device -- more difficult but worth it
    majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
    if (majorNumber<0){
        printk(KERN_ALERT "EBBChar failed to register a major number\n");
        return majorNumber;
    }
    printk(KERN_INFO "EBBChar: registered correctly with major number %d\n", majorNumber);

    // Register the device class
    ebbcharClass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(ebbcharClass)){                // Check for error and clean up if there is
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to register device class\n");
        return PTR_ERR(ebbcharClass);          // Correct way to return an error on a pointer
    }
    printk(KERN_INFO "EBBChar: device class registered correctly\n");

    // Register the device driver
    ebbcharDevice = device_create(ebbcharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
    if (IS_ERR(ebbcharDevice)){               // Clean up if there is an error
        class_destroy(ebbcharClass);           // Repeated code but the alternative is goto statements
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to create the device\n");
        return PTR_ERR(ebbcharDevice);
    }
    printk(KERN_INFO "EBBChar: device class created correctly\n"); // Made it! device was initialized
    gpioNumber = 0; // Gpio initialization
    return 0;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit ebbchar_exit(void){
    device_destroy(ebbcharClass, MKDEV(majorNumber, 0));     // remove the device
    class_unregister(ebbcharClass);                          // unregister the device class
    class_destroy(ebbcharClass);                             // remove the device class
    unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
    printk(KERN_INFO "EBBChar: Goodbye from the LKM!\n");
}

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){
    numberOpens++;
    printk(KERN_INFO "EBBChar: Device has been opened %d time(s)\n", numberOpens);
    return 0;
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
//    if(snprintf(receivedMessage, sizeof(receivedMessage), buffer) > sizeof(receivedMessage)){
//        printk(KERN_ALERT "Writing caused buffer overflowed, possible data loss\n");
//    }
    kstrtol(buffer, 0, &gpioNumber);
    //  kstrtol(message, 0, &gpioNumber);
    printk(KERN_INFO "EBBChar: Received %zu characters from the user\n", len);
    printk(KERN_INFO "Current gpio number : %ld\n", gpioNumber);

    return len;
}

/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
    printk(KERN_INFO "EBBChar: Device successfully closed\n");
    return 0;
}

/**
 * Called when the user read from the pseudo file
 * @param file_ptr
 * @param __user
 * @return
 */
static ssize_t device_file_printGpio(struct file *file_ptr, char __user *user_buffer, size_t count, loff_t *position){
    printk(KERN_INFO "Current gpio number : %ld\n", gpioNumber);
    snprintf(message, sizeof(message), "%ld", gpioNumber);
    copy_to_user(user_buffer, message, sizeof(message));
    return count;
}


/*
 * This function is called when somebody tries to
 * write into our device file.
 */
static ssize_t device_write(struct file *file, const char __user * buffer, size_t length, loff_t * offset){
    int i;

    #ifdef DEBUG
    printk(KERN_INFO "device_write(%p,%s,%d)", file, buffer, length);
    #endif
    // TODO CHECK FOR OVERFLOW
    for (i = 0; i < length && i < BUF_LEN; i++)
    get_user(messageReceived[i], buffer + i);

    Message_Ptr = messageReceived;
    kstrtol(messageReceived, 0, &gpioNumber);



/*
* Again, return the number of input characters used
*/
    return i;
}

/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
 *  identify the initialization function at insertion time and the cleanup function (as
 *  listed above)
 */
module_init(ebbchar_init);
module_exit(ebbchar_exit);
