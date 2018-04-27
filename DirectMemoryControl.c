#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/epoll.h>
#include <sys/mman.h>


#define GPIO_BASE_ADDR 0xE000A000
#define GPIO_END_ADDR 0xE000AFFF
#define GPIO_MAP_SIZE 0x10000
#define GPIO_SIZE (GPIO_END_ADDR - GPIO_BASE_ADDR)
#define GPIO_PATH "/sys/class/gpio/gpio"
#define GPIO_PATH_EXPORT "/sys/class/gpio/export"
#define GPIO_PATH_UNEXPORT "/sys/class/gpio/unexport"
#define GPIO_VALUE "/value"
#define GPIO_EDGE "/edge"
#define GPIO_DIRECTION "/direction"
#define LINUX_OFFSET 54
#define GPIO_OUT 0
#define GPIO_IN 1
#define EDGE_RISING 0
#define EDGE_FALLING 1
#define EDGE_BOTH 2


int main(int argc, char *argv[]) {
    volatile void *gpio_addr;
    volatile unsigned int *gpio_ptr;
    int fd = open("/dev/mem", O_RDWR);
    int led = 7 + LINUX_OFFSET;
    // mmap the GPIO device into user space

    gpio_ptr = mmap(NULL, GPIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (gpio_ptr == MAP_FAILED) {
        printf("Mmap call failure.\n");
        return -1;
    }

    int gpioContent, gpioContentDiff;
    int i;
    for (i = 0; i < 20; i++) {
        gpioContent = *gpio_ptr;
        sleep(1);
        gpioContentDiff = *gpio_ptr;
        printf("Difference from a second ago : %d ", gpioContent - gpioContentDiff);
    }


    return 0;
}
