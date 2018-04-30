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
#define GPIO_MAP_SIZE (GPIO_END_ADDR - GPIO_BASE_ADDR)
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

void memory_Visualizer();

int main(int argc, char *argv[]) {

    memory_Visualizer();
    return 0;
}

/**
 * Read the content from the GPIO Base address to its End address
 * Byte by byte
 * It goes from 0xE000A000 to 0xE000AFFF so 16 reading are necessary
 *
 */
void memory_Visualizer() {
    volatile void *gpio_addr;
    volatile unsigned int *gpio_ptr;
    int fd = open("/dev/mem", O_RDWR);

    printf("Mapped size : %d or %#010x\n", GPIO_MAP_SIZE, GPIO_MAP_SIZE);
    volatile unsigned char gpioContent;
    int i;
    while (1) {


        // Byte decomposition from 0x000 to 0xFFF to print all memory content
        for (i = 0; i < 16; i++) {
            gpioContent = *gpio_ptr;
            // mmap the GPIO device into user space and Read a Byte
//            gpio_ptr = mmap(NULL, sizeof(char), PROT_READ | PROT_WRITE, MAP_SHARED, fd, (i * 0x100));
            gpio_ptr = mmap(NULL, 1, PROT_READ | PROT_WRITE, MAP_SHARED, fd, i);
            if (gpio_ptr == MAP_FAILED) {
                printf("Mmap call failure.\n");
                exit(EXIT_FAILURE);
            }

            printf("L%d : %d\r\n", i, gpioContent);
        }
        printf(" \r\n");
        sleep(1);
    }

}