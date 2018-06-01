//
// Created by mathhann on 01/06/18.
//

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


#define TRANSMITTER_BASE_ADDR 0x43C00000
#define TRANSMITTER_HIGH_ADDR 0x43C0FFFF
#define RECEIVER_BASE_ADDR 0x43C10000
#define RECEIVER_HIGH_ADDR 0x43C1FFFF
#define IP_SIZE(ipBaseAddr, ipHighAddr) (((ipHighAddr) + 1) - (ipBaseAddr))


int ip_Mem_Map(uint32_t baseAddress, volatile void **memorySpacePointer, size_t size);

int ip_Mem_Unmap(int fd, volatile void *memorySpacePointer, size_t size);

int main(void) {

    volatile void *transmitterPointer;
    volatile void *receiverPointer;

    int fd = ip_Mem_Map(TRANSMITTER_BASE_ADDR, &transmitterPointer,
                        IP_SIZE(TRANSMITTER_BASE_ADDR, TRANSMITTER_HIGH_ADDR));
    int fd2 = ip_Mem_Map(RECEIVER_BASE_ADDR, &receiverPointer, IP_SIZE(RECEIVER_BASE_ADDR, RECEIVER_HIGH_ADDR));

    volatile uint32_t *transmitterValue = transmitterPointer;
    volatile uint32_t *receiverValue = receiverPointer;

    *transmitterValue = 7u;


    printf("Value read : %x\n", *receiverValue);

    ip_Mem_Unmap(fd, transmitterPointer, IP_SIZE(TRANSMITTER_BASE_ADDR, TRANSMITTER_HIGH_ADDR));
    ip_Mem_Unmap(fd2, receiverPointer, IP_SIZE(RECEIVER_BASE_ADDR, RECEIVER_HIGH_ADDR));
    return EXIT_SUCCESS;
}
/**
 * Map memory space for register control
 *
 * @param baseAddress Base address of the space to map
 * @param memorySpacePointer Address of the pointer to assign to the allocate space
 * @param size Size of the space that will allocated
 * @return The file descriptor on /dev/mem to close it later
 */
int ip_Mem_Map(uint32_t baseAddress, volatile void **memorySpacePointer, size_t size) {
    int fd = open("/dev/mem", O_RDWR);
    *memorySpacePointer = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, baseAddress);
    if (memorySpacePointer == MAP_FAILED) {
        perror("Memory mapping failed");
        exit(EXIT_FAILURE);
    }
    return fd;
}

/**
 * Unmap previously mapped memory space
 * Close opened file descriptor
 * @param fd
 * @param memorySpacePointer
 * @param size
 * @return
 */
int ip_Mem_Unmap(int fd, volatile void *memorySpacePointer, size_t size) {
    int flag = munmap((void *) memorySpacePointer, size);
    if (flag < 0) {
        perror("Memory unmapping failed");
        return -1;
    }
    close(fd);
    return 0;
}

