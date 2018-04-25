

#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/epoll.h>


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

int gpio_SetDataDirection(int portAddr, int mode);

int gpio_Write(int portAddr, int gpioValue);

int gpio_SetDataDirection_Range(int firstPortAddr, int lastPortAddr, int mode);

int gpio_Unexport(int portAddr);

int gpio_Write_Range(int firstPortAddr, int lastPortAddr, int valueMask[]);

int gpio_Read(int portAddr, void *buf, size_t count);

int gpio_Clocked_Read(int portAddr, int clkAddr, char *buf, size_t count, char *edgeOption, size_t edgeOptionSize);

int
gpio_Clocked_Read_Simple(int portAddr, int clkAddr, char buf[], size_t count, useconds_t clkReadRate, int edgeOption);

int gpio_Clocked_Epoll();

int gpio_Clocked_Poll(int portAddr);

int gpio_Clocked_Poll_Monitor(int clkAddr, int portAddr);

int main(int argc, char *argv[]) {


    int clkIn = 32, dataIn = 33;
    gpio_SetDataDirection(clkIn, GPIO_IN);
    gpio_SetDataDirection(dataIn, GPIO_IN);
    gpio_Clocked_Poll_Monitor(clkIn, dataIn);
    gpio_Unexport(clkIn);
    gpio_Unexport(dataIn);

    return EXIT_SUCCESS;
}


// TODO split gpio export and mode set
// TODO Handle ports already exported "Device or resource bus"
/**
 * Mandatory initialization for using port
 * Set the port as an Input or Output
 *
 * Write the port in gpio/export.
 * Write the mode in gpio*Port* /direction.
 * Write the value in gpio*Port /value
 *
 * @param portAddr Address of the port used (refer to the .xdc file)0
 * @param mode Output : 0, Input : 1.
 * @return 0 In case of success
 */
int gpio_SetDataDirection(int portAddr, int mode) {


    if (mode != GPIO_OUT && mode != GPIO_IN) {
        perror("Enter a binary value for mode");
        return -1;
    }

    char stringAddr[10];
    char directionPath[80];
    char valuePath[80];
    int portAddrOffsetted = portAddr + LINUX_OFFSET;

    snprintf(stringAddr, sizeof(stringAddr), "%d", portAddrOffsetted);
    snprintf(directionPath, sizeof(directionPath), "%s%d%s", GPIO_PATH, portAddrOffsetted, GPIO_DIRECTION);
    snprintf(valuePath, sizeof(valuePath), "%s%d%s", GPIO_PATH, portAddrOffsetted, GPIO_VALUE);

    // Exporting the port
    int fileDescriptor = open(GPIO_PATH_EXPORT, O_WRONLY);
    if (fileDescriptor == -1)
        perror("Opening gpio/export failed\r\n");

    int returnFlag = (int) write(fileDescriptor, stringAddr, sizeof(stringAddr));

    if (returnFlag == -1)
        perror("Writing to gpio/export failed\r\n");

    returnFlag = close(fileDescriptor);

    if (returnFlag == -1)
        perror("Closing gpio/export failed\r\n");

    // Writing in/out in the direction file
    fileDescriptor = open(directionPath, O_WRONLY);
    if (fileDescriptor == -1)
        perror("Opening gpio/gpio*Port*/direction failed\r\n");

    if (mode == GPIO_OUT)
        returnFlag = (int) write(fileDescriptor, "out", strlen("out"));
    else
        returnFlag = (int) write(fileDescriptor, "in", strlen("in"));


    if (returnFlag == -1)
        perror("Writing to gpio/gpio*Port*/direction failed\r\n");

    returnFlag = close(fileDescriptor);

    if (returnFlag == -1)
        perror("Closing gpio/gpio*Port*/direction failed\r\n");
    return 0;
}

/**
 * Unexport the selected port, good practice to unexport all used port at the end of a program
 *
 * @param portAddr Port to unexport
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Unexport(int portAddr) {


    char stringAddr[10];
    int portAddrOffsetted = portAddr + LINUX_OFFSET;

    snprintf(stringAddr, sizeof(stringAddr), "%d", portAddrOffsetted);

    // Writing the port address
    int fileDescriptor = open(GPIO_PATH_UNEXPORT, O_WRONLY);
    if (fileDescriptor == -1)
        perror("Opening gpio/unexport failed\r\n");

    int returnFlag = (int) write(fileDescriptor, stringAddr, sizeof(stringAddr));

    if (returnFlag == -1)
        perror("Writing to gpio/unexport failed\r\n");

    returnFlag = close(fileDescriptor);

    if (returnFlag == -1)
        perror("Closing gpio/unexport failed\r\n");

    return 0;
}

/**
 * Write the chosen value to the corresponding GPIO pin
 *
 * @param portAddr Address of the port used (refer to the .xdc file)
 * @param gpioValue Value passed to the gpio pin (1 or 0)
 * @return 0 In case of success
 */
int gpio_Write(int portAddr, int gpioValue) {

    if (gpioValue != 0 && gpioValue != 1) {
        perror("Enter a binary value");
        return -1;
    }

    char stringValue[2];
    char valuePath[80];
    int portAddrOffsetted = portAddr + LINUX_OFFSET;

    snprintf(stringValue, sizeof(gpioValue), "%d", gpioValue);

    snprintf(valuePath, sizeof(valuePath), "%s%d%s", GPIO_PATH, portAddrOffsetted, GPIO_VALUE);

    // Writing the value in the value file
    int fileDescriptor = open(valuePath, O_WRONLY);
    if (fileDescriptor == -1)
        perror("Opening gpio/gpio*Port*/value failed\r\n");

    int returnFlag = (int) write(fileDescriptor, stringValue, sizeof(stringValue));

    if (returnFlag == -1)
        perror("Writing to gpio/gpio*Port*/value failed\r\n");

    returnFlag = close(fileDescriptor);

    if (returnFlag == -1)
        perror("Closing gpio/gpio*Port*/value failed\r\n");

    return 0;
}

/**
 * Read content found for the corresponding GPIO pin and put it in a buffer
 *
 * @param portAddr Target port
 * @param buf Buffer to store the read result
 * @param count Number of bytes to read
 * @return the number of bytes read
 */
int gpio_Read(int portAddr, void *buf, size_t count) {


    char valuePath[80];
    int portAddrOffsetted = portAddr + LINUX_OFFSET;

    snprintf(valuePath, sizeof(valuePath), "%s%d%s", GPIO_PATH, portAddrOffsetted, GPIO_VALUE);

    // Reading the value from the value pseudo file
    int fileDescriptor = open(valuePath, O_RDONLY);
    if (fileDescriptor == -1)
        perror("Opening gpio/gpio*Port*/value failed\r\n");

    int returnFlag = (int) read(fileDescriptor, buf, count);

    if (returnFlag == -1)
        perror("Reading gpio/gpio*Port*/value failed\r\n");

    returnFlag = close(fileDescriptor);

    if (returnFlag == -1)
        perror("Closing gpio/gpio*Port*/value failed\r\n");

    return returnFlag;
}

/**
 * Set a whole range of port as Inputs our Outputs
 *
 * @param firstPortAddr The address of the first port of the range
 * @param lastPortAddr The address of the last port of the range
 * @param mode Output : 0, Input : 1
 * @return 0 In case of success
 */
int gpio_SetDataDirection_Range(int firstPortAddr, int lastPortAddr, int mode) {

    if (firstPortAddr > lastPortAddr) {
        perror("First address greater than last address");
        return -1;
    }

    int index;
    for (index = 0; index <= (lastPortAddr - firstPortAddr); index++) {
        gpio_SetDataDirection(firstPortAddr + index, mode);
    }

    return 0;
}

/**
 * Set the value of a whole range of port given a corresponding Array
 *
 * @param firstPortAddr The address of the first port of the range
 * @param lastPortAddr The address of the last port of the range
 * @param valueMask Array of desired value for each port
 * @return 0 In case of success
 */
int gpio_Write_Range(int firstPortAddr, int lastPortAddr, int valueMask[]) {

    if (firstPortAddr > lastPortAddr) {
        perror("First address greater than last address");
        return -1;
    }

    int index;
    for (index = 0; index <= (lastPortAddr - firstPortAddr); index++) {
        gpio_Write(firstPortAddr + index, valueMask[index]);
    }
    return 0;
}

/**
 * Write the content of a buffer in the selected port
 *
 * @param portAddr Address of the port used (refer to the .xdc file)
 * @param valueBuffer Array of value to write
 * @param bufferSize Size of the buffer of value
 * @return 0 In case of success
 */
int gpio_write_buffer(int portAddr, int valueBuffer[], int bufferSize) {
    int index;
    for (index = 0; index < bufferSize; index++) {
        gpio_Write(portAddr, valueBuffer[index]);
    }
    return 0;
}

/**
 * Read the data on the portAddr synchronous to the clk
 *
 * @param portAddr
 * @param clkAddr
 * @param buf
 * @param count
 * @param edgeOption
 * @return
 */
// TODO remove unused comment
// TODO make it work
int gpio_Clocked_Read(int portAddr, int clkAddr, char *buf, size_t count, char *edgeOption, size_t edgeOptionSize) {

    //todo add edge check

//    char stringPortAddr[10];
    char stringCLKAddr[10];
//    char portPath[60];
//    char clkPath[60];
    char portValuePath[80];
    char clkValuePath[80];
    char edgePath[80];
    int portAddrOffSetted = portAddr + LINUX_OFFSET;
    int clkAddrOffSetted = clkAddr + LINUX_OFFSET;

//    snprintf(stringPortAddr, sizeof(stringPortAddr), "%d", portAddrOffSetted);
    snprintf(stringCLKAddr, sizeof(stringCLKAddr), "%d", clkAddrOffSetted);

    // Setting the port
//    strcpy(portPath, GPIO_PATH);
//    strcat(portPath, stringPortAddr);
//    snprintf(portPath, sizeof(portPath), "%s%d",GPIO_PATH, portAddr);

//    strcpy(portValuePath, portPath);
//    strcat(portValuePath, GPIO_VALUE);
    snprintf(portValuePath, sizeof(portValuePath), "%s%d%s", GPIO_PATH, portAddrOffSetted, GPIO_VALUE);

//    // Setting the clock
//    strcpy(clkPath, GPIO_PATH);
//    strcat(clkPath, stringCLKAddr);
//
//
//    strcpy(edgePath, clkPath);
//    strcat(edgePath, GPIO_EDGE);
    snprintf(edgePath, sizeof(edgePath), "%s%d%s", GPIO_PATH, clkAddrOffSetted, GPIO_EDGE);

//    strcpy(clkValuePath, clkPath);
//    strcat(clkValuePath, GPIO_VALUE);
    snprintf(clkValuePath, sizeof(clkValuePath), "%s%d%s", GPIO_PATH, clkAddrOffSetted, GPIO_VALUE);


    // Writing rising/edge/both in the edge file
    int edge_fileDescriptor = open(edgePath, O_WRONLY);
    if (edge_fileDescriptor == -1)
        perror("Opening gpio/gpio*CLK*/edge failed\r\n");


    if (write(edge_fileDescriptor, edgeOption, edgeOptionSize) == -1)
        perror("Writing to gpio/gpio*CLK*/edge failed\r\n");


    if (close(edge_fileDescriptor) == -1)
        perror("Closing gpio/gpio*CLK*/edge failed\r\n");


    int portFileDescriptor = open(portValuePath,/* O_APPEND |*/ O_RDONLY);
    if (portFileDescriptor == -1)
        perror("Opening gpio/gpio*Port*/value failed\r\n");

    int clkFileDescriptor;
    struct pollfd fds;
    char buffer[2];

    if ((clkFileDescriptor = open(clkValuePath, O_RDONLY)) < 0) {
        perror("Opening clk value port failed");
        return (-1);
    }

    int index;
    fds.fd = clkFileDescriptor;
    fds.events = POLLPRI;

    // Fill the buffer with the result of the reading
    for (index = 0; index < count - 1; index++) {
        if (poll(&fds, 1, -1) < 0) {
            perror("poll");
            break;
        }
        lseek(clkFileDescriptor, 0, SEEK_SET);
        if (read(portFileDescriptor, &buffer, 2) < 0) {
            perror("read");
            break;
        }
        buffer[1] = '\0';
        buf[index] = buffer[0];

        printf("Character read : %c\r\n", buffer[0]);

        if (read(clkFileDescriptor, &buffer, 2) != 2) {
            perror("Read");
            break;
        }
        buffer[1] = '\0';
    }
    buf[count - 1] = '\0';


    if (close(clkFileDescriptor) == -1)
        perror("Closing gpio/gpio*CLK Port*/value failed\r\n");

    if (close(portFileDescriptor) == -1)
        perror("Closing gpio/gpio*Port*/value failed\r\n");


    return 0;

}

/**
 * Function used to read the value on a port on each change of state on the clock
 * Will fill the buffer with the values read
 * Will read count - 1 value
 *
 * @param portAddr Address of the port to monitor
 * @param clkAddr Address of the clock
 * @param buf Buffer in witch the values will be put
 * @param count Size of the buffer
 * @param clkReadRate Refreshing rate of the reading of the clock
 * @param edgeOption Define when to read the value, options are EDGE_BOTH, EDGE_RISING and EDGE_FALLING
 * @return 0 in case of success
 */
int
gpio_Clocked_Read_Simple(int portAddr, int clkAddr, char buf[], size_t count, useconds_t clkReadRate, int edgeOption) {


    if (edgeOption != EDGE_BOTH && edgeOption != EDGE_FALLING && edgeOption != EDGE_RISING) {
        perror("edgeOption invalid parameter\r\n");
        exit(EXIT_FAILURE);
    }


    char bufferReader[2];
    char bufferReaderDif[2];
    char bufferReaderPortValue[2];
    size_t index = 0;

    // Read the first state of the clock
    if (gpio_Read(clkAddr, bufferReader, 2) < 0) {
        perror("Reading first clock value failed\r\n");
        exit(EXIT_FAILURE);
    }

    while (index < count - 1) {
        if (gpio_Read(clkAddr, bufferReaderDif, 2) < 0) {
            perror("Reading clock value failed\r\n");
            exit(EXIT_FAILURE);
        }
        // If the clock state as changed
        if ((bufferReaderDif[0] != bufferReader[0])) {
            bufferReader[0] = bufferReaderDif[0];
            // And the edge option is checked
            if (edgeOption == EDGE_BOTH || (edgeOption == EDGE_RISING && bufferReaderDif[0] == '1') ||
                (edgeOption == EDGE_FALLING && bufferReaderDif[0] == '0')) {
                // Read the value in the port monitored
                if (gpio_Read(portAddr, bufferReaderPortValue, 2) < 0) {
                    perror("Reading monitored port value failed\r\n");
                    exit(EXIT_FAILURE);
                }
                buf[index] = bufferReaderPortValue[0];
                index++;
            }
        }
        usleep(clkReadRate);
    }
    buf[count - 1] = '\0';

    return 0;
}

// TODO make it work
int gpio_Clocked_Epoll() {
    int n;
    int epfd = epoll_create(1);
    int fd = open("/sys/class/gpio/gpio86/value", O_RDWR | O_NONBLOCK);
    printf("open returned %d: %s\n", fd, strerror(errno));
    if (fd > 0) {
        char buf = 0;

        struct epoll_event ev;
        struct epoll_event events;
        ev.events = EPOLLET;
        ev.data.fd = fd;

        n = epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev);
        printf("epoll_ctl returned %d: %s\n", n, strerror(errno));

        while (1) {
            n = epoll_wait(epfd, &events, 1, -1);
            printf("epoll returned %d: %s\n", n, strerror(errno));

            if (n > 0) {
                n = lseek(fd, 0, SEEK_SET);
                printf("seek %d bytes: %s\n", n, strerror(errno));
                n = read(fd, &buf, 1);
                printf("read %d bytes: %s\n", n, strerror(errno));
                printf("buf = 0x%x\n", buf);
            }
        }
    }
    return (0);
}

int gpio_Clocked_Poll(int portAddr) {
    char valuePath[60], edgePath[60];
    snprintf(valuePath, sizeof(valuePath), "%s%d%s", GPIO_PATH, portAddr + LINUX_OFFSET, GPIO_VALUE);
    snprintf(edgePath, sizeof(edgePath), "%s%d%s", GPIO_PATH, portAddr + LINUX_OFFSET, GPIO_EDGE);
    int fd;
    struct pollfd fds;
    struct timeval tv;
    char buffer[2];

    int edgeFd = open(edgePath, O_WRONLY);
    if (edgeFd < 0) {
        perror("Opening edge failed");
        exit(EXIT_FAILURE);
    }

    if (write(edgeFd, "rising", strlen("rising")) < 0) {
        perror("Writing edge failed");
        exit(EXIT_FAILURE);
    }

    close(edgeFd);

    if ((fd = open(valuePath, O_RDONLY)) < 0) {
        perror("Opening value path failed");
        exit(EXIT_FAILURE);
    }
    while (1) {
        fds.fd = fd;
        fds.events = POLLPRI;
        if (poll(&fds, 1, -1) < 0) {
            perror("poll");
            break;
        }
        gettimeofday(&tv, NULL);
        lseek(fd, 0, SEEK_SET);
        if (read(fd, &buffer, 2) != 2) {
            perror("read");
            break;
        }
        buffer[1] = '\0';
        fprintf(stdout, "[%ld.%06ld]: %s\n", tv.tv_sec, tv.tv_usec, buffer);
    }
    close(fd);
    return 0;
}

int gpio_Clocked_Poll_Monitor(int clkAddr, int portAddr) {

    char clkValuePath[60], edgePath[60], portValuePath[60];
    snprintf(clkValuePath, sizeof(clkValuePath), "%s%d%s", GPIO_PATH, clkAddr + LINUX_OFFSET, GPIO_VALUE);
    snprintf(edgePath, sizeof(edgePath), "%s%d%s", GPIO_PATH, clkAddr + LINUX_OFFSET, GPIO_EDGE);
    snprintf(portValuePath, sizeof(portValuePath), "%s%d%s", GPIO_PATH, portAddr + LINUX_OFFSET, GPIO_VALUE);

    int fd;
    struct pollfd fds;
    struct timeval tv;
    char buffer[2], portBufferReader[2];

    int edgeFd = open(edgePath, O_WRONLY);
    if (edgeFd < 0) {
        perror("Opening edge failed");
        exit(EXIT_FAILURE);
    }

    if (write(edgeFd, "rising", strlen("rising")) < 0) {
        perror("Writing edge failed");
        exit(EXIT_FAILURE);
    }

    close(edgeFd);

    int portFd = open(portValuePath, O_RDONLY);
    if (portFd < 0) {
        perror("Opening monitored port failed");
        exit(EXIT_FAILURE);
    }


    if ((fd = open(clkValuePath, O_RDONLY)) < 0) {
        perror("Opening value path failed");
        exit(EXIT_FAILURE);
    }
    while (1) {
        fds.fd = fd;
        fds.events = POLLPRI;
        if (poll(&fds, 1, -1) < 0) {
            perror("poll");
            break;
        }
        gettimeofday(&tv, NULL);
        lseek(fd, 0, SEEK_SET);
        lseek(portFd, 0, SEEK_SET);
        read(portFd, portBufferReader, 2);
        if (read(fd, &buffer, 2) != 2) {
            perror("read");
            break;
        }
        buffer[1] = '\0';
        portBufferReader[1] = '\0';

        fprintf(stdout, "[%ld.%06ld]: %s\n", tv.tv_sec, tv.tv_usec, buffer);
        printf("Data read : %s", portBufferReader);
    }
    close(fd);
    close(portFd);
    return 0;
}