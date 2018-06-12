//
// Created by Mathieu Hannoun on 12/06/18.
// GPIOFS stand for General Purpose Input/Output File System
// It is a library used for the GPIO control on the ARM processor embedded on a Zedboard
// For better performances use the GPIODMC library
//


#include "GPIOFS.h"

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

    char stringCLKAddr[10];

    char portValuePath[80];
    char clkValuePath[80];
    char edgePath[80];
    int portAddrOffSetted = portAddr + LINUX_OFFSET;
    int clkAddrOffSetted = clkAddr + LINUX_OFFSET;

    snprintf(stringCLKAddr, sizeof(stringCLKAddr), "%d", clkAddrOffSetted);
    snprintf(portValuePath, sizeof(portValuePath), "%s%d%s", GPIO_PATH, portAddrOffSetted, GPIO_VALUE);
    snprintf(edgePath, sizeof(edgePath), "%s%d%s", GPIO_PATH, clkAddrOffSetted, GPIO_EDGE);
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

/**
 * Monitor a pin and read it's content each rising edge on the clk pin
 *
 * @param clkAddr Address of the clock
 * @param portAddr Address of the port to monitor
 * @return will not return, infinite loop
 */
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

/**
 * Set the desired edge in the corresponding pseudo file of a clock
 * @param clkAddr Address of the clock
 * @param edgeOption EDGE_RISING / EDGE_FALLING / EDGE_BOTH
 * @return 0 in case of success
 */
int gpio_Set_Edge(int clkAddr, int edgeOption) {
    if (edgeOption != EDGE_BOTH && edgeOption != EDGE_FALLING && edgeOption != EDGE_RISING) {
        perror("edgeOption invalid parameter\r\n");
        exit(EXIT_FAILURE);
    }

    char edgePath[60];
    snprintf(edgePath, sizeof(edgePath), "%s%d%s", GPIO_PATH, clkAddr + LINUX_OFFSET, GPIO_EDGE);

    int edgeFd = open(edgePath, O_WRONLY);
    if (edgeFd < 0) {
        perror("Opening edge failed");
        exit(EXIT_FAILURE);
    }
    if (edgeOption == EDGE_RISING) {
        if (write(edgeFd, "rising", strlen("rising")) < 0) {
            perror("Writing edge failed");
            exit(EXIT_FAILURE);
        }
    } else if (edgeOption == EDGE_FALLING) {
        if (write(edgeFd, "falling", strlen("falling")) < 0) {
            perror("Writing edge failed");
            exit(EXIT_FAILURE);
        }
    } else {
        if (write(edgeFd, "both", strlen("both")) < 0) {
            perror("Writing edge failed");
            exit(EXIT_FAILURE);
        }
    }
    close(edgeFd);
    return 0;
}

/**
 * Return a file descriptor for the opened pseudo file corresponding to the value of the port
 *
 * @param portAddr Port to open
 * @param flags Flag passed to the open() function
 * @return The file descriptor
 */
int gpio_Open(int portAddr, int flags) {
    char portPath[60];
    snprintf(portPath, sizeof(portPath), "%s%d%s", GPIO_PATH, portAddr + LINUX_OFFSET, GPIO_VALUE);
    int portFd = open(portPath, flags);
    return portFd;
}

// TODO add timeOUT
/**
 * Wait for an specified edge on the clock port to fill the buffer with data read
 * Will return only after the buffer is filled
 * Discard the first character
 * Read count - 1 character
 *
 * @param portAddr
 * @param clkAddr
 * @param buf Buffer that will be filled
 * @param count Size of the buffer in character
 * @param edgeOption EDGE_RISING / EDGE_FALLING / EDGE_BOTH
 * @return The number of character read (should be count - 1)
 */
int
gpio_Read_Clocked_Poll(int portAddr, int clkAddr, char buf[], size_t count, int edgeOption) {

    if (edgeOption != EDGE_BOTH && edgeOption != EDGE_FALLING && edgeOption != EDGE_RISING) {
        perror("edgeOption invalid parameter\r\n");
        exit(EXIT_FAILURE);
    }

    char clkValuePath[60], edgePath[60], portValuePath[60];
    snprintf(clkValuePath, sizeof(clkValuePath), "%s%d%s", GPIO_PATH, clkAddr + LINUX_OFFSET, GPIO_VALUE);
    snprintf(edgePath, sizeof(edgePath), "%s%d%s", GPIO_PATH, clkAddr + LINUX_OFFSET, GPIO_EDGE);
    snprintf(portValuePath, sizeof(portValuePath), "%s%d%s", GPIO_PATH, portAddr + LINUX_OFFSET, GPIO_VALUE);

    int fd;
    struct pollfd fds;
    char buffer[2], portBufferReader[2];

    int edgeFd = open(edgePath, O_WRONLY | O_SYNC);
    if (edgeFd < 0) {
        perror("Opening edge failed");
        exit(EXIT_FAILURE);
    }
    if (edgeOption == EDGE_RISING) {
        if (write(edgeFd, "rising", strlen("rising")) < 0) {
            perror("Writing edge failed");
            exit(EXIT_FAILURE);
        }
    } else if (edgeOption == EDGE_FALLING) {
        if (write(edgeFd, "falling", strlen("falling")) < 0) {
            perror("Writing edge failed");
            exit(EXIT_FAILURE);
        }
    } else {
        if (write(edgeFd, "both", strlen("both")) < 0) {
            perror("Writing edge failed");
            exit(EXIT_FAILURE);
        }
    }
    close(edgeFd);

    int portFd = open(portValuePath, O_RDONLY | O_SYNC);
    if (portFd < 0) {
        perror("Opening monitored port failed");
        exit(EXIT_FAILURE);
    }


    if ((fd = open(clkValuePath, O_RDONLY | O_SYNC)) < 0) {
        perror("Opening value path failed");
        exit(EXIT_FAILURE);
    }
    size_t index = 0;

    // read it a first time
    fds.fd = fd;
    fds.events = POLLPRI;
    if (poll(&fds, 1, -1) < 0) {
        perror("poll");
        exit(EXIT_FAILURE);
    }
    lseek(fd, 0, SEEK_SET);
    if (read(fd, &buffer, 2) != 2) {
        perror("read");
        exit(EXIT_FAILURE);
    }
    buffer[1] = '\0';

    // Actual reading loop
    for (index = 0; index < count - 1; index++) {
        fds.fd = fd;
        fds.events = POLLPRI;
        if (poll(&fds, 1, -1) < 0) {
            perror("poll");
            break;
        }
        lseek(fd, 0, SEEK_SET);
        lseek(portFd, 0, SEEK_SET);
        read(portFd, portBufferReader, 2);
        if (read(fd, &buffer, 2) != 2) {
            perror("read");
            break;
        }
        buffer[1] = '\0';
        portBufferReader[1] = '\0';
        buf[index] = portBufferReader[0];
    }
    close(fd);
    close(portFd);
    buf[index] = '\0';
    return index;
}

/**
 * Work the same way as gpio_Read_Clocked_Poll but this time without openging or closing pseudos files
 * Thus making it faser
 * @param portFd File descriptor of the port to monitor
 * @param clkFd File descriptor of the clock
 * @param buf Buffer that will be filled
 * @param count Size of the buffer in character
 * @return The number of character read (should be count - 1)
 */
size_t
gpio_Read_Clocked_Poll_Fast(int portFd, int clkFd, char buf[], size_t count) {

    struct pollfd fds;
    char buffer[2], portBufferReader[2];
    size_t index = 0;

    // reads it a first time
    fds.fd = clkFd;
    fds.events = POLLPRI;
    if (poll(&fds, 1, -1) < 0) {
        perror("poll");
        exit(EXIT_FAILURE);
    }
    lseek(clkFd, 0, SEEK_SET);
    if (read(clkFd, &buffer, 2) != 2) {
        perror("read");
        exit(EXIT_FAILURE);
    }
    buffer[1] = '\0';

    // Actual reading loop
    for (index = 0; index < count - 1; index++) {
        fds.fd = clkFd;
        fds.events = POLLPRI;
        if (poll(&fds, 1, -1) < 0) {
            perror("poll");
            break;
        }
        lseek(clkFd, 0, SEEK_SET);
        lseek(portFd, 0, SEEK_SET);
        read(portFd, portBufferReader, 2);
        if (read(clkFd, &buffer, 2) != 2) {
            perror("read");
            break;
        }
        buffer[1] = '\0';
        portBufferReader[1] = '\0';
        buf[index] = portBufferReader[0];
    }

    buf[index] = '\0';
    return index;
}


int reader_Fast() {

    int clkIn = 32, dataIn = 33;
    char bufferReader[20][9];

    gpio_SetDataDirection(clkIn, GPIO_IN);
    gpio_SetDataDirection(dataIn, GPIO_IN);
    gpio_Set_Edge(clkIn, EDGE_RISING);

    int clkFd = gpio_Open(clkIn, O_RDONLY);
    if (clkFd < 0) {
        perror("Port opening impossible");
        exit(EXIT_FAILURE);
    }
    int dataFd = gpio_Open(dataIn, O_RDONLY);
    if (dataFd < 0) {
        perror("Port opening impossible");
        exit(EXIT_FAILURE);
    }

    for (int index = 0; index < 20; index++) {
        gpio_Read_Clocked_Poll_Fast(dataFd, clkFd, bufferReader[index], 9);
    }
    for (int i = 0; i < 20; i++) {
        printf("index : %d, Data Read : %s\r\n", i, bufferReader[i]);
    }
    close(clkFd);
    close(dataFd);

    gpio_Unexport(clkIn);
    gpio_Unexport(dataIn);

    return 0;
}

/**
 * Write a value directly on the pseudo file without opening it again
 * @param fd File descriptor, pseudo file opened
 * @param value Value to write '0' or '1'
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Fast(int fd, char value) {

    if (value != '0' && value != '1') {
        perror("Writing failed, value must be 1 or 0");
        return -1;
    }
    char bufferWriter[2];
    bufferWriter[0] = value;
    bufferWriter[1] = '\0';
    write(fd, bufferWriter, 2);
    return 0;
}

/**
 * Will test the function gpio_Write_Fast by measuring it's average write frequency.
 * Port Ja1 is used
 * @param nbTest
 * @param internalRepetition
 */
void gpio_Write_Fast_Tester(int nbTest, int internalRepetition) {
    int dataOut = 24;

    gpio_SetDataDirection(24, GPIO_OUT);
    int fd = gpio_Open(24, O_WRONLY);
    usleep(10);
    int index = 0;

    struct timeval tv, tv2;
    long testResult[nbTest];
    long testResultUS[nbTest];

    for (int j = 0; j < nbTest; j++) {
        gettimeofday(&tv, NULL);
        // Insert function to test here
        for (index = 0; index < internalRepetition; index++) {
            gpio_Write_Fast(fd, '0');
            gpio_Write_Fast(fd, '1');
        }
        gettimeofday(&tv2, NULL);

        testResult[j] = (int) (tv2.tv_sec * 1000000 + tv2.tv_usec) - (int) (tv.tv_sec * 1000000 + tv.tv_usec);
    }
    long result = 0;
    for (int k = 1; k < nbTest - 1; k++) {
        result += testResult[k];
    }

    double averageTimeElapsed = ((double) result) / (nbTest - 2);

    printf("Average time for internal function measured is : %lf\r\n us", averageTimeElapsed);

    double period = averageTimeElapsed / internalRepetition;
    printf("Average period is : %lf us\r\n", period);

    double frequency = ((double) 1.00 / period) * 1000000;
    printf("Average frequency is : %lf Hz\r\n", frequency);

    close(fd);
    gpio_Unexport(24);
}

/**
 * Export all the port of a transceiver and its clock in the correct direction
 * @param transceiver Transceiver to export
 * @param mode GPIO_IN or GPIO_OUT
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Export_Transceiver(CustomTransceiver transceiver, int mode) {
    int i, flag;
    for (i = 0; i < transceiver.nb_Data_Pins; i++) {
        flag = gpio_SetDataDirection(*(transceiver.pins_Ports + i), mode);
        if (flag < 0)
            return -1;
        usleep(1);
    }
    flag = gpio_SetDataDirection(transceiver.clk_Port, mode);
    if (flag < 0)
        return -1;
}

/**
 * Unexport all the port of a transceiver and its clock
 * @param transceiver Transceiver to unexport
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Unexport_Transceiver(CustomTransceiver transceiver) {
    int i, flag;
    for (i = 0; i < transceiver.nb_Data_Pins; i++) {
        flag = gpio_Unexport(*(transceiver.pins_Ports + i));
        if (flag < 0)
            return -1;
        usleep(1);
    }
    flag = gpio_Unexport(transceiver.clk_Port);
    if (flag < 0)
        return -1;

    return 0;
}

/**
 * Open all the port of the transceiver and its clock
 * Assigning them files descriptors
 * @param transceiver Transceiver to set up
 * @param flags Opening flags (O_WRONLY)
 * @return 0 in case of success, -1 in case of failure
 */
CustomTransceiver gpio_Open_Transceiver(CustomTransceiver transceiver, int flags) {
    int i;
    transceiver.pins_Fds = malloc(sizeof(int) * transceiver.nb_Data_Pins);
    if (transceiver.pins_Fds == NULL) {
        perror("Memory allocation for port files descriptor failed");
        exit(EXIT_FAILURE);
    }
    for (i = 0; i < transceiver.nb_Data_Pins; i++) {
        *(transceiver.pins_Fds + i) = gpio_Open(*(transceiver.pins_Ports + i), flags);
        if (*(transceiver.pins_Fds + i) < 0) {
            perror("Transceiver opening port files descriptors failed");
            exit(EXIT_FAILURE);
        }
        usleep(2);
    }
    transceiver.clk_fd = gpio_Open(transceiver.clk_Port, flags);
    if (transceiver.clk_fd < 0) {
        perror("Transceiver opening clock file descriptor failed");
        exit(EXIT_FAILURE);
    }
    return transceiver;
}

/**
 * Close all the files descriptor opened for the transceiver
 * Including the clock file descriptor
 * @param transceiver Transceiver to close
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Close_Transceiver(CustomTransceiver transceiver) {
    int i, flag;
    for (i = 0; i < transceiver.nb_Data_Pins; i++) {
        flag = close(*(transceiver.pins_Fds + i));
        if (flag < 0) {
            perror("Transceiver closing port files descriptors failed");
            return -1;
        }
        usleep(1);
    }
    transceiver.clk_fd = close(transceiver.clk_fd);
    if (transceiver.clk_fd < 0) {
        perror("Transceiver closing clock file descriptor failed");
        return -1;
    }
    return 0;
}

/**
 * Write port by port a data, then rise the clock
 * Data array must be same number as the data pin numbers
 * @param transceiver Transceiver containing all the information about clock and port
 * @param data Array of bit to transmit
 * @param dataSize Size of the array of data
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Rising_half_Transceiver(CustomTransceiver transceiver, char data[], int dataSize) {
    int i, flag;
    if (dataSize != transceiver.nb_Data_Pins) {
        perror("Data size mismatch transceiver port number");
        return -1;
    }
    // Write data
    for (i = 0; i < transceiver.nb_Data_Pins; i++) {
        flag = gpio_Write_Fast(*(transceiver.pins_Fds + i), data[i]);
        if (flag < 0) {
            perror("Transceiver Writing data failed");
            return -1;
        }
    }
    // Write Clk high
    flag = gpio_Write_Fast(transceiver.clk_fd, '1');
    if (flag < 0) {
        perror("Transceiver writing clock failed");
        return -1;
    }
    return 0;
}

/**
 * Write port by port a data, then lower the clock
 * Data array must be same number as the data pin numbers
 * @param transceiver Transceiver containing all the information about clock and port
 * @param data Array of bit to transmit
 * @param dataSize Size of the array of data
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Falling_half_Transceiver(CustomTransceiver transceiver, char data[], int dataSize) {
    int i, flag;
    if (dataSize != transceiver.nb_Data_Pins) {
        perror("Data size mismatch transceiver port number");
        return -1;
    }
    // Write data
    for (i = 0; i < transceiver.nb_Data_Pins; i++) {
        flag = gpio_Write_Fast(*(transceiver.pins_Fds + i), data[i]);
        if (flag < 0) {
            perror("Transceiver Writing data failed");
            return -1;
        }
    }
    // Write Clk low
    flag = gpio_Write_Fast(transceiver.clk_fd, '0');
    if (flag < 0) {
        perror("Transceiver writing clock failed");
        return -1;
    }
    return 0;
}

/**
 * Writing data on both edge of the clock
 *
 * @param transceiver Transceiver containing all the information about clock and port
 * @param dataFirst First set of data to transmit on rising edge
 * @param dataSecond Second set of data to transmit on falling edge
 * @param dataSize Size of the array of data
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Both(CustomTransceiver transceiver, char dataFirst[], char dataSecond[], int dataSize) {
    if (dataSize != transceiver.nb_Data_Pins) {
        perror("Data size mismatch transceiver port number");
        return -1;
    }
    int flag = gpio_Write_Rising_half_Transceiver(transceiver, dataFirst, dataSize);
    if (flag < 0) {
        perror("Writing first part of data failed");
        return -1;
    }
    flag = gpio_Write_Falling_half_Transceiver(transceiver, dataSecond, dataSize);
    if (flag < 0) {
        perror("Writing second part of data failed");
        return -1;
    }
    return 0;
}

/**
 * Write port by port a data, then rise the clock and lower it after
 * Data array must be same number as the data pin numbers
 * Complete cycle of data transmission on rising edge
 * @param transceiver Transceiver containing all the information about clock and port
 * @param data Array of bit to transmit
 * @param dataSize Size of the array of data
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Rising_Transceiver(CustomTransceiver transceiver, char data[], int dataSize) {
    if (dataSize != transceiver.nb_Data_Pins) {
        perror("Data size mismatch transceiver port number");
        return -1;
    }
    int flag = gpio_Write_Rising_half_Transceiver(transceiver, data, dataSize);
    if (flag < 0) {
        perror("Writing data failed");
        return -1;
    }
    // Write Clk low
    flag = gpio_Write_Fast(transceiver.clk_fd, '0');
    if (flag < 0) {
        perror("Transceiver writing clock failed");
        return -1;
    }
    return 0;
}

/**
 * Write port by port a data, then lower the clock and rise it after
 * Data array must be same number as the data pin numbers
 * Complete cycle of data transmission on falling edge
 * @param transceiver Transceiver containing all the information about clock and port
 * @param data Array of bit to transmit
 * @param dataSize Size of the array of data
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Falling_Transceiver(CustomTransceiver transceiver, char data[], int dataSize) {
    if (dataSize != transceiver.nb_Data_Pins) {
        perror("Data size mismatch transceiver port number");
        return -1;
    }
    int flag = gpio_Write_Falling_half_Transceiver(transceiver, data, dataSize);
    if (flag < 0) {
        perror("Writing data failed");
        return -1;
    }
    // Write Clk low
    flag = gpio_Write_Fast(transceiver.clk_fd, '1');
    if (flag < 0) {
        perror("Transceiver writing clock failed");
        return -1;
    }
    return 0;
}

void transceiver_Print_Info(CustomTransceiver transceiver) {
    printf("Clock port : %d\tFd : %d\r\nNumber of data pins : %d\r\n", transceiver.clk_Port, transceiver.clk_fd,
           transceiver.nb_Data_Pins);
    int index;
    for (index = 0; index < transceiver.nb_Data_Pins; index++) {
        printf("Port : %d\tFd : %d\r\n", *(transceiver.pins_Ports + index), *(transceiver.pins_Fds + index));
    }
}

/**
 * Will test the function gpio_Write_Rising_Transceiver by measuring it's average write frequency.
 * Will write 7 bits and its clocks in one time.
 * Whole Pmod JA is used
 * @param nbTest
 * @param internalRepetition
 */
void transceiver_Write_Tester(int nbTest, int internalRepetition, CustomTransceiver transceiver) {
    int dataSize = 3;
    char dataToTransmit[] = {'1', '1', '1'};
    char dataToTransmit2[] = {'0', '0', '0'};
    int index = 0;
    struct timeval tv, tv2;
    long testResult[nbTest];

    for (int j = 0; j < nbTest; j++) {
        gettimeofday(&tv, NULL);
        for (index = 0; index < internalRepetition; index++) {
            // Insert function to test here

            //gpio_Write_Rising_Transceiver(transceiver, dataToTransmit, dataSize);
            gpio_Write_Both(transceiver, dataToTransmit, dataToTransmit2, dataSize);
        }
        gettimeofday(&tv2, NULL);

        testResult[j] = (int) (tv2.tv_sec * 1000000 + tv2.tv_usec) - (int) (tv.tv_sec * 1000000 + tv.tv_usec);
    }
    long result = 0;
    for (int k = 1; k < nbTest - 1; k++) {
        result += testResult[k];
    }

    double averageTimeElapsed = ((double) result) / (nbTest - 2);

    printf("Average time for internal function measured is : %lf\r\n us", averageTimeElapsed);

    double period = averageTimeElapsed / internalRepetition;
    printf("Average period is : %lf us\r\n", period);

    double frequency = ((double) 1.00 / period) * 1000000;
    printf("Average frequency is : %lf Hz\r\n", frequency);

}