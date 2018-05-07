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
#define GPIO_MAP_SIZE 0x1000
#define ZYNQ_GPIO_UPPER_MASK 0xFFFF0000
#define GPIO_BANK2_PROTECTION_MASK 0b1111111
#define GPIO_OUT 0
#define GPIO_IN 1
#define EDGE_RISING 0
#define EDGE_FALLING 1
#define EDGE_BOTH 2
#define GPIO_OFFSET_OUTPUT_BANK(bankNumber) (0x0040 + ((bankNumber) * 0x04))
#define GPIO_OFFSET_DIRM(bankNumber) (0x0204 + ((bankNumber) * 0x40))
#define GPIO_OFFSET_OEN(bankNumber) (0x0208 + ((bankNumber) * 0x40))
#define APER_CLK_CTRL 0x0000012C
#define SYSTEM_LVL_CTR_BASE 0xF8000000
#define PIN_NUMBER_PER_BANK 32

/**
 * Transceiver responsible for handling data ports used for transmission
 * For now pins ports must belong to the same bank
 * Port array must be in most significant bit to least significant bit order
 * Set clk to negative value to disable it
 */
typedef struct {
    int nb_Data_Pins;
    int *pins_Ports;
    int clk_Port;

} CustomMemTransceiver;


int gpio_Mem_Set_Value(unsigned int port, int value);

int gpio_Mem_Set_Data_Direction(int port, int direction);

int gpio_Mem_Set_Transceiver_Direction(CustomMemTransceiver transceiver, int direction);


int gpio_Mem_Map();

int gpio_Mem_Unmap(int fd);

int gpio_Mem_Write(int port, int value);

int gpio_Mem_Write_Transceiver_Slow(CustomMemTransceiver transceiver, int *dataArray, int dataSize);

int gpio_Mem_Write_Direct_Data(int bank, __uint32_t value);

int open_Amba_clk();

int enable_gpio_clock();

int disable_gpio_clock();

int gpio_Entire_Bank_On(int bankNumber);

void print_Bank_Addr();

int *
gpio_Mem_Formatting_Data(CustomMemTransceiver transceiver, const int *dataArray, int dataSize, int *resultArraySize);

void print_Formatted_Data(const int *dataArray, int dataSize);

void gpio_mem_speed_test(int nbTest, int internalRepetition, int delay);

// TODO make it local
volatile unsigned int *g_CLOCK_ADDRESS;
volatile void *g_MEMORY_MAP;

int main(int argc, char *argv[]) {

//
//    int port = 32;
//    int fd = gpio_Mem_Map();
//    open_Amba_clk();
//    usleep(100);
//    enable_gpio_clock();
//    gpio_Mem_Set_Data_Direction(port, GPIO_OUT);
//    usleep(100);
//    //while(1){
//    gpio_Mem_Write(port, 1);
//    usleep(1);
//    //}
//    usleep(100);
//    disable_gpio_clock();
//    gpio_Mem_Unmap(fd);
//


//    int data[4] = {1, 0, 1, 0};
//    CustomMemTransceiver transceiver;
//    int pinPorts[4] = {10, 9, 8, 7};
//    transceiver.pins_Ports = pinPorts;
//    transceiver.nb_Data_Pins = 4;
//    transceiver.clk_Port = -1;
////    int *returnedDataSize = malloc(sizeof(int));
////    *returnedDataSize = 0;
////    int *dataToPrint = gpio_Mem_Formatting_Data(transceiver, data, 4, returnedDataSize);
////    printf("Returned size : %d\r\n", *returnedDataSize);
////    print_Formatted_Data(dataToPrint, *returnedDataSize);
//
//
//    int fd = gpio_Mem_Map();
//    open_Amba_clk();
//    usleep(100);
//    enable_gpio_clock();
//    usleep(100);
//    gpio_Mem_Set_Transceiver_Direction(transceiver, GPIO_OUT);
//    usleep(100);
////    gpio_Mem_Write_Direct_Data(3, (__uint32_t) dataToPrint[0]);
//    gpio_Mem_Write_Transceiver_Slow(transceiver, data, 4);
//    usleep(100);
//    disable_gpio_clock();
//    usleep(100);
//    gpio_Mem_Unmap(fd);


    gpio_mem_speed_test(10, 1000000, 0);


    return 0;
}


// TODO split this function into opening port and writing value
// TODO split again the function which find the port corresponding bank
/**
 * DEPRECATED
 *
 * Check in what bank the port is in set it as an output and write in its register the value
 * If 1 : (Current Content) or (1 shifted port time)
 * If 0 : (Current Content) and (Not (1 shifted port time))
 * @param port
 * @param value
 * @return
 */
int gpio_Mem_Set_Value(unsigned int port, int value) {
    int bank;
    int fd = open("/dev/mem", O_RDWR);
    volatile void *gpio_ptr = mmap(NULL, GPIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_ADDR);
    if (gpio_ptr == MAP_FAILED) {
        perror("Clk memory mapping failed");
        exit(EXIT_FAILURE);
    }
    if (port < PIN_NUMBER_PER_BANK)
        bank = 2;
    else {
        bank = 3;
        port = port - PIN_NUMBER_PER_BANK;
    }

    volatile unsigned int *setDirectionAddr;
    volatile unsigned int *setValueAddr;
    volatile unsigned int *setOutputEnableAddr;
    setDirectionAddr = gpio_ptr + GPIO_OFFSET_DIRM(bank);
    setOutputEnableAddr = gpio_ptr + GPIO_OFFSET_OEN(bank);
    setValueAddr = gpio_ptr + GPIO_OFFSET_OUTPUT_BANK(bank);


    // Direction out
    *setDirectionAddr |= (1u << port);
    *setOutputEnableAddr |= (1u << port);
    // Writing data
    if (value == 0) {
        *setValueAddr &= ~(1u << port);
    } else {
        *setValueAddr |= (1u << port);

    }
    //close(fd);
    return 0;
}

/**
 * DEPRECATED
 *
 * @param bankNumber
 * @return
 */
int gpio_Entire_Bank_On(int bankNumber) {
    int fd = open("/dev/mem", O_RDWR);
    volatile void *gpio_ptr = mmap(NULL, GPIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_ADDR);
    if (gpio_ptr == MAP_FAILED) {
        perror("Clk memory mapping failed");
        exit(EXIT_FAILURE);
    }
    volatile unsigned int *setDirectionAddr;
    volatile unsigned int *setValueAddr;
    volatile unsigned int *setOutputEnableAddr;
    setDirectionAddr = gpio_ptr + GPIO_OFFSET_DIRM(bankNumber);
    setOutputEnableAddr = gpio_ptr + GPIO_OFFSET_OEN(bankNumber);
    setValueAddr = gpio_ptr + GPIO_OFFSET_OUTPUT_BANK(bankNumber);

    // Direction out for all the bank
    *setDirectionAddr |= 0xFFFFFFFF;
    *setOutputEnableAddr |= 0xFFFFFFFF;
    sleep(1);
    // Writing 1 on all the bank
    *setValueAddr |= 0xFFFFFFFF;
    //close(fd);
    return 0;

}

int open_Amba_clk() {
    int fd = open("/dev/mem", O_RDWR);
    volatile void *gpio_ptr = mmap(NULL, GPIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, SYSTEM_LVL_CTR_BASE);
    if (gpio_ptr == MAP_FAILED) {
        perror("Clk memory mapping failed");
        exit(EXIT_FAILURE);
    }
    volatile unsigned int *clkAddr;
    clkAddr = gpio_ptr + APER_CLK_CTRL;
    g_CLOCK_ADDRESS = clkAddr;
    //close(fd);
    return 0;
}

int enable_gpio_clock() {
    *g_CLOCK_ADDRESS |= (1u << 22);
}

int disable_gpio_clock() {
    *g_CLOCK_ADDRESS &= ~(1u << 22);
}

void print_Bank_Addr() {
    int index;
    for (index = 0; index < 4; index++) {
        printf("Output : %x\tDirm : %x\tOEN : %x\r\n", GPIO_OFFSET_OUTPUT_BANK(index), GPIO_OFFSET_DIRM(index),
               GPIO_OFFSET_OEN(index));
    }
}

int gpio_Mem_Set_Data_Direction(int port, int direction) {
    int bank;
    if (port < PIN_NUMBER_PER_BANK)
        bank = 2;
    else {
        bank = 3;
        port = port - PIN_NUMBER_PER_BANK;
    }
    volatile unsigned int *setDirectionAddr;
    volatile unsigned int *setOutputEnableAddr;

    setDirectionAddr = g_MEMORY_MAP + GPIO_OFFSET_DIRM(bank);
    setOutputEnableAddr = g_MEMORY_MAP + GPIO_OFFSET_OEN(bank);

    // Set the direction
    if (direction == GPIO_IN) {
        *setDirectionAddr &= ~(1u << port);
    } else {
        *setDirectionAddr |= (1u << port);

    }
    // TODO check if sleeping is necessary
    usleep(10);
    // Enable or disable the output
    if (direction == GPIO_IN) {
        *setOutputEnableAddr &= ~(1u << port);
    } else {
        *setOutputEnableAddr |= (1u << port);

    }
    return 0;
}

/**
 * Set the direction for all the ports of a transceiver and its clock if it is not null
 * @param transceiver Transceiver containing all the pins information
 * @param direction GPIO_IN/GPIO_OUT
 * @return 0 in case of success
 */
int gpio_Mem_Set_Transceiver_Direction(CustomMemTransceiver transceiver, int direction) {
    int index;
    for (index = 0; index < transceiver.nb_Data_Pins; index++) {
        gpio_Mem_Set_Data_Direction(transceiver.pins_Ports[index], direction);
    }
    if (transceiver.clk_Port < 0)
        gpio_Mem_Set_Data_Direction(transceiver.clk_Port, direction);
    return 0;
}


// todo add structure and pointer arguments
/**
 * Map the gpio driver memory space
 * Used before any operation to initialize g_MEMORY_MAP
 * @return the file descriptor used for opening /dev/mem
 */
int gpio_Mem_Map() {
    int fd = open("/dev/mem", O_RDWR);
    g_MEMORY_MAP = mmap(NULL, GPIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_ADDR);
    if (g_MEMORY_MAP == MAP_FAILED) {
        perror("Memory mapping failed");
        exit(EXIT_FAILURE);
    }
    return fd;
}

/**
 * Unmap previously mapped memory space
 * Close opened file descriptor
 * @param fd file descriptor returned by gpio_Mem_Map
 * @return 0 in case of success, -1 in case of faillure
 */
int gpio_Mem_Unmap(int fd) {
    int flag = munmap((void *) g_MEMORY_MAP, GPIO_MAP_SIZE);
    if (flag < 0) {
        perror("Memory unmapping failed");
        return -1;
    }
    close(fd);
    return 0;
}

/**
 * Write the chosen value to a gpio port
 * @param port Gpio port as defined in the .xdc file
 * @param value Value to write, non zero is treated as one
 * @return 0 in case of success
 */
int gpio_Mem_Write(int port, int value) {
    int bank;
    if (port < PIN_NUMBER_PER_BANK)
        bank = 2;
    else {
        bank = 3;
        port = port - PIN_NUMBER_PER_BANK;
    }
    volatile unsigned int *setValueAddr;
    setValueAddr = g_MEMORY_MAP + GPIO_OFFSET_OUTPUT_BANK(bank);
    // Writing data
    if (value == 0) {
        *setValueAddr &= ~(1u << port);
    } else {
        *setValueAddr |= (1u << port);

    }
    return 0;
}

/**
 * Write value on port one by one without a clk signal
 * The data arrays size must be equal the transceiver port number
 * @param transceiver
 * @param dataArray Array of binary data, not '0' is treated as 1
 * @param dataSize Size of the data array
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Mem_Write_Transceiver_Slow(CustomMemTransceiver transceiver, int *dataArray, int dataSize) {
    if (dataSize != transceiver.nb_Data_Pins) {
        perror("Data array and port number mismatch");
        return -1;
    }
    int index;
    for (index = 0; index < transceiver.nb_Data_Pins; index++) {
        gpio_Mem_Write(transceiver.pins_Ports[index], dataArray[index]);
    }
    return 0;
}


/**
 * NOT FUNCTIONAL
 * Write data on an entire bank
 * @param bank Bank to write data on
 * @param value Value used to generated output
 * @return 0 in case o success
 */
int gpio_Mem_Write_Direct_Data(int bank, __uint32_t value) {
    // If bank 2, mask the data to prevent change in OLED display

    volatile unsigned int *setValueAddr;
    setValueAddr = g_MEMORY_MAP + GPIO_OFFSET_OUTPUT_BANK(bank);

    if (bank == 2){
        int temp;
        temp = (*setValueAddr & GPIO_BANK2_PROTECTION_MASK);
        value = temp | value;

    }
    // Writing data
    *setValueAddr = (unsigned) value;
    return 0;
}


/**
 * Convert an array of binary values to an array of "Writable" values for the actual transceiver
 * Handle the allocation of the newly created array
 *
 * @param transceiver Transceiver containing all the information about used ports
 * @param dataArray Array of binary data, not '0' is treated as '1'
 * @param dataSize Size of the data Array
 * @param resultArraySize Size of the returned array
 * @return The newly formatted data array
 */
int *
gpio_Mem_Formatting_Data(CustomMemTransceiver transceiver, const int *dataArray, int dataSize, int *resultArraySize) {
    int requiredRows;
    if (dataSize % transceiver.nb_Data_Pins == 0) {
        requiredRows = dataSize / transceiver.nb_Data_Pins;
    }
        // If sizes don't match, add a cell and fill the void with '0' latter
    else {
        requiredRows = (dataSize / transceiver.nb_Data_Pins) + 1;

    }
    int *usableDataArray = malloc(requiredRows * sizeof(int));
    if (usableDataArray == NULL) {
        perror("Memory allocation failed");
        return NULL;
    }
    *resultArraySize = requiredRows;

    int rowIndex, columnIndex, port;
    unsigned int cellValue;
    for (rowIndex = 0; rowIndex < requiredRows; rowIndex++) {
        cellValue = 0;
        for (columnIndex = 0; columnIndex < transceiver.nb_Data_Pins; columnIndex++) {

            port = transceiver.pins_Ports[columnIndex];

            // If the port is in bank 3
            if (port >= PIN_NUMBER_PER_BANK)
                port = port - PIN_NUMBER_PER_BANK;

            if (dataArray[columnIndex + (transceiver.nb_Data_Pins * rowIndex)] == 0) {
                cellValue &= ~(1u << port);
            } else {
                cellValue |= (1u << port);
            }
        }
        usableDataArray[rowIndex] = cellValue;
    }
    return usableDataArray;
}

void print_Formatted_Data(const int *dataArray, int dataSize) {
    int index;
    for (index = 0; index < dataSize; index++) {
        printf("%x\r\n", dataArray[index]);
    }
}

void gpio_mem_speed_test(int nbTest, int internalRepetition, int delay){

    int port = 32;
    int fd = gpio_Mem_Map();
    open_Amba_clk();
    usleep(10);
    enable_gpio_clock();
    usleep(10);
    gpio_Mem_Set_Data_Direction(port, GPIO_OUT);
    usleep(10);

    int index = 0;
    struct timeval tv, tv2;
    long testResult[nbTest];
    int value = 0;
    for (int j = 0; j < nbTest; j++) {
        gettimeofday(&tv, NULL);
        for (index = 0; index < internalRepetition; index++) {
            value = 0;
            // Insert function to test here
            gpio_Mem_Write(port,value);
            value = 1;
            for (int i = 0; i < delay; ++i) {
                // nothing
            }
            gpio_Mem_Write(port,value);

        }
        fsync(fd);
        gettimeofday(&tv2, NULL);
        testResult[j] = (int) (tv2.tv_sec * 1000000 + tv2.tv_usec) - (int) (tv.tv_sec * 1000000 + tv.tv_usec);
    }
    usleep(100);
    disable_gpio_clock();
    gpio_Mem_Unmap(fd);

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
