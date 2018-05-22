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
#define GPIO_DATA_RO(bankNumber) (0x60 + ((bankNumber)* 0x04))
#define APER_CLK_CTRL 0x0000012C
#define SYSTEM_LVL_CTR_BASE 0xF8000000
#define PIN_NUMBER_PER_BANK 32
#define DELAY_MAX 10000
#define TIMEOUT_MAX_COUNT 1000
#define TC_HIGH 1
#define TC_LOW 0
#define RC_HIGH 1
#define RC_LOW 0


/**
 * Transceiver responsible for handling data ports used for transmission
 * For now pins, tr and rc ports must belong to the same bank
 * Port array must be in most significant bit to least significant bit order
 * Set tc and rc to negative value to disable them
 */
typedef struct {
    int nb_Data_Pins;
    int *pins_Ports;
    int tc_Port;
    int rc_Port;
    int bank;
} CustomMemTransceiver;


int gpio_Mem_Set_Value(unsigned int port, int value);

int gpio_Mem_Set_Data_Direction(int port, int direction);

int gpio_Mem_Set_Transceiver_Direction(CustomMemTransceiver transceiver, int direction);

int gpio_Mem_Set_Data_Direction_Range(int firstPort, int lastPort, int direction);

int gpio_Mem_Map();

int gpio_Mem_Unmap(int fd);

int gpio_Mem_Write(int port, int value);

int gpio_Mem_Write_Transceiver_Slow(CustomMemTransceiver transceiver, int *dataArray, int dataSize);

int gpio_Mem_Write_Direct_Data(int bank, __uint32_t value);

int gpio_Mem_Chip_To_Chip_Writer(CustomMemTransceiver transceiver, int *writableDataArray, int writableDataArraySize);

__uint32_t *gpio_Mem_Chip_To_Chip_Reader(CustomMemTransceiver transceiver, __uint32_t *resultArraySize);

unsigned int gpio_Mem_Read_Bank(int bank);

int open_Amba_clk();

int enable_gpio_clock();

int disable_gpio_clock();

int gpio_Entire_Bank_On(int bankNumber);

void print_Bank_Addr();

__uint32_t gpio_Reading_Mask_From_Transceiver(CustomMemTransceiver transceiver);

int *
gpio_Mem_Formatting_Binary_Data(CustomMemTransceiver transceiver, const int *dataArray, int dataSize,
                                int *resultArraySize);

int *gpio_Mem_Formatting_Integer_Data(CustomMemTransceiver transceiver, const uint32_t *dataArray, int dataSize);

void print_Formatted_Data(__uint32_t *dataArray, int dataSize);

int gpio_Bank_From_port(int port);

int gpio_Mem_Shift_Data(CustomMemTransceiver transceiver, uint32_t data);

__uint32_t gpio_Mem_Unshift_Data(CustomMemTransceiver transceiver, __uint32_t data);

uint32_t gpio_Mem_Wait_For_Value(CustomMemTransceiver transceiver, int value, int *timeOutFlag);

int gpio_Mem_Transceiver_Send_Data(CustomMemTransceiver transceiver, __uint32_t value, int tcState);

void gpio_Mem_CleanUp(int fd, int bank);

void gpio_mem_speed_test(int nbTest, int internalRepetition, int delay);

void gpio_mem_multiple_speed_test(int nbTest, int internalRepetition, int delay);

void gpio_Mem_Protocol_Reader();

void gpio_Mem_Protocol_Writer();

uint32_t *gpio_Unmarshall(CustomMemTransceiver transceiver, int *data, size_t dataSize, size_t *returnedDataSize);

uint32_t *gpio_Marshall(CustomMemTransceiver transceiver, uint32_t *data, size_t dataSize, size_t *returnedDataSize);

uint32_t *
gpio_Marshall_Simplified(CustomMemTransceiver transceiver, uint32_t *data, size_t dataSize, size_t *returnedDataSize);

uint32_t *gpio_Unmarshall_Simplified(CustomMemTransceiver transceiver, uint32_t *data, size_t dataSize,
                                     size_t *returnedDataSize);

int gpio_Marshall_Unmarshall_Validity_Check(int nbDataPins);

// TODO make it local
volatile unsigned int *g_CLOCK_ADDRESS;
volatile void *g_MEMORY_MAP;

int main(int argc, char *argv[]) {

//    CustomMemTransceiver transceiver;
//    transceiver.nb_Data_Pins = 6;
//    size_t returnedDataSize = 0;
//    int data[12] = {0b111111, 0b111111, 0b111111, 0b111111, 0b111111, 0b111111, 0b111111, 0b111111, 0, 0, 0b111100, 12};
//
//    uint32_t *returnedData = gpio_Unmarshall(transceiver, data, 12, &returnedDataSize);
//
//
//    printf("Returned data size : %x\r\n", returnedDataSize);
//    printf("First value : %x\r\n", returnedData[0]);
//    printf("Second value : %x\r\n", returnedData[1]);
//    int notMatchingPinsNumber[32] = {0};
//    int j = 0;
//    for (int i = 1; i < 33; ++i) {
//        if (gpio_Marshall_Unmarshall_Validity_Check(i) < 0) {
//            notMatchingPinsNumber[j] = i;
//            j++;
//        }
//    }
//    printf("\nNot matching pin number :\n");
//    for (int k = 0; k < j; ++k) {
//        printf("%d,", notMatchingPinsNumber[k]);
//    }
//    gpio_Marshall_Unmarshall_Validity_Check(6);
    for (int i = 1; i < 33; ++i) {
        gpio_Marshall_Unmarshall_Validity_Check(i);

    }

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
    // usleep(1);
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
    if (transceiver.tc_Port > 0)
        gpio_Mem_Set_Data_Direction(transceiver.tc_Port, GPIO_OUT);
    if (transceiver.rc_Port > 0)
        gpio_Mem_Set_Data_Direction(transceiver.rc_Port, GPIO_IN);
    return 0;
}

/**
 * Set the direction for a range of port
 * @param firstPort Starting point for the setting of direction
 * @param lastPort Ending point for the setting of direction
 * @param direction GPIO_IN/GPIO_OUT
 * @return 0 in case of success
 */
int gpio_Mem_Set_Data_Direction_Range(int firstPort, int lastPort, int direction) {
    int index;
    for (index = 0; index < ((lastPort - firstPort) + 1); index++) {
        gpio_Mem_Set_Data_Direction(firstPort + index, direction);
    }
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
 * @return 0 in case of success, -1 in case of failure
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
 * Write data on an entire bank
 *
 * @param bank Bank to write data on
 * @param value Value used to generated output
 * @return 0 in case of success
 */
int gpio_Mem_Write_Direct_Data(int bank, __uint32_t value) {
    volatile unsigned int *setValueAddr;
    setValueAddr = g_MEMORY_MAP + GPIO_OFFSET_OUTPUT_BANK(bank);
    // Writing data
    *setValueAddr = value;
    return 0;
}

/**
 * Start the communication with another device
 * Start by setting Tc to high
 * First data send is the number of data that will follow
 * Complementing Tc state and awaiting response from receiver to continue
 * Finally reset the state of Tc to '0'
 *
 * @param transceiver Data structure containing all ports informations
 * @param writableDataArray Formatted data array, must be ready to write on the port accordingly to the current transceiver
 * @param writableDataArraySize Size of the data array
 * @return 0 in case of success, -1 in case of timeout
 */
int gpio_Mem_Chip_To_Chip_Writer(CustomMemTransceiver transceiver, int *writableDataArray, int writableDataArraySize) {
    int timeOutFlagValue = 0;
    int *timeOutPointer = &timeOutFlagValue;
    int tcState = TC_HIGH;
    // Start the communication request by setting Tc pin to high
    gpio_Mem_Transceiver_Send_Data(transceiver, 0, TC_HIGH);
    // Wait for the acknowledgement response
    gpio_Mem_Wait_For_Value(transceiver, RC_HIGH, timeOutPointer);
    if (*timeOutPointer != 0) {
        perror("Transmission request timed out, no response from receiver");
        return -1;
    }
    gpio_Mem_Transceiver_Send_Data(transceiver,
                                   (__uint32_t) gpio_Mem_Shift_Data(transceiver, (uint32_t) writableDataArraySize),
                                   TC_LOW);
    gpio_Mem_Wait_For_Value(transceiver, RC_LOW, timeOutPointer);
    if (*timeOutPointer != 0) {
        perror("Transmission of the size timed out, no response from receiver");
        return -1;
    }
    int index;
    for (index = 0; index < writableDataArraySize; ++index) {
        gpio_Mem_Transceiver_Send_Data(transceiver, (__uint32_t) writableDataArray[index], tcState);
        // Waiting for response from the receiver, Rc must in same state as Tc
        gpio_Mem_Wait_For_Value(transceiver, tcState, timeOutPointer);
        if (*timeOutPointer != 0) {
            perror("Transmission timed out, no response from receiver");
            return -1;
        }
        tcState = !tcState;
    }
    // Reset state of Tc
    gpio_Mem_Transceiver_Send_Data(transceiver, 0, TC_LOW);
    return 0;
}

/**
 * Reader for device to device communication
 * This function must be started after a communication request as been read (typically a Rc High state)
 * Start by a acknowledgement of the communication request
 * Then wait for the number of data the emitter will transmit
 * Allocate memory in consequence
 * Do not forget to this memory later
 *
 * @param transceiver Data structure containing all ports informations
 * @param resultArraySize Size of the array received from the emitter
 * @return The received data array received from the emitter
 */
__uint32_t *gpio_Mem_Chip_To_Chip_Reader(CustomMemTransceiver transceiver, __uint32_t *resultArraySize) {
    int timeOutFlagValue = 0;
    int *timeOutPointer = &timeOutFlagValue;
    int tcState = TC_HIGH;

    // Acknowledgement of the transmission request
    gpio_Mem_Transceiver_Send_Data(transceiver, 0, TC_HIGH);

    // While waiting for response generate the mask
    __uint32_t mask = gpio_Reading_Mask_From_Transceiver(transceiver);

    __uint32_t valueRead = gpio_Mem_Wait_For_Value(transceiver, RC_LOW, timeOutPointer);
    if (*timeOutPointer != 0) {
        perror("Transmission validation timed out, no response from emitter");
        return NULL;
    }
    // Obtain the number of time the emitter will transmit data
    *resultArraySize = gpio_Mem_Unshift_Data(transceiver, (valueRead & mask));

    // Instantiation of the array
    __uint32_t *readArray = malloc(*resultArraySize * sizeof(int));

    // Confirmation of ready to send state
    gpio_Mem_Transceiver_Send_Data(transceiver, 0, TC_LOW);
    int index = 0;
    for (index = 0; index < *resultArraySize; ++index) {
        valueRead = gpio_Mem_Wait_For_Value(transceiver, tcState, timeOutPointer);
        if (*timeOutPointer != 0) {
            perror("Transmission reception timed out, no response from emitter");
            return NULL;
        }
        gpio_Mem_Transceiver_Send_Data(transceiver, 0, tcState);
        // Partial unmarshalling of the data
        readArray[index] = gpio_Mem_Unshift_Data(transceiver, (valueRead & mask));
        tcState = !tcState;
    }
    // TODO check waiting function in writer to reduce this time
    usleep(10000);
    // Reset the state at the end
    gpio_Mem_Transceiver_Send_Data(transceiver, 0, TC_LOW);
    return readArray;
}

/**
 * Write a buffer, 32 bits by 32 bits
 *
 * @param bank Bank to write data on
 * @param dataArray Array of writable values
 * @param dataSize Size of the array
 * @return 0 in case of success
 */
int gpio_Mem_Write_Buffer(int bank, int *dataArray, size_t dataSize) {
    size_t index;
    for (index = 0; index < dataSize; index++) {
        gpio_Mem_Write_Direct_Data(bank, (__uint32_t) dataArray[index]);
    }
    return 0;
}

/**
 * Read the entirety of a bank RO register
 * @param bank Bank to read from
 * @return value read
 */
unsigned int gpio_Mem_Read_Bank(int bank) {
    volatile unsigned int *readValueAddr;
    readValueAddr = g_MEMORY_MAP + GPIO_DATA_RO(bank);
    return (*readValueAddr);
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
gpio_Mem_Formatting_Binary_Data(CustomMemTransceiver transceiver, const int *dataArray, int dataSize,
                                int *resultArraySize) {
    int requiredRows;
    if (dataSize % transceiver.nb_Data_Pins == 0) {
        requiredRows = dataSize / transceiver.nb_Data_Pins;
    }
        // If sizes don't match, add a cell and fill the void with '0' latter
    else {
        requiredRows = (dataSize / transceiver.nb_Data_Pins) + 1;

    }
    // TODO handle the free of this array later
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

/**
 * Shift data array to adapt to given transceiver
 * Warning does not check for overflow of data, may result in data loss
 * Use this function carefully
 * Do not forget to free the resulted array later
 *
 * @param transceiver Structure containing all the port information
 * @param dataArray Array of unsigned integer value to b converted
 * @param dataSize Size of the data array
 * @return Pointer to the writable data array
 */
int *gpio_Mem_Formatting_Integer_Data(CustomMemTransceiver transceiver, const uint32_t *dataArray, int dataSize) {
    int *usableDataArray = malloc(dataSize * sizeof(int));
    int index;
    for (index = 0; index < dataSize; ++index) {
        usableDataArray[index] = gpio_Mem_Shift_Data(transceiver, dataArray[index]);
    }
    return usableDataArray;
}

void print_Formatted_Data(__uint32_t *dataArray, int dataSize) {
    int index;
    for (index = 0; index < dataSize; index++) {
        printf("%x\r\n", dataArray[index]);
    }
}

/**
 * Return the bank from witch the given port belong
 * @param port Port to evaluate
 * @return Bank number
 */
int gpio_Bank_From_port(int port) {
    if (port < PIN_NUMBER_PER_BANK)
        return 2;
    else
        return 3;
}

/**
 * Since reading a bank return a 32 bit, it is mandatory to mask it to obtain the researched data
 * Mask read data as follow : readData &= mask
 *
 * @param transceiver Transceiver containing the port to mask
 * @return  A 32 bit mask
 */
__uint32_t gpio_Reading_Mask_From_Transceiver(CustomMemTransceiver transceiver) {
    int index;
    __uint32_t mask = 0;
    for (index = 0; index < transceiver.nb_Data_Pins; index++) {
        if (transceiver.bank == 2)
            mask |= (1u << transceiver.pins_Ports[index]);
        else
            mask |= (1u << (transceiver.pins_Ports[index] - PIN_NUMBER_PER_BANK));
    }
    return mask;
}


/**
 * Shift the data to adapt to the transceiver
 * Data size and transceiver offset are not checked
 * Warning risk of overflowing and data loss
 *
 * @param transceiver
 * @param data Data to convert in a writable way
 * @return The converted data
 */
int gpio_Mem_Shift_Data(CustomMemTransceiver transceiver, uint32_t data) {

    if (gpio_Bank_From_port(transceiver.pins_Ports[transceiver.nb_Data_Pins - 1]) == 2) {
        data = data << transceiver.pins_Ports[transceiver.nb_Data_Pins - 1];
    } else {
        data = data << (transceiver.pins_Ports[transceiver.nb_Data_Pins - 1] - PIN_NUMBER_PER_BANK);
    }
    return data;
}


/**
 * Used to shift data read accordingly to port offset
 * Warning : use this functionly only if transmission ports are consecutive
 *
 * @param transceiver
 * @param data Data to shift
 * @return Shifted data
 */
__uint32_t gpio_Mem_Unshift_Data(CustomMemTransceiver transceiver, __uint32_t data) {
    if (gpio_Bank_From_port(transceiver.pins_Ports[transceiver.nb_Data_Pins - 1]) == 2) {
        data = data >> transceiver.pins_Ports[transceiver.nb_Data_Pins - 1];
    } else {
        data = data >> (transceiver.pins_Ports[transceiver.nb_Data_Pins - 1] - PIN_NUMBER_PER_BANK);
    }
    return data;

}


/**
 * Wait for a particular value on the Rc chanel of a transceiver
 * If the value is not directly read, the waiting become exponential
 * Sleep is used to not hog resources
 *
 * @param transceiver Transceiver responsible for the communication
 * @param value Value to wait for
 * @param timeOutFlag If this function hit timeOut will be put to 1
 * @return return the value read on the transceiver RO register, if timeOut 0 is returned do not use this as timeOut check, use the flag pointer
 */
uint32_t gpio_Mem_Wait_For_Value(CustomMemTransceiver transceiver, int value, int *timeOutFlag) {
    uint32_t readValue = gpio_Mem_Read_Bank(transceiver.bank);
    uint32_t maskedValue;
    if (transceiver.bank == 2)
        maskedValue = readValue & (1u << transceiver.rc_Port);
    else
        maskedValue = readValue & (1u << (transceiver.rc_Port - PIN_NUMBER_PER_BANK));

    maskedValue = (uint32_t) (maskedValue != 0);
    if (maskedValue == value)
        return readValue;
    else {
        __useconds_t delay = 1;
        int timeOutTimer = 0;
        while (timeOutTimer < TIMEOUT_MAX_COUNT) {
            usleep(delay);
            readValue = gpio_Mem_Read_Bank(transceiver.bank);
            if (transceiver.bank == 2)
                maskedValue = readValue & (1u << transceiver.rc_Port);
            else
                maskedValue = readValue & (1u << (transceiver.rc_Port - PIN_NUMBER_PER_BANK));

            maskedValue = (uint32_t) (maskedValue != 0);
            if (maskedValue == value)
                return readValue;
            else {
                if (delay < DELAY_MAX) {
                    delay *= 10;
                } else {
                    timeOutTimer++;
                }
            }
        }
        *timeOutFlag = 1;
        return 0;
    }
}

/**
 * Will add to the data, the Tc state allowing serial reading
 * If tc is set to -1, it will be ignored ( 1 << -1 = 0)
 *
 * @param transceiver Transceiver responsible for handling communication
 * @param value "Writable" data
 * @param tcState Current state of the Tc port
 * @return The result of gpio_Mem_Write_Direct_Data
 */
int gpio_Mem_Transceiver_Send_Data(CustomMemTransceiver transceiver, __uint32_t value, int tcState) {
    if (transceiver.bank == 2) {
        if (tcState == 0) {
            value &= ~(1u << transceiver.tc_Port);
        } else {
            value |= (1u << transceiver.tc_Port);
        }
    } else {
        if (tcState == 0) {
            value &= ~(1u << (transceiver.tc_Port - PIN_NUMBER_PER_BANK));
        } else {
            value |= (1u << (transceiver.tc_Port - PIN_NUMBER_PER_BANK));
        }
    }

    return gpio_Mem_Write_Direct_Data(transceiver.bank, value);

}

/**
 * Reset the value on the entire bank, disable the clock and close the memory mapping
 * Note disabling gpio clock prevent other process to use gpio
 *
 * @param fd File descriptor of "/dev/mem"
 * @param bank Bank to reset
 */
void gpio_Mem_CleanUp(int fd, int bank) {
    gpio_Mem_Write_Direct_Data(bank, 0u);
    usleep(10);
//    disable_gpio_clock();
    usleep(10);
    gpio_Mem_Unmap(fd);

}

void gpio_mem_speed_test(int nbTest, int internalRepetition, int delay) {

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
            gpio_Mem_Write(port, value);
            value = 1;
            for (int i = 0; i < delay; ++i) {
                // nothing
            }
            gpio_Mem_Write(port, value);

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

    printf("Average time for internal function measured is : %lf us\r\n", averageTimeElapsed);

    double period = averageTimeElapsed / internalRepetition;
    printf("Average period is : %lf us\r\n", period);

    double frequency = ((double) 1.00 / period) * 1000000;
    printf("Average frequency is : %lf Hz\r\n", frequency);
}

void gpio_mem_multiple_speed_test(int nbTest, int internalRepetition, int delay) {


    int data1[8] = {1, 0, 1, 0, 1, 0, 1, 0};
    int data2[8] = {0, 1, 0, 1, 0, 1, 0, 1};

    CustomMemTransceiver transceiver;
    int pinPorts[8] = {31, 30, 29, 28, 27, 26, 25, 24};
    transceiver.pins_Ports = pinPorts;
    transceiver.nb_Data_Pins = 8;
    transceiver.tc_Port = -1;
    int *returnedDataSize = malloc(sizeof(int));
    *returnedDataSize = 0;
    int *dataToPrint = gpio_Mem_Formatting_Binary_Data(transceiver, data1, 8, returnedDataSize);
    int *dataToPrint2 = gpio_Mem_Formatting_Binary_Data(transceiver, data2, 8, returnedDataSize);

    printf("Returned size : %d\r\n", *returnedDataSize);
    print_Formatted_Data(dataToPrint, *returnedDataSize);
    print_Formatted_Data(dataToPrint2, *returnedDataSize);


    int fd = gpio_Mem_Map();
    usleep(10);
    open_Amba_clk();
    usleep(10);
    enable_gpio_clock();
    usleep(10);
    gpio_Mem_Set_Transceiver_Direction(transceiver, GPIO_OUT);
    usleep(10);

    int index = 0;
    struct timeval tv, tv2;
    long testResult[nbTest];
    for (int j = 0; j < nbTest; j++) {
        gettimeofday(&tv, NULL);
        for (index = 0; index < internalRepetition; index++) {
            // Insert function to test here
            gpio_Mem_Write_Direct_Data(2, (__uint32_t) dataToPrint[0]);

            for (int i = 0; i < delay; ++i) {
                // nothing
            }
            gpio_Mem_Write_Direct_Data(2, (__uint32_t) dataToPrint2[0]);

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

    printf("Average time for internal function measured is : %lf us\r\n", averageTimeElapsed);

    double period = averageTimeElapsed / internalRepetition;
    printf("Average period is : %lf us\r\n", period);

    double frequency = ((double) 1.00 / period) * 1000000;
    printf("Average frequency is : %lf Hz\r\n", frequency);
}

void gpio_Mem_Protocol_Reader() {

    CustomMemTransceiver transceiver;
    int pinPort[6] = {39, 38, 37, 36, 35, 34};
    transceiver.pins_Ports = pinPort;
    transceiver.nb_Data_Pins = 6;
    transceiver.tc_Port = 33;
    transceiver.rc_Port = 32;
    transceiver.bank = 3;

    __uint32_t mask = gpio_Reading_Mask_From_Transceiver(transceiver);
    printf("mask : %x\n", mask);

    int fd = gpio_Mem_Map();
    open_Amba_clk();
    usleep(100);
    enable_gpio_clock();
    gpio_Mem_Set_Transceiver_Direction(transceiver, GPIO_IN);
    usleep(100);

    printf("reset\n");
    gpio_Mem_Transceiver_Send_Data(transceiver, 0, 0);
    usleep(100);
    int flag = 0;
    __uint32_t resultSize = 0;
    __uint32_t *resultArray;
    struct timeval tv, tv2;
    printf("Waiting for HIGH on port %d\n", transceiver.rc_Port);
    gpio_Mem_Wait_For_Value(transceiver, RC_HIGH, &flag);
    // Start of time measuring
    gettimeofday(&tv, NULL);
    if (flag != 0) {
        perror("No start from emitter");
        gpio_Mem_CleanUp(fd, transceiver.bank);
        return;
    }
    printf("Starting to listen\n");
    resultArray = gpio_Mem_Chip_To_Chip_Reader(transceiver, &resultSize);
    if (resultArray == NULL) {
        perror("Result array is null");
        free(resultArray);
        gpio_Mem_CleanUp(fd, transceiver.bank);
        return;
    }
    gettimeofday(&tv2, NULL);
    double testResult = (double) (tv2.tv_sec * 1000000 + tv2.tv_usec) - (double) (tv.tv_sec * 1000000 + tv.tv_usec);
    printf("Time for reception measured is : %lf us\r\n", testResult);

    double period = testResult / 63;
    printf("Average period is : %lf us\r\n", period);

    double frequency = ((double) 1.00 / period) * 1000000;
    printf("Average frequency is : %lf Hz\r\n", frequency);
    print_Formatted_Data(resultArray, resultSize);
    free(resultArray);
    gpio_Mem_CleanUp(fd, transceiver.bank);
}


void gpio_Mem_Protocol_Writer() {
    CustomMemTransceiver transceiver;
    int pinPort[6] = {31, 30, 29, 28, 27, 26};
    transceiver.pins_Ports = pinPort;
    transceiver.nb_Data_Pins = 6;
    transceiver.tc_Port = 24;
    transceiver.rc_Port = 25;
    transceiver.bank = 2;

//    __uint32_t data[5] = {2u, 4u, 5u, 7u, 8u};
//    int *printableData = gpio_Mem_Formatting_Integer_Data(transceiver, data, 5);

    __uint32_t data[63] = {0};
    for (int i = 0; i < 63; ++i) {
        data[i] = (__uint32_t) i;
    }
    int *printableData = gpio_Mem_Formatting_Integer_Data(transceiver, data, 63);
    int fd = gpio_Mem_Map();
    open_Amba_clk();
    usleep(100);
    enable_gpio_clock();
    gpio_Mem_Set_Transceiver_Direction(transceiver, GPIO_OUT);
    usleep(100);
    gpio_Mem_Transceiver_Send_Data(transceiver, 0, TC_LOW);
    printf("Reset\r\n");
    if (gpio_Mem_Chip_To_Chip_Writer(transceiver, printableData, 63) < 0) {
        perror("Writing function failed");
    }
    gpio_Mem_Transceiver_Send_Data(transceiver, 0, TC_LOW);
    free(printableData);
    gpio_Mem_CleanUp(fd, transceiver.bank);

}

/**
 * DEPRECATED
 *
 * Convert received data to 32 bits values according to the number of data pins in use
 * Will read all the data array, adding values found to current 32 bits number
 * Warning Only working for transceiver data pins number divider of 32
 *
 * @param transceiver Transceiver containing all the port informations
 * @param data Data received from another board using a trasnceiver structure
 * @param dataSize Size of the data array
 * @param returnedDataSize Size of the returned unmarshalled array
 * @return Unmarshalled 32 bits value array
 */
uint32_t *gpio_Unmarshall(CustomMemTransceiver transceiver, int *data, size_t dataSize, size_t *returnedDataSize) {
//    printf("\n\n\t\t**UNMARSHALLING **\n\n");

    int numberOfReadingPerWord;
    numberOfReadingPerWord = 32u / transceiver.nb_Data_Pins;
//    printf("Number Of reading per word : %d\n", numberOfReadingPerWord);

    size_t numberOfDataBits = dataSize * transceiver.nb_Data_Pins;
//    printf("Number of data bits : %d\n", numberOfDataBits);
    // If total number of bit is not divisible by 32
//    if (32u % numberOfDataBits) {
//        *returnedDataSize = (numberOfDataBits / 32u) + 1;
//    } else {
    // Discard remaining data that do not form a word
    *returnedDataSize = (numberOfDataBits / 32u);
//    printf("Returned data size : %d\n", *returnedDataSize);
//    }

    // Array memory allocation
    uint32_t *unmarshalledData = malloc(*returnedDataSize * sizeof(uint32_t));
    int lastingBits = 32;
    uint32_t mainValue = 0;
    uint32_t truncatedValue = 0;
    int overflowFlag = 0;
    int difference;
    int resultArrayIndex = 0;
    int invertedReadingIndex = numberOfReadingPerWord - 1;
//    printf("Inverted reading index : %d\n", invertedReadingIndex);
    // Number of bits that will overflow
    int offset = 32u % transceiver.nb_Data_Pins;
    for (int i = 0; i < dataSize; ++i) {
        difference = transceiver.nb_Data_Pins - lastingBits;
        // Overflow of data
        // 2 truncated words sharing same portion (array space)
//        printf("Data[%d] : 0x%x\n", i, data[i]);

        if (lastingBits < transceiver.nb_Data_Pins) {
            overflowFlag = 1;
            mainValue += data[i] >> (difference);
            // Masking to keep only the least significant part of the word
            // mask = pow(2, difference) - 1;
            truncatedValue = data[i] & (0xffffffffu >> lastingBits);
            lastingBits = 32 - difference;
            // Updating number of bits that will overflow
            offset = lastingBits % transceiver.nb_Data_Pins;
            invertedReadingIndex = numberOfReadingPerWord - 1;

            // The word is complete
            unmarshalledData[resultArrayIndex] = mainValue;
            mainValue = 0;
            resultArrayIndex++;
        }
            // Only one part of a word on this portion
        else {
            // Handle the truncated starting part of a word
            if (overflowFlag) {
                // TODO HANDLE SHIFTING FOR TRUNCATED VALUE
                mainValue += truncatedValue << ((invertedReadingIndex * transceiver.nb_Data_Pins) + offset);
                truncatedValue = 0;
                overflowFlag = 0;
                invertedReadingIndex--;
            }
            mainValue += ((uint32_t) data[i]) << ((invertedReadingIndex * transceiver.nb_Data_Pins) + offset);
            invertedReadingIndex--;

            lastingBits -= transceiver.nb_Data_Pins;
            // The word is complete
            if (lastingBits == 0) {
                unmarshalledData[resultArrayIndex] = mainValue;
                resultArrayIndex++;
                invertedReadingIndex = numberOfReadingPerWord - 1;
                lastingBits = 32u;
                mainValue = 0;
            }
        }
    }
    return unmarshalledData;
}

/**
 * DEPRECATED
 *
 * WARNING  : only functioning for number of data pins divider of 32
 * @param transceiver Transceiver containing all the port informations
 * @param data
 * @param dataSize
 * @param returnedDataSize
 * @return
 */
uint32_t *gpio_Marshall(CustomMemTransceiver transceiver, uint32_t *data, size_t dataSize, size_t *returnedDataSize) {

    int numberOfWritingPerWord;
    numberOfWritingPerWord = 32u / transceiver.nb_Data_Pins;
    // We need to create one more array space in case of overflow
    if ((32u * dataSize) % transceiver.nb_Data_Pins) {
//        *returnedDataSize = dataSize * numberOfWritingPerWord + 1;
        // TODO check if necessary
        *returnedDataSize = (dataSize * 32) / transceiver.nb_Data_Pins + 1;
    } else {
//        *returnedDataSize = dataSize * numberOfWritingPerWord;
        *returnedDataSize = (dataSize * 32) / transceiver.nb_Data_Pins;
    }

    // Array memory allocation
    uint32_t *marshalledData = malloc(*returnedDataSize * sizeof(uint32_t));
    int remainingBits = 32;
    // Keep only the size of a sending
    uint32_t mask = 0xffffffff >> (32 - transceiver.nb_Data_Pins);
    uint32_t valueToStore = 0;
    uint32_t overflowedValue = 0;
    int numberSharedBitsStrong = 0;
    int precedingOffset = 0;
    int overflowFlag = 0;
    int generalIndex = 0;

    for (int index = 0; index < dataSize; ++index) {
        // Decrementing loop, storing chunk by chunk in a one dimensional array
//        numberSharedBitsStrong = (32 - numberSharedBitsStrong) % transceiver.nb_Data_Pins;
        numberSharedBitsStrong = remainingBits % transceiver.nb_Data_Pins;
        // If the precedent value overflowed we need to write one more for the next value
        for (int leftToWrite = numberOfWritingPerWord - 1 + (overflowFlag != 0), internalIndex = 0;
             leftToWrite >= 0; --leftToWrite, ++internalIndex, ++generalIndex) {
            valueToStore = data[index] >> (leftToWrite * transceiver.nb_Data_Pins + numberSharedBitsStrong);
            // Index shifting by the index of the word that is actually marshalled, time the number of writting necessary for a word
            if (overflowFlag) {
//                remainingBits -= (transceiver.nb_Data_Pins - precedingOffset);
                valueToStore += overflowedValue;
                overflowFlag = 0;
            } else {
                remainingBits -= transceiver.nb_Data_Pins;
            }
//            marshalledData[(index * numberOfWritingPerWord) + internalIndex] = valueToStore & mask;
            marshalledData[generalIndex] = valueToStore & mask;

//            printf("Creating index : %d\t Remaining : %d\n", (index * numberOfWritingPerWord) + internalIndex,
//                   remainingBits);
            printf("Creating index : %d\t Remaining : %d\n", generalIndex,
                   remainingBits);
        }
        // If there is an overflow
        if (remainingBits) {
            // Mask the data to keep only the data that overflowed and will share next array space
            if (index < dataSize) {
                overflowedValue = data[index] & (0xffffffff >> (32 - remainingBits));
            }
            // Shift the value that overflow to fit in the next array space
            overflowedValue = overflowedValue << (transceiver.nb_Data_Pins - numberSharedBitsStrong);
            remainingBits = 32 - (transceiver.nb_Data_Pins - numberSharedBitsStrong);
            overflowFlag = 1;
        } else {
            remainingBits = 32;
        }
        precedingOffset = numberSharedBitsStrong;
    }
    // Handle the last overflow
    if (overflowFlag) {
        marshalledData[*returnedDataSize - 1] = overflowedValue & mask;
    }
    return marshalledData;
}

/**
 * Divide 32 bits word in smaller sub words of the size of the transceiver data pin number
 * Handle correctly transceiver with data pin number not divider of 32
 * Will always have the same number of sub words for a 32 bits word
 *
 * @param transceiver Transceiver containing all the port informations
 * @param data 32 bits word data array to convert
 * @param dataSize Size of the data array
 * @param returnedDataSize Size of the returned data array
 * @return Array of sub word
 */
uint32_t *
gpio_Marshall_Simplified(CustomMemTransceiver transceiver, uint32_t *data, size_t dataSize, size_t *returnedDataSize) {
    // Number of writing necessary for a 32 bits word
    int numberOfWritingPerWord = (32u / transceiver.nb_Data_Pins) + ((32u % transceiver.nb_Data_Pins) != 0);
    int numberOfOverflowedBits = 32u % transceiver.nb_Data_Pins;
    uint32_t mask = 0xffffffff >> (32 - transceiver.nb_Data_Pins);
    *returnedDataSize = numberOfWritingPerWord * dataSize;
    uint32_t *marshalledData = malloc(*returnedDataSize * sizeof(uint32_t));
    uint32_t valueToStore = 0;

    for (int wordIndex = 0, resultArrayIndex = 0; wordIndex < dataSize; ++wordIndex) {
        for (int splitIndex = numberOfWritingPerWord - 1, multiplierCoefficient = 1;
             splitIndex >= 0; --splitIndex, ++resultArrayIndex, ++multiplierCoefficient) {
            // Case of last part of a word with overflowed bits
            if (splitIndex == 0 && numberOfOverflowedBits != 0) {
                valueToStore = data[wordIndex] << (transceiver.nb_Data_Pins - numberOfOverflowedBits);
            } else {
                valueToStore = data[wordIndex] >> (32 - (transceiver.nb_Data_Pins * multiplierCoefficient));
            }

            marshalledData[resultArrayIndex] = valueToStore & mask;
        }
    }

    return marshalledData;
}

/**
 * Unmarshall sub words received into 32 bits word
 * Will shift sub words accordingly and add them to the value to store
 * Use this only with the data marshalled in gpio_Marshall_Simplified
 *
 * @param transceiver Transceiver containing all the port informations
 * @param data Sub word data array to unmarshall
 * @param dataSize Size of the sub word array
 * @param returnedDataSize Size of the returned data array
 * @return 32 bits word data array
 */
uint32_t *gpio_Unmarshall_Simplified(CustomMemTransceiver transceiver, uint32_t *data, size_t dataSize,
                                     size_t *returnedDataSize) {
    int numberOfWritingPerWord = (32u / transceiver.nb_Data_Pins) + ((32u % transceiver.nb_Data_Pins) != 0);
    int numberOfOverflowedBits = 32u % transceiver.nb_Data_Pins;
    if (dataSize % numberOfWritingPerWord) {
        perror("Unmarshalling failed : mismatch in data size and number of writing per word");
        return NULL;
    }
    *returnedDataSize = dataSize / numberOfWritingPerWord;
    uint32_t *unmarshalledData = malloc(*returnedDataSize * sizeof(uint32_t));
    int arrayIndex = 0;
    for (int entireWordIndex = 0; entireWordIndex < *returnedDataSize; ++entireWordIndex) {
        unmarshalledData[entireWordIndex] = 0;
        for (int splitedWordIndex = 0, multiplierCoefficient = 1;
             splitedWordIndex < numberOfWritingPerWord; ++splitedWordIndex, ++arrayIndex, ++multiplierCoefficient) {
            if (splitedWordIndex == numberOfWritingPerWord - 1 && numberOfOverflowedBits != 0) {
                unmarshalledData[entireWordIndex] +=
                        data[arrayIndex] >> (transceiver.nb_Data_Pins - numberOfOverflowedBits);
            } else {
                unmarshalledData[entireWordIndex] +=
                        data[arrayIndex] << (32 - (transceiver.nb_Data_Pins * multiplierCoefficient));
            }
        }
    }
    return unmarshalledData;
}


/**
 * Test the marshalling and unmarshalling function automatically
 * Compare observed result to expected values to check data validity
 *
 * @param nbDataPins Number of data pins used of transmission
 * @return
 */
int gpio_Marshall_Unmarshall_Validity_Check(int nbDataPins) {
    int returnedFlag = 0;
    int faultSum = 0;
    uint32_t dataToMarshall[5] = {7, 0xffffffff, 0x0, 0x45, 0x78};
    CustomMemTransceiver transceiver2;
    transceiver2.nb_Data_Pins = nbDataPins;
    size_t returnedDataSize2 = 0;
//    uint32_t *marshalledData = gpio_Marshall(transceiver2, dataToMarshall, 3, &returnedDataSize2);
    uint32_t *marshalledData = gpio_Marshall_Simplified(transceiver2, dataToMarshall, 5, &returnedDataSize2);

    size_t finalSize;

    // TODO change unmarshall function and simplify it
    uint32_t *finalResult = gpio_Unmarshall_Simplified(transceiver2, marshalledData, returnedDataSize2, &finalSize);
    printf("\n\n**Pins in usage : %d\n", nbDataPins);
    for (int j = 0; j < finalSize; ++j) {
        if (dataToMarshall[j] != finalResult[j]) {
            printf("Returned unmarshalled result does not match, %d Data pins:\nExpected : %x\tObserved : %x\n",
                   nbDataPins, dataToMarshall[j], finalResult[j]);
            faultSum++;
            returnedFlag = -1;
        }
    }
    if (returnedFlag < 0) {
        printf("Returned marshalled data size : %d\n", returnedDataSize2);
        printf("Bad marshalling/unmarshalling : %d/%d\n", faultSum, 5);
        for (int i = 0; i < returnedDataSize2; ++i) {
            printf("Value %d \t: %x \n", i, marshalledData[i]);
        }
    }
    if(returnedFlag == 0){
        printf("Succes\n");
    }
//    for (int k = 0; k < finalSize; ++k) {
//        printf("Value unmarshalled : %x \n", finalResult[k]);
//    }

    free(marshalledData);
    free(finalResult);
    return returnedFlag;
}

