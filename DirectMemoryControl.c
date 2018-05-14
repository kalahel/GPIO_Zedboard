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


// TODO make it local
volatile unsigned int *g_CLOCK_ADDRESS;
volatile void *g_MEMORY_MAP;

int main(int argc, char *argv[]) {

//    gpio_Mem_Protocol_Reader();
//    gpio_Mem_Protocol_Writer();

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
 * Do not forget to free the result array later
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