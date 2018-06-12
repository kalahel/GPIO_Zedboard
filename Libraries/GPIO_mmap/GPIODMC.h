//
// Created by Mathieu Hannoun on 11/06/18.
// GPIODMC stand for General Purpose Input/Output Direct memory control
// It is a library used for the GPIO control on the ARM processor embedded on a Zedboard
//

#ifndef GPIO_ZEDBOARD_GPIODMC_H

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



#define GPIO_ZEDBOARD_GPIODMC_H
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
 * For now, pins tr, rc and data ports must belong to the same bank
 * Port array must be in most significant bit to least significant bit order
 * Set tc and rc to negative value to disable them
 * pins_Ports is the array of data pins used for data transmission
 */
typedef struct {
    int nb_Data_Pins;
    int *pins_Ports;
    int tc_Port;
    int rc_Port;
    int bank;
} CustomMemTransceiver;


/**
 * Set a given port to be an input or an output, enable or disable its output accordingly
 *
 * @param port The pin to set, only one pin at a time
 * @param direction GPIO_IN or GPIO_OUT, no internal check of value
 * Warning : Passing others value will result in unpredictable behaviors
 * @return 0 in case of success
 */
int gpio_Mem_Set_Data_Direction(int port, int direction);

/**
 * Set the direction for all the ports of a transceiver and tc, rc port if theirs values are positive
 *
 * @param transceiver Transceiver containing all the pins informations
 * @param direction GPIO_IN/GPIO_OUT, set all the data pins to the same direction
 * @return 0 in case of success
 */
int gpio_Mem_Set_Transceiver_Direction(CustomMemTransceiver transceiver, int direction);

/**
 * DEPRECATED
 * Set the direction for a range of port on the same bank
 *
 * @param firstPort Starting point for the setting of direction
 * @param lastPort Ending point for the setting of direction
 * @param direction GPIO_IN/GPIO_OUT
 * @return 0 in case of success
 */
int gpio_Mem_Set_Data_Direction_Range(int firstPort, int lastPort, int direction);

/**
 * Open /dev/mem
 * Map the gpio driver memory space
 * Initialize the global variable linked to the memory mapping
 * Used before any operation to initialize space pointer g_MEMORY_MAP
 *
 * @return the file descriptor used for opening /dev/mem
 */
int gpio_Mem_Map();

/**
 * Unmap previously mapped memory space (g_MEMORY_MAP)
 * Close opened file descriptor
 *
 * @param fd file descriptor returned by gpio_Mem_Map
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Mem_Unmap(int fd);

/**
 * Write the chosen value to a gpio port
 *
 * @param port Gpio port as defined in the .xdc file
 * @param value Binary value to write, non zero values are treated as one
 * @return 0 in case of success
 */
int gpio_Mem_Write(int port, int value);

/**
 * DEPRECATED
 * Write value on pins one by one without a clk signal
 * The data arrays size must be equal the transceiver port number
 *
 * @param transceiver Transceiver containing all the pins informations
 * @param dataArray Array of binary data, non zero values are treated as one
 * @param dataSize Size of the data array
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Mem_Write_Transceiver_Slow(CustomMemTransceiver transceiver, int *dataArray, int dataSize);

/**
 * Write a value on an entire bank register
 * Effectively setting up the value for 32 pins
 *
 * @param bank Bank to write data on
 * @param value Value used to generated output
 * @return 0 in case of success
 */
int gpio_Mem_Write_Direct_Data(int bank, __uint32_t value);

/**
 * Start the communication with another device
 * Start by setting Tc to high
 * First data send is the number of data that will follow
 * Complementing Tc state and awaiting response from receiver to continue
 * Finally reset the state of Tc to '0'
 * Does not handle the unshifting of the data
 *
 * @param transceiver Data structure containing all ports informations
 * @param writableDataArray Formatted data array, must be ready to write on the port accordingly to the current transceiver.
 * Use gpio_Marshall_Simplified() then gpio_Mem_Formatting_Integer_Data() to make the data usable by this function
 * @param writableDataArraySize Size of the data array
 * @return 0 in case of success, -1 in case of timeout
 */
int gpio_Mem_Chip_To_Chip_Writer(CustomMemTransceiver transceiver, int *writableDataArray, int writableDataArraySize);

/**
 * Reader for device to device communication
 * This function must be started after a communication request as been read (typically a Rc High state)
 * Start by a acknowledgement of the communication request
 * Then wait for the number of data the emitter will transmit
 * Handle the unshifting of the data
 * Allocate memory in consequence
 * Do not forget to free this memory later
 *
 * @param transceiver Data structure containing all ports informations
 * @param resultArraySize The pointer passed will be set to the size of the array received from the emitter
 * @return The received data array received from the emitter
 */
__uint32_t *gpio_Mem_Chip_To_Chip_Reader(CustomMemTransceiver transceiver, __uint32_t *resultArraySize);

/**
 * Read the entirety of a bank RO register
 *
 * @param bank Bank to read from
 * @return value read
 */
unsigned int gpio_Mem_Read_Bank(int bank);

/**
 * Open the GPIO clock by using mmap(2)
 * Initialize the global variable linked to the clock register
 * Must be called before any GPIO operation
 *
 * @return 0 in case of success, mmap error value in case of failure
 * TODO must be changed to return used file descriptor
 * TODO may be changed to eliminate secondary mapping, using only g_MEMORY_MAP with offset
 */
int open_Amba_clk();

/**
 * Set the register value for the GPIO clock to '1'
 *
 * @return 0 in case of success
 */
int enable_gpio_clock();

/**
 * Set the register value for the GPIO clock to '0'
 * Preventing usage of GPIO for all the users on the board
 *
 * @return 0 in case of success
 */
int disable_gpio_clock();

/**
 * DEPRECATED
 * Will use printf to display information about current BANK memory address
 */
void print_Bank_Addr();

/**
 * Since reading a bank return a 32 bit, it is mandatory to mask it to obtain correct data
 * Mask is generated from data pins of the transceiver
 * Mask read data as follow : readData &= mask
 *
 * @param transceiver Transceiver containing the port to mask
 * @return  A 32 bit mask generated from and for the given transceiver
 */
__uint32_t gpio_Reading_Mask_From_Transceiver(CustomMemTransceiver transceiver);

/**
 * DEPRECATED
 * Convert an array of binary values to an array of "Writable" values for the actual transceiver
 * Handle the allocation of the newly created array
 *
 * Usage of this function is not recommended for performance issue
 * Use marshalling/shifting functions instead
 *
 * @param transceiver Transceiver containing all the information about used ports
 * @param dataArray Array of binary data, not '0' is treated as '1'
 * @param dataSize Size of the data Array
 * @param resultArraySize Size of the returned array
 * @return The newly formatted data array
 */
int *
gpio_Mem_Formatting_Binary_Data(CustomMemTransceiver transceiver, const int *dataArray, int dataSize,
                                int *resultArraySize);

/**
 * Shift data array to adapt to given transceiver
 * Use gpio_Mem_Shift_Data to generate output array
 * Warning does not check for overflow of data, may result in data loss
 * Use this function carefully
 * Do not forget to free the resulted array later
 *
 * @param transceiver Structure containing all the port information
 * @param dataArray Array of unsigned integer value to b converted
 * @param dataSize Size of the data array
 * @return Pointer to the writable data array
 */
int *gpio_Mem_Formatting_Integer_Data(CustomMemTransceiver transceiver, const uint32_t *dataArray, int dataSize);

/**
 * DEPRECATED
 * Use printf to print an array of 32 bits value
 *
 * @param dataArray Data to print
 * @param dataSize Size of the given array
 */
void print_Formatted_Data(__uint32_t *dataArray, int dataSize);

/**
 * Return the bank from witch the given port belong
 * @param port Port to evaluate
 * @return Bank number
 */
int gpio_Bank_From_port(int port);

/**
 * Shift the data to adapt to the transceiver
 * Data size and transceiver offset are not checked
 * Warning risk of overflowing and data loss
 *
 * @param transceiver Structure containing all the port information
 * @param data Data to convert in a writable way
 * @return The converted data
 */
int gpio_Mem_Shift_Data(CustomMemTransceiver transceiver, uint32_t data);

/**
 * Used to unshift data read accordingly to port offset
 * Warning : use this function only if transmission ports are consecutive
 *
 * @param transceiver
 * @param data Data to shift
 * @return Unshifted data
 */
__uint32_t gpio_Mem_Unshift_Data(CustomMemTransceiver transceiver, __uint32_t data);

/**
 * Wait for a particular value on the Rc chanel of a transceiver
 * If the value is not directly read, the waiting become exponential
 * Sleep(2) is used to not hog resources
 * TODO replace Sleep(2) by shorter active waiting
 *
 * @param transceiver Transceiver responsible for the communication
 * @param value Value to wait for, MUST BE '1' OR '0', non zero values are NOT treated as '1'
 * @param timeOutFlag If this function hit timeOut will be put to 1
 * @return return the value read on the transceiver RO register, if timeOut 0 is returned do not use this as timeOut check, use the flag pointer
 */
uint32_t gpio_Mem_Wait_For_Value(CustomMemTransceiver transceiver, int value, int *timeOutFlag);

/**
 * Write on the value register the data to send
 * Add to the data, the Tc state allowing serial reading
 * If tc is set to -1, it will be ignored ( 1 << -1 = 0)
 *
 * @param transceiver Transceiver responsible for handling communication
 * @param value "Writable" data. Data is considered "Writable" after passing through gpio_Marshall_Simplified() then gpio_Mem_Formatting_Integer_Data()
 * @param tcState Current state of the Tc port
 * @return The result of gpio_Mem_Write_Direct_Data
 */
int gpio_Mem_Transceiver_Send_Data(CustomMemTransceiver transceiver, __uint32_t value, int tcState);

/**
 * Reset the value on the entire bank and close the memory mapping
 * Note : disabling gpio clock prevent other process to use GPIO
 *
 * @param fd File descriptor of "/dev/mem"
 * @param bank Bank to reset
 */
void gpio_Mem_CleanUp(int fd, int bank);

/**
 * TEST FUNCTION
 * Generate a square signal for nbTest * internalRepetition period
 * Measure the internal frequency and period then display it with printf()
 *
 * @param nbTest Number of time the measure will be taken
 * @param internalRepetition Number of repetition between two measures. Warning value must be high enough to make measurement possible
 * @param delay Active delay wait, set it to 0 to obtain correct measurements
 */
void gpio_mem_speed_test(int nbTest, int internalRepetition, int delay);

/**
 * TEST FUNCTION
 * Work the same way as gpio_mem_speed_test() but this time using multiple pins
 * Pin used : 31, 30, 29, 28, 27, 26, 25, 24
 *
 * @param nbTest Number of time the measure will be taken
 * @param internalRepetition Number of repetition between two measures. Warning value must be high enough to make measurement possible
 * @param delay Active delay wait, set it to 0 to obtain correct measurements
 */
void gpio_mem_multiple_speed_test(int nbTest, int internalRepetition, int delay);

/**
 * TEST FUNCTION
 * Test function for the reading part of the protocol
 * Initialize data structures and open memory mapping
 * Wait for the writer and display received result
 */
void gpio_Mem_Protocol_Reader();

/**
 * TEST FUNCTION
 * Test function for the writing part of the protocol
 * Initialize data structures and open memory mapping
 * Send 63 value from 0 to 62
 */
void gpio_Mem_Protocol_Writer();

/**
 * Send an entire set of data using a transceiver and the current protocol
 * Start by marshalling then shifting correctly the data to have "writable" data
 * Set Rc to low and data to '0'
 * Start communication using gpio_Mem_Chip_To_Chip_Writer()
 * Set Rc to low and data to '0' once the communication is over
 *
 * @param transceiver Transceiver responsible for handling communication
 * @param data 32 bits data to transmit, not marshalled
 * @param dataSize Size of the data array
 */
void gpio_Protocol_Send(CustomMemTransceiver transceiver, uint32_t *data, size_t dataSize);

/**
 * Receive an entire set of data using a transceiver and the current protocol
 * Wait for the Rc high
 * Then use gpio_Mem_Chip_To_Chip_Reader() to received data from emitter
 * Unmarshall received data
 *
 * @param transceiver Transceiver responsible for handling communication
 * @param returnedDataSize Size of the array returned
 * @return 32 bits unmarshalled word
 */
uint32_t *gpio_Protocol_Receive(CustomMemTransceiver transceiver, size_t *returnedDataSize);

/**
 * TEST FUNCTION
 * Test function for protocol send
 * Initialize data structure and open memory map
 */
void gpio_Protocol_Send_Test();

/**
 * TEST FUNCTION
 * Test function for protocol received
 * Initialize data structure and open memory map
 * Print data received
 */
void gpio_Protocol_Receive_Test();

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
gpio_Marshall_Simplified(CustomMemTransceiver transceiver, uint32_t *data, size_t dataSize, size_t *returnedDataSize);


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
                                     size_t *returnedDataSize);

/**
 * TEST FUNCTION
 * Test the marshalling and unmarshalling function automatically
 * Compare observed result to expected values to check data validity
 *
 * @param nbDataPins Number of data pins used of transmission
 * @return
 */
int gpio_Marshall_Unmarshall_Validity_Check(int nbDataPins);

#endif //GPIO_ZEDBOARD_GPIODMC_H
