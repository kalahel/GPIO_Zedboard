//
// Created by Mathieu Hannoun on 12/06/18.
// GPIOFS stand for General Purpose Input/Output File System
// It is a library used for the GPIO control on the ARM processor embedded on a Zedboard
// For better performances use the GPIODMC library
//

#ifndef GPIO_ZEDBOARD_GPIOFS_H
#define GPIO_ZEDBOARD_GPIOFS_H

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


/**
 * Structure used for transceiver
 * Has nb_Data_Pins number of data pins
 * One clock
 *
 * nb_Data_Pins : Total number of data pins (clk not included)
 * pins_Ports : Array of pins ports
 * pins_Fds : Files descriptors opened on the pseudo files of the pins
 * clk_Port : Port of the clock
 * clk_fd : File descriptor for the clock
 */
typedef struct {
    int nb_Data_Pins;
    int *pins_Ports;
    int *pins_Fds;
    int clk_Port;
    int clk_fd;
} CustomTransceiver;

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
int gpio_SetDataDirection(int portAddr, int mode);

/**
 * Set a whole range of port as Inputs our Outputs
 *
 * @param firstPortAddr The address of the first port of the range
 * @param lastPortAddr The address of the last port of the range
 * @param mode Output : 0, Input : 1
 * @return 0 In case of success
 */
int gpio_SetDataDirection_Range(int firstPortAddr, int lastPortAddr, int mode);

/**
 * Unexport the selected port, good practice to unexport all used port at the end of a program
 *
 * @param portAddr Port to unexport
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Unexport(int portAddr);

/**
 * Read content found for the corresponding GPIO pin and put it in a buffer
 *
 * @param portAddr Target port
 * @param buf Buffer to store the read result
 * @param count Number of bytes to read
 * @return the number of bytes read
 */
int gpio_Read(int portAddr, void *buf, size_t count);

/**
 * Return a file descriptor for the opened pseudo file corresponding to the value of the port
 *
 * @param portAddr Port to open
 * @param flags Flag passed to the open() function
 * @return The file descriptor
 */
int gpio_Open(int portAddr, int flags);

/**
 * Export all the port of a transceiver and its clock in the correct direction
 *
 * @param transceiver Transceiver to export
 * @param mode GPIO_IN or GPIO_OUT
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Export_Transceiver(CustomTransceiver transceiver, int mode);

/**
 * Unexport all the port of a transceiver and its clock
 *
 * @param transceiver Transceiver to unexport
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Unexport_Transceiver(CustomTransceiver transceiver);

/**
 * Open all the port of the transceiver and its clock
 * Assigning them files descriptors
 *
 * @param transceiver Transceiver to set up
 * @param flags Opening flags (O_WRONLY, etc...)
 * @return 0 in case of success, -1 in case of failure
 */
CustomTransceiver gpio_Open_Transceiver(CustomTransceiver transceiver, int flags);

/**
 * Close all the files descriptor opened for the transceiver
 * Including the clock file descriptor
 *
 * @param transceiver Transceiver to close
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Close_Transceiver(CustomTransceiver transceiver);

/**
 * Set the desired edge in the corresponding pseudo file of a clock
 *
 * @param clkAddr Address of the clock
 * @param edgeOption EDGE_RISING / EDGE_FALLING / EDGE_BOTH
 * @return 0 in case of success
 */
int gpio_Set_Edge(int clkAddr, int edgeOption);

/**
 * TEST FUNCTION
 * Wait for rising edge on given port
 * Check period for reading
 *
 * @param portAddr Port to poll on
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Clocked_Poll(int portAddr);

/**
 * TEST FUNCTION
 * Monitor a pin and read it's content each rising edge on the clk pin
 *
 * @param clkAddr Address of the clock
 * @param portAddr Address of the port to monitor
 * @return will not return, infinite loop
 */
int gpio_Clocked_Poll_Monitor(int clkAddr, int portAddr);

/**
 * Wait for an specified edge on the clock port to fill the buffer with data read
 * Will return only after the buffer is filled
 * Discard the first character
 * Read count - 1 character
 * TODO add exponential time out
 *
 * @param portAddr Data pins address
 * @param clkAddr Clock pins address
 * @param buf Buffer that will be filled
 * @param count Size of the buffer in character
 * @param edgeOption EDGE_RISING / EDGE_FALLING / EDGE_BOTH
 * @return The number of character read (should be count - 1)
 */
int gpio_Read_Clocked_Poll(int portAddr, int clkAddr, char buf[], size_t count, int edgeOption);

/**
 * TEST FUNCTION
 * Will use gpio_Read_Clocked_Poll_Fast() to read transmission sent by emitter
 * Display the result with printf
 *
 * @return 0 in case of success
 */
int reader_Fast();

/**
 * Write a value directly on the pseudo file without opening it again
 *
 * @param fd File descriptor, pseudo file opened
 * @param value Value to write '0' or '1'
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Fast(int fd, char value);

/**
 * TEST FUNCTION
 * Will test the function gpio_Write_Fast by measuring it's average write frequency.
 * Generate a square signal for nbTest * internalRepetition period
 * Measure the internal frequency and period then display it with printf()
 * Port Ja1 is used
 *
 * @param nbTest Number of time the measure will be taken
 * @param internalRepetition Number of repetition between two measures. Warning value must be high enough to make measurement possible
 */
void gpio_Write_Fast_Tester(int nbTest, int internalRepetition);

/**
 * Write port by port a data, then rise the clock
 * Data array must be same number as the data pin numbers
 *
 * @param transceiver Transceiver containing all the information about clock and ports
 * @param data Array of bit to transmit
 * @param dataSize Size of the array of data
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Rising_half_Transceiver(CustomTransceiver transceiver, char data[], int dataSize);

/**
 * Write port by port a data, then lower the clock
 * Data array must be same number as the data pin numbers
 *
 * @param transceiver Transceiver containing all the information about clock and ports
 * @param data Array of bit to transmit
 * @param dataSize Size of the array of data
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Falling_half_Transceiver(CustomTransceiver transceiver, char data[], int dataSize);

/**
 * Write port by port a data, then rise the clock and lower it after
 * Data array must be same number as the data pin numbers
 * Complete cycle of data transmission on rising edge
 *
 * @param transceiver Transceiver containing all the information about clock and ports
 * @param data Array of bit to transmit
 * @param dataSize Size of the array of data
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Rising_Transceiver(CustomTransceiver transceiver, char data[], int dataSize);

/**
 * Write port by port a data, then lower the clock and rise it after
 * Data array must be same number as the data pin numbers
 * Complete cycle of data transmission on falling edge
 *
 * @param transceiver Transceiver containing all the information about clock and ports
 * @param data Array of bit to transmit
 * @param dataSize Size of the array of data
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Falling_Transceiver(CustomTransceiver transceiver, char data[], int dataSize);

/**
 * Writing data on both edge of the clock
 *
 * @param transceiver Transceiver containing all the information about clock and ports
 * @param dataFirst First set of data to transmit on rising edge
 * @param dataSecond Second set of data to transmit on falling edge
 * @param dataSize Size of the array of data
 * @return 0 in case of success, -1 in case of failure
 */
int gpio_Write_Both(CustomTransceiver transceiver, char dataFirst[], char dataSecond[], int dataSize);

/**
 * Display transceiver informations by using printf(2) to print clock pin, its file descriptor, data pin(s) and it/their file(s) descriptor(s)
 * @param transceiver Transceiver containing all the information about clock and ports
 */
void transceiver_Print_Info(CustomTransceiver transceiver);

/**
 * TEST FUNCTION
 * Will test the function gpio_Write_Rising_Transceiver by measuring it's average write frequency.
 * Will write 7 bits and its clocks in one time.
 * Whole Pmod JA is used
 *
 * @param nbTest Number of time the measure will be taken
 * @param internalRepetition Number of repetition between two measures. Warning value must be high enough to make measurement possible
 */
void transceiver_Write_Tester(int nbTest, int internalRepetition, CustomTransceiver transceiver);

#endif //GPIO_ZEDBOARD_GPIOFS_H
