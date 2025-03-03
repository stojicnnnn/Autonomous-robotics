/*
 * i2c.c
 *
 *  Created on: 22.02.2025.
 *      Author: Nikola
 */

#include "biblioteka/i2c.h"

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * Standard mode (100kHz) on PB8 (SCL) and PB9 (SDA)
 * Configures GPIO pins, enables I2C1 clock, and sets required register values.
 */
void I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  // Enable I2C1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock

    /*
     RCC short for Reset and Clock Control
     Peripheral in STM32 microcontrollers responsible for enabling and configuring clock signals for different peripherals
     Defined in the STM32 header files (stm32f4xx.h)

     APB1ENR - Advanced Peripheral Bus 1 Enable Register - 32-bit register inside RCC
     Each bit in APB1ENR corresponds to a different peripheral (e.g., timers, UART, I2C, SPI, etc.)

     RCC_APB1ENR_I2C1EN bit mask for I2C1 clock enable bit.
     Defined in stm32f4xx.h
     21st bit in APB1ENR controls I2C1 clock,0 disabled,1 enabled

     |= bitwise OR assignment operator guarantees that I2C1 clock enable bit is set to 1
     Because of this,I2C1 peripheral can recieve clock signal
     All the other set bits in APB1ENR are not changed

     0000 0000 0000 0000 0000 0000 0000 0001  (32-bit representation)
     << shifts bits to the left by a specified number of positions
     (1 << 21) means shifting 1 21 positions to the left
     0000 0010 0000 0000 0000 0000 0000 0000  (32-bit representation after shifting)

     RCC_AHB1ENR_GPIOBEN bit used to enable the clock for GPIO port B

     */

    // Configure PB6 and PB7 as alternate function I2C1
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9); //Clear mode bits for PB8 PB9,other unchanged,force them to 00,input mode
    GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1); // Set bits 1 for pins PB8 and PB9 to 1,while leaving bits 0 to 0
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9); // Open-drain mode,required for i2c for bidirectional comm
    GPIOB->PUPDR |= (1 << (8 * 2)) | (1 << (9 * 2));
    GPIOB->AFR[1] |= (4 << ((8 - 8) * 4)) | (4 << ((9 - 8) * 4)); // Set alternate function I2C1

/*
 * Each GPIO port in STM32 (e.g., GPIOA, GPIOB, etc.) has a MODER (Mode Register)
 * MODER determines whether each GPIO pin operates as:
	Input mode (00)
	Output mode (01)
	Alternate function mode (10)
	Analog mode (11)

	AFR[0] → Configures pins 0-7
	FR[1] → Configures pins 8-15
	PB6 and PB7 are in AFR[0]

	Each pin can be set to one of 16 alternate functions (0 to 15)
	Alternate Function 4 (AF4) is used for I2C1

	The value 4 (binary 0100) corresponds to Alternate Function 4 (AF4), which is I2C1
	(6 * 4) = 24, so 4 << 24 → Writes 0100 at bits 24-27 pb6
	(7 * 4) = 28, so 4 << 28 → Writes 0100 at bits 28-31 pb7

	8 * 4 = 32, so 32 - 32 = 0 (First 4 bits of AFR[1])
	4 << 0 → Places 0100 in bits 0-3
	This configures PB8 and PB9 to Alternate Function 4 (AF4), which is the I2C1 SCL/SDA function
 *
 */
    // Reset and configure I2C1
    I2C1->CR1 = I2C_CR1_SWRST; //Clears all settings and resets the I2C1 to its default state
   // I2C1->CR1 = 0; //Clears the reset bit, bringing I2C1 back to normal operation
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2 = 16; // Set peripheral clock frequency (16MHz)
    I2C1->CCR = 80; // Configure standard mode (100kHz)
    //Formula for Standard Mode CCR = PCLK1 / (2*I2C Clock speed)

    I2C1->TRISE = 17; //Configure maximum rise time
    /*
	Configure maximum rise time
    TRISE helps synchronize SCL/SDA transitions
    Synchronizes the rise time of SDA/SCL signals
    Rise time in I2C is the time it takes for the SDA and SCL to transition from (0) voltage level to (1)
    TRISE = (PCLK1×tr / 1 000 000 ) +1
    PCLK1 Peripheral clock frequency APB1 clock
    t_r = Max rise time in nanoseconds
    Pull-up resistors and bus capacitance affect rise time
    If rise time is too slow, communication fails or slows down
    */
    I2C1->CR1 |= I2C_CR1_PE; // Enable I2C1
}

void I2C1_Start(void) {
    I2C1->CR1 |= I2C_CR1_START | I2C_CR1_ACK;
    while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait for start condition flag
}
/*
	I2C1->CR1 (Control Register 1) manages the behavior of the I2C peripheral
	I2C_CR1_START (Bit 8) is used to generate a Start condition on the bus
	The Master (STM32) pulls SDA low while SCL is high, signaling a START condition
	The I2C peripheral begins communication with a slave

	I2C1->SR1 (Status Register 1) contains status flags that indicate the I2C state
	I2C_SR1_SB (Bit 0) is the Start Bit (SB) flag
	While loop waits until SB is set to 1, meaning the start condition has been successfully sent
 */


void I2C1_Stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}
/*
	I2C1->CR1 (Control Register 1) controls the behavior of the I2C peripheral
	I2C_CR1_STOP (Bit 9) is used to generate a Stop Condition on the I2C bus
	The |= operation ensures only the STOP bit is set, keeping other settings unchanged
 */


void I2C1_Write(uint8_t data) {
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_TXE)); // Wait for data to be transmitted
}

/*
	I2C1->DR (Data Register) holds the byte to be transmitted over the I2C bus
	The STM32 loads the byte into the shift register
	The I2C hardware starts sending the byte over the SDA line

	I2C1->SR1 (Status Register 1) contains status flags indicating I2C events
	I2C_SR1_TXE (Bit 7) stands for "Transmit Data Register Empty"
	The while loop waits until the TXE flag is set (1), meaning:
		The previous byte has been shifted out.
		The Data Register (DR) is empty and ready for the next byte
 */

uint8_t I2C1_Read(uint8_t ack) {
    if (ack) {
        I2C1->CR1 |= I2C_CR1_ACK;
    } else {
        I2C1->CR1 &= ~I2C_CR1_ACK;
    }
    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // Wait for received data
    return I2C1->DR;
}

/*
	ACK/NACK determines whether the master acknowledges receipt of the byte:
	If ack == 1, send ACK (expecting more bytes).
	If ack == 0, send NACK (final byte received, preparing for Stop condition)

	I2C1->SR1 (Status Register 1) contains flags that indicate the I2C state.
	I2C_SR1_RXNE (Bit 6) is the Receive Data Register Not Empty flag.
	The while loop waits until the RXNE flag is set, meaning:
	The byte has been received from the slave.
	The Data Register (DR) contains the received byte

	While loop ensures the master does not read an empty buffer
	Prevents reading before data is available- race conditions

	I2C1->DR (Data Register) holds the received byte.
	The function returns the received byte to the caller
 */
void I2C1_WriteRegister(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
    I2C1_Start();
    I2C1_Write(deviceAddr << 1); // Send device address with write flag
    I2C1_Write(regAddr); // Send register address
    I2C1_Write(data); // Send data byte
    I2C1_Stop();
}

uint8_t I2C1_ReadRegister(uint8_t deviceAddr, uint8_t regAddr) {
    uint8_t data;
    I2C1_Start();
    I2C1_Write(deviceAddr << 1); // Send device address with write flag
    I2C1_Write(regAddr); // Send register address
    I2C1_Start(); // Restart condition for read mode
    I2C1_Write((deviceAddr << 1) | 1); // Send device address with read flag
    data = I2C1_Read(0); // Read data with NACK
    I2C1_Stop();
    return data;
}

uint8_t I2C1_CheckDevice(uint8_t address)
{
    I2C1_Start();
    I2C1_Write(address);  // Send Slave Address

    if (I2C1->SR1 & I2C_SR1_AF) {  // Check for NACK (Acknowledge Failure)
        I2C1_Stop();
        return 0;  // Device NOT detected
    }

    I2C1_Stop();
    return 1;  // Device detected
}

