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

#define I2C_100Khz	(80) //Avoiding magic numbers
#define SD_MODE_MAX_RISE_TIME	(17) //This can be taken from STMCubeMX
#define RCC_AHB1ENR__GPIOB	(1UL<<1)
#define RCC_APB1ENR__I2C1	(1UL<<21)
#define I2C_CR1__PE (1UL<<0)
#define I2C_SR2__BUSY (1UL<<1)
#define I2C_CR1__START (1UL<<8)
#define I2C_SR1__SB (1UL<<0)
#define I2C_SR1__ADDR (1UL<<1)
#define I2C_SR1__TxE (1UL<<7)
#define I2C_CR1__ACK  (1UL<<10)
#define I2C_CR1__STOP (1UL<<9)
#define I2C_CR1__RxNE (1UL<<6)
#define I2C_SR1__BTF   (1UL<<2)

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


	 * Explanation: info taken from RM0090:GPIO port mode register (GPIOx_MODER) (x = A..I/J/K)
	 * 		We are interested in the bits 19,18 (port 9) and bits 17,16 (port 8)
	 *		These bits are written by software to configure the I/O direction mode.
	 *		...
	 *		10: Alternate function mode
	 *		...
	 *
	* Set PB8 and PB9 modes to alternate functions *//* Set PB8 and PB9 modes to alternate functions *
	GPIOB->MODER |= (1UL << 19); //'1' port 9
	GPIOB->MODER &= ~(1UL << 18); //'0' port 9 //1UL<<19 means value 1 as unsigned int shifted by 19 bits to the left

	GPIOB->MODER |= (1UL << 17); //'1' port 8
	GPIOB->MODER &= ~(1UL << 16); //'0' port 8

	*
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
	 *
	/* ****************************************************************************************************************************************************************
	 * Explanation: info taken from RM0090: GPIO port output type register (GPIOx_OTYPER) (x = A..I/J/K)
	 * 		We are interested in the bits 9 and 8
	 * 		OTy: Port x configuration bits (y = 0..15)
	 * 			These bits are written by software to configure the output type of the I/O port.
	 * 			0: Output push-pull (reset state)
	 * 			1: Output open-drain
	 * **************************************************************************************************************************************************************** */
	/* Set PB8 and PB9 output type to open drain */
	GPIOB->OTYPER |= (1UL << 9); //'1'
	GPIOB->OTYPER |= (1UL << 8); //'1' the rest of the bits are by defaut '0' since the reset value is 0x0 for this registry

	/* Enable the pull-ups for PB8 and PB9 */
	GPIOB->PUPDR &= ~(1UL << 19); //'0' port 9
	GPIOB->PUPDR |= (1UL << 18); //'1' port 9

	GPIOB->PUPDR &= ~(1UL << 17); //'0' port 8
	GPIOB->PUPDR |= (1UL << 16); //'1' port 9

	/* Set alternate functions to AF4 */

	//PB9
	GPIOB->AFR[1] &= ~(1UL << 7); // '0'
	GPIOB->AFR[1] |= (1UL << 6); //'1'
	GPIOB->AFR[1] &= ~(1UL << 5); // '0'
	GPIOB->AFR[1] &= ~(1UL << 4); // '0'

	//PB8
	GPIOB->AFR[1] &= ~(1UL << 3); // '0'
	GPIOB->AFR[1] |= (1UL << 2); //'1'
	GPIOB->AFR[1] &= ~(1UL << 1); // '0'
	GPIOB->AFR[1] &= ~(1UL << 0); // '0'

	// I2C Control register 1 (I2C_CR1)	in RM(refference manual)
	I2C1->CR1 |= (1UL << 15); //Clears all settings and resets the I2C1 to its default state
	/* Revert from reset mode */
	I2C1->CR1 &= ~(1UL << 15); //Clears the reset bit, bringing I2C1 back to normal operation

	/* ******************************************************************************************************************************************************************
	 * Explanation: info taken from RM: I2C Control register 2 (I2C_CR2)
	 * We are going to be interested in bits 5..0 FREQ[5:0]: Peripheral clock frequency
	 * 		The FREQ bits must be configured with the APB clock frequency value (I2C peripheral connected to APB).
	 *		The minimum allowed frequency is 2 MHz, the maximum frequency is limited by the maximum APB frequency and cannot exceed 50 MHz (peripheral intrinsic maximum limit).
	 * 			0b000000: Not allowed
	 * 			0b000001: Not allowed
	 * 			0b000010: 2 MHz
	 * 			...
	 * 			0b110010: 50 MHz
	 * 			Higher than 0b101010: Not allowed
	 * 			So 0b(binary_representation_of_decimal Mhz)
	 * ***************************************************************************************************************************************************************** */
	/* Set the peripheral clock frequency */
	I2C1->CR2 = (1UL << 4); //'010000' 16Mhz
	I2C1->CCR = I2C_100Khz;  // Configure standard mode (100kHz)
	//same as  I2C1->CCR = 80; Formula for Standard Mode CCR = PCLK1 / (2*I2C Clock speed)

	I2C1->TRISE = SD_MODE_MAX_RISE_TIME; //Configure maximum rise time
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

void i2c1_BurstWrite(char slave_address, char memory_address, int number_bytes_to_write, char *data)
{
	volatile int temporary;
	/*
	 * Note on volatile
	 * The volatile keyword ensures that variables are always read from memory instead of registers to avoid compiler optimizations.
	 * It is essential in embedded systems for:  Hardware registers (GPIO, UART, Timers)
	 * Interrupts (shared variables between main and ISR)
	 * Polling loops (waiting for hardware status changes)
	 * This means that even though in the code it may seem like a variable is not being changed,it can still change
	 * Due to nature of embedded system
	 * */


	/* Wait until the bus is not busy */
	while(I2C1->SR2 & I2C_SR2__BUSY);

	/* Generate a START condition */
	I2C1->CR1 |=I2C_CR1__START;

	/* Wait for the START bit to be set */
	while(!(I2C1->SR1 & I2C_SR1__SB));

	/* Transmit the address of the slave and write */
	I2C1->DR = slave_address<<1;

	/* Wait until address flag is set */
	while(!(I2C1->SR1 & I2C_SR1__ADDR));

	/* 	Clear the address flag */
	temporary=I2C1->SR2;

	/* Wait until the data registry/transmitter is empty */
	while(!(I2C1->SR1 & I2C_SR1__TxE));

	/* Send memory address */
	I2C1->DR=memory_address;

	for(int i=0;i<number_bytes_to_write;i++)
	{
		/* Wait until transmitter is empty */
		while(!(I2C1->SR1 & I2C_SR1__TxE));

		/* Transmit memory address */
		I2C1->DR = *data++; //Pointer arithmetic is generally faster in embedded C,alternative is we use a copy of array,data[i
		//Using pointers,we actually dont copy the array,we only point to a byte in array and then with ++ operator,we move it to next
	}

	/* *************************************************************************************************************************************************************************
		 * Explanation: info taken from RM0090: I2C Status register 1 (I2C_SR1)
		 * We are interested in bit no. 2 BTF: Byte transfer finished
		 * 		0: Data byte transfer not done
				1: Data byte transfer succeeded
	   ************************************************************************************************************************************************************************* */
	/* Wait until the transfer is finished */
	while(!(I2C1->SR1 & I2C_SR1__BTF));

	/* Generate stop */
	I2C1->CR1 |= I2C_CR1__STOP;

}
