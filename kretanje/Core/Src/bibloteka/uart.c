/*
 * uart.c
 *
 *  Created on: 17. mar 2024.
 *      Author: HP
 */

#include "biblioteka/uart.h"

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

#define MAX_VELICINA_BUFFERA 30

volatile static uint8_t input = 0;

volatile static uint8_t buffer[MAX_VELICINA_BUFFERA];
volatile static uint8_t velicina = 0;
volatile static uint8_t index_pisi = 0;
volatile static uint8_t index_citaj = 0;

static void
uart6_init ();

void
uart_init ()
{
  uart6_init ();
}

static void
uart6_init ()
{
  RCC->AHB1ENR |= (0b1 << 0); // Dozvola takta na GPIO Port-u A
  RCC->APB2ENR |= (0b1 << 5); // Dozvola takta na USART6 periferiji

  uint8_t const TX_PIN = 11; //PC6  PA11
  uint8_t const RX_PIN = 12; //PC7   PA12

  GPIOA->MODER &= ~(0b11 << TX_PIN * 2);
  GPIOA->MODER |= (0b10 << TX_PIN * 2);
 GPIOA->MODER &= ~(0b11 << RX_PIN * 2);
  GPIOA->MODER |= (0b10 << RX_PIN * 2);

  // Podešavanje da TX pin bude pull up open drain
  // kako bi radio kao half duplex
  GPIOA->OTYPER |= (0b1 << TX_PIN);
  GPIOA->PUPDR &= ~(0b11 << TX_PIN * 2);
  GPIOA->PUPDR |= (0b01 << TX_PIN * 2);

  uint8_t const AF = 8;

  GPIOA->AFR[TX_PIN / 8] &= ~(0b1111 << (TX_PIN % 8) * 4);
  GPIOA->AFR[TX_PIN / 8] |= (AF << (TX_PIN % 8) * 4);
  GPIOA->AFR[RX_PIN / 8] &= ~(0b1111 << (RX_PIN % 8) * 4);
  GPIOA->AFR[RX_PIN / 8] |= (AF << (RX_PIN % 8) * 4);

  USART6->CR1 &= ~(0b1 << 12); // Dužina poruke
  USART6->CR2 &= ~(0b11 << 12); // Broj stop bitova

  // Baudrate = 9600
  //USART6->BRR = 0;
  USART6->BRR &= ~(0xFFFF);
  USART6->BRR |= ((546 << 4) | (14 << 0));

  // Uključivanje TX i RX pinova
  USART6->CR1 |= ((0b1 << 2) | (0b1 << 3));

  // Uključivanje prekida za prihvatanje poruke
  USART6->CR1 |= (0b1 << 5);

  // Uključivanje Half-duplex komunikacije
  USART6->CR2 &= ~((0b1 << 11) | (0b1 << 14));
  USART6->CR3 &= ~((0b1 << 1) | (0b1 << 5));
  USART6->CR3 |= (0b1 << 3);

  // Odabir prekidne rutike koja se izvršava
  uint8_t const USART6_PREKID = 71;
  NVIC->ISER[USART6_PREKID / 32] |= (0b1 << USART6_PREKID % 32);

  // Uključivanje UART-a
  USART6->CR1 |= (0b1 << 13);
}

void
uart_send_byte (uint8_t podatak)
{
  USART6->DR = podatak;

  while (!(USART6->SR & (0b1 << 6)))
    {
      __NOP();
    }
  USART6->SR &= ~(0b1 << 6);
}

// Piši u buffer
void
uart_pisi (uint8_t podatak)
{
  if (velicina != MAX_VELICINA_BUFFERA)
    {
      buffer[index_pisi] = podatak;
      index_pisi = (index_pisi + 1) % MAX_VELICINA_BUFFERA;
      velicina++;
    }
  else
    {
      buffer[index_pisi] = podatak;
      index_pisi = (index_pisi + 1) % MAX_VELICINA_BUFFERA;
      index_citaj = (index_citaj + 1) % MAX_VELICINA_BUFFERA;
    }
}

// Čitaj iz buffera
uint8_t
uart_citaj ()
{
  uint8_t podatak;

  if (velicina != 0)
    {
      podatak = buffer[index_citaj];
      index_citaj = (index_citaj + 1) % MAX_VELICINA_BUFFERA;
      velicina--;
    }

  return podatak;
}

bool
uart_buffer_prazan ()
{
  if (velicina == 0)
    {
      return true;
    }
  else
    {
      return false;
    }
}

void
USART6_IRQHandler ()
{
  if (USART6->SR & (0b1 << 5))
    {
      //input = USART6->DR;
      uart_pisi(USART6->DR);
    }
}
