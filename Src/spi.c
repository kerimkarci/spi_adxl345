/*
 * spi.c
 *
 *  Created on: Jan 30, 2024
 *      Author: Kerim
 */

#include "spi.h"

#define SPI1EN		(1<<12)
#define GPIOAEN		(1<<0)
#define SR_TXE		(1<<1)
#define SR_RXNE		(1<<0)

#define SR_BSY		(1<<7)


//PA5 -> CLK
//PA6 -> MISO
//PA7 -> MOSI

//PA9 -> SLAVE SELECT

void spi_gpio_init(void)
{
	/*enable clock acces to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA5,PA6,PA7 mode to alternate function*/

	/*PA5*/
	GPIOA->MODER &= ~(1<<10);
	GPIOA->MODER |=  (1<<11);

	/*PA6*/
	GPIOA->MODER &= ~(1<<12);
	GPIOA->MODER |=  (1<<13);

	/*PA7*/
	GPIOA->MODER &= ~(1<<14);
	GPIOA->MODER |=  (1<<15);

	/*Set PA9 as output pin*/
	GPIOA->MODER |=  (1<<18);
	GPIOA->MODER &= ~(1<<19);

	/*Set PA5,PA6,PA7 alternate function type to SPI1*/
	/*PA5*/
	GPIOA->AFR[0] |=  (1<<20);
	GPIOA->AFR[0] &= ~(1<<21);
	GPIOA->AFR[0] |=  (1<<22);
	GPIOA->AFR[0] &= ~(1<<23);
	/*PA6*/
	GPIOA->AFR[0] |=  (1<<24);
	GPIOA->AFR[0] &= ~(1<<25);
	GPIOA->AFR[0] |=  (1<<26);
	GPIOA->AFR[0] &= ~(1<<27);
	/*PA7*/
	GPIOA->AFR[0] |=  (1<<28);
	GPIOA->AFR[0] &= ~(1<<29);
	GPIOA->AFR[0] |=  (1<<30);
	GPIOA->AFR[0] &= ~(1<<31);
}

void spi1_config(void)
{
	/*Enable clock access to SPI1 module*/
	RCC->APB2ENR |= SPI1EN;

	/*Set the clock to fPCLK/4*/
	SPI1->CR1 |=  (1<<3);
	SPI1->CR1 &= ~(1<<4);
	SPI1->CR1 &= ~(1<<5);

	/*Set CPOL to 1 and CPHA to 1*/
	SPI1->CR1 |= (1<<0);
	SPI1->CR1 |= (1<<1);

	/*Enable full duplex*/
	SPI1->CR1 &= ~(1<<10);

	/*Set MSB first*/
	SPI1->CR1 &= ~(1<<7);

	/*Set mode to Master*/
	SPI1->CR1 |= (1<<2);

	/*Set 8 bit data mode*/
	SPI1->CR1 |= (1<<11);

	/*Select software slave management by
	 * setting SSM=1 and SSI=1*/
	SPI1->CR1 |= (1<<8);
	SPI1->CR1 |= (1<<9);

	/*Enable SPI module*/
	SPI1->CR1 |= (1<<6);
}

void spi1_transmit(uint8_t *data, uint32_t size)
{
	uint32_t i=0;
	uint8_t temp;

	while(i<size)
	{
		/*wait until TXE is set*/
		while(!(SPI1->SR & (SR_TXE))){}

		/*Write the data to the data register*/
		SPI1->DR =data[i];
		i++;
	}
	/*wait until TXE is set*/
	while(!(SPI1->SR & (SR_TXE))){}


	/*Wait for BUSY flag to reset*/
	while((SPI1->SR & (SR_BSY))){}

	/*Clear OVR flag*/
	temp = SPI1->DR;
	temp = SPI1->SR;
}

void spi1_receive(uint8_t *data, uint32_t size)
{
	while(size)
	{
		/*Send dummy data*/
		SPI1->DR = 0;

		/*Wait for RXNE flag to be set*/
		while(!(SPI1->SR & (SR_RXNE))){}

		/*Read the data from data register*/
		*data++ = (SPI1->DR);
		size--;
	}
}

void cs_enable(void)
{
	/*Set it to 0 to enable it*/
	GPIOA->ODR &= ~(1<<9);
}

/*Pull high to disable*/
void cs_disable(void)
{
	/*Set it to 1 to disable it*/
	GPIOA->ODR |= (1<<9);
}








