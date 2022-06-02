#include "UART.h"

void UART1_Init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // enable clock
	RCC->CCIPR &= ~RCC_CCIPR_USART1SEL; // clear clock selection
	RCC->CCIPR |= RCC_CCIPR_USART1SEL_0; // set clock to system
}

void UART2_Init(void) {
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // enable clock
	RCC->CCIPR &= ~RCC_CCIPR_USART2SEL; // clear clock selection
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0; // set clock to system
}

void UART1_GPIO_Init(void) {
	// pb6, pb7 transmistters and recievers
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	
	GPIOB->MODER &= ~GPIO_MODER_MODE6; // clear pb6 mode
	GPIOB->MODER &= ~GPIO_MODER_MODE7; // clear pb7 mode
	GPIOB->MODER |= GPIO_MODER_MODE6_1; // pb6 to alt
	GPIOB->MODER |= GPIO_MODER_MODE7_1; // pb7 to alt
	
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL6; // clear alt function pb6
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL7; // clear alt function pb7
	GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL6 & ~GPIO_AFRL_AFSEL6_3); // pb6 to alt7
	GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL7 & ~GPIO_AFRL_AFSEL7_3); // pb7 to alt7
	
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT6; // pb6 to push pull
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT7; // pb7 to push pull
	
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6; // pb6 to very high speed
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7; // pb7 to very high speed
	
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD6; // clear pb6
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0; // pb6 to pullup 
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD7; // clear pb7
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_0; // pb7 to pullup
}

void UART2_GPIO_Init(void) {
	// pa2, pa3 transmistters and recievers
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	GPIOA->MODER &= ~GPIO_MODER_MODE2; // clear pa2 mode
	GPIOA->MODER &= ~GPIO_MODER_MODE3; // clear pa2 mode
	GPIOA->MODER |= GPIO_MODER_MODE2_1; // pa2 to alt
	GPIOA->MODER |= GPIO_MODER_MODE3_1; // pa2 to alt
	
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2; // clear alt function pa2
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL3; // clear alt function pa3
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL2 & ~GPIO_AFRL_AFSEL2_3); // pa2 to alt7
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL3 & ~GPIO_AFRL_AFSEL3_3); // pa3 to alt7
	
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT2; // pa2 to push pull
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT3; // pa3 to push pull
	
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2; // pa2 to very high speed
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3; // pa3 to very high speed
	
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD2; // clear pa2
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD2_0; // pa2 to pullup 
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3; // clear pa3
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD3_0; // pa3 to pullup
}

void USART_Init(USART_TypeDef* USARTx) {
	USARTx->CR1 &= ~USART_CR1_UE; // disable usart
	USARTx->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // word length 8 bits
	USARTx->CR1 &= ~USART_CR1_OVER8; // oversampling by 8
	USARTx->CR2 &= ~USART_CR2_STOP; // stop bits to 1
	
	// USART enable flag: nooo you can't just set this value to 8333
	USARTx->BRR = 0x208D; // haha usart go brrrrr
	
	USARTx->CR1 |= USART_CR1_RE; // enable reciever
	USARTx->CR1 |= USART_CR1_TE; // enable transmitter
	
	USARTx->CR1 |= USART_CR1_UE; // enable usart
}

uint8_t USART_Read (USART_TypeDef * USARTx) {
	// SR_RXNE (Read data register not empty) bit is set by hardware
	while (!(USARTx->ISR & USART_ISR_RXNE));  // Wait until RXNE (RX not empty) bit is set
	// USART resets the RXNE flag automatically after reading DR
	return ((uint8_t)(USARTx->RDR & 0xFF));
	// Reading USART_DR automatically clears the RXNE flag 
}

void USART_Write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {
	int i;
	// TXE is cleared by a write to the USART_DR register.
	// TXE is set by hardware when the content of the TDR 
	// register has been transferred into the shift register.
	for (i = 0; i < nBytes; i++) {
		while (!(USARTx->ISR & USART_ISR_TXE));   	// wait until TXE (TX empty) bit is set
		// Writing USART_DR automatically clears the TXE flag 	
		USARTx->TDR = buffer[i] & 0xFF;
		USART_Delay(300);
	}
	while (!(USARTx->ISR & USART_ISR_TC));   		  // wait until TC bit is set
	USARTx->ISR &= ~USART_ISR_TC;
}   

void USART_Delay(uint32_t us) {
	uint32_t time = 100*us/7;    
	while(--time);   
}
