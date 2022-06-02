

/*

	Eric Buckland Final Project

	
	https://stm32f4-discovery.net/2018/06/tutorial-control-ws2812b-leds-stm32/
	http://fabioangeletti.altervista.org/blog/stm32-interface-ws2812b/?doing_wp_cron=1528043483.7364630699157714843750

*/

#include "stm32l476xx.h"
#include "UART.h"
#include "I2C.h"
#include "SysClock.h"
#include "stm32l476xx.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <inttypes.h>

void init_colors(void);

static struct Color {
	uint8_t R;
	uint8_t G;
	uint8_t B;
} Off, White, Grey, Red, Blue, Green;

void init_colors(void) {
	Off.R = 0x0; Off.B = 0x0; Off.G = 0x0;
	White.R = 0xFF; White.B = 0xFF; White.G = 0xFF;
	Grey.R = 0x22; Grey.B = 0x22; Grey.G = 0x22;
	Red.R = 0x60; Red.B = 0x0; Red.G = 0x0;
	Blue.R = 0x0; Blue.B = 0x60; Blue.G = 0x0;
	Green.R = 0x0; Green.B = 0x0; Green.G = 0x60;
};

// bring clock speed down to 1mb

void sysclock_init(void);
void led_set_color(int i, int j, struct Color* color);
void led_set_color_all(struct Color* color);
void led_init(void);
void led_update(void);
void animation_arrow_up(struct Color* color);
void animation_arrow_down(struct Color* color);

#define NUM_LEDS_X 8
#define NUM_LEDS_Y 8
#define NUM_LEDS (NUM_LEDS_X * NUM_LEDS_Y)
#define LED_BYTES 3

#define SPACING_BYTES 1
#define COLOR_SPACING_BYTES 0

#define RAW_LED_BYTES (8 * LED_BYTES)
#define RAW_LED_BYTES_WSPACING (RAW_LED_BYTES + COLOR_SPACING_BYTES*LED_BYTES + RAW_LED_BYTES*SPACING_BYTES)
#define BLANK_LEDS 6
#define HIGH_BYTES 0
#define TOTAL_RAW_BYTES (RAW_LED_BYTES_WSPACING*(BLANK_LEDS * 2 + NUM_LEDS))


// 80 MHz / (0+1) * 1.25us - 1 = 99
// 80 MHz / (1+1) * 1.25us - 1 = 49
// 80 MHz / (3+1) * 1.25us - 1 = 24
#define PSC_VAL 0
#define ARR_VAL 99

static uint8_t led_colors[NUM_LEDS*LED_BYTES];
static uint8_t is_updating = 0;
static uint8_t current_led = 0;
static uint8_t raw_array[TOTAL_RAW_BYTES];

void Init_USARTx(int x) {
	if(x == 1) {
		UART1_Init();
		UART1_GPIO_Init();
		USART_Init(USART1);
	} else if(x == 2) {
		UART2_Init();
		UART2_GPIO_Init();
		USART_Init(USART2);
	} else {
		// Do nothing...
	}
}

int main(void) {
		System_Clock_Init();
	
		Init_USARTx(2);
	
		printf("START\n");
	
    led_init();
		
	
		uint8_t SlaveAddress = 0x48 << 1; //0b1001000
		uint8_t Data_Receive;
		uint8_t Data_Send = 0;
	
		
		while(1) {
			// First, send a command to the sensor for reading the temperature
			I2C_SendData(I2C1, SlaveAddress, &Data_Send, 1); // send one 0 byte
		
			// Next, get the measurement
			I2C_ReceiveData(I2C1, SlaveAddress, &Data_Receive, 1); // read 1 byte
		
			int temp = Data_Receive & 0x7F;
			if (Data_Receive >> 7 == 1) {
				temp = temp * -1;
			}
			
			if (temp < 37) {
				break;
			}
			
			// Some delay
			for(int i = 0; i < 100000; ++i); 
		}
		
	
		//animation_arrow_up(&Green);
	
		led_set_color_all(&Off);
	
		led_update();
		
		while (1) {
			char rxByte;
			scanf("%c", &rxByte);
			if (tolower(rxByte) == 'u') {
				init_colors();
				//printf("UP\n");
				animation_arrow_up(&Green);
			} else if (tolower(rxByte) == 'd') {
				init_colors();
				//printf("DOWN\n");
				animation_arrow_down(&Red);
			}
		}
	
}

void animation_arrow_up(struct Color* color) {
	for (int i = NUM_LEDS_X; i >= 0; i--) {
		//printf("%d, %d, %d\n",Off.R,Off.G, Off.B); 
		led_set_color_all(&Off);
		
		led_set_color(i, 3, color);
		led_set_color(i, 4, color);
		
		led_set_color(i+1, 2, color);
		led_set_color(i+1, 3, color);
		led_set_color(i+1, 4, color);
		led_set_color(i+1, 5, color);
		
		led_set_color(i+2, 1, color);
		led_set_color(i+2, 2, color);
		led_set_color(i+2, 3, color);
		led_set_color(i+2, 4, color);
		led_set_color(i+2, 5, color);
		led_set_color(i+2, 6, color);
		
		led_set_color(i+3, 0, color);
		led_set_color(i+3, 1, color);
		led_set_color(i+3, 2, color);
		led_set_color(i+3, 3, color);
		led_set_color(i+3, 4, color);
		led_set_color(i+3, 5, color);
		led_set_color(i+3, 6, color);
		led_set_color(i+3, 7, color);
		
		led_set_color(i+4, 3, color);
		led_set_color(i+4, 4, color);
		
		led_set_color(i+5, 3, color);
		led_set_color(i+5, 4, color);
		
		led_set_color(i+6, 3, color);
		led_set_color(i+6, 4, color);
		
		led_set_color(i+7, 3, color);
		led_set_color(i+7, 4, color);
		
		led_update();
		
		for (int j = 0; j < ((NUM_LEDS_X-i) * (NUM_LEDS_X-i) * 30000); j++);
	}
}

void animation_arrow_down(struct Color* color) {
	for (int i = 0; i < NUM_LEDS_X+1; i++) {
		led_set_color_all(&Off);
		
		led_set_color(i-1, 3, color);
		led_set_color(i-1, 4, color);
		
		led_set_color(i-2, 2, color);
		led_set_color(i-2, 3, color);
		led_set_color(i-2, 4, color);
		led_set_color(i-2, 5, color);
		
		led_set_color(i-3, 1, color);
		led_set_color(i-3, 2, color);
		led_set_color(i-3, 3, color);
		led_set_color(i-3, 4, color);
		led_set_color(i-3, 5, color);
		led_set_color(i-3, 6, color);
		
		led_set_color(i-4, 0, color);
		led_set_color(i-4, 1, color);
		led_set_color(i-4, 2, color);
		led_set_color(i-4, 3, color);
		led_set_color(i-4, 4, color);
		led_set_color(i-4, 5, color);
		led_set_color(i-4, 6, color);
		led_set_color(i-4, 7, color);
		
		led_set_color(i-5, 3, color);
		led_set_color(i-5, 4, color);
		
		led_set_color(i-6, 3, color);
		led_set_color(i-6, 4, color);
		
		led_set_color(i-7, 3, color);
		led_set_color(i-7, 4, color);
		
		led_set_color(i-8, 3, color);
		led_set_color(i-8, 4, color);
		
		led_update();
		
		for (int j = 0; j < (i*i * 30000); j++);
	}
}



void led_set_color_all(struct Color* color) {
	//printf("led_set_color_all\n");
	for (int i = 0; i < NUM_LEDS_Y; i++) {
		for (int j = 0; j < NUM_LEDS_X; j++) {
			led_set_color(i, j, color);
			//printf("(%d,%d", i,j);
		}
	}
}

void led_set_color(int i, int j, struct Color* color) {
	if (i >= NUM_LEDS_X || j >= NUM_LEDS_Y || i < 0 || j < 0) {
		return;
	}
	//printf("index before %d %d\n", i , j);
	if (j%2 == 1) {
		i = 7 - i;
	}
	//printf("index after %d %d\n", i , j);
	
	//printf("set %d %d start index %d to %d %d %d\n", i, j,i + (8 * j), color->R , color->G, color->B);
	// needs to be in GRB order
	led_colors[(i + NUM_LEDS_Y * j)*3] = color->G;
	led_colors[(i + NUM_LEDS_Y * j)*3 + 1] = color->R;
	led_colors[(i + NUM_LEDS_Y * j)*3 + 2] = color->B;
}

void led_update(void) {
	//printf("led_update\n");
	
	/*
	printf("led_colors\n");
	for (int i = 0; i < NUM_LEDS*LED_BYTES; i++) {
		printf("%d,",led_colors[i]);
	}
	*/
	
	
	int i = 0;
	
	for (; i < HIGH_BYTES; i++) {
		raw_array[i] = ARR_VAL/2;
	}
	
	for (; i < RAW_LED_BYTES_WSPACING*BLANK_LEDS; i++) {
		raw_array[i] = 0;
	}

	for (int j = 0; i < RAW_LED_BYTES_WSPACING*(NUM_LEDS + BLANK_LEDS); j++) {
			for (int  k = 0; k < 8; k++, i++) {
				raw_array[i] = (led_colors[j] & (1 << k) ? (ARR_VAL*.65) : (ARR_VAL*.35));
				
				for (int  l = 0; l < SPACING_BYTES; l++) {
					i++;
					raw_array[i] = 0;
				}
				//printf("setting index %d to %d\n",i, raw_array[i]);
			}
			for (int  k = 0; k < COLOR_SPACING_BYTES; k++, i++) {
				raw_array[i] = 0;
			}
	}
	
	for (; i < TOTAL_RAW_BYTES-HIGH_BYTES; i++) {
		raw_array[i] = 0;
	}
	for (; i < TOTAL_RAW_BYTES; i++) {
		raw_array[i] = ARR_VAL/2;
	}
	
	//printf("\nraw_array\n");
	/*
	for (int i = 0; i < TOTAL_RAW_BYTES; i++) {
		printf("%d,",raw_array[i]);
		if (!((i+1)%RAW_LED_BYTES_WSPACING)) {
			for (int i = 0; i < 2000; i++);
			printf("\n");
		}
	}*/
	
	//return;
	
	//printf("led_update - setup bits\n");
	
	DMA1_Channel5->CCR &= ~DMA_CCR_EN; // disable channel

	DMA1_Channel5->CMAR = (uint32_t)raw_array; // set the buffer to data
	DMA1_Channel5->CNDTR = TOTAL_RAW_BYTES; // set number of bytes
	
	DMA1_Channel5->CCR |= DMA_CCR_TCIE; // enable full transfer event
	
	DMA1_Channel5->CCR &= ~DMA_CCR_MSIZE; // memory size
	//DMA1_Channel5->CCR |= DMA_CCR_MSIZE_1;
	DMA1_Channel5->CCR &= ~DMA_CCR_PSIZE; // peripheral size
	DMA1_Channel5->CCR |= DMA_CCR_PSIZE_1;
	
	TIM2->CNT = 0;
	
	/*
	// FOR DEGUB
	TIM2->DIER |= TIM_DIER_CC1IE; // enable capture compare interrupt
	TIM2->DIER |= TIM_DIER_UIE; // enable DMA update interrupt
	NVIC_EnableIRQ(TIM2_IRQn); // enable interrupt
	NVIC_SetPriority(TIM2_IRQn, 0); // set priority
	*/
	
	DMA1_Channel5->CCR |= DMA_CCR_EN; // enable channel
	TIM2->CCER |= TIM_CCER_CC1E; // enable channel 1
	TIM2->CR1 |= TIM_CR1_CEN; // enable timer

	is_updating = 1;
	
	while (is_updating);
	
	//printf("led_update - done\n");
}

void DMA1_Channel5_IRQHandler(void) {
    if ((DMA1->ISR & DMA_ISR_TCIF5) == DMA_ISR_TCIF5) { // check if TC
        TIM2->CR1 &= ~TIM_CR1_CEN; // disable timer
				TIM2->CCER &= ~TIM_CCER_CC1E; // disable channel 1
				DMA1_Channel5->CCR &= ~DMA_CCR_EN; // disable channel
			
				DMA1->IFCR |= DMA_IFCR_CTCIF5; // clear flag
        DMA1->IFCR &= ~DMA_IFCR_CTCIF5; // clear the clear flag
			
				is_updating = 0;
    } 
}

void led_init(void) {
		//printf("led_init\n");
		// using gpioa pa5, tim2ch1, dma1ch5
	
		// RCC

		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // enable gpioa clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // enable tim 2
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // enable dma clock
	
		// GPIO

		GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5; // clear AFRL5
		GPIOA->AFR[0] |= GPIO_AFRL_AFSEL5_0; // set AFRL5 to af1
		GPIOA->MODER &= ~GPIO_MODER_MODE5; // clear mode
		GPIOA->MODER |= GPIO_MODER_MODE5_1; // set to alt function
		GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5; // set to no pullup no pulldown
		GPIOA->OTYPER &= ~GPIO_OTYPER_OT5; // set output type to pushpull
		GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR5; // set to low

		// DMA

		DMA1_Channel5->CCR &= ~DMA_CCR_EN; // disable channel	
	
		DMA1_CSELR->CSELR &= ~DMA_CSELR_C5S; // clear DMA 5 selection
		DMA1_CSELR->CSELR |= ( 1 << 18 ) & DMA_CSELR_C5S; // set DMA 5 to tim2ch1
	
		DMA1_Channel5->CCR |= DMA_CCR_DIR; // set to read from memory
		
		DMA1_Channel5->CCR |= DMA_CCR_PL; // set priority to very high
		
		//DMA1_Channel5->CCR |= DMA_CCR_CIRC; //  circular mode
		
		//DMA1_Channel5->CCR |= DMA_CCR_PINC; // peripheral increment
		
		DMA1_Channel5->CCR |= DMA_CCR_MINC; // enable mecmory increment
		
		//DMA1_Channel5->CCR &= ~DMA_CCR_PSIZE; // clear peripheral size
		//DMA1_Channel5->CCR &= ~DMA_CCR_MSIZE; // clear memory size

		DMA1_Channel5->CPAR &= ~DMA_CPAR_PA; // clear peripheral address
		DMA1_Channel5->CPAR |= (uint32_t)&TIM2->CCR1; // set peripheral address to tim2

		NVIC_EnableIRQ(DMA1_Channel5_IRQn); // enable interrupt
		NVIC_SetPriority(DMA1_Channel5_IRQn, 0); // set priority
	
		// TIM
		
		TIM2->CR1 &= ~TIM_CR1_CEN; // disable counter
		TIM2->CR1 &= ~TIM_CR1_DIR; // set to upcounter
		TIM2->ARR = ARR_VAL; // set ARR
		TIM2->PSC = PSC_VAL; // set prescaler
		
		TIM2->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // output compare to PWM mode 1
		
		TIM2->DIER |= TIM_DIER_CC1DE; // enable DMA
	
		TIM2->CR1 &= ~TIM_CR1_CKD; // disable clock division
		//TIM2->CR2 &= ~TIM_CR2_MMS; // set UG to trigger output
		TIM2->CR1 |= TIM_CR1_ARPE; // auto reload preload enable
		
		TIM2->CCER &= ~TIM_CCER_CC1P; // enable active high output polarity
		TIM2->CCER |= TIM_CCER_CC1E; // enable capture compare
		
		TIM2->CCR1 = 0; // clear capture compare
		//TIM2->CCR1 |= 4; // set capture compare to 4
		
		//TIM2->CCMR1 &= ~TIM_CCMR1_OC1FE; // disable fast output
		//TIM2->CCMR1 &= ~TIM_CCMR1_OC1M; // clear output compare
		
		//TIM2->DIER |= TIM_DIER_UDE; // update DMA enable
		//TIM2->DIER |= TIM_DIER_TDE; // trigger request enable
		
		//TIM2->DCR &= ~TIM_DCR_DBA; // clear address offset 
		//TIM2->DCR |= 0x34 >> 2; // set offset to ccr1
		//printf("address: %d\n", (uint32_t)&TIM2->CCR1 - (uint32_t)TIM2_BASE);
		//TIM2->DCR |= TIM_DCR_DBL_1; // DMA burst for 3 transfers
		
		//TIM2->EGR |= TIM_EGR_UG; // update generation
		
		// FOR DEGUB
		//NVIC_EnableIRQ(TIM2_IRQn); // enable interrupt
		//NVIC_SetPriority(TIM2_IRQn, 0); // set priority
}





/*
void TIM2_IRQHandler(void) {
	//printf("interrupt\n");
	if ((TIM2->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF) {
		printf("cc1\n");
		TIM2->SR &= ~TIM_SR_CC1IF;
		
	}
	else if ((TIM2->SR & TIM_SR_UIF) == TIM_SR_UIF) {
		printf("uif\n");
		TIM2->SR &= ~TIM_SR_UIF;
		
	}
}*/

/*

// type 1 to wait until complete
void led_update(uint8_t type) {
	printf("led_update\n");
	if (is_updating) {
		return;
	}
	is_updating = 1;
	
	led_start_reset_pulse(1);
	
	if (type) {
		while (is_updating);
	}
}

// type - 1 for start pulse, 2 for end pulse 
void led_start_reset_pulse(uint8_t type) {
	
	//printf("led_start_reset_pulse\n");
	is_reset_pulse = type;

	memset(led_data_buffer, 0, sizeof(led_data_buffer));
	
	// set the first bits to timing high
	if (type == 1) {
		led_data_buffer[0] = TIM2->ARR / 2;
	}
	DMA1_Channel5->CCR &= ~DMA_CCR_EN; // disable channel
	DMA1_Channel5->CCR &= ~DMA_CCR_CIRC; // disable circular mode
	
	DMA1_Channel5->CMAR = (uint32_t)led_data_buffer; // set the buffer to data
	DMA1_Channel5->CNDTR = 10; // send 10 bytes of zero after high
	
	DMA1->IFCR |= DMA_IFCR_CHTIF5; // clear flag
	DMA1->IFCR &= ~DMA_IFCR_CHTIF5; // clear the clear flag
	DMA1->IFCR |= DMA_IFCR_CTCIF5; // clear flag
  DMA1->IFCR &= ~DMA_IFCR_CTCIF5; // clear the clear flag
	
	DMA1_Channel5->CCR &= ~DMA_CCR_HTIE; // disable half transfer event
	DMA1_Channel5->CCR |= DMA_CCR_TCIE; // enable full transfer event
	
	DMA1_Channel5->CCR |= DMA_CCR_EN; // enable channel
	TIM2->CCER |= TIM_CCER_CC1E; // enable channel 1
	TIM2->CR1 |= TIM_CR1_CEN; // enable timer
}

// type is 0 for half transfer, 1 for full transfer
void led_update_sequence(uint8_t type) {
	
	
	// FOR DEGUBBBB
	if (num_capture_comp_bits > num_capture_comp_bits_last) {
		printf("capture comp %d\n", num_capture_comp_bits);
		num_capture_comp_bits_last = num_capture_comp_bits;
	}
	
	
	
	//printf("update: %d, %d\n", type, is_reset_pulse);
	
	if (is_reset_pulse == 2) {
		TIM2->CR1 &= ~TIM_CR1_CEN; // disable timer
		TIM2->CCER &= ~TIM_CCER_CC1E; // disable channel 1
		DMA1_Channel5->CCR &= ~DMA_CCR_EN; // disable channel
		// disable dma stream?
		
		is_updating = 0;
		return;
	}
	else if (is_reset_pulse == 1) {
		//printf("bytes left: %d\n", DMA1_Channel5->CNDTR);
		if (!type) {
			return;
		}
		TIM2->CR1 &= ~TIM_CR1_CEN; // disable timer
		TIM2->CCER &= ~TIM_CCER_CC1E; // disable channel 1
		DMA1_Channel5->CCR &= ~DMA_CCR_EN; // disable channel
		// disable dma stream?
		
		is_reset_pulse = 0;
		current_led = 0;
	}
	else {
		//printf("current led:%d\n", current_led);
		current_led ++;
	}
	
	if (current_led < NUM_LEDS) {
		// fill buffer
		if (current_led == 0 || type == 0) {
			fill_led_buffer(current_led, &led_data_buffer[0]);
		}
		else {
			fill_led_buffer(current_led, &led_data_buffer[RAW_LED_BYTES]);
		}
		
		// start transfer
		if (current_led == 0) {
			current_led++;
			fill_led_buffer(current_led, &led_data_buffer[RAW_LED_BYTES]);
			
			DMA1_Channel5->CCR &= ~DMA_CCR_EN; // disable channel
			DMA1_Channel5->CCR |= DMA_CCR_CIRC; // enable circular mode
			DMA1_Channel5->CMAR = (uint32_t)led_data_buffer; // set the buffer to data
			DMA1_Channel5->CNDTR = 2*RAW_LED_BYTES; // set amount of data to trasmit
			
			DMA1->IFCR |= DMA_IFCR_CHTIF5; // clear flag
			DMA1->IFCR &= ~DMA_IFCR_CHTIF5; // clear the clear flag
			DMA1->IFCR |= DMA_IFCR_CTCIF5; // clear flag
      DMA1->IFCR &= ~DMA_IFCR_CTCIF5; // clear the clear flag
			
			DMA1_Channel5->CCR |= DMA_CCR_HTIE; // enable half transfer event
			DMA1_Channel5->CCR |= DMA_CCR_EN; // enable channel
			
			TIM2->CR1 |= TIM_CR1_CEN; // enable timer
			TIM2->CCER |= TIM_CCER_CC1E; // enable channel 1
			
		}
	
	}
	
	// if the transfer is complete, disable timer, send reset bits
	else if (type) {
		//printf("transfer complete: %d\n", current_led);
		TIM2->CCER &= ~TIM_CCER_CC1E; // disable timer channel 1
		DMA1_Channel5->CCR &= ~DMA_CCR_EN; // disable DMA channel
		
		led_start_reset_pulse(2);
	}
}

void fill_led_buffer(uint8_t current_led, uint32_t* buffer_pointer) {
		//printf("fill_led_buffer\n");
		uint16_t arr = TIM2->ARR;
    if (current_led < NUM_LEDS) {
				//printf("capture comp %d\n", num_capture_comp_bits);
				for (int i = 0; i < 24; i++) {
						//printf("%d %d\n", LED_BYTES * current_led + (i/8), (led_colors[LED_BYTES * current_led + (i/8)] & (1 << (7 - i/8))) ? (2 * arr / 3) : (arr / 3));
						buffer_pointer[i] = (led_colors[LED_BYTES * current_led + (i/8)] & (1 << (7 - i/8))) ? (2 * arr / 3) : (arr / 3);
	
					
						//// DEGUB STUFF DELETE
						if (debug_var_index < 3) {
							debug_var[debug_var_index] = buffer_pointer[i];
							debug_var_index++;
						}
						num_bits_queued++;
				}
    }
}
*/

