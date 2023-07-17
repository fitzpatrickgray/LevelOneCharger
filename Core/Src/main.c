#include "main.h"

//ADC_HandleTypeDef hadc;
//TIM_HandleTypeDef htim1;
//UART_HandleTypeDef huart1;
//DMA_HandleTypeDef hdma_usart1_tx;
//WWDG_HandleTypeDef hwwdg;

enum state{
	IDLE,
	CONNECTED,
	CHARGING,
	ERROR
};

uint8_t prevState = IDLE;
uint8_t state = IDLE;

uint16_t len = 8;

uint8_t open[] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x01, 0x0A, 0x48};		// Relay Open
uint8_t close[] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x02, 0x0B, 0x08};		// Relay Close

uint16_t voltage = 0;

#define OPEN_RELAY (DMA1_Channel2->CCR |= DMA_CCR_EN)
#define CLOSE_RELAY (DMA1_Channel3->CCR |= DMA_CCR_EN)

void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART1_UART_Init(void);
static void DMA_Init(void);
static void ADC_Init(void);
static void TIM3_Init(void);
static void TIM2_Init(void);
static void ping(void);



//static void WWDG_Init(void);

int main(void)
{

  HAL_Init();

  SystemClock_Config();
  GPIO_Init();
  USART1_UART_Init();
  DMA_Init();
  ADC_Init();
  TIM3_Init();
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_EnableIRQ(ADC1_IRQn);
//  WWDG_Init();
	while (1)
	{
		if(state == IDLE && prevState != IDLE)
		{

		}
		else if(state == CONNECTED && prevState != CONNECTED)
		{

		}
		else if(state == CHARGING && prevState != CHARGING)
		{

		}
		else if(state == ERROR && prevState != ERROR)
		{

		}
	}
}

void DMA1_Channel2_3_IRQHandler(void)
{
	if((DMA1_Channel2->CCR & DMA_CCR_EN) != 0)
	{
		DMA1_Channel2->CCR &= ~(DMA_CCR_EN);
		DMA1->IFCR |= DMA_IFCR_CTCIF2;
	}
	else if((DMA1_Channel3->CCR & DMA_CCR_EN) != 0)
	{
		DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
	}
}

void TIM3_IRQHandler(void)
{
	if(TIM3->SR && TIM_SR_UIF)
	{
		ADC1->CR |= ADC_CR_ADSTART;
	}
}

void ADC1_IRQHandler(void)
{
	if(TIM3->CNT < 200) // Detect if on PEAK or TROPH of CP Signal
	{

		voltage = ADC1->DR;
		if(998 < voltage <= 1024) //
		{
			state = IDLE;
		}
		else if(868 < voltage <= 921)
		{
			state = CONNECTED;
		}
		else if(745 < voltage <= 791)
		{
			state = CHARGING;
		}
		else
		{
			// Add Error Handler Later On
			state = ERROR;
		}
	}
	else
	{
		if(voltage > 25)
		{
			state = ERROR;
		}
	}
}

void ping(void)
{
	GPIOA->BSRR |= GPIO_BSRR_BS_0;
	for(int i = 0; i <= 20; i++);
	GPIOA->BSRR |= GPIO_BSRR_BR_0;
	TIM3->SR &= ~(TIM_SR_UIF);
}

void SystemClock_Config(void)
{
	// System Clocks, configuration, 8 MHz, and 14 MHz
	RCC->CFGR &= ~(RCC_CFGR_SW | RCC_CFGR_PPRE | RCC_CFGR_HPRE);
	RCC->CR |= RCC_CR_HSION;
	RCC->CR2 |= RCC_CR2_HSI14ON;

	// Peripheral Clocks, enable GPIO, enable (USART, ADC), use HSI for USART
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_DMAEN;
	RCC->CFGR3 |= RCC_CFGR3_USART1SW_HSI;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_ADCEN;
	// Enable Timer 3 (CP Ctrl) & Timer 2 (TIMEOUT)
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN;

//	RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;

	//Calibration
//	RCC->CR |= RCC_CR2_HSI14CAL
}

static void GPIO_Init(void)
{
	/*			           ___
	 *			    PA9	  |___|   VIN
	 * 			    PA10     	  GND
	 * 			    NRST          NRST
	 * 			    GND	          +5V
	 * 			    PA12          PA2 --- USART1_TX
	 * 			    PB0	          PA7 --- USART1_DE
	 * 			    PB7		  PA6
	 * 			    PB6           PA5
	 *             ADC_IN9 ---- PB1    ___    PA4
	 * 			    PF0	  |   |   PA3
	 * 			    PF1	  |___|   PA1
	 * 			    PA8	     	  PA0 --- GPIO_Test
	 * 			    PA11	  AREF
	 * 			    PB5	          +3V
	 *             TIM3_CH1 --- PB4	          PB3
	 */

	// Unlock GPIO
	GPIOA->LCKR &= ~(GPIO_LCKR_LCK2 | GPIO_LCKR_LCK1 | GPIO_LCKR_LCK0);
	GPIOB->LCKR &= ~(GPIO_LCKR_LCK4);

	// USART DE & TX
	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER1_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR2 | GPIO_OSPEEDR_OSPEEDR1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR1_0;
	GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFRL2_Pos) | (1 << GPIO_AFRL_AFRL1_Pos);

	// TIM3_CH1 & ADC_IN9 Setup
	GPIOB->MODER |= GPIO_MODER_MODER4_1; // Setup PB4 as Alternate Function
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR4 || GPIO_OSPEEDR_OSPEEDR1;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4_1);
	GPIOB->AFR[0] |= (1 << GPIO_AFRL_AFRL4_Pos);

	// Random GPIO for testing
	GPIOA->MODER |= GPIO_MODER_MODER0_0; // General purpose output
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR0;


	// Lock GPIO
	GPIOA->LCKR |= GPIO_LCKR_LCK2 | GPIO_LCKR_LCK1;
	GPIOB->LCKR |= GPIO_LCKR_LCK4;
}

static void USART1_UART_Init(void)
{
	// DISABLE
	USART1->CR1 &= ~(USART_CR1_UE);

	USART1->CR1 &= ~(USART_CR1_OVER8 | USART_CR1_PS); // ---------------------------------------------- Oversampling and Parity Selection
	USART1->CR1 |= USART_CR1_DEAT_0 | USART_CR1_DEDT_0 | USART_CR1_TE | USART_CR1_PCE | USART_CR1_M; // Enable TX, Parity, M[0] Word Length
	USART1->CR2 &= ~(USART_CR2_STOP); // -------------------------------------------------------------- Stop Bit #
	USART1->CR3 &= ~(USART_CR3_DEP); //                                                                 RS-485 DE Polarity
	USART1->CR3 |= USART_CR3_DEM | USART_CR3_DMAT; // ------------------------------------------------- Enable the RS-485 DE and DMA Transmit
	USART1->BRR = 0x1A1; //                                                                             19200 Baud

	// ENABLE
	USART1->CR1 |= USART_CR1_UE;
}

static void DMA_Init(void)
{
	// DISABLE
	DMA1_Channel2->CCR &= ~(DMA_CCR_EN);
	DMA1_Channel3->CCR &= ~(DMA_CCR_EN);

	DMA1_Channel2->CCR &= ~(DMA_CCR_MEM2MEM | DMA_CCR_MSIZE | DMA_CCR_PSIZE | DMA_CCR_PINC); // Memory to Peripheral Sel, Memory Size, Peripheral Size, Peripheral Increment Enable
	DMA1_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_TCIE; // -------- Circular, Direction, Transmit Complete Interrupt Enable
	DMA1_Channel2->CPAR = (uint32_t)&(USART1->TDR); //                                          Peripheral Address = USART Transmit Data Register
	DMA1_Channel2->CMAR = (uint32_t)open; // -------------------------------------------------- Memory Address = Open[] Message;
	DMA1_Channel2->CNDTR = DMA_CNDTR_NDT & len; //                                              Length of Data len(Open)

	// Same Sauce as DMA Channel 2 except Memory points to Close[] message
	DMA1_Channel3->CCR &= ~(DMA_CCR_MEM2MEM | DMA_CCR_MSIZE | DMA_CCR_PSIZE | DMA_CCR_PINC);
	DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_TCIE;
	DMA1_Channel3->CPAR = (uint32_t)&(USART1->TDR);
	DMA1_Channel3->CMAR = (uint32_t)close;
	DMA1_Channel3->CNDTR = DMA_CNDTR_NDT & len;

	// ENABLE
	DMA1_Channel2->CCR |= DMA_CCR_EN;
//	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

static void TIM3_Init(void)
{
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER &= ~(TIM_CCER_CC1P);

	TIM3->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_OPM | TIM_CR1_CKD | TIM_CR1_UDIS); // Upcounting Direction, Disable One Pulse Mode, No Clock Division, Restrict Updates
	TIM3->CR1 |= TIM_CR1_CMS | TIM_CR1_ARPE | TIM_CR1_URS; // ---------------- Center-Aligned Mode 3, Update Request Source, Auto Reload Preload Enable
	TIM3->DIER |= TIM_DIER_UIE;// TIM_DIER_CC1IE; //                                           Enable Capture/Compare Ch. 1 Interrupt
	TIM3->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) | TIM_CCMR1_OC1PE; // ----------- Enable PWM Mode and Output Compare Preload
	TIM3->CR2 |= TIM_CR2_MMS_1; //                                             Trigger for ADC Capture

	TIM3->PSC = 3;
	TIM3->ARR = 999;
	TIM3->CCR1 = 200;


	TIM3->EGR |= TIM_EGR_UG;  // Update Register Values
	TIM3->CR1 |= TIM_CR1_CEN; // Clock Enable
}

static void TIM2_Init(void)
{

}


static void ADC_Init(void)
{

	ADC1->CR |= ADC_CR_ADSTP;
	while ((ADC1->CR & ADC_CR_ADSTP) != 0)
	{

	}
	ADC1->CR |= ADC_CR_ADDIS;
	while ((ADC1->CR & ADC_CR_ADDIS) != 0)
	{

	}

	ADC1->CFGR2 &= ~(ADC_CFGR2_CKMODE);
	ADC1->IER |= ADC_IER_EOCIE; // ADC End of Completion & ADC Ready

	ADC1->CFGR1 &= ~(ADC_CFGR1_CONT | ADC_CFGR1_DISCEN | ADC_CFGR1_RES); // Not Continuous or Discontinuous & 12 bit Resolution
	ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0 | ADC_CFGR1_EXTSEL_1 | ADC_CFGR1_EXTSEL_0; // Hardware Detection on Rising Edge & External Select Trigger 3
	ADC1->CHSELR |= ADC_CHSELR_CHSEL9; // Channel 9 Selection


	ADC1->CR &= ~(ADC_CR_ADSTP | ADC_CR_ADDIS);

	ADC1->CR |= ADC_CR_ADCAL;
	while((ADC1->CR & ADC_CR_ADCAL) != 0)
	{

	}

	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0)
	{

	}

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

