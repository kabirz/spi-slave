#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_pwr.h"

uint8_t ubButtonPress = 0;

uint8_t aTxBuffer[] = "**** SPI_TwoBoards_FullDuplex_IT communication **** SPI_TwoBoards_FullDuplex_IT communication **** SPI_TwoBoards_FullDuplex_IT communication ****";
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
uint8_t ubTransmitIndex = 0;

uint8_t aRxBuffer[sizeof(aTxBuffer)];
uint8_t ubNbDataToReceive = sizeof(aTxBuffer);
uint8_t ubReceiveIndex = 0;

#define LED_PORT                GPIOB
#define LED_PIN                 LL_GPIO_PIN_0
#define LED_PORT_CLK_ENABLE()	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)

void     SystemClock_Config(void);
void     Configure_SPI(void);
void     Activate_SPI(void);
void     WaitAndCheckEndOfTransfer(void);
uint8_t  Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);
static void CPU_CACHE_Enable(void);

void SysTick_Handler(void)
{
	static int counter = 0;
	counter++;

	// 1 Hz blinking
	if ((counter % 500) == 0)
		LL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

void initGPIO()
{
	LED_PORT_CLK_ENABLE();

	LL_GPIO_SetPinMode(LED_PORT, LED_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(LED_PORT, LED_PIN, LL_GPIO_OUTPUT_PUSHPULL);
}


int main(void)
{
	/* Enable the CPU Cache */
	CPU_CACHE_Enable();
	/* Configure the system clock to 216 MHz */
	//SystemClock_Config();
	SystemCoreClockUpdate();
	// SysTick_Config(SystemCoreClock / 1000);
	initGPIO();

	/* Configure the SPI1 parameters */
	Configure_SPI();

	/* Enable the SPI1 peripheral */
	Activate_SPI();

	WaitAndCheckEndOfTransfer();

	for (;;)
		__WFI();
}

void Configure_SPI(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_5);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_DOWN);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_DOWN);

	/* (2) Configure NVIC for SPI1 transfer complete/error interrupts **********/
	/* Set priority for SPI1_IRQn */
	NVIC_SetPriority(SPI1_IRQn, 0);
	/* Enable SPI1_IRQn           */
	NVIC_EnableIRQ(SPI1_IRQn);

	/* (3) Configure SPI1 functional parameters ********************************/

	/* Enable the peripheral clock of GPIOA */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

	/* Configure SPI1 communication */
	// LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV8);
	LL_SPI_SetTransferDirection(SPI1,LL_SPI_FULL_DUPLEX);
	LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
	LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);
	/* Reset value is LL_SPI_MSB_FIRST */
	//LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
	LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
	LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

	/* Configure SPI1 transfer interrupts */
	/* Enable RXNE  Interrupt             */
	LL_SPI_EnableIT_RXNE(SPI1);
	/* Enable TXE   Interrupt             */
	LL_SPI_EnableIT_TXE(SPI1);
	/* Enable Error Interrupt             */
	LL_SPI_EnableIT_ERR(SPI1);
}

void Activate_SPI(void)
{
	/* Enable SPI1 */
	LL_SPI_Enable(SPI1);
}

void WaitAndCheckEndOfTransfer(void)
{
	/* 1 - Wait end of transmission */
	while (ubTransmitIndex != ubNbDataToTransmit)
	{
	}
	/* Disable TXE Interrupt */
	LL_SPI_DisableIT_TXE(SPI1);

	/* 2 - Wait end of reception */
	while (ubNbDataToReceive > ubReceiveIndex)
	{
	}
	/* Disable RXNE Interrupt */
	LL_SPI_DisableIT_RXNE(SPI1);

	/* 3 - Compare Transmit data to receive data */
	if(Buffercmp8((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, ubNbDataToTransmit))
	{
		/* Processing Error */
	}
	else
	{
		/* Turn On Led if data are well received */
	}
}


uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
{
	while (BufferLength--)
	{
		if (*pBuffer1 != *pBuffer2)
		{
			return 1;
		}

		pBuffer1++;
		pBuffer2++;
	}

	return 0;
}


void SystemClock_Config(void)
{
	/* Enable HSE clock */
	// LL_RCC_HSE_EnableBypass();
	LL_RCC_HSE_Enable();
	while(LL_RCC_HSE_IsReady() != 1)
	{
	};

	/* Set FLASH latency */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_7);

	/* Enable PWR clock */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* Activation OverDrive Mode */
	LL_PWR_EnableOverDriveMode();
	while(LL_PWR_IsActiveFlag_OD() != 1)
	{
	};

	/* Activation OverDrive Switching */
	LL_PWR_EnableOverDriveSwitching();
	while(LL_PWR_IsActiveFlag_ODSW() != 1)
	{
	};

	/* Main PLL configuration and activation */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 432, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();
	while(LL_RCC_PLL_IsReady() != 1)
	{
	};

	/* Sysclk activation on the main PLL */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{
	};

	/* Set APB1 & APB2 prescaler */
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

	/* Set systick to 1ms */
	SysTick_Config(216000000 / 1000);

	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	SystemCoreClock = 216000000;
}


void  SPI1_Rx_Callback(void)
{
	aRxBuffer[ubReceiveIndex++] = LL_SPI_ReceiveData8(SPI1);
}

void  SPI1_Tx_Callback(void)
{
	LL_SPI_TransmitData8(SPI1, ubTransmitIndex++);
}

void SPI1_TransferError_Callback(void)
{
	/* Disable RXNE  Interrupt             */
	LL_SPI_DisableIT_RXNE(SPI1);

	/* Disable TXE   Interrupt             */
	LL_SPI_DisableIT_TXE(SPI1);

	/* Set LED1 to Blinking mode to indicate error occurs */
}

static void CPU_CACHE_Enable(void)
{
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

void SPI1_IRQHandler(void)
{
  /* Check RXNE flag value in ISR register */
  if(LL_SPI_IsActiveFlag_RXNE(SPI1))
  {
    /* Call function Slave Reception Callback */
    SPI1_Rx_Callback();
    LL_GPIO_TogglePin(LED_PORT, LED_PIN);
  }
  /* Check RXNE flag value in ISR register */
  else if(LL_SPI_IsActiveFlag_TXE(SPI1))
  {
    /* Call function Slave Reception Callback */
    SPI1_Tx_Callback();
    LL_GPIO_TogglePin(LED_PORT, LED_PIN);
  }
  /* Check STOP flag value in ISR register */
  else if(LL_SPI_IsActiveFlag_OVR(SPI1))
  {
    /* Call Error function */
    SPI1_TransferError_Callback();
  }
}
