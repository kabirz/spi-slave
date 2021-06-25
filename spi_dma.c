#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_pwr.h"
#include "stm32f7xx_ll_dma.h"

uint8_t ubButtonPress = 0;

uint8_t aTxBuffer[] = "**** SPI_TwoBoards_FullDuplex_DMA communication **** SPI_TwoBoards_FullDuplex_DMA communication **** SPI_TwoBoards_FullDuplex_DMA communication ****";
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
uint8_t ubTransmissionComplete = 0;

uint8_t aRxBuffer[sizeof(aTxBuffer)];
uint8_t ubNbDataToReceive = sizeof(aTxBuffer);
uint8_t ubReceptionComplete = 0;

#define LED_PORT                GPIOB
#define LED_PIN                 LL_GPIO_PIN_0
#define LED_PORT_CLK_ENABLE()	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)

void     SystemClock_Config(void);
void     Configure_DMA(void);
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

	/* Configure DMA channels for the SPI1  */
	Configure_DMA();

	/* Enable the SPI1 peripheral */
	Activate_SPI();

	WaitAndCheckEndOfTransfer();

	for (;;)
		__WFI();
}

void Configure_DMA(void)
{
	/* DMA2 used for SPI1 Transmission
	 * DMA2 used for SPI1 Reception
	 */
	/* (1) Enable the clock of DMA2 and DMA2 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	/* (2) Configure NVIC for DMA transfer complete/error interrupts */
	NVIC_SetPriority(DMA2_Stream2_IRQn, 0);
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	NVIC_SetPriority(DMA2_Stream3_IRQn, 0);
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);

	/* (3) Configure the DMA2_Stream2 functional parameters */
	LL_DMA_ConfigTransfer(DMA2,
			LL_DMA_STREAM_2,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
			LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
			LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA2,
			LL_DMA_STREAM_2,
			LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)aRxBuffer,
			LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_2));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, ubNbDataToReceive);


	LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_3);

	/* (4) Configure the DMA2_Stream3 functional parameters */
	LL_DMA_ConfigTransfer(DMA2,
			LL_DMA_STREAM_3,
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
			LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
			LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_3, (uint32_t)aTxBuffer, LL_SPI_DMA_GetRegAddr(SPI1),
			LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_3));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, ubNbDataToTransmit);

	LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_3, LL_DMA_CHANNEL_3);

	/* (5) Enable DMA interrupts complete/error */
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_2);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_3);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_3);
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

	/* Configure SPI1 DMA transfer interrupts */
	/* Enable DMA RX Interrupt */
	LL_SPI_EnableDMAReq_RX(SPI1);
	/* Enable DMA TX Interrupt */
	LL_SPI_EnableDMAReq_TX(SPI1);
}

void Activate_SPI(void)
{
	/* Enable SPI1 */
	LL_SPI_Enable(SPI1);
	/* Enable DMA Channels */
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_3);
}

void WaitAndCheckEndOfTransfer(void)
{
	/* 1 - Wait end of transmission */
	while (ubTransmissionComplete != 1)
	{
	}
	/* Disable DMA2 Tx Channel */
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
	/* 2 - Wait end of reception */
	while (ubReceptionComplete != 1)
	{
	}
	/* Disable DMA2 Rx Channel */
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
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

void DMA2_ReceiveComplete_Callback(void)
{
	ubReceptionComplete = 1;
}

void DMA2_TransmitComplete_Callback(void)
{
	ubTransmissionComplete = 1;
}

void SPI1_TransferError_Callback(void)
{
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);

	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
}

static void CPU_CACHE_Enable(void)
{
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

