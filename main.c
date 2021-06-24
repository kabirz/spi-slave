#include <stm32f7xx_ll_gpio.h>
#include <stm32f7xx_ll_cortex.h>
#include <stm32f7xx_ll_rcc.h>
#include <stm32f7xx_ll_spi.h>
#include <stm32f7xx_ll_bus.h>

#define LED_PORT                GPIOB
#define LED_PIN                 LL_GPIO_PIN_0
#define LED_PORT_CLK_ENABLE()	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)
LL_SPI_InitTypeDef spi_initstruct;

static void CPU_CACHE_Enable(void)
{
	SCB_EnableICache();
	SCB_EnableDCache();
}

void Configure_SPI1(void)
{
	/* (1) Enables GPIO clock and configures the SPI1 pins ********************/
	/* Enable the peripheral clock of GPIOA */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/* Configure SCK Pin connected to pin 10 of CN7 connector */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);

	/* Configure MOSI Pin connected to pin 14 of CN7 connector */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_DOWN);

	/* (2) Configure SPI1 functional parameters ********************************/
	/* Enable the peripheral clock of GPIOA */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

	/* Configure SPI1 communication */
	spi_initstruct.BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV32;
	spi_initstruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	spi_initstruct.ClockPhase        = LL_SPI_PHASE_1EDGE;
	spi_initstruct.ClockPolarity     = LL_SPI_POLARITY_LOW;
	spi_initstruct.BitOrder          = LL_SPI_MSB_FIRST;
	spi_initstruct.DataWidth         = LL_SPI_DATAWIDTH_8BIT;
	spi_initstruct.NSS               = LL_SPI_NSS_SOFT;
	spi_initstruct.CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE;
	spi_initstruct.Mode              = LL_SPI_MODE_SLAVE;

	/* Initialize SPI instance according to parameters defined in initialization structure. */  
	if (LL_SPI_Init(SPI1, &spi_initstruct) != SUCCESS)
	{
		/* Initialization Error */
	}

	/* Initialize FFIFO Threshold */
	LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

	/* Configure SPI1 DMA transfer interrupts */
	/* Enable DMA TX Interrupt */
	LL_SPI_EnableDMAReq_TX(SPI1);
}



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
	CPU_CACHE_Enable();
	// 1kHz ticks
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
	initGPIO();
	Configure_SPI1();

	for (;;)
		__WFI();

	return 0;
}
