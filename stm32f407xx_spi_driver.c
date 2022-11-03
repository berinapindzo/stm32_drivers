
#include <stm32f407xx_spi_driver.h>

// peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}


	} else
		if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
		else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
		else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}

}



// initialization

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	// peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// configure SPI_CR1 register

	uint32_t tempreg = 0;

	// 1. configure device mode

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// 2. configure bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode should be cleared
		tempreg &= ~(1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode should be set
		tempreg |= (1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
		{
		// BIDI mode should be cleared and RXONLY must be set
		tempreg &= ~(1 << 15);
		tempreg |= (1 << 10);
		}


	// 3. configure SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	// 4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	// 5. configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	// 6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;



}
void SPI_DeInit(SPI_RegDef_t *pSPIx);


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName) return FLAG_SET;
	return FLAG_RESET;
}



// data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// 1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. check DFF
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// 1. load data in DR register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);



void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}
		else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
			{
				pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
			}
			else
			{
				pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
			}
}

// IRQ configuration and IRQ handling

void SPI_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

