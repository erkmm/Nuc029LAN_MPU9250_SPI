/**************************************************************************//**
 * @file     main.c
 * @version  V3.0
 * $Revision: 2 $
 * $Date: 14/10/17 8:20p $
 * @brief    NUC029 Series SPI Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC029xAN.h"

#define TEST_COUNT 16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void SPI_Init(void);
void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len);
void mpu9250_write_reg(uint8_t reg, uint8_t data);


/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init SPI */
    SPI_Init();

    int16_t accel_data;
    uint8_t imu_data[14];
    mpu9250_write_reg(28, 0x08);

    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFO_CTL = (2 << SPI_FIFO_CTL_TX_THRESHOLD_Pos) | SPI_FIFO_CTL_TIMEOUT_INTEN_Msk | SPI_FIFO_CTL_TX_INTEN_Msk;
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
   // NVIC_EnableIRQ(SPI0_IRQn);

    while (1)
     {
   		  mpu9250_read_reg(59, imu_data, sizeof(imu_data));
   		  accel_data = ((int16_t)imu_data[0]<<8) + imu_data[1];
   		  printf("data is : %d \n",accel_data);
   		  for(volatile int time=0; time< 0x100000; time++){}
     }
    /* Disable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFO_CTL = 0;
   // NVIC_DisableIRQ(SPI0_IRQn);
    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI0 peripheral clock */
    CLK->APBCLK &= (~CLK_APBCLK_SPI0_EN_Msk);

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable external 12MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Select HXT as the clock source of HCLK */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HXT;

    /* Select HXT as the clock source of UART; select HCLK as the clock source of SPI0. */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~(CLK_CLKSEL1_UART_S_Msk | CLK_CLKSEL1_SPI0_S_Msk))) | (CLK_CLKSEL1_UART_S_HXT | CLK_CLKSEL1_SPI0_S_HCLK);

    /* Enable UART and SPI0 clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_SPI0_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);

    /* Setup SPI0 multi-function pins */
    SYS->P1_MFP &= ~(SYS_MFP_P14_Msk | SYS_MFP_P15_Msk | SYS_MFP_P16_Msk | SYS_MFP_P17_Msk);
    SYS->P1_MFP |= (SYS_MFP_P14_SPISS0 | SYS_MFP_P15_MOSI_0 | SYS_MFP_P16_MISO_0 | SYS_MFP_P17_SPICLK0);
}

void UART0_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LCR = UART_LCR_WLS_Msk;
    /* Using mode 2 calculation: UART bit rate = UART peripheral clock rate / (BRD setting + 2) */
    /* UART peripheral clock rate 12MHz; UART bit rate 115200 bps. */
    /* 12000000 / 115200 bps ~= 104 */
    /* 104 - 2 = 0x66. */
    UART0->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (0x66);
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI0 as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI0->CNTRL = SPI_CNTRL_FIFO_Msk | SPI_MASTER | SPI_CNTRL_TX_NEG_Msk;
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI0->SSR = SPI_SSR_AUTOSS_Msk | SPI_SSR_SSR_Msk;
    /* Set IP clock divider. SPI clock rate = HCLK / ((5+1)*2) = 1MHz */
    SPI0->DIVIDER = (SPI0->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | 5;
}

void SPI0_IRQHandler(void)
{
    /* Check RX EMPTY flag */
    while((SPI0->STATUS & SPI_STATUS_RX_EMPTY_Msk) == 0)
    {
        /* Read RX FIFO */
        g_au32DestinationData[g_u32RxDataCount++] = SPI0->RX0;
    }
    /* Check TX FULL flag and TX data count */
    while(((SPI0->STATUS & SPI_STATUS_TX_FULL_Msk) == 0) && (g_u32TxDataCount < TEST_COUNT))
    {
        /* Write to TX FIFO */
        SPI0->TX0 = g_au32SourceData[g_u32TxDataCount++];
    }
    if(g_u32TxDataCount >= TEST_COUNT)
        SPI0->FIFO_CTL &= (~SPI_FIFO_CTL_TX_INTEN_Msk); /* Disable TX FIFO threshold interrupt */

    /* Check the RX FIFO time-out interrupt flag */
    if(SPI0->STATUS & SPI_STATUS_TIMEOUT_Msk)
    {
        /* If RX FIFO is not empty, read RX FIFO. */
        while((SPI0->STATUS & SPI_STATUS_RX_EMPTY_Msk) == 0)
            g_au32DestinationData[g_u32RxDataCount++] = SPI0->RX0;
    }
}


/*
void SpiFlash_ReadData(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;
    unsigned int Counter;

    // configure transaction length as 8 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI0, 8);
    // /CS: active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI0);

    // send Command: 0x03, Read data
    au32SourceData = 0x03;
    SPI0->TX0[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(24);
    SPI_SET_DATA_WIDTH(SPI0, 24);
    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI0->TX0[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 8 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI0, 8);

    for(Counter = 0; Counter < ByteCount; Counter++)
    {
        // receive
        au32SourceData = 0x0;
        SPI0->TX0[0] = au32SourceData;
        SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

        // wait
        while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

        // dump Rx register
        au32DestinationData = SPI0->RX0[0];
        DataBuffer[Counter] = (unsigned char) au32DestinationData;
    }

    // /CS: de-active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI0);
}*/
//void GetGyroData() {
//    unsigned int GyroAddress = 0x43;
//    volatile int GyroData = 0;
//    unsigned char DataBuffer[6];  // Allocate memory for the buffer
//
//    SPI_SET_DATA_WIDTH(SPI0, 8);
//
//    SPI_SET_SS0_LOW(SPI0);
//
//    for (int count = 0; count < 6; count++) {
//        SPI0->TX0 = GyroAddress + count;
//        SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;
//
//        while (SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}
//
//        GyroData = SPI0->RX0;
//        DataBuffer[count] = (unsigned char) GyroData;
//    }
//
//    SPI_SET_SS0_HIGH(SPI0);
//
//    // Process DataBuffer as needed}

void mpu9250_write_reg(uint8_t reg, uint8_t data)
 {
	SPI_SET_DATA_WIDTH(SPI0, 8);

	SPI_SET_SS_LOW(SPI0);
	SPI0->TX0 = reg;
	SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;
	while (SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

	SPI_SET_DATA_WIDTH(SPI0, 8);

	SPI0->TX0 = data;
	SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;
	while (SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

	//HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	//HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	SPI_SET_SS_HIGH(SPI0);
}
void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
	uint8_t temp_data = 0x80 | reg;
	SPI_SET_DATA_WIDTH(SPI0, 8);
	SPI_SET_SS_LOW(SPI0);

	SPI0->TX0 = temp_data;
	SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;
	while (SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

	SPI_SET_DATA_WIDTH(SPI0, 8);
	for (int count = 0; count < len; count++) {
		SPI0->TX0 = 0x00;
		SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;
		while (SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

		data[count] = SPI0->RX0;
	}

	//HAL_SPI_Transmit(&hspi1, &temp_data, 1, 100);
	//HAL_SPI_Receive(&hspi1, data, len, 100);
	SPI_SET_SS_HIGH(SPI0);
}
