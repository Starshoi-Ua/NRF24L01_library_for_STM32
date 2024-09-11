#include "NRF24.h"
//------------------------------------------------
extern SPI_HandleTypeDef hspi2;
//------------------------------------------------
#define TX_ADR_WIDTH 3
#define TX_PLOAD_WIDTH 3
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xb3,0xb4,0x01};
uint8_t RX_BUF[TX_PLOAD_WIDTH] = {0};
//------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
	 micros *= (SystemCoreClock / 1000000) / 9;
	 /* Wait till done*/
	 while (micros--) ;
}
//---------------------------------------
uint8_t NRF24_ReadReg(uint8_t addr)
{
 uint8_t dt=0, cmd;
 CS_ON;
 HAL_SPI_TransmitReceive(&hspi2, &addr, &dt, 1, 1000);
 if (addr!=STATUS)
 {
	 	 cmd=0xFF;
	 	 HAL_SPI_TransmitReceive(&hspi2, &cmd, &dt, 1, 1000);
 }
 CS_OFF;
 return dt;
}
//------------------------------------------------
void NRF24_WriteReg(uint8_t addr, uint8_t dt)
{
	addr |= W_REGISTER;
	CS_ON;
	HAL_SPI_Transmit(&hspi2,&addr,1,1000);
	HAL_SPI_Transmit(&hspi2,&dt,1,1000);
	CS_OFF;
}
//------------------------------------------------
void NRF24_ToggleFeatures(void)
{
	uint8_t dt[1] = {ACTIVATE};
	CS_ON;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);
	DelayMicro(1);
	dt[0] = 0x73;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);
	CS_OFF;
}
//-----------------------------------------------
void NRF24_Read_Buf(uint8_t addr, uint8_t *pBuf, uint8_t bytes)
{
	CS_ON;
	HAL_SPI_Transmit(&hspi2,&addr,1,1000);
	HAL_SPI_Receive(&hspi2,pBuf,bytes,1000);
	CS_OFF;
}
//------------------------------------------------
void NRF24_Write_Buf(uint8_t addr, uint8_t*pBuf, uint8_t bytes)
{
	addr |= W_REGISTER;
	CS_ON;
	HAL_SPI_Transmit(&hspi2,&addr,1,1000);
	DelayMicro(1);
	HAL_SPI_Transmit(&hspi2,pBuf,bytes,1000);
	CS_OFF;
}
//------------------------------------------------
void NRF24_FlushRX(void)
{
	uint8_t dt[1] = {FLUSH_RX};
	CS_ON;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);
	DelayMicro(1);
	CS_OFF;
}
//------------------------------------------------
void NRF24_FlushTX(void)
{
	uint8_t dt[1] = {FLUSH_TX};
	CS_ON;
	HAL_SPI_Transmit(&hspi2,dt,1,1000);
	DelayMicro(1);
	CS_OFF;
}
//------------------------------------------------
void NRF24L01_RX_Mode(void)
{
	//NRF24_FlushRX(); // Flush buffers
	uint8_t regval=0x00;
	regval = NRF24_ReadReg(CONFIG);
	regval |= (1<<PRIM_RX);
	NRF24_WriteReg(CONFIG, regval); // PRIM_RX on
	CE_SET;
	HAL_Delay(1);
}
//------------------------------------------------
void NRF24L01_TX_Mode(void)
{
	//NRF24_FlushRX(); // Flush buffers
	uint8_t regval=0x00;
	regval = NRF24_ReadReg(CONFIG);
	regval &= ~(1<<PRIM_RX);
	NRF24_WriteReg(CONFIG, regval); // PRIM_RX off
	CE_SET;
	//HAL_Delay(1);
	//CE_RESET;
}
//------------------------------------------------
void NRF24L01_Transmit(uint8_t *pBuf, uint8_t bytes)
{
	// Writing data to tx pload
	NRF24_Write_Buf(WR_TX_PLOAD, pBuf, bytes);
	//entering tx mode
	NRF24L01_TX_Mode();
	//sending checking
	uint8_t status=0x00;
	while((GPIO_PinState)IRQ == GPIO_PIN_SET) {}
	status = NRF24_ReadReg(STATUS);
	if(status&TX_DS) //tx_ds == 0x20
	{
		LED_TGL;
		NRF24_WriteReg(STATUS, 0x20);
	}
	else
	if(status&MAX_RT)
	{
		NRF24_WriteReg(STATUS, 0x10);
		NRF24_FlushTX();
	}
	CE_RESET;
}
//------------------------------------------------
void NRF24L01_Receive(uint8_t *pBuf)
{
	NRF24L01_RX_Mode();
	uint8_t status=0x00;
	while((GPIO_PinState)IRQ == GPIO_PIN_SET) {}
	status = NRF24_ReadReg(STATUS);
	if(status & 0x40)
	{
		NRF24_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);
		for(int i = 0; i < TX_PLOAD_WIDTH; i++)
			pBuf[i] = RX_BUF[i];
		NRF24_WriteReg(STATUS, 0x40);
		LED_TGL;
	}
	CE_RESET;
}
//------------------------------------------------
void NRF24_init(void)
{
	HAL_Delay(11);
	NRF24_WriteReg(CONFIG, 0x02); // Set PWR_UP bit
	HAL_Delay(2);
	NRF24_WriteReg(CONFIG, 0x0b);  // Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:1 (Receiver)
	NRF24_WriteReg(EN_RXADDR, 0x02); // data pipe 1 enable
	NRF24_WriteReg(EN_AA, 0x00); // pipe no auto ankowledgement
	NRF24_WriteReg(SETUP_AW, 0x01); // setup address width =3 bytes
	NRF24_WriteReg(SETUP_RETR, 0x5F);  // // 1500us, 15 retransmits
	NRF24_WriteReg(STATUS, 0x70); //Reset flags for IRQ
	NRF24_WriteReg(RF_CH, 75); // frequency 2475MHz
	NRF24_WriteReg(RF_SETUP, 0x04); //TX_PWR:-6dBm, Datarate:1Mbps ////
	NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
	NRF24_Write_Buf(RX_ADDR_P1, TX_ADDRESS, TX_ADR_WIDTH);
	NRF24_WriteReg(RX_PW_P1, TX_PLOAD_WIDTH); //Number of bytes in RX payload in data pipe 1
}
//---------------------------------------
