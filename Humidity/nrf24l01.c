#include "nrf24l01.h"

const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xE7,0xE7,0xE7,0xE7,0xE7};


#define cs_low	GPIO_PinOutClear(gpioPortC,Nrf24l01_CSPIN)
#define cs_high	GPIO_PinOutSet(gpioPortC,Nrf24l01_CSPIN)

#define ce_set		GPIO_PinOutSet(gpioPortC,Nrf24l01_CEPIN)
#define ce_reset	GPIO_PinOutClear(gpioPortC,Nrf24l01_CEPIN)

extern bool data_geldi;

uint8_t NRF_Write_reg(uint8_t add,uint8_t* data,uint8_t size)
{
	uint8_t status,cnt=0;;
	cs_low;
	
	delay_();
	
	status=USART_SpiTransfer(USART1,add|0x20);
	
	for(cnt=0;cnt<size;cnt++)
		status=USART_SpiTransfer(USART1,data[cnt]);	
	
	delay_();
	cs_high;

	return status;

}

uint8_t NRF_Write_reg2(uint8_t add,uint8_t data)
{
	uint8_t status;
	cs_low;
	
	delay_();
	
	status=USART_SpiTransfer(USART1,add);
	
	USART_SpiTransfer(USART1,data);	
	
	delay_();
	cs_high;

	return status;

}

void multi_read(uint8_t reg,uint8_t* p,uint8_t size)
{
	uint8_t status=0,rxdata=0,i=0;;
	
	cs_low;
	
	status=USART_SpiTransfer(USART1,reg);
	
	
	NVIC_EnableIRQ(USART1_RX_IRQn);	
	
	for(i=0;i<size;i++)
	{
		USART_SpiTransfer(USART1,0xFF);	
	
	//while(!(USART_StatusGet(USART1)& USART_STATUS_RXDATAV));  
		while (!data_geldi);
		data_geldi=0;
		*p= USART_RxDataGet(USART1);
		p++;
		
	}
	NVIC_DisableIRQ(USART1_RX_IRQn);
	cs_high;

}


uint8_t NRF_Read_reg(uint8_t add)
{
	uint8_t status=0,rxdata=0;
	
	cs_low;
	
	status=USART_SpiTransfer(USART1,add);
	
	
	NVIC_EnableIRQ(USART1_RX_IRQn);	
	USART_SpiTransfer(USART1,0xFF);	
	
	//while(!(USART_StatusGet(USART1)& USART_STATUS_RXDATAV));  
	while (!data_geldi);
	data_geldi=0;
	rxdata= USART_RxDataGet(USART1);
	NVIC_DisableIRQ(USART1_RX_IRQn);
	
	cs_high;

	return rxdata;

}

void Nrf24l01_init(void)
{


	


}

void Config_transmitter(void)
{
		uint8_t reg1=0;
		uint8_t cmd[1];
		uint8_t add1[5];
		
	
		ce_reset;
		
		delay_();
		cs_low;
		delay_();
		//cmd[0]=0x38;
		USART_SpiTransfer(USART1,0x20);
		USART_SpiTransfer(USART1,0x38);
		
		//NRF_Write_reg(0x00,cmd,1);
		delay_();
    cs_high;
		
    
		delay_();
		cs_low;
		delay_();
		USART_SpiTransfer(USART1,0x24);
		USART_SpiTransfer(USART1,0x00);
		
		//cmd[0]=0x00;
		//NRF_Write_reg(0x04,cmd,1);
		delay_();
    cs_high;
		
		delay_();

		cs_low;
		delay_();
		USART_SpiTransfer(USART1,0x23);
		USART_SpiTransfer(USART1,0x03);
		
		//cmd[0]=0x03;
		//NRF_Write_reg(0x03,cmd,1);
		delay_();
		
		
    cs_high;
    
		delay_();
    
		cs_low;
		delay_();
		USART_SpiTransfer(USART1,0x26);
		USART_SpiTransfer(USART1,0x07);
		
		//cmd[0]=0x26;
		//NRF_Write_reg(0x06,cmd,1);
		delay_();
    cs_high;
		
		
		delay_();
    
		cs_low;
		
		delay_();
		USART_SpiTransfer(USART1,0x25);
		USART_SpiTransfer(USART1,0x02);
		
		//cmd[0]=0x02;
		//NRF_Write_reg(0x05,cmd,1);
		
		delay_();
		
    cs_high;    

		
		delay_();

		cs_low;
		
		delay_();
		
		USART_SpiTransfer(USART1,0x30);
		USART_SpiTransfer(USART1,0xE7);
		USART_SpiTransfer(USART1,0xE7);
		USART_SpiTransfer(USART1,0xE7);
		USART_SpiTransfer(USART1,0xE7);
		USART_SpiTransfer(USART1,0xE7);
		
		
		
		delay_();
		//NRF_Write_reg(0x10,&xtr_add[0],5);
    cs_high;

    
    delay_();
		
		
		cs_low;
		
		delay_();
		
		USART_SpiTransfer(USART1,0x21);
		USART_SpiTransfer(USART1,0x00);
		
		//cmd[0]=0x00;
		//NRF_Write_reg(0x01,cmd,1);
		
		delay_();
		
    cs_high;  

		
		ce_set;
    

}

uint8_t reg=0;

void Nrf24l01_transmit(uint8_t* data,uint8_t size)
{

		uint8_t i=0;
	
		ce_reset;
	
		cs_low;
	
	delay_();
	
		USART_SpiTransfer(USART1,0x27);
		USART_SpiTransfer(USART1,0x7E);
		//cmd[0]=0x7E;
		//NRF_Write_reg(0x07,cmd,1);
	
	delay_();
    cs_high;  

	
	delay_();

		cs_low;
	
	delay_();
		USART_SpiTransfer(USART1,0x20);
		USART_SpiTransfer(USART1,0x3A);
		//cmd[0]=0x3A;
		//NRF_Write_reg(0x00,cmd,1);
		
		delay_();
    cs_high;  

		
		delay_();
	
		cs_low;
		
		delay_();
		USART_SpiTransfer(USART1,0xE1);
		//cmd[0]=0x00;
		//NRF_Write_reg(0xE1,cmd,0);
		delay_();
    cs_high; 

		delay_();
	
		cs_low;
		delay_();
		
		USART_SpiTransfer(USART1,0xA0);
		for(i=0;i<30;i++)
		{
			USART_SpiTransfer(USART1,*data);
			data++;
		}
	
		//NRF_Write_reg(0xA0,data,size);
		
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
	
    cs_high;  
		
	
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();

		
		ce_set;
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		delay_();
		//cs_low;
	
		//delay_();

		//USART_SpiTransfer(USART1,0x20);
		//USART_SpiTransfer(USART1,0x38);
		
		//delay_();
    //cs_high;  
		//ce_reset;


}//

void delay_ms(uint16_t ms)
{
	extern bool bekle;
	
	uint16_t sayac=0;
	
	while(sayac<ms)
	{
		while(!bekle);
		sayac++;
	
	}
	
	


}

void delay_(void)
{
	__NOP;
	__NOP;

	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
	__NOP;
}

void NRF24L01_RX_Mode(void)
{
		ce_reset;
		
		delay_();
		cs_low;
		delay_();
		//cmd[0]=0x38;
		USART_SpiTransfer(USART1,0x20);
		USART_SpiTransfer(USART1,0x3B);
		
		//NRF_Write_reg(0x00,cmd,1);
		delay_();
    cs_high;
		
    
		//delay_();
		//cs_low;
		//delay_();
		//USART_SpiTransfer(USART1,0x24);
		//USART_SpiTransfer(USART1,0x00);
		
		//cmd[0]=0x00;
		///NRF_Write_reg(0x04,cmd,1);
		//delay_();
    //cs_high;
		
		delay_();

		cs_low;
		delay_();
		USART_SpiTransfer(USART1,0x23);
		USART_SpiTransfer(USART1,0x03);
		
		//cmd[0]=0x03;
		//NRF_Write_reg(0x03,cmd,1);
		delay_();
		
		
    cs_high;
    
		delay_();
    
		cs_low;
		delay_();
		USART_SpiTransfer(USART1,0x26);
		USART_SpiTransfer(USART1,0x07);
		
		//cmd[0]=0x26;
		//NRF_Write_reg(0x06,cmd,1);
		delay_();
    cs_high;
		
		
		delay_();
    
		cs_low;
		
		delay_();
		USART_SpiTransfer(USART1,0x25);
		USART_SpiTransfer(USART1,0x02);
		
		//cmd[0]=0x02;
		//NRF_Write_reg(0x05,cmd,1);
		
		delay_();
		
    cs_high;    

		
		delay_();

		cs_low;
		
		delay_();
		
		USART_SpiTransfer(USART1,0x2A);
		USART_SpiTransfer(USART1,0xE7);
		USART_SpiTransfer(USART1,0xE7);
		USART_SpiTransfer(USART1,0xE7);
		USART_SpiTransfer(USART1,0xE7);
		USART_SpiTransfer(USART1,0xE7);
		
		
		
		delay_();
		//NRF_Write_reg(0x10,&xtr_add[0],5);
    cs_high;

    
    delay_();
		
				cs_low;
		
		delay_();
		
		USART_SpiTransfer(USART1,0x31);
		USART_SpiTransfer(USART1,0x1E);

		
		
		
		delay_();
		//NRF_Write_reg(0x10,&xtr_add[0],5);
    cs_high;

    
    delay_();
		
		
		cs_low;
		
		delay_();
		
		USART_SpiTransfer(USART1,0x21);
		USART_SpiTransfer(USART1,0x00);
		
		//cmd[0]=0x00;
		//NRF_Write_reg(0x01,cmd,1);
		
		delay_();
		
    cs_high;  

		
		ce_set;
    

}	

uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
		uint8_t status,uint8_t_ctr;	 
		ce_reset;
		cs_low;           //??SPI??
		status=USART_SpiTransfer(USART1,reg);
  	for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
		{
			USART_SpiTransfer(USART1,*pBuf++);
			//SPI1_ReadWriteByte(*pBuf++); //????	 
			
		}
  	cs_high;       //??SPI??
		delay_();						//important
  	return status;          //????????
}	

uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		    							   
	//SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi???9Mhz(24L01???SPI???10Mhz)   
	sta=NRF_Read_reg(STATUS);  //?????????    	 
	NRF_Write_reg2(WRITE_REG_NRF+STATUS,sta); //??TX_DS?MAX_RT????
	if(sta&RX_OK)//?????
	{
		multi_read(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//????
		NRF_Write_reg2(FLUSH_RX,0xff);//??RX FIFO??? 
		return 0; 
	}	   
	return 1;//???????
}	