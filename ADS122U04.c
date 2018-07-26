#include "ADS122U04.h"
#include "TFT.h"
#include <stdio.h>
uint8_t adsreg[5];
void ADS_Init(void){
  ADS122U04_EXTI_Init();
  ADS122U04_USART_Init();
}
void ADS122U04_EXTI_Init(void){
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(ADS_INT_GPIO_CLK,ENABLE);//开时钟
	/*配置中断优先级*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel=ADS_INT_EXTI_IRQ ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/*初始化中断GPIO*/
	RCC_APB2PeriphClockCmd( ADS_INT_GPIO_CLK , ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADS_INT_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ADS_INT_GPIO_PORT, &GPIO_InitStructure);
	/*初始化EXTI */
	GPIO_EXTILineConfig(ADS_INT_EXTI_PORTSOURCE, ADS_INT_EXTI_PINSOURCE);
  EXTI_InitStructure.EXTI_Line=ADS_INT_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Falling;//  EXTI_Trigger_Falling  EXTI_Trigger_Rising_Falling
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}
void ADS122U04_USART_Init(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  ADS_USART_GPIO_APBxClkCmd(ADS_USART_GPIO_CLK, ENABLE);
  // 打开串口外设的时钟
  ADS_USART_APBxClkCmd(ADS_USART_CLK, ENABLE);
  // 将USART Tx的GPIO配置为推挽复用模式
  GPIO_InitStructure.GPIO_Pin = ADS_USART_TX_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ADS_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  // 将USART Rx的GPIO配置为浮空输入模式
  GPIO_InitStructure.GPIO_Pin = ADS_USART_RX_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(ADS_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  //串口结构体配置
  USART_InitStructure.USART_BaudRate = ADS_USART_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(ADS_USARTx, &USART_InitStructure);
  // 使能串口
  USART_Cmd(ADS_USARTx, ENABLE);
}
void ADS_Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t DATA){
	/* 发送一个字节数据到USART */
	uint32_t USARTTimeout;
	USART_SendData(pUSARTx,DATA);
	USARTTimeout = USART_FLAG_TIMEOUT;
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
		if((USARTTimeout--) == 0) return ;
}
void ADS_Usart_SendCMD( USART_TypeDef * pUSARTx, uint8_t CMD){
	ADS_Usart_SendByte( pUSARTx, AD_SYNC_HEAD);
	ADS_Usart_SendByte( pUSARTx,CMD);
}
uint8_t ADS_Usart_ReceiveByte( USART_TypeDef * pUSARTx){
	uint8_t retVal=0;
	uint32_t USARTTimeout;
	USARTTimeout = USART_FLAG_TIMEOUT;
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_RXNE) == RESET)
		if((USARTTimeout--) == 0) return 0;
	retVal=USART_ReceiveData(pUSARTx);
	return retVal;
}

uint8_t ADS_Read_Reg( USART_TypeDef * pUSARTx,uint8_t Address){
	ADS_Usart_SendByte( pUSARTx, AD_SYNC_HEAD);
	ADS_Usart_SendByte( pUSARTx,Address);
	return ADS_Usart_ReceiveByte(pUSARTx);
}
void ADS_Write_Reg( USART_TypeDef * pUSARTx,int8_t Address,uint8_t data){
	ADS_Usart_SendByte( pUSARTx, AD_SYNC_HEAD);
	ADS_Usart_SendByte( pUSARTx,Address);
	ADS_Usart_SendByte( pUSARTx,data);
}
void ADS_Write_All_Reg(ADS_Reg_TypeDef * pADS_Reg){
  ADS_Reg_Generator(pADS_Reg);
  ADS_Write_Reg(ADS_USARTx,0x40,adsreg[0]);
  ADS_Write_Reg(ADS_USARTx,0x42,adsreg[1]);
  ADS_Write_Reg(ADS_USARTx,0x44,adsreg[2]);
  ADS_Write_Reg(ADS_USARTx,0x46,adsreg[3]);
  ADS_Write_Reg(ADS_USARTx,0x48,adsreg[4]);
}
void ADS_Reg_Generator(ADS_Reg_TypeDef * pADS_Reg){
  adsreg[0]=((pADS_Reg->ADS_MUX&0x0F)<<4)|((pADS_Reg->ADS_GAIN&0x07)<<1)|(pADS_Reg->PGA_BYPASS&0x01);
  adsreg[1]=((pADS_Reg->Data_Rate&0x03)<<5)|((pADS_Reg->Operating&0x01)<<4)|((pADS_Reg->Conversion&0x01)<<3)|((pADS_Reg->VREF&0x03)<<1)|(pADS_Reg->Temperature_Sensor&0x01);
  adsreg[2]=((pADS_Reg->DRDY&0x01)<<7)|((pADS_Reg->DCNT&0x01)<<6)|((pADS_Reg->ADS_CRC&0x03)<<4)|((pADS_Reg->BCS&0x01)<<3)|(pADS_Reg->IDAC&0x07);
  adsreg[3]=((pADS_Reg->L1MUX&0x07)<<5)|((pADS_Reg->L2MUX&0x07)<<2)|(pADS_Reg->AUTO);
  adsreg[4]=((pADS_Reg->GPIO2DIR&0x01)<<6)|((pADS_Reg->GPIO1DIR&0x01)<<5)|((pADS_Reg->GPIO0DIR&0x01)<<4)|((pADS_Reg->GPIO2SEL&0x01)<<3)|((pADS_Reg->GPIO2DAT&0x01)<<2)|((pADS_Reg->GPIO1DAT&0x01)<<1)|(pADS_Reg->GPIO0DAT&0x01);
}
void ADS_AUTO_Setting(ADS_Reg_TypeDef * pADS_Reg){
  pADS_Reg->ADS_MUX=ADS_MUX_P0_N1;
  pADS_Reg->ADS_GAIN=ADS_GAIN_1;
  pADS_Reg->PGA_BYPASS=PGA_DISABLED;

  pADS_Reg->Data_Rate=DR_NORMAL_20;
  pADS_Reg->Operating=MODE_NORMAL;
  pADS_Reg->Conversion=CM_CONTINUOUS;
  pADS_Reg->VREF=VREF_INTERNAL_2048;
  pADS_Reg->Temperature_Sensor=TS_DISABLED;

  pADS_Reg->DRDY=DRDY_NOT_READY;
  pADS_Reg->DCNT=DCNT_DISABLED;
  pADS_Reg->ADS_CRC=CRC_DISABLED;
  pADS_Reg->BCS=BCS_CURRENT_SOURCE_OFF;
  pADS_Reg->IDAC=IDAC_CURRENT_OFF;

  pADS_Reg->L1MUX=L1MUX_DISABLED;
  pADS_Reg->L2MUX=L2MUX_DISABLED;
  pADS_Reg->AUTO=ADS_DATA_MODE_AUTO;

  pADS_Reg->GPIO2DIR=ADS_GPIO2_DIR_OUTPUT;
  pADS_Reg->GPIO1DIR=ADS_GPIO1_DIR_OUTPUT;
  pADS_Reg->GPIO0DIR=ADS_GPIO0_DIR_OUTPUT;
  pADS_Reg->GPIO2SEL=ADS_GPIO2_SEL_DRDY;
  pADS_Reg->GPIO2DAT=ADS_GPIO_LOW;
  pADS_Reg->GPIO1DAT=ADS_GPIO_LOW;
  pADS_Reg->GPIO0DAT=ADS_GPIO_LOW;

  ADS_Write_All_Reg(pADS_Reg);
}
void ADS_AUTO_Run(void){
  // Power-up;
  // Delay to allow power supplies to settle and power-up reset to complete; minimum of 50 μs;
  ADS_Reg_TypeDef ADS_Reg;
	Delay_ms(1);
  // Configure the UART interface of the microcontroller to 8-N-1 format;
  ADS_Init();
	// Configure the microcontroller GPIO connected to the GPIO2/DRDY pin as a falling edge triggered
  // interrupt input;
  // Send the synchronization word to the device (55h);
  // Send the RESET command (06h) to make sure the device is properly reset after power-up;
	ADS_Usart_SendCMD(ADS_USARTx,ADS_CMD_RESET);
	// Delay for a minimum of td(RSRX);
	Delay_ms(1);
  ADS_AUTO_Setting(&ADS_Reg);
	// Send the synchronization word to the device (55h);
  // Send the START/SYNC command (08h) to start converting in continuous conversion mode;
	 ADS_Usart_SendCMD(ADS_USARTx, ADS_CMD_START);




}
void ADS_AUTO_Stop(void){
	// Send the synchronization word (55h);
  // Send the POWERDOWN command (02h) to stop conversions and put the device in power-down mode;
	ADS_Usart_SendCMD(ADS_USARTx,ADS_CMD_POWERDOWN);

}
void ADS_AUTO_Handeler(void){
//	u8 arr[15];
	uint32_t MSB;
	uint32_t HSB;
	uint32_t LSB;
	int32_t DATA;
	DATA=0;
	LSB=ADS_Usart_ReceiveByte( ADS_USARTx);
	HSB=ADS_Usart_ReceiveByte( ADS_USARTx);
	MSB=ADS_Usart_ReceiveByte( ADS_USARTx);
	if(MSB>=0x80){
		DATA=(MSB<<16)+(HSB<<8)+LSB;
		DATA=0x1000000-DATA;
		DATA=-DATA;
	}else{
		DATA=(MSB<<16)+(HSB<<8)+LSB;
	}
//	sprintf(arr,"Voltage: %1.7f",(double)DATA/4096000);
//	LCD_DisplayString(0,0,arr);
  // Loop
  // {
  //   Wait for GPIO2/DRDY to transition low;
  //   Send the synchronization word (55h);
  //   Send the RDATA command (10h);
  //   Receive 3 bytes of data from TX;
  // }
}
void ADS_One_Shot_Setting(ADS_Reg_TypeDef * pADS_Reg){
  pADS_Reg->ADS_MUX=ADS_MUX_P0_N1;
  pADS_Reg->ADS_GAIN=ADS_GAIN_1;
  pADS_Reg->PGA_BYPASS=PGA_DISABLED;

  pADS_Reg->Data_Rate=DR_NORMAL_1000;
  pADS_Reg->Operating=MODE_NORMAL;
  pADS_Reg->Conversion=CM_CONTINUOUS;
  pADS_Reg->VREF=VREF_INTERNAL_2048;
  pADS_Reg->Temperature_Sensor=TS_DISABLED;

  pADS_Reg->DRDY=DRDY_NOT_READY;
  pADS_Reg->DCNT=DCNT_DISABLED;
  pADS_Reg->ADS_CRC=CRC_DISABLED;
  pADS_Reg->BCS=BCS_CURRENT_SOURCE_OFF;
  pADS_Reg->IDAC=IDAC_CURRENT_OFF;

  pADS_Reg->L1MUX=L1MUX_DISABLED;
  pADS_Reg->L2MUX=L2MUX_DISABLED;
  pADS_Reg->AUTO=ADS_DATA_MODE_MANUAL;//important

  pADS_Reg->GPIO2DIR=ADS_GPIO2_DIR_OUTPUT;
  pADS_Reg->GPIO1DIR=ADS_GPIO1_DIR_OUTPUT;
  pADS_Reg->GPIO0DIR=ADS_GPIO0_DIR_OUTPUT;
  pADS_Reg->GPIO2SEL=ADS_GPIO2_SEL_DRDY;
  pADS_Reg->GPIO2DAT=ADS_GPIO_LOW;
  pADS_Reg->GPIO1DAT=ADS_GPIO_LOW;
  pADS_Reg->GPIO0DAT=ADS_GPIO_LOW;

  ADS_Write_All_Reg(pADS_Reg);
}
void ADS_One_Shot_Run(void){
  ADS_Reg_TypeDef ADS_Reg;
	Delay_ms(1);
  ADS122U04_USART_Init();
	ADS_Usart_SendCMD(ADS_USARTx,ADS_CMD_RESET);
	Delay_ms(1);
  ADS_One_Shot_Setting(&ADS_Reg);
  ADS_Usart_SendCMD(ADS_USARTx, ADS_CMD_START);
}
void ADS_One_Shot_Stop(void){
  ADS_Usart_SendCMD(ADS_USARTx,ADS_CMD_POWERDOWN);
}
double ADS_One_Shot_Receiver(USART_TypeDef * pUSARTx){
  uint32_t MSB;
  uint32_t HSB;
  uint32_t LSB;
  int32_t DATA;
  DATA=0;
  ADS_Usart_SendCMD( pUSARTx, ADS_CMD_RREG+4);
  if((((ADS_Usart_ReceiveByte( pUSARTx)&0x80)>>7)&0x01)==0x01){
		ADS_Usart_SendCMD( pUSARTx, ADS_CMD_RDATA);
    LSB=ADS_Usart_ReceiveByte( pUSARTx);
    HSB=ADS_Usart_ReceiveByte( pUSARTx);
    MSB=ADS_Usart_ReceiveByte( pUSARTx);
    if(MSB>=0x80){
      DATA=(MSB<<16)+(HSB<<8)+LSB;
      DATA=0x1000000-DATA;
      DATA=-DATA;
    }else{
      DATA=(MSB<<16)+(HSB<<8)+LSB;
    }
    return (double)DATA/4096000;
  }else{
    return 0.0;
  }
}
