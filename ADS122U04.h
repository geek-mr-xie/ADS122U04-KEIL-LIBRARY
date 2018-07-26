#ifndef __ADS122U04_H
#define __ADS122U04_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stdint.h"
#include "delay.h"

#define ADS_INT_GPIO_PORT         GPIOA
#define ADS_INT_GPIO_CLK          (RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO)
#define ADS_INT_GPIO_PIN          GPIO_Pin_15
#define ADS_INT_EXTI_PORTSOURCE   GPIO_PortSourceGPIOA
#define ADS_INT_EXTI_PINSOURCE    GPIO_PinSource15
#define ADS_INT_EXTI_LINE         EXTI_Line15
#define ADS_INT_EXTI_IRQ          EXTI15_10_IRQn

#define ADS_IRQHandler            EXTI15_10_IRQHandler


#define  ADS_USARTx                   USART1
#define  ADS_USART_CLK                RCC_APB2Periph_USART1
#define  ADS_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  ADS_USART_BAUDRATE           115200 //max 120000

// USART GPIO Òý½Åºê¶¨Òå
#define  ADS_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  ADS_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd

#define  ADS_USART_TX_GPIO_PORT       GPIOA
#define  ADS_USART_TX_GPIO_PIN        GPIO_Pin_9
#define  ADS_USART_RX_GPIO_PORT       GPIOA
#define  ADS_USART_RX_GPIO_PIN        GPIO_Pin_10

//#define  ADS_USART_IRQ                USART1_IRQn
//#define  ADS_USART_IRQHandler         USART1_IRQHandler

#define USART_FLAG_TIMEOUT         ((uint32_t)0x5000)
#define USART_LONG_TIMEOUT         ((uint32_t)0xFFFF)


typedef struct{
  uint8_t ADS_MUX;
	uint8_t ADS_GAIN;
	uint8_t PGA_BYPASS;
	uint8_t Data_Rate;
	uint8_t Operating;
	uint8_t Conversion;
	uint8_t VREF;
	uint8_t Temperature_Sensor;
	uint8_t DRDY;
	uint8_t DCNT;
	uint8_t ADS_CRC;
	uint8_t BCS;
	uint8_t IDAC;
	uint8_t L1MUX;
	uint8_t L2MUX;
	uint8_t AUTO;
	uint8_t GPIO2DIR;
	uint8_t GPIO1DIR;
	uint8_t GPIO0DIR;
	uint8_t GPIO2SEL;
	uint8_t GPIO2DAT;
	uint8_t GPIO1DAT;
	uint8_t GPIO0DAT;
} ADS_Reg_TypeDef;


//CMD//////////////////////////////////////////////
//CMD
#define AD_SYNC_HEAD			 	0x55
#define ADS_CMD_RESET				0x06
#define ADS_CMD_START				0x08
#define ADS_CMD_POWERDOWN		0x03
#define ADS_CMD_RDATA				0x10
#define ADS_CMD_RREG				0x20
#define ADS_CMD_WREG				0x40
//////REG0 BEGIN////////////////////////////////
//////MUX DEFs /////////////////////////////////
#define ADS_MUX_P0_N1 0x00
#define ADS_MUX_P0_N2 0x01
#define ADS_MUX_P0_N3 0x02
#define ADS_MUX_P1_N0 0x03
#define ADS_MUX_P1_N2 0x04
#define ADS_MUX_P1_N3 0x05
#define ADS_MUX_P2_N3 0x06
#define ADS_MUX_P3_N2 0x07
#define ADS_MUX_P0_NS 0x08
#define ADS_MUX_P1_NS 0x09
#define ADS_MUX_P2_NS 0x0A
#define ADS_MUX_P3_NS 0x0B
#define ADS_MUX_VREF      0x0C
#define ADS_MUX_AVOLD_4   0x0D
#define ADS_MUX_AVOLAVER  0x0E
#define ADS_MUX_RESERVE   0x0F
//////GAIN DEFs /////////////////////////////////
#define ADS_GAIN_1    0x00
#define ADS_GAIN_2    0x01
#define ADS_GAIN_4    0x02
#define ADS_GAIN_8    0x03
#define ADS_GAIN_16   0x04
#define ADS_GAIN_32   0x05
#define ADS_GAIN_64   0x06
#define ADS_GAIN_128  0x07
//////PGA_BYPASS DEFs /////////////////////////////////
#define PGA_ENABLED  0x00
#define PGA_DISABLED 0x01
/////REG0 END//////////////////////////////////////////////////

/////REG1 BEGIN//////////////////////////////////////////////////

/////DR DEFs//////////////////////////////////////////////
#define DR_NORMAL_20     0x00
#define DR_NORMAL_45     0x01
#define DR_NORMAL_90     0x02
#define DR_NORMAL_175    0x03
#define DR_NORMAL_330    0x04
#define DR_NORMAL_600    0x05
#define DR_NORMAL_1000   0x06
//in turbo mode times by 2
#define DR_TURBO_40     0x00
#define DR_TURBO_90     0x01
#define DR_TURBO_180     0x02
#define DR_TURBO_350    0x03
#define DR_TURBO_660    0x04
#define DR_TURBO_1200    0x05
#define DR_TURBO_2000   0x06
//MODE DEFs////////////////////////////////////////////////////////
#define MODE_NORMAL 0x00
#define MODE_TURBO 0x01
//CM DEFs////////////////////////////////////////////////////
#define CM_SINGAL_SHOT 0x00
#define CM_CONTINUOUS  0x01
//VREF DEFS/////////////////////////////////////////////////
#define VREF_INTERNAL_2048  0x00
#define VREF_EXTERNAL_REF   0x01
#define VREF_ANALOG_1       0x02
#define VREF_ANALOG_2       0x03
//TS DEFs///////////////////////////////////////////////////
#define TS_DISABLED  0x00
#define TS_ENABLED   0x01
/////REG1 END//////////////////////////////////////////////////

/////REG2 BEGIN//////////////////////////////////////////////////

//DRDY DEFs
#define DRDY_NOT_READY 0x00
#define DRDY_READY 0x01
//DCNT DEFS
#define DCNT_DISABLED 0x00
#define DCNT_ENABLED 0x01
//CRC  DEFs
#define CRC_DISABLED         0x00
#define CRC_INVERTED_ENABLED 0x01
#define CRC_CRC16_ENABLED    0x02
#define CRC_RESERVED        0x03
//BCS DEFS
#define BCS_CURRENT_SOURCE_OFF  0x00
#define BCS_CURRENT_SOURCE_ON   0x01
//IDAC DEFS
#define IDAC_CURRENT_OFF      0x00
#define IDAC_CURRENT_10_uA    0x01
#define IDAC_CURRENT_50_uA    0x02
#define IDAC_CURRENT_100_uA   0x03
#define IDAC_CURRENT_250_uA   0x04
#define IDAC_CURRENT_500_uA   0x05
#define IDAC_CURRENT_1000_uA  0x06
#define IDAC_CURRENT_1500_uA  0x07
/////REG2 END//////////////////////////////////////////////////

/////REG3 BEGIN//////////////////////////////////////////////////

//L1MUX
#define L1MUX_DISABLED   0x00
#define L1MUX_TO_AIN0   0x01
#define L1MUX_TO_AIN1   0x02
#define L1MUX_TO_AIN2   0x03
#define L1MUX_TO_AIN3   0x04
#define L1MUX_TO_REFP   0x05
#define L1MUX_TO_REFN   0x06
#define L1MUX_RESERVED  0x07
//L2MUX
#define L2MUX_DISABLED   0x00
#define L2MUX_TO_AIN0   0x01
#define L2MUX_TO_AIN1   0x02
#define L2MUX_TO_AIN2   0x03
#define L2MUX_TO_AIN3   0x04
#define L2MUX_TO_REFP   0x05
#define L2MUX_TO_REFN   0x06
#define L2MUX_RESERVED
//AUTO DEFS
#define ADS_DATA_MODE_MANUAL 0x00
#define ADS_DATA_MODE_AUTO   0x01

/////REG3 END//////////////////////////////////////////////////


/////REG4 BEGIN//////////////////////////////////////////////////
//GPIO DIRECTION
#define ADS_GPIO2_DIR_INPUT  0x00
#define ADS_GPIO2_DIR_OUTPUT 0x01
#define ADS_GPIO1_DIR_INPUT  0x00
#define ADS_GPIO1_DIR_OUTPUT 0x01
#define ADS_GPIO0_DIR_INPUT  0x00
#define ADS_GPIO0_DIR_OUTPUT 0x01
//GPIO2 DRDY
#define ADS_GPIO2_SEL_DAT  0x00
#define ADS_GPIO2_SEL_DRDY 0x01
//GPIO
#define ADS_GPIO_LOW  0x00
#define ADS_GPIO_HIGH 0x01

/////REG4 END//////////////////////////////////////////////////

void ADS_Init(void);
void ADS122U04_EXTI_Init(void);
void ADS122U04_USART_Init(void);
void ADS_AUTO_Run(void);
void ADS_Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t DATA);
void ADS_Usart_SendCMD( USART_TypeDef * pUSARTx, uint8_t CMD);
uint8_t ADS_Usart_ReceiveByte( USART_TypeDef * pUSARTx);
uint8_t ADS_Read_Reg( USART_TypeDef * pUSARTx,uint8_t Address);
void ADS_Write_Reg( USART_TypeDef * pUSARTx,int8_t Address,uint8_t data);
void ADS_Reg_Generator(ADS_Reg_TypeDef * pADS_Reg);
void ADS_AUTO_Handeler(void);

void ADS_One_Shot_Setting(ADS_Reg_TypeDef * pADS_Reg);
void ADS_One_Shot_Run(void);
void ADS_One_Shot_Stop(void);
double ADS_One_Shot_Receiver(USART_TypeDef * pUSARTx);

extern int32_t ADSData;
#endif
