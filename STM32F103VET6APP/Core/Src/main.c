/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
//TIM1: CH1,CH1N used to drive DC motor(drv8848).
//72MHz/Prescale=72MHz/2=36MHz
//ARR(AutoReloadRegister)=360,36KHz/360=100KHz.
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/**********************************************************/
//TIM1-CH1/CH1N: output PWM to drive DC motor.
//TIM5-CH1/CH1N: DC motor encoder A/B phase input.
/**********************************************************/
//TIM2-CH1/CH1N: output PWMto drive Stepper Left/Right motor.
//TIM3: used to help to stop TIM2-CH1.
/**********************************************************/
//TIM8-CH2: output PWM to drive Stepper Up/Down motor.
//TIM4: used to help to stop TIM8-CH2.
/**********************************************************/

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 128
};
/* Definitions for ZPidTask */
osThreadId_t ZPidTaskHandle;
const osThreadAttr_t ZPidTask_attributes = {
		.name = "ZPidTask",
		.priority = (osPriority_t) osPriorityRealtime,
		.stack_size = 256
};
/* Definitions for ZModBusTask */
osThreadId_t ZModBusTaskHandle;
const osThreadAttr_t ZModBusTask_attributes = {
		.name = "ZModBusTask",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 512
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void *argument);
void ZPidTaskLoop(void *argument);
void ZModBusTaskLoop(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <string.h>
#include <math.h>
uint32_t g_CRC32JAMCRCCalculate(uint8_t *s, int len);

//the flag.
uint8_t g_flagDCMotorPeakCurrent = 0;

//each command is 24 bytes at least.
//Sync(4)+Length(4)+Key(4)+Address(4)+Data(4)+Padding(0)+CRC32(4)
#define CMD_LENGTH_MIN	24
uint8_t g_Uart1DmaBuffer[CMD_LENGTH_MIN];

#define CMD_POLL_SIZE	(CMD_LENGTH_MIN*50)
uint8_t g_Uart1DmaPoll[CMD_POLL_SIZE];
uint32_t g_Uart1DmaPollLength = 0;

//the flag.
uint8_t g_FlagUart1GetNewData = 0;
typedef enum {
	UnPack_Sync_Filed = 0,
	UnPack_Length_Field,
	UnPack_Key_Field,
	UnPack_Address_Field,
	UnPack_Data_Field,
	UnPack_Padding_Field,
	UnPack_CRC32_Field,
} ZModBusUnPackFSM;
typedef struct {
	ZModBusUnPackFSM m_fsm;
	uint8_t m_flagEncrypt;
	uint8_t m_length;
	uint32_t m_key;
	uint32_t m_address;
	uint32_t m_data;
	uint32_t m_crc32;
	uint8_t m_crc32Buffer[CMD_LENGTH_MIN];
	uint8_t m_crc32BufferLen;

	//DC motor & encoder related.
	uint32_t m_uTargetEncoder;
	float maxOut;
	float inteLimit;
	float inputMaxErr;

	float kP;
	float kI;
	float kD;

	float setV;
	float getV;

	float err;
	float last_err;

	float pOut;
	float iOut;
	float dOut;
	float out;
	//other ModBus related variables.
	uint32_t m_uBatteryPercent;
	uint32_t m_uRSSIPercent;
	uint32_t m_uDistance;
	uint32_t m_uOutputVol;
	uint32_t m_uVideoCtl;
	uint32_t m_uFPGAReg;
	uint32_t m_uCrossXY;

	//for Flash Read & Write.
	//each block data structure is
	//Sync Prefix:(uint32_t)
	//m_uOutputVol:(uint32_t)
	//m_uVideoCtl:(uint32_t)
	//m_uFPGAReg:(uint32_t)
	//m_uCrossXY:(uint32_t)
	//so the total bytes is 5*(uint32_t)=20 bytes.
	uint32_t newAddr;
	uint32_t curAddr;
} ZModBusObject;

ZModBusObject modbusObj;
enum {
	nReg_Battery_R = 0x000001,						//read battery register.
	nReg_RSSI_R = 0x000002, 						//read RSSI register.
	nReg_Distance_R = 0x000003, 					//read Distance register.
	nReg_LenMotorCtl_W = 0x800004,				//write LenMotorCtl register.
	nReg_2DBracketCtl_W = 0x800005, 		//write 2D bracket ctrl register.
	nReg_OutVolume_R = 0x000006,				//read output volume register.
	nReg_OutVolume_W = 0x800006,				//write output volume register.
	nReg_VideoCtl_R = 0x000007, 					//read videoCtl register.
	nReg_VideoCtl_W = 0x800007, 					//write videoCtl register.
	nReg_FPGA_R = 0x000008, 						//read FPGA register.
	nReg_FPGA_W = 0x800008, 						//write FPGA register.
	nReg_CrossXY_R = 0x000009,						//read CrossXY register.
	nReg_CrossXY_W = 0x800009,						//write CrossXY register.
	nReg_AutoFocus1_R = 0x3F0000,					//read AutoFocus1 register.
	nReg_AutoFocus1_W = 0xBF0000,					//write AutoFocus1 register.
};
#define MODBUS_ADDR_ME			(0x01<<24) //0x01000000
#define MODBUS_ADDR_APP			(0xFF<<24) //0xFF000000
#define MODBUS_ADDR_BROADCAST	(0x00<<24) //0x00000000

//ADC1-CHannel10:	BatteryVoltage
//ADC1-Channel12: 	RSSI
uint16_t g_ADC1DMABuffer[30][2];

//根据数据手册得知STM32F103VET6是大容量产品
//具有512K Internal Flash和64K SRAM
//Page 0: 0x0800 0000 ~ 0x0800 07FF,2Kbytes
//Page 1: 0x0800 0800 ~ 0x0800 0FFF,2Kbytes
//Page 2: 0x0800 1000 ~ 0x0800 17FF,2Kbytes
//Page 3: 0x0800 1800 ~ 0x0800 1FFF,2Kbytes
// ...............
//Page255:0x0807 F800 ~ 0x0807 FFFF,2Kbytes
//所以说从Page0开始存放的是程序代码，总共容量是2K*256=512Kbytes
//如果程序只占用了一部分，那么剩下的一部分可以用于存放数据.
//这里将数据首地址定义为0x0801E000
//因此有(0x0801E000-0x08000000)=122880bytes=120K
//这就相当于给程序留出了前120K的空间，余下的空间用于数据存储
#include "stm32f1xx_hal_flash_ex.h"
#define ZSY_DATA_ADDR_BASE 		0x0801E000
#define PAGE_SIZE               (uint32_t)FLASH_PAGE_SIZE  /* Page size */
#define RD_HALF_WORD(addr)		(*(uint16_t*)(addr))
#define RD_WORD(addr)			(*(uint16_t*)(addr)) | (*(uint16_t*)((addr)+2)<<16)
void zsy_ErasePage(void)
{
	/* -1- Unlock the Flash Bank Program Erase controller */
	HAL_FLASH_Unlock();

	/* -2- Clear All pending flags */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

	/* -3- erase the FLASH pages */
	FLASH_PageErase(ZSY_DATA_ADDR_BASE);
	FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
	CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

	/* -5- Lock the Flash Bank Program Erase controller */
	HAL_FLASH_Lock();
}
void zsy_FindUnusedBlk(void)
{
	while(modbusObj.curAddr<(ZSY_DATA_ADDR_BASE+PAGE_SIZE))
	{
		uint32_t u32RdData = RD_WORD(modbusObj.curAddr);
		//if the first word is 0xFFFFFFFF,it means this is not used yet.
		if(u32RdData == 0xFFFFFFFF)
		{
			modbusObj.newAddr = modbusObj.curAddr;
			return;
		}
		modbusObj.curAddr += 20;
	}

	//if the current address reached the boundary, it means all page was used.
	//so erase all page,and write from the begin address again.
	if(modbusObj.curAddr >= (ZSY_DATA_ADDR_BASE+PAGE_SIZE))
	{
		zsy_ErasePage();
		modbusObj.curAddr = ZSY_DATA_ADDR_BASE;
		modbusObj.newAddr = ZSY_DATA_ADDR_BASE;
	}
}
void zsy_FlushToFlash()
{
	//first,find a unused block.
	zsy_FindUnusedBlk();

	//unlock,write,lock.
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,modbusObj.newAddr+0,0x19870901);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,modbusObj.newAddr+4,modbusObj.m_uOutputVol);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,modbusObj.newAddr+8,modbusObj.m_uVideoCtl);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,modbusObj.newAddr+12,modbusObj.m_uFPGAReg);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,modbusObj.newAddr+16,modbusObj.m_uCrossXY);
	HAL_FLASH_Lock();
}
//to find out the newest used block,
//usually load once time at start up.
void zsy_LoadFromFlash(void)
{
	//set initial address.
	modbusObj.curAddr=ZSY_DATA_ADDR_BASE;
	modbusObj.newAddr=ZSY_DATA_ADDR_BASE;

	//search loop.
	while(modbusObj.curAddr < (ZSY_DATA_ADDR_BASE+PAGE_SIZE))
	{
		uint32_t u32RdData = RD_WORD(modbusObj.curAddr);
		if(u32RdData==0x19870901)
		{
			//this block is used,try to check next one.
			modbusObj.curAddr += 20;

		}else{
			//find the unused block,it's 0xFFFFFFFF. so,back a block.
			modbusObj.curAddr-=20;

			//to avoid down-overflow,so here check to afford the beginning search.
			if(modbusObj.curAddr<ZSY_DATA_ADDR_BASE)
			{
				modbusObj.curAddr=ZSY_DATA_ADDR_BASE;
			}else if(modbusObj.curAddr>(ZSY_DATA_ADDR_BASE+PAGE_SIZE))
			{
				modbusObj.curAddr=(ZSY_DATA_ADDR_BASE+PAGE_SIZE);
			}
			//read one block data.
			uint32_t uBlkPrefix=RD_WORD(modbusObj.curAddr+0);
			if(uBlkPrefix==0x19870901)
			{
				modbusObj.m_uOutputVol=RD_WORD(modbusObj.curAddr+4);
				modbusObj.m_uVideoCtl=RD_WORD(modbusObj.curAddr+8);
				modbusObj.m_uFPGAReg=RD_WORD(modbusObj.curAddr+12);
				modbusObj.m_uCrossXY=RD_WORD(modbusObj.curAddr+16);
			}else{
				//maybe the first running , no one used block in Flash.
				//this is the default parameters.
				modbusObj.m_uOutputVol=100;
				modbusObj.m_uVideoCtl=1;
				modbusObj.m_uFPGAReg=1;
				modbusObj.m_uCrossXY=0;
			}
			//output the default value.
//			uint8_t buffer[128];
//			sprintf("Log:%d,%d,%d,%d\n",///<
//					modbusObj.m_uOutputVol,modbusObj.m_uVideoCtl,///<
//					modbusObj.m_uFPGAReg,modbusObj.m_uCrossXY);
//			HAL_UART_Transmit(&huart1, buffer, strlen(buffer), 1000);
			return;
		}
	}
}
//when the current of DC motor gets to grow
//this interrupt occurs,stop PWM output.
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc) {
	static uint32_t nDCPeakCurrentCnt = 0;
	if (hadc->Instance == ADC2) {
		if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_AWD) != RESET) {
			if ((g_flagDCMotorPeakCurrent == 0)
					&& (nDCPeakCurrentCnt++ >= 100)) {
				nDCPeakCurrentCnt = 0;
				//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
				//			HAL_GPIO_WritePin(NSLEEP_GPIO_Port, NSLEEP_Pin, GPIO_PIN_RESET);

				//>50%:clockwise,<50%:anti-clockwise,=50%:stop.
				//CCR/ARR=180/360=50%.
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 180);

				//reset encoder to 0.
				//here we do not set to 0,because it reaches the boundary.
				//0-1=65535 or 65535+1=0.
				//so here we use 500 as the ZeroPoint encoder value.
				__HAL_TIM_SET_COUNTER(&htim5, 500);

				//reset flag.
				g_flagDCMotorPeakCurrent = 1;
			}
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		//HAL_UART_Transmit(&huart1,g_UART1DMABuffer,CMD_LENGTH_MIN,100);
		if (g_Uart1DmaPollLength + CMD_LENGTH_MIN < CMD_POLL_SIZE) {
			memcpy(&g_Uart1DmaPoll[g_Uart1DmaPollLength], g_Uart1DmaBuffer,
					CMD_LENGTH_MIN);
			g_Uart1DmaPollLength += CMD_LENGTH_MIN;
			g_FlagUart1GetNewData = 1;
		}
	}
}
float zsy_PIDCalculate(ZModBusObject *obj, float getV, float setV) {
	float fKp = 1.5;
	float fKi = 0;
	float fKd = 0;

	obj->getV = getV;
	obj->setV = setV;
	obj->err = setV - getV;

	obj->pOut = fKp * obj->err;
	obj->iOut += fKi * obj->err;
	obj->dOut = fKd * (obj->err - obj->last_err);

	obj->out = obj->pOut + obj->iOut + obj->dOut;

	return obj->out;
}
uint32_t zsyHtonl(uint32_t uData) {
	uint32_t uByteConverted;
	uint8_t *p = (uint8_t*) &uByteConverted;
	p[0] = (uData >> 24) & 0xFF;
	p[1] = (uData >> 16) & 0xFF;
	p[2] = (uData >> 8) & 0xFF;
	p[3] = (uData >> 0) & 0xFF;
	return uByteConverted;
}
void zsyTxResponse(uint32_t regAddr, uint32_t regData) {
	uint32_t uResponse[6];
	uResponse[0] = zsyHtonl(0x44454354);				//sync.
	uResponse[1] = zsyHtonl(16);				//length.
	uResponse[2] = 0;				//key.
	uResponse[3] = zsyHtonl(regAddr | MODBUS_ADDR_APP);				//address.
	uResponse[4] = zsyHtonl(regData);				//key.
	uResponse[5] = g_CRC32JAMCRCCalculate((uint8_t*) uResponse, 20);	//CRC32.

	HAL_UART_Transmit(&huart1, (uint8_t*) uResponse, sizeof(uResponse), 1000);
}
void zsy_Delay()
{
	uint8_t 		i;

	for (i = 0; i < 200; i++)
	{
		;
	}
}
void zsyParseModBusFrame(ZModBusObject *obj) {
	if (obj->m_flagEncrypt) {
		//do not support encrypted frame in this version.
		return;
	}
	//1.check address is not mine or not broadcast address.
	if (!(((obj->m_address & 0xFF000000) == MODBUS_ADDR_ME)
			|| ((obj->m_address & 0xFF000000) == 0))) {
		//do not response any.
		return;
	}
	switch (obj->m_address & 0x00FFFFFF) {
	case nReg_Battery_R:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 00 00 01 00 00 00 01 5C 9C 78 76
	{
		//calculate the average value.
		uint32_t uAverageVal[2];
		for(uint32_t i=0;i<2;i++)
		{
			uint32_t sum=0;
			for(uint32_t j=0;j<30;j++)
			{
				sum+=g_ADC1DMABuffer[j][i];
			}
			uAverageVal[i]=sum/30;

			//			char buffer[32];
			//			sprintf(buffer,"%d:%d\n",i,uAverageVal[i]);
			//			HAL_UART_Transmit(&huart1,buffer,strlen(buffer),100);
			//			float it=uAverageVal[i]/4096.0*12.0;
			//			sprintf(buffer,"%.2f\n\r",it);
			//			HAL_UART_Transmit(&huart1,buffer,strlen(buffer),100);
		}
		//here we do not convert ADC value to Real Voltage Value.
		//obj->m_uBatteryPercent=(uint32_t)(uAverageVal[0]/4096.0*2.5);
		float fPercent=uAverageVal[0]/4096.0*100;
		obj->m_uBatteryPercent=fPercent;
		zsyTxResponse(nReg_Battery_R, obj->m_uBatteryPercent);
	}
	break;
	case nReg_RSSI_R:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 00 00 02 00 00 00 01 1B 3C 02 A6
	{
		//calculate the average value.
		uint32_t uAverageVal[2];
		for(uint32_t i=0;i<2;i++)
		{
			uint32_t sum=0;
			for(uint32_t j=0;j<30;j++)
			{
				sum+=g_ADC1DMABuffer[j][i];
			}
			uAverageVal[i]=sum/30;

			//			char buffer[32];
			//			sprintf(buffer,"%d:%d\n",i,uAverageVal[i]);
			//			HAL_UART_Transmit(&huart1,buffer,strlen(buffer),100);
			//			float it=uAverageVal[i]/4096.0*12.0;
			//			sprintf(buffer,"%.2f\n\r",it);
			//			HAL_UART_Transmit(&huart1,buffer,strlen(buffer),100);
		}
		//here we do not convert ADC value to Real Voltage Value.
		//obj->m_uRSSIPercent=(uint32_t)(uAverageVal[1]/4096.0*2.5);
		float fPercent=uAverageVal[1]/4096.0*100;
		obj->m_uRSSIPercent=fPercent;
		zsyTxResponse(nReg_RSSI_R, obj->m_uRSSIPercent);
	}
	break;
	case nReg_Distance_R:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 00 00 03 00 00 00 01 26 5C 2B 16
		zsyTxResponse(nReg_RSSI_R, obj->m_uDistance);
		break;
	case nReg_LenMotorCtl_W: {
		uint8_t uWhichMotor = (obj->m_data >> 24) & 0xFF;
		uint8_t uMotorAction = (obj->m_data >> 16) & 0xFF;
		uint16_t uMotorIncrease = 1800;
		switch (uWhichMotor) {
		case 0x1:				//left motor.
			if (uMotorAction == 0x1)				//clockwise.
			{
				//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 04 01 01 00 01 5A EB EF 5A
				(obj->m_uTargetEncoder) += uMotorIncrease;
			} else if (uMotorAction == 0x2)				//anti-clockwise.
			{
				//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 04 01 02 00 01 58 AD 51 01
				(obj->m_uTargetEncoder) -= uMotorIncrease;
			}
			break;
		case 0x2:				//right motor.
			break;
		case 0xFF:				//all motors.
			break;
		default:
			break;
		}
		//CAUTION!!! here to prevent encoder value to overflow.
		//0-1=65535    65535+1=0
		if (obj->m_uTargetEncoder < 180) {
			obj->m_uTargetEncoder = 180;
		} else if (obj->m_uTargetEncoder >= 65535) {
			obj->m_uTargetEncoder = 65535;
		}
		zsyTxResponse(nReg_LenMotorCtl_W, obj->m_data);
	}
	break;
	case nReg_2DBracketCtl_W: {
		uint8_t uWhichMotor = (obj->m_data >> 24) & 0xFF;
		uint8_t uMotorAction = (obj->m_data >> 16) & 0xFF;
		uint16_t uMotorIncrease = 500;
		switch (uWhichMotor) {
		case 0x1:		//Up/Down direction motor.
			if (uMotorAction == 0x1)		//clockwise.
			{
				//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 05 01 01 00 01 67 8B C6 EA
				HAL_GPIO_WritePin(SM_UpDown_NSLEEP_GPIO_Port,
						SM_UpDown_NSLEEP_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(SM_UpDown_DIR_GPIO_Port, SM_UpDown_DIR_Pin,
						GPIO_PIN_RESET);
				HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
				HAL_TIM_Base_Start_IT(&htim3);
			} else if (uMotorAction == 0x2)		//anti-clockwise.
			{
				//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 05 01 02 00 01 65 CD 78 B1
				HAL_GPIO_WritePin(SM_UpDown_NSLEEP_GPIO_Port,
						SM_UpDown_NSLEEP_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(SM_UpDown_DIR_GPIO_Port, SM_UpDown_DIR_Pin,
						GPIO_PIN_SET);
				HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
				HAL_TIM_Base_Start_IT(&htim3);
			}
			break;
		case 0x2:		//Left/Right direction motor.
			if (uMotorAction == 0x1)		//clockwise.
			{
				//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 05 02 01 00 01 75 3E 69 08
				HAL_GPIO_WritePin(SM_LeftRight_NSLEEP_GPIO_Port,
						SM_LeftRight_NSLEEP_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(SM_LeftRight_DIR_GPIO_Port,
						SM_LeftRight_DIR_Pin, GPIO_PIN_RESET);
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
				HAL_TIM_Base_Start_IT(&htim4);
			} else if (uMotorAction == 0x2)		//anti-clockwise.
			{
				//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 05 02 02 00 01 77 78 D7 5F
				HAL_GPIO_WritePin(SM_LeftRight_NSLEEP_GPIO_Port,
						SM_LeftRight_NSLEEP_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(SM_LeftRight_DIR_GPIO_Port,
						SM_LeftRight_DIR_Pin, GPIO_PIN_SET);
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
				HAL_TIM_Base_Start_IT(&htim4);
			}
			break;
		default:
			break;
		}
		zsyTxResponse(nReg_2DBracketCtl_W, obj->m_data);
	}
	break;
	case nReg_OutVolume_R:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 00 00 06 00 00 00 01 11 43 5b 9a
		zsyTxResponse(nReg_OutVolume_R, obj->m_uOutputVol);
		break;
	case nReg_OutVolume_W:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 06 00 00 00 64 5b 72 db 43
		//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 06 00 00 00 01 66 aa 4e 94
		//m62429 has two input/output channel.
		//Data Input Format
		//D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10
		//D0:	0:1ch,1:2ch
		//D1:	0:Both channels at a time,1:A channel at a time.
		//D2-D8: volume.
		//D9: 1,fixed value.
		//D10:1,fixed value.
		//Volume Code from Datasheet.
		//To reach 1dB precision，we must combine D2~D6 and D7~D8 two bit field.
		//we adjust two channels so D0D1=10,and D9D10=11 fixed value.
		//0dB: ATT1=0dB,ATT2=0dB,10101,11 ->10,10101,11,11=0x55F
		//-1dB: ATT1=0dB,ATT2=-1dB,10101,01 ->10,10101,01,11=0x557
		//-2dB: ATT1=0dB,ATT2=-2dB,10101,10 ->10,10101,10,11=0x55B
		//-3dB: ATT1=0dB,ATT2=-3dB,10101,00 ->10,10101,00,11=0x553
		//-4dB: ATT1=-4dB,ATT2=0dB,00101,11 ->10,00101,11,11=0x45F
		//-5dB: ATT1=-4dB,ATT2=-1dB,00101,01 ->10,00101,01,11=0x457
		//-6dB: ATT1=-4dB,ATT2=-2dB,00101,10 ->10,00101,10,11=0x45B
		//-7dB: ATT1=-4dB,ATT2=-3dB,00101,00 ->10,00101,00,11=0x453
		//-8dB: ATT1=-8dB,ATT2=0dB,11001,11 ->10,11001,11,11=0x59F
		//......................
		//-10dB: ATT1=-8dB,ATT2=-2dB,11001,10 ->10,11001,10,11=0x59B
		//......................
		//-20dB: ATT1=-20dB,ATT2=0dB,00001,11 ->10,00001,11,11=0x41F
		//......................
		//-30dB: ATT1=-28dB,ATT2=-2dB,01110,10 ->10,01110,10,11=0x4EB
		//......................
		//-40dB: ATT1=-40dB,ATT2=0dB,11010,11 ->10,11010,11,11=0x5AF
		//......................
		//-50dB: ATT1=-48dB,ATT2=-2dB,10010,10 ->10,10010,11,11=0x52F
		//......................
		//-83dB: ATT1=-80dB,ATT2=-4dB,10000,00 ->10,10000,00,11=0x503
	{
		uint32_t  volume=obj->m_data;
		uint32_t uVolumedB;
		if (volume > 80 && volume <= 100)
		{
			uVolumedB			= 0x55F;				/*0dB*/
			obj->m_uOutputVol		= 100;
		}
		else if (volume > 60 && volume <= 80)
		{
			uVolumedB			= 0x59B;				/*-10dB*/
			obj->m_uOutputVol		= 80;
		}
		else if (volume > 40 && volume <= 60)
		{
			uVolumedB			= 0x41F;				/*-20dB*/
			obj->m_uOutputVol		= 60;
		}
		else if (volume > 20 && volume <= 40)
		{
			uVolumedB			= 0x4EB;				/*-30dB*/
			obj->m_uOutputVol		= 40;
		}
		else if (volume > 10 && volume <= 20)
		{
			uVolumedB			= 0x5AF;				/*-40dB*/
			obj->m_uOutputVol		= 20;
		}
		else if (volume > 0 && volume <= 10)
		{
			uVolumedB			= 0x52F;				/*-50dB*/
			obj->m_uOutputVol		= 10;
		}
		else
		{
			uVolumedB			= 0x503;				/*-83dB*/
			obj->m_uOutputVol		= 0;
		}

		for (uint8_t i = 0; i < 11; i++)
		{
			HAL_GPIO_WritePin(GPIOD, M62429_DAT_Pin, GPIO_PIN_RESET);
			zsy_Delay();
			HAL_GPIO_WritePin(GPIOD, M62429_CLK_Pin, GPIO_PIN_RESET);
			zsy_Delay();

			if ((uVolumedB & 0x400))
			{
				HAL_GPIO_WritePin(GPIOD, M62429_DAT_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOD, M62429_DAT_Pin, GPIO_PIN_RESET);
			}

			zsy_Delay();
			HAL_GPIO_WritePin(GPIOD, M62429_CLK_Pin, GPIO_PIN_SET);
			zsy_Delay();
			uVolumedB <<= 1;
		}
		HAL_GPIO_WritePin(GPIOD, M62429_DAT_Pin, GPIO_PIN_SET);
		zsy_Delay();
		HAL_GPIO_WritePin(GPIOD, M62429_CLK_Pin, GPIO_PIN_RESET);
		zsy_Delay();
		HAL_GPIO_WritePin(GPIOD, M62429_CLK_Pin, GPIO_PIN_SET);
		zsyTxResponse(nReg_OutVolume_W, obj->m_uOutputVol);

		//output volume was changed,we should remember it.
		zsy_FlushToFlash();
	}
	break;
	case nReg_VideoCtl_R:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 00 00 07 00 00 00 01 2C 23 72 2A
		zsyTxResponse(nReg_VideoCtl_R, obj->m_uVideoCtl);
		break;
	case nReg_VideoCtl_W:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 07 00 00 00 02 3d 3c c9 62
		if(obj->m_data==0x1)
		{
			obj->m_uVideoCtl=1;
		}else if(obj->m_data==0x2)
		{
			obj->m_uVideoCtl=2;
		}else{
			//other value does not support.
		}
		zsyTxResponse(nReg_VideoCtl_W, obj->m_data);

		//video control register was changed,we should remember it.
		zsy_FlushToFlash();
		break;
	case nReg_FPGA_R:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 00 00 08 00 00 00 01 51 8C 1A 07
		zsyTxResponse(nReg_FPGA_R, obj->m_uFPGAReg);
		break;
	case nReg_FPGA_W:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 08 00 00 00 02 40 93 a1 4f
		obj->m_uFPGAReg=obj->m_data;
		zsyTxResponse(nReg_FPGA_W, obj->m_uFPGAReg);

		//FPGA register was changed,we should remember it.
		zsy_FlushToFlash();
		break;
	case nReg_CrossXY_R:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 00 00 09 01 23 04 56 7F 38 AD 38
		zsyTxResponse(nReg_CrossXY_R, obj->m_uCrossXY);
		break;
	case nReg_CrossXY_W:
		//44 45 43 54 00 00 00 10 00 00 00 00 01 80 00 09 01 23 04 56 08 D1 B8 36
		obj->m_uCrossXY=obj->m_data;
		zsyTxResponse(nReg_CrossXY_W, obj->m_data);

		//CrossXY register was changed,we should remember it.
		zsy_FlushToFlash();
		break;
	case nReg_AutoFocus1_R:
		zsyTxResponse(nReg_AutoFocus1_R, obj->m_data);
		break;
	case nReg_AutoFocus1_W:
		zsyTxResponse(nReg_AutoFocus1_W, obj->m_data);
		break;
	default:
		break;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_TIM1_Init();
	MX_TIM8_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	/* USER CODE BEGIN 2 */

	//zsy:read Flash to restore previous parameters.
	zsy_LoadFromFlash();

	//zsy:start TIM1-CH1/CH1N to driver DC motor.
	//zsy:DC motor to get ZeroPoint through Peak Current at startup.
	HAL_GPIO_WritePin(NSLEEP_GPIO_Port, NSLEEP_Pin, GPIO_PIN_SET);
	//>50%:clockwise,<50%:anti-clockwise,=50%:stop.
	//CCR/ARR=180/360=50%.
	//set Duty Cycle to 0% to get the maximum speed.
	//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

	//zsy:start encoder.
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);

	//delay 100ms to bypass the DC motor startup peak current.
	//if here less than 100ms, the DC motor maybe cannot startup.
	//when it startup,peak current ADC IRQ trigger immediately.
	//so keep it more than 100ms.
	HAL_Delay(100);

	//zsy:start ADC.
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) g_ADC1DMABuffer, 30 * 2);
	HAL_ADC_Start(&hadc2);

	//zsy:nSLEEP=0 to save power.
	HAL_GPIO_WritePin(SM_UpDown_NSLEEP_GPIO_Port, SM_UpDown_NSLEEP_Pin,
			GPIO_PIN_RESET);
	//zsy:nSLEEP=0 to save power.
	HAL_GPIO_WritePin(SM_LeftRight_NSLEEP_GPIO_Port, SM_LeftRight_NSLEEP_Pin,
			GPIO_PIN_SET);

	//zsy:uart1 receive with DMA.
	HAL_UART_Receive_DMA(&huart1, g_Uart1DmaBuffer, CMD_LENGTH_MIN);

	//zsy:modbus protocol.
	modbusObj.m_fsm = UnPack_Sync_Filed;
	modbusObj.m_uBatteryPercent = 0;
	modbusObj.m_uRSSIPercent = 0;
	modbusObj.m_uDistance = 0;

	//if this initial value is 0,it may cause error.
	//here we do not set to 0,because it reaches the boundary.
	//0-1=65535 or 65535+1=0.
	//so here we use 500 as the ZeroPoint encoder value.
	modbusObj.m_uTargetEncoder = 500;

	/* USER CODE END 2 */
	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of ZPidTask */
	ZPidTaskHandle = osThreadNew(ZPidTaskLoop, NULL, &ZPidTask_attributes);

	/* creation of ZModBusTask */
	ZModBusTaskHandle = osThreadNew(ZModBusTaskLoop, NULL, &ZModBusTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analog WatchDog 1
	 */
	AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
	AnalogWDGConfig.HighThreshold = 2000;
	AnalogWDGConfig.LowThreshold = 0;
	AnalogWDGConfig.Channel = ADC_CHANNEL_8;
	AnalogWDGConfig.ITMode = ENABLE;
	if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 2;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 360;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 4;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 10000;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 7200;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 10000;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 7200;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 65535;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 4;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 1000;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SM_LeftRight_NSLEEP_GPIO_Port, SM_LeftRight_NSLEEP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NSLEEP_GPIO_Port, NSLEEP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, SM_LeftRight_DIR_Pin|SM_UpDown_NSLEEP_Pin|SM_UpDown_DIR_Pin|M62429_DAT_Pin
			|M62429_CLK_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SM_LeftRight_NSLEEP_Pin */
	GPIO_InitStruct.Pin = SM_LeftRight_NSLEEP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SM_LeftRight_NSLEEP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NSLEEP_Pin */
	GPIO_InitStruct.Pin = NSLEEP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(NSLEEP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SM_LeftRight_DIR_Pin SM_UpDown_NSLEEP_Pin SM_UpDown_DIR_Pin M62429_DAT_Pin
                           M62429_CLK_Pin */
	GPIO_InitStruct.Pin = SM_LeftRight_DIR_Pin|SM_UpDown_NSLEEP_Pin|SM_UpDown_DIR_Pin|M62429_DAT_Pin
			|M62429_CLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//CRC32 algorithm.
uint32_t g_CRC32Table[] = { 0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL,
		0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L, 0x0edb8832L,
		0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL,
		0xe7b82d07L, 0x90bf1d91L, 0x1db71064L, 0x6ab020f2L, 0xf3b97148L,
		0x84be41deL, 0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
		0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL,
		0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L, 0x3b6e20c8L, 0x4c69105eL,
		0xd56041e4L, 0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL,
		0xa50ab56bL, 0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L,
		0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L, 0x26d930acL,
		0x51de003aL, 0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L,
		0xcfba9599L, 0xb8bda50fL, 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L,
		0xb10be924L, 0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,
		0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L,
		0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L, 0x7807c9a2L, 0x0f00f934L,
		0x9609a88eL, 0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L,
		0xe6635c01L, 0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL,
		0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L, 0x65b0d9c6L,
		0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L,
		0x8cd37cf3L, 0xfbd44c65L, 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L,
		0xd4bb30e2L, 0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
		0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L,
		0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L, 0x5005713cL, 0x270241aaL,
		0xbe0b1010L, 0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L,
		0xce61e49fL, 0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L,
		0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL, 0xedb88320L,
		0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL,
		0x04db2615L, 0x73dc1683L, 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL,
		0x7a6a5aa8L, 0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
		0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL,
		0x806567cbL, 0x196c3671L, 0x6e6b06e7L, 0xfed41b76L, 0x89d32be0L,
		0x10da7a5aL, 0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L,
		0x60b08ed5L, 0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L,
		0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL, 0xd80d2bdaL,
		0xaf0a1b4cL, 0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L,
		0x316e8eefL, 0x4669be79L, 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L,
		0x5268e236L, 0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
		0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L,
		0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL, 0x9b64c2b0L, 0xec63f226L,
		0x756aa39cL, 0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L,
		0x05005713L, 0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L,
		0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L, 0x86d3d2d4L,
		0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL,
		0x6fb077e1L, 0x18b74777L, 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL,
		0x11010b5cL, 0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,
		0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L,
		0xd06016f7L, 0x4969474dL, 0x3e6e77dbL, 0xaed16a4aL, 0xd9d65adcL,
		0x40df0b66L, 0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL,
		0x30b5ffe9L, 0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L,
		0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL, 0xb3667a2eL,
		0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L,
		0x5a05df1bL, 0x2d02ef8dL };

uint32_t g_CRC32JAMCRCCalculate(uint8_t *s, int len) {
	int i;
	uint32_t crc32val = 0;

	crc32val ^= 0xFFFFFFFF;

	for (i = 0; i < len; i++) {
		crc32val = g_CRC32Table[(crc32val ^ s[i]) & 0xFF]
								^ ((crc32val >> 8) & 0x00FFFFFF);
	}

	return labs(crc32val ^ 0xFFFFFFFF);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ZPidTaskLoop */
/**
 * @brief Function implementing the ZPidTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ZPidTaskLoop */
void ZPidTaskLoop(void *argument)
{
	/* USER CODE BEGIN ZPidTaskLoop */
	/* Infinite loop */

	ZModBusObject *obj = (ZModBusObject*) argument;

	for (;;) {
#if 0
		//do PID after reached ZeroPoint.
		if (g_flagDCMotorPeakCurrent) {
			float fPIDOut;
			uint32_t uCurrentEncoder;
			uint32_t uPWMCtl = 0;

			//setup CCR，Duty Cycle=CCR/ARR,>50%:clockwise,<50%:anti-clockwise,=50%:stop.
			//0%:clockwise at maximum speed,100%:anti-clockwise at maximum speed.
			//at initial we set TIM1 ARR=360, so if we want duty cycle equals 50%，then 50%=CCR/ARR -> CCR=180.
			//so here CCR=180-pidOut,this is we expect value.
			//if (target-current) --->>>0 ,180-pidOut--->>>180，now duty cycle--->>>50%,motor stop.
			//   when (target-current) located in (0~180) range:
			//   1.if diff=1,then 180-pidOut=180-1=179,duty cycle=179/360=49.72%
			//   2.if diff=10,then 180-pidOut=180-10=170,duty cycle=170/360=47.22%
			//   3.if diff=160,then 180-pidOut=180-160=20,duty cycle=20/360=5.5%
			//   4.if diff=180,then 180-pidOut=180-180=0,duty cycle=0/360=0% (clockwise maximum speed)

			//if (target-current) located in (-180~0) range:
			//   1.if diff=-1,then 180-pidOut=180-(-1)=181,duty cycle=181/360=50.27%
			//   2.if diff=-10,then 180-pidOut=180-(-10)=190,duty cycle=190/360=52.77%
			//   3.if diff=-160,then 180-pidOut=180-(-160)=340,duty cycle=340/360=94.44%
			//   4.if diff=-180�?,then 180-pidOut=180-(-180)=360,duty cycle=360/360=100% (anti-clockwise maximum speed)
			uCurrentEncoder = __HAL_TIM_GET_COUNTER(&htim5);
			fPIDOut = zsy_PIDCalculate(obj, obj->m_uTargetEncoder,
					uCurrentEncoder);
			if (fPIDOut > 180) {
				fPIDOut = 180;
			} else if (fPIDOut < -180) {
				fPIDOut = -180;
			}

			uPWMCtl = 180 - fPIDOut;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, uPWMCtl);
		}
#endif
		osDelay(1);
	}
	/* USER CODE END ZPidTaskLoop */
}

/* USER CODE BEGIN Header_ZModBusTaskLoop */
/**
 * @brief Function implementing the ZModBusTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ZModBusTaskLoop */
void ZModBusTaskLoop(void *argument)
{
	/* USER CODE BEGIN ZModBusTaskLoop */
	/* Infinite loop */
	for (;;) {
		//do parse out UART1 ModBus protocol.
		if (g_FlagUart1GetNewData) {
			uint32_t i = 0;
			for (i = 0; i < g_Uart1DmaPollLength; i += 4) {
				uint32_t t0 = g_Uart1DmaPoll[i + 0];
				uint32_t t1 = g_Uart1DmaPoll[i + 1];
				uint32_t t2 = g_Uart1DmaPoll[i + 2];
				uint32_t t3 = g_Uart1DmaPoll[i + 3];
				uint32_t uRxData = (t0 << 24) | (t1 << 16) | (t2 << 8)
																		| (t3 << 0);
				switch (modbusObj.m_fsm) {
				case UnPack_Sync_Filed:
					if (uRxData == 0x44454354) {
						modbusObj.m_flagEncrypt = 0;
						modbusObj.m_fsm = UnPack_Length_Field;
					}
					if (uRxData == 0x454E4354) {
						modbusObj.m_flagEncrypt = 1;
						modbusObj.m_fsm = UnPack_Length_Field;
					} else {
						//something wrong,bypass this int(4 bytes).
					}
					memcpy(modbusObj.m_crc32Buffer, (void*) &g_Uart1DmaPoll[i],
							sizeof(uint32_t));
					modbusObj.m_crc32BufferLen = sizeof(uRxData);
					break;
				case UnPack_Length_Field:
					//Sync(4)+Length(4)+Key(4)+Address(4)+Data(4)+Padding(0)+CRC32(4)
					//So Length=Key(4)+Address(4)+Data(4)+Padding(0)+CRC32(4)=16 at least.
					if (uRxData < 16) {
						modbusObj.m_fsm = UnPack_Sync_Filed;
					} else {
						modbusObj.m_length = uRxData;
						modbusObj.m_fsm = UnPack_Key_Field;
						memcpy(
								&modbusObj.m_crc32Buffer[modbusObj.m_crc32BufferLen],
								(void*) &g_Uart1DmaPoll[i], sizeof(uint32_t));
						modbusObj.m_crc32BufferLen += sizeof(uRxData);
					}
					break;
				case UnPack_Key_Field:
					modbusObj.m_key = uRxData;
					modbusObj.m_fsm = UnPack_Address_Field;
					memcpy(&modbusObj.m_crc32Buffer[modbusObj.m_crc32BufferLen],
							(void*) &g_Uart1DmaPoll[i], sizeof(uint32_t));
					modbusObj.m_crc32BufferLen += sizeof(uRxData);
					break;
				case UnPack_Address_Field:
					modbusObj.m_address = uRxData;
					modbusObj.m_fsm = UnPack_Data_Field;
					memcpy(&modbusObj.m_crc32Buffer[modbusObj.m_crc32BufferLen],
							(void*) &g_Uart1DmaPoll[i], sizeof(uint32_t));
					modbusObj.m_crc32BufferLen += sizeof(uRxData);
					break;
				case UnPack_Data_Field:
					modbusObj.m_data = uRxData;
					modbusObj.m_fsm = UnPack_CRC32_Field;
					memcpy(&modbusObj.m_crc32Buffer[modbusObj.m_crc32BufferLen],
							(void*) &g_Uart1DmaPoll[i], sizeof(uint32_t));
					modbusObj.m_crc32BufferLen += sizeof(uRxData);
					break;
				case UnPack_CRC32_Field:
					modbusObj.m_crc32 = uRxData;
					modbusObj.m_fsm = UnPack_Sync_Filed;
					uint32_t uCalcCRC32 = g_CRC32JAMCRCCalculate(
							modbusObj.m_crc32Buffer,
							modbusObj.m_crc32BufferLen);
					if (modbusObj.m_crc32 == uCalcCRC32) {
						zsyParseModBusFrame(&modbusObj);
					}
					break;
				default:
					modbusObj.m_fsm = UnPack_Sync_Filed;
					break;
				}
			}

			g_Uart1DmaPollLength = 0;
			//reset flag.
			g_FlagUart1GetNewData = 0;
			HAL_UART_Receive_DMA(&huart1, g_Uart1DmaBuffer, CMD_LENGTH_MIN);
		}
		osDelay(10);
	}
	/* USER CODE END ZModBusTaskLoop */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM7) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM4) {
		HAL_GPIO_WritePin(SM_LeftRight_NSLEEP_GPIO_Port,
				SM_LeftRight_NSLEEP_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	} else if (htim->Instance == TIM3) {
		HAL_GPIO_WritePin(SM_UpDown_NSLEEP_GPIO_Port, SM_UpDown_NSLEEP_Pin,
				GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	}
	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
