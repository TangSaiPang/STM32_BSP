/**
  ******************************************************************************
  * 文件名程: bsp_MB_slave.h 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2019-04-16
  * 功    能: MODBUS-API
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-H7Multi使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
#ifndef __BSP_MB_SLAVE_H__
#define __BSP_MB_SLAVE_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  Code ;  	        // 功能码
  __IO uint8_t byteNums; 	        // 字节数
  __IO uint16_t Addr ;            // 操作内存的起始地址
  __IO uint16_t Num; 	            // 寄存器或者线圈的数量
  __IO uint16_t _CRC;       	      // CRC校验码
  __IO uint8_t *ValueReg; 	      // 10H功能码的数据
  __IO uint16_t *PtrHoldingbase;  // HoldingReg内存首地址
  __IO uint16_t *PtrHoldingOffset;// HoldingReg内存首地址
}PDUData_TypeDef;

typedef struct
{
  uint16_t IN1;
	
	/* 01H 05H 读写单个强制线圈 */
	uint16_t D01;
	uint16_t D02;
	uint16_t D03;
	uint16_t D04;

}REG_VALUE;

/* 宏定义 --------------------------------------------------------------------*/
#define MB_SLAVEADDR            0x0001
#define MB_ALLSLAVEADDR         0x00FF

#define FUN_CODE_01H            0x01  // 功能码01H 
#define FUN_CODE_02H            0x02  // 功能码02H
#define FUN_CODE_03H            0x03  // 功能码03H
#define FUN_CODE_04H            0x04  // 功能码04H
#define FUN_CODE_05H            0x05  // 功能码05H
#define FUN_CODE_06H            0x06  // 功能码06H
#define FUN_CODE_10H            0x10  // 功能码10H

/* 本例程所支持的功能码,需要添加新功能码还需要在.c文件里面添加 */
#define IS_NOT_FUNCODE(code)  (!((code == FUN_CODE_01H)||\
                                 (code == FUN_CODE_02H)||\
                                 (code == FUN_CODE_03H)||\
                                 (code == FUN_CODE_04H)||\
                                 (code == FUN_CODE_05H)||\
                                 (code == FUN_CODE_06H)||\
                                 (code == FUN_CODE_10H)))

#define EX_CODE_NONE           0x00  // 异常码 无异常
#define EX_CODE_01H            0x01  // 异常码
#define EX_CODE_02H            0x02  // 异常码
#define EX_CODE_03H            0x03  // 异常码
#define EX_CODE_04H            0x04  // 异常码

#define COIL_D01		0x01
#define COIL_D02		0x02
#define COIL_D03		0x03
#define COIL_D04		0x04

#define REG_IN1		  0x0020
/* 03H 读保持寄存器 */
/* 06H 写保持寄存器 */
/* 10H 写多个保存寄存器 */
#define HOLD_REG_01		0x0010
#define HOLD_REG_02		0x0011
#define HOLD_REG_03		0x0012

/* 扩展变量 ------------------------------------------------------------------*/
extern PDUData_TypeDef PduData;

/* 函数声明 ------------------------------------------------------------------*/
uint16_t MB_CRC16(uint8_t *pushMsg,uint8_t usDataLen);
void MB_Parse_Data(void);
void MB_WriteNumHoldingReg_10H(uint8_t _addr, uint16_t _reg, uint16_t _num,uint8_t *_databuf);
uint8_t MB_Analyze_Execute(void );
uint8_t MB_JudgeNum(uint16_t _Num,uint8_t _FunCode,uint16_t ByteNum);
uint8_t MB_JudgeAddr(uint16_t _Addr,uint16_t _Num);
void MB_Exception_RSP(uint8_t _FunCode,uint8_t _ExCode);
void MB_RSP(uint8_t _FunCode);

#endif /* __BSP_MB_SLAVE_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
