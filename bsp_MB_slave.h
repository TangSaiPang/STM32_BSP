/**
  ******************************************************************************
  * �ļ�����: bsp_MB_slave.h 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2019-04-16
  * ��    ��: MODBUS-API
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-H7Multiʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
#ifndef __BSP_MB_SLAVE_H__
#define __BSP_MB_SLAVE_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  Code ;  	        // ������
  __IO uint8_t byteNums; 	        // �ֽ���
  __IO uint16_t Addr ;            // �����ڴ����ʼ��ַ
  __IO uint16_t Num; 	            // �Ĵ���������Ȧ������
  __IO uint16_t _CRC;       	      // CRCУ����
  __IO uint8_t *ValueReg; 	      // 10H�����������
  __IO uint16_t *PtrHoldingbase;  // HoldingReg�ڴ��׵�ַ
  __IO uint16_t *PtrHoldingOffset;// HoldingReg�ڴ��׵�ַ
}PDUData_TypeDef;

typedef struct
{
  uint16_t IN1;
	
	/* 01H 05H ��д����ǿ����Ȧ */
	uint16_t D01;
	uint16_t D02;
	uint16_t D03;
	uint16_t D04;

}REG_VALUE;

/* �궨�� --------------------------------------------------------------------*/
#define MB_SLAVEADDR            0x0001
#define MB_ALLSLAVEADDR         0x00FF

#define FUN_CODE_01H            0x01  // ������01H 
#define FUN_CODE_02H            0x02  // ������02H
#define FUN_CODE_03H            0x03  // ������03H
#define FUN_CODE_04H            0x04  // ������04H
#define FUN_CODE_05H            0x05  // ������05H
#define FUN_CODE_06H            0x06  // ������06H
#define FUN_CODE_10H            0x10  // ������10H

/* ��������֧�ֵĹ�����,��Ҫ����¹����뻹��Ҫ��.c�ļ�������� */
#define IS_NOT_FUNCODE(code)  (!((code == FUN_CODE_01H)||\
                                 (code == FUN_CODE_02H)||\
                                 (code == FUN_CODE_03H)||\
                                 (code == FUN_CODE_04H)||\
                                 (code == FUN_CODE_05H)||\
                                 (code == FUN_CODE_06H)||\
                                 (code == FUN_CODE_10H)))

#define EX_CODE_NONE           0x00  // �쳣�� ���쳣
#define EX_CODE_01H            0x01  // �쳣��
#define EX_CODE_02H            0x02  // �쳣��
#define EX_CODE_03H            0x03  // �쳣��
#define EX_CODE_04H            0x04  // �쳣��

#define COIL_D01		0x01
#define COIL_D02		0x02
#define COIL_D03		0x03
#define COIL_D04		0x04

#define REG_IN1		  0x0020
/* 03H �����ּĴ��� */
/* 06H д���ּĴ��� */
/* 10H д�������Ĵ��� */
#define HOLD_REG_01		0x0010
#define HOLD_REG_02		0x0011
#define HOLD_REG_03		0x0012

/* ��չ���� ------------------------------------------------------------------*/
extern PDUData_TypeDef PduData;

/* �������� ------------------------------------------------------------------*/
uint16_t MB_CRC16(uint8_t *pushMsg,uint8_t usDataLen);
void MB_Parse_Data(void);
void MB_WriteNumHoldingReg_10H(uint8_t _addr, uint16_t _reg, uint16_t _num,uint8_t *_databuf);
uint8_t MB_Analyze_Execute(void );
uint8_t MB_JudgeNum(uint16_t _Num,uint8_t _FunCode,uint16_t ByteNum);
uint8_t MB_JudgeAddr(uint16_t _Addr,uint16_t _Num);
void MB_Exception_RSP(uint8_t _FunCode,uint8_t _ExCode);
void MB_RSP(uint8_t _FunCode);

#endif /* __BSP_MB_SLAVE_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
