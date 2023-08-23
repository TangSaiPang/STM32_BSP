/**
  ******************************************************************************
  * �ļ�����: bsp_MB_slave.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2018-09-01
  * ��    ��: MODBUS�ӻ�
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-H7Multiʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/  
#include "bsp_MB_slave.h"
#include "bsp_RS485.h"
#include "bsp_GeneralTIM.h"
#include "bsp_led.h"
#include "bsp_debug_usart.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
PDUData_TypeDef PduData;
REG_VALUE R_value;

// CRC ��λ�ֽ�ֵ��
static const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
// CRC ��λ�ֽ�ֵ��
static const uint8_t auchCRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

__IO uint8_t  LED_state=0;


/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
static uint8_t MB_JudgeAddr(uint16_t _Addr,uint16_t _RegNum);
static uint8_t MB_JudgeNum(uint16_t _RegNum,uint8_t _FunCode,uint16_t _ByteNum);
static uint16_t MB_RSP_01H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _CoilNum );
static uint16_t MB_RSP_02H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _CoilNum);
static uint8_t MB_RSP_03H(uint16_t _TxCount,uint16_t *_AddrOffset,uint16_t _RegNum );
static uint8_t MB_RSP_04H(uint16_t _TxCount,uint16_t _AddrOffset,uint16_t _RegNum );
static uint8_t MB_RSP_05H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _RegDATA);
static uint8_t MB_RSP_06H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _RegNum ,uint16_t *_AddrAbs);
static uint8_t MB_RSP_10H(uint16_t _TxCount,uint16_t _AddrOffset,uint16_t _RegNum ,uint16_t *_AddrAbs ,uint8_t* _Datebuf);

/* ������ --------------------------------------------------------------------*/

/** 
  * ��������: Modbus CRC16 У����㺯��
  * �������: pushMsg:������������׵�ַ,usDataLen:���ݳ���
  * �� �� ֵ: CRC16 ������
  * ˵    ��: �������Ǹ�λ��ǰ,��Ҫת�����ܷ���
  */
uint16_t MB_CRC16(uint8_t *_pushMsg,uint8_t _usDataLen)
{
  uint8_t uchCRCHi = 0xFF;
  uint8_t uchCRCLo = 0xFF;
  uint16_t uIndex;
  while(_usDataLen--)
  {
    uIndex = uchCRCLo ^ *_pushMsg++;
    uchCRCLo = uchCRCHi^auchCRCHi[uIndex];
    uchCRCHi = auchCRCLo[uIndex];
  }
  return (uchCRCHi<<8|uchCRCLo);
}

/* ��ȡ����֡,���н�������֡ */
void MB_Parse_Data()
{
  PduData.Code = Rx_Buf[1];                   // ������
  PduData.Addr = ((Rx_Buf[2]<<8) | Rx_Buf[3]);// �Ĵ�����ʼ��ַ
  PduData.Num  = ((Rx_Buf[4]<<8) | Rx_Buf[5]);// ����(Coil,Input,Holding Reg,Input Reg)
  PduData._CRC = MB_CRC16((uint8_t*)&Rx_Buf,RxCount-2);             // CRCУ����
  PduData.byteNums = Rx_Buf[6];                                     // ����ֽ���
  PduData.ValueReg = (uint8_t*)&Rx_Buf[7];                          // �Ĵ���ֵ��ʼ��ַ
  PduData.PtrHoldingOffset = PduData.PtrHoldingbase + PduData.Addr; // ���ּĴ�������ʼ��ַ
}

/** 
  * ��������: �Խ��յ������ݽ��з�����ִ��
  * �������: ��
  * �� �� ֵ: �쳣���0x00
  * ˵    ��: �жϹ�����,��֤��ַ�Ƿ���ȷ.��ֵ�����Ƿ����,����û����ͷ�����Ӧ�ź�
  */
uint8_t MB_Analyze_Execute(void )
{
  uint16_t ExCode = EX_CODE_NONE;
  /* У�鹦���� */
  if( IS_NOT_FUNCODE(PduData.Code) ) // ��֧�ֵĹ�����
  {
    /* Modbus�쳣��Ӧ */
    ExCode = EX_CODE_01H;            // �쳣��01H
    return ExCode;
  }
  /* ���ݹ�����ֱ����ж� */
  switch(PduData.Code)
  {
    /* ������Ϊ01H�������02��������һ����,��ʵҲûʲô��һ��
     * ֻ�ǲ�����ַ���ܲ�һ��,��һ���Ͼ�����ʵ��,������main����
     * ���뵥�����ڴ�ʹ�ò�ͬ�Ĺ�����,��ʵ��Ӧ���б����������ʹ��
     * ��ͬ���ڴ�ռ�
     */
/* ---- 01H  02H ��ȡ��ɢ������(Coil Input)---------------------- */
    case FUN_CODE_01H:
    case FUN_CODE_02H:
      /* �ж���Ȧ�����Ƿ���ȷ */  
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,1);
      if(ExCode != EX_CODE_NONE )
        return ExCode;      
      
      /* �жϵ�ַ�Ƿ���ȷ*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      break;
/* ---- 03H  04H ��ȡ����/����Ĵ���---------------------- */
    case FUN_CODE_03H:
    case FUN_CODE_04H:
      /* �жϼĴ��������Ƿ���ȷ */
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,PduData.byteNums);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      
      /* �жϵ�ַ�Ƿ���ȷ*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      break;
/* ---- 05H д�뵥����ɢ��---------------------- */
    case FUN_CODE_05H:
      break;
/* ---- 06H д�������ּĴ��� ---------------------- */
    case FUN_CODE_06H:     
      break;
/* ---- 10H д������ּĴ��� ---------------------- */
    case FUN_CODE_10H:
      /* �жϼĴ��������Ƿ���ȷ */
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,PduData.byteNums);
      if(ExCode != EX_CODE_NONE )
        return ExCode;          
      /* �жϵ�ַ�Ƿ���ȷ*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);		      		
      if(ExCode != EX_CODE_NONE )
        return ExCode;  			
      break;
  }
  /* ����֡û���쳣 */
  return ExCode; //   EX_CODE_NONE
}

/**
  * ��������: ������Ӧ
  * �������: _FunCode :������
  * �� �� ֵ: ��
  * ˵    ��: ��ͨ������֡û���쳣ʱ���ҳɹ�ִ��֮��,������Ӧ����֡
  */
void MB_RSP(uint8_t _FunCode)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;	
	Tx_Buf[TxCount++] = MB_SLAVEADDR;		 /* ��վ��ַ */
	Tx_Buf[TxCount++] = _FunCode;        /* ������   */	
  switch(_FunCode)
  {
    case FUN_CODE_01H:
			/* ��ȡ��Ȧ״̬ */
			TxCount = MB_RSP_01H(TxCount,PduData.Addr,PduData.Num);
		  break;
    case FUN_CODE_02H:
			/* ��ȡ��ɢ���� */
      TxCount = MB_RSP_02H(TxCount,PduData.Addr,PduData.Num);
      break;		 
    case FUN_CODE_03H:
			 /* ��ȡ���ּĴ��� */
			TxCount = MB_RSP_03H(TxCount,(uint16_t*)PduData.PtrHoldingOffset,PduData.Num);
	  	break;
    case FUN_CODE_04H:
			/* ��ȡ����Ĵ��� */
			TxCount =	MB_RSP_04H(TxCount,PduData.Addr,PduData.Num);      
      break;
    case FUN_CODE_05H:
			/* д������Ȧ */
      TxCount = MB_RSP_05H(TxCount,PduData.Addr,PduData.Num);
      break;
    case FUN_CODE_06H:
			/* д�������ּĴ��� */
      TxCount = MB_RSP_06H(TxCount,PduData.Addr,PduData.Num, (uint16_t*)PduData.PtrHoldingOffset);
      break;
    case FUN_CODE_10H:
			/* д������ּĴ��� */
      TxCount = MB_RSP_10H(TxCount,PduData.Addr,PduData.Num ,(uint16_t*)PduData.PtrHoldingOffset,(uint8_t*)PduData.ValueReg);
      break;
  }
  crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  UART_Tx((uint8_t*)Tx_Buf, TxCount);
}

/**
  * ��������: ��ȡ��Ȧ״̬����/д��
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_CoilNum:��Ȧ����
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ��ȡ��ɢ���,�������Tx_Buf
  */
static uint16_t MB_RSP_01H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _CoilNum )
{
	/*
		��������:
			01 �ӻ���ַ
			01 ������
			00 �Ĵ�����ʼ��ַ���ֽ�
			02 �Ĵ�����ʼ��ַ���ֽ�
			00 �Ĵ����������ֽ�
			08 �Ĵ����������ֽ�
			9C CRCУ����ֽ�
			0C CRCУ����ֽ�

		�ӻ�Ӧ��: 	1����ON��0����OFF��ʹ��LED��״̬�����棩�������ص���Ȧ����Ϊ8�ı�����������������ֽ�δβʹ��0����. BIT0��Ӧ��1��
			01 �ӻ���ַ
			01 ������
			01 �����ֽ���
			02 ����1(��Ȧ0002H-��Ȧ0011H)
			D0 CRCУ����ֽ�
			49 CRCУ����ֽ�

		����1:
		���ͣ�	01 01 00 02 00 08   9C 0C	  --- ��ѯD02��ʼ��8���̵���״̬
		���أ�	01 01 01 01         90 48   --- ��ѯ��8��״̬Ϊ��0000 0001 �ڶ���LEDΪ��
		
		����2:
		���ͣ�	01 01 00 01 00 10   6C 06	  --- ��ѯD01��ʼ��16���̵���״̬
		���أ�	01 01 02 FF FF      B8 4C   --- ��ѯ�������ֽ�����Ϊ0xFFFF
				
	*/	
//  uint16_t i = 0;
//	uint16_t m;	
//	uint8_t status[10];	
//	
//  /* ���㷵���ֽ�����_CoilNum��������λΪ��λ�� */
//  m = (_CoilNum+7)/8;
//  /* �����ֽ�����������*/
//	Tx_Buf[_TxCount++] = m; 
//	if ((_AddrOffset >= COIL_D01) && (_CoilNum > 0))
//  {
//		/* ����ȡ����Ȧ״̬�������� */
//		for (i = 0; i < m; i++)
//		{
//			status[i] = 0;
//		}		
//		/* ��ȡ��Ӧ��Ȧ״̬��������д��status[] */
//		for (i = 0; i < _CoilNum; i++)
//		{
//			/* ��LED��״̬��д��״̬�Ĵ�����ÿһλ */
//			if (Get_LEDx_State(i + 1 + _AddrOffset - COIL_D01))		
//			{  
//				status[i / 8] |= (1 << (i % 8));
//			}			
//		}    		
//	}
//	/* ��䷢������ */
//	for (i = 0; i < m; i++)
//	{
//		Tx_Buf[_TxCount++] = status[i];	/* �̵���״̬ */
//	}		
	
	/*----------------------------�ָ���----------------------------------*/
	Tx_Buf[_TxCount++] = 2; 
  /* ��䷵������ */ 	
	if (_AddrOffset == COIL_D01)
	{
		Tx_Buf[_TxCount++] = R_value.D01>>8;
		Tx_Buf[_TxCount++] = R_value.D01;	
	}
	else if (_AddrOffset == COIL_D02)
	{
		Tx_Buf[_TxCount++] = R_value.D02>>8;
		Tx_Buf[_TxCount++] = R_value.D02;	
	}
	else if (_AddrOffset == COIL_D03)
	{
		Tx_Buf[_TxCount++] = R_value.D03>>8;
		Tx_Buf[_TxCount++] = R_value.D03;	
	}
	else if (_AddrOffset == COIL_D04)
	{
		Tx_Buf[_TxCount++] = R_value.D04>>8;
		Tx_Buf[_TxCount++] = R_value.D04;	
	}
	else
	{
		Tx_Buf[_TxCount++] = 0;
		Tx_Buf[_TxCount++] = 0;	
	}		

  return _TxCount;
}

/**
  * ��������: ��ȡ��ɢ���루ֻ����
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_CoilNum:��Ȧ����
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ��ȡ��ɢ���,�������Tx_Buf
  */
static uint16_t MB_RSP_02H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _CoilNum)
{
	/*
		��������:
			01 �ӻ���ַ
			02 ������
			00 �Ĵ�����ʼ��ַ���ֽ�
			01 �Ĵ�����ʼ��ַ���ֽ�
			00 �Ĵ����������ֽ�
			08 �Ĵ����������ֽ�
			28 CRCУ����ֽ�
			0C CRCУ����ֽ�

		�ӻ�Ӧ��: 	1����ON��0����OFF��ʹ��LED��״̬�����棩�������ص���Ȧ����Ϊ8�ı�����������������ֽ�δβʹ��0����. BIT0��Ӧ��1��
			01 �ӻ���ַ
			02 ������
			01 �����ֽ���
			02 ����1(��Ȧ0002H-��Ȧ0011H)
			D0 CRCУ����ֽ�
			49 CRCУ����ֽ�

		����:
		���ͣ�	01 01 00 02 00 08   9C 0C	  --- ��ѯD02��ʼ��8���̵���״̬
		���أ�	01 01 01 02         D0 49   --- ��ѯ��8��״̬Ϊ��0000 0010 �ڶ���LEDΪ��
	*/	
  uint16_t i = 0;
	uint16_t m;	
	uint8_t status[10];	
	
  /* ���㷵���ֽ�����_CoilNum��������λΪ��λ�� */
  m = (_CoilNum+7)/8;
  /* �����ֽ�����������*/
	Tx_Buf[_TxCount++] = m; 
	  
	if ((_AddrOffset >= COIL_D01) && (_CoilNum > 0))
  {
		/* ����ȡ����Ȧ״̬�������� */
		for (i = 0; i < m; i++)
		{
			status[i] = 0;
		}		
		/* ��ȡ��Ӧ��Ȧ״̬��������д��status[] */
		for (i = 0; i < _CoilNum; i++)
		{
			/* ��LED��״̬��д��״̬�Ĵ�����ÿһλ */
			if (Get_LEDx_State(i + 1 + _AddrOffset - COIL_D01))		
			{  
				status[i / 8] |= (1 << (i % 8));
			}			
		}
	}
	/* ��䷢������ */
	for (i = 0; i < m; i++)
	{
		/* �̵���״̬ */
		Tx_Buf[_TxCount++] = status[i];	
	}	
  return _TxCount;
}

/**
  * ��������: ��ȡ���ּĴ�������/д��
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_RegNum:�Ĵ�������
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ��ȡ���ּĴ���������,�������Tx_Buf
  */
static uint8_t MB_RSP_03H(uint16_t _TxCount,uint16_t *_AddrOffset,uint16_t _RegNum )
{
	/*
		�ӻ���ַΪ01H�����ּĴ�������ʼ��ַΪ0010H��������ַΪ0011H���ôβ�ѯ�ܹ�����2�����ּĴ�����
		��������:
			01 �ӻ���ַ
			03 ������
			00 �Ĵ�����ַ���ֽ�
			10 �Ĵ�����ַ���ֽ�
			00 �Ĵ����������ֽ�
			02 �Ĵ����������ֽ�
			C5 CRC���ֽ�
			CE CRC���ֽ�

		�ӻ�Ӧ��: 	���ּĴ����ĳ���Ϊ2���ֽڡ����ڵ������ּĴ������ԣ��Ĵ������ֽ������ȱ����䣬
					���ֽ����ݺ󱻴��䡣���ּĴ���֮�䣬�͵�ַ�Ĵ����ȱ����䣬�ߵ�ַ�Ĵ����󱻴��䡣
			01 �ӻ���ַ
			03 ������
			04 �ֽ���
			12 ����1���ֽ�(0010H)
			34 ����1���ֽ�(0010H)
			02 ����2���ֽ�(0011H)
			03 ����2���ֽ�(0100H)
			FF CRC���ֽ�
			F4 CRC���ֽ�

		��һ�����ּĴ�������:
			���ͣ�	01 03 00 10 00 01            85 CF ---- �� 0010Hһ���Ĵ�������
			���أ�	01 03 02 12 34               B5 33 ---- ����10H������д������ݣ�10H���������ܣ�
	*/
	
  /* ��䷵�ؼĴ������� */
  Tx_Buf[_TxCount++] = _RegNum*2;
  /* ���ر��ּĴ����ڵ����� */
  for(uint8_t i = 0;i< _RegNum;i++)
  {
    Tx_Buf[_TxCount++] = ((*_AddrOffset)>>8);
    Tx_Buf[_TxCount++] = *_AddrOffset++;
  }
  return _TxCount;
}

/**
  * ��������: ��ȡ����Ĵ�����������03Hָ�����ƣ���������Ҳ���ƣ���ֻ����
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_RegNum:�Ĵ�������
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ��ȡ���ּĴ���������,�������Tx_Buf
  */
static uint8_t MB_RSP_04H(uint16_t _TxCount,uint16_t _AddrOffset,uint16_t _RegNum )
{
	/*
		��������:
			01 �ӻ���ַ
			04 ������
			00 �Ĵ�����ʼ��ַ���ֽ�
			20 �Ĵ�����ʼ��ַ���ֽ�
			00 �Ĵ����������ֽ�
			02 �Ĵ����������ֽ�
			70 CRC���ֽ�
			01 CRC���ֽ�

		�ӻ�Ӧ��:  ����Ĵ�������Ϊ2���ֽڡ����ڵ�������Ĵ������ԣ��Ĵ������ֽ������ȱ����䣬
				���ֽ����ݺ󱻴��䡣����Ĵ���֮�䣬�͵�ַ�Ĵ����ȱ����䣬�ߵ�ַ�Ĵ����󱻴��䡣
			01 �ӻ���ַ
			04 ������
			02 �ֽ���
			02 ����1���ֽ�(0020H)
			03 ����1���ֽ�(0020H)
			00 ����2���ֽ�(0021H)
			00 ����2���ֽ�(0021H)
			82 CRC���ֽ�
			3C CRC���ֽ�

		����:
			���ͣ�	01 04 00 20 00 02      70 01  --- �� 0020H IN1 ��ʼ��2���ֽ���������
			���أ�	01 04 02 02 03 00 00   82 3C  --- ���أ�02 03 00 00 4�����ݣ�����KEY1������ı� R_value.IN1��ֵ��
	*/	
	uint8_t i;
	uint16_t reg_value[64];
  /* ��䷵�ؼĴ������� */
  Tx_Buf[_TxCount++] = _RegNum;
	/* ��ȡ���ּĴ������� */
	for(i = 0; i < _RegNum; i++)
	{
		switch (_AddrOffset)
		{
			/* ���Բ��� */
			case REG_IN1:
				reg_value[i] = R_value.IN1;
				break;
		
			default:
				reg_value[i] = 0;
				break;
		}
		_AddrOffset++;
	}
	
  /* ��䷵������ */
  for(i = 0;i< _RegNum;i++)
  {
		
    Tx_Buf[_TxCount++] = reg_value[i]>>8;
    Tx_Buf[_TxCount++] = reg_value[i]& 0xFF;
  }
  return _TxCount;
}

/**
  * ��������: д������Ȧ����/д��
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_RegDATA:д������
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ���Tx_Buf
  */
static uint8_t MB_RSP_05H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _RegDATA)
{
	/*
		��������: д������Ȧ�Ĵ������򵥵�01~03�Ĵ�����ַ��ӦLED1~LED3,�����ݴ����D01��D02��D03��D04��4����Ա��
		05Hָ�����õ�����Ȧ��״̬
			01 �ӻ���ַ
			05 ������
			00 �Ĵ�����ַ���ֽ�
			01 �Ĵ�����ַ���ֽ�
			FF ����1���ֽ�
			FF ����1���ֽ�
			9D CRCУ����ֽ�
			BA CRCУ����ֽ�

		�ӻ�Ӧ��:
			01 �ӻ���ַ
			05 ������
			00 �Ĵ�����ַ���ֽ�
			01 �Ĵ�����ַ���ֽ�
			FF �Ĵ���1���ֽ�
			FF �Ĵ���1���ֽ�
			9D CRCУ����ֽ�
			BA CRCУ����ֽ�

		����:
		���ͣ�	01 05 00 04 FF FF   8D BB   -- ��������0xFFFF��0x04��ַ��Ȧ��
		���أ�	01 05 00 04 FF FF   8D BB   -- ����ԭʼ����
	*/	
	
  /* ����ֵַ */
  Tx_Buf[_TxCount++] = _AddrOffset>>8;
  Tx_Buf[_TxCount++] = _AddrOffset;
	
	if (_AddrOffset == COIL_D01)
	{
		R_value.D01 = _RegDATA;	
    LED1_ON();		
	}
	else if (_AddrOffset == COIL_D02)
	{
		R_value.D02 = _RegDATA;
		LED2_ON();
	}
	else if (_AddrOffset == COIL_D03)
	{
		R_value.D03 = _RegDATA;
		LED3_ON();
	}
	else if (_AddrOffset == COIL_D04)
	{
		R_value.D04 = _RegDATA;
		LED1_TOGGLE();
		LED2_TOGGLE();
		LED3_TOGGLE();
	}
	else
	{
	  R_value.D01 = 0;
		R_value.D02 = 0;
		R_value.D03 = 0;
		R_value.D04 = 0;
	}
	
	Tx_Buf[_TxCount++] = _RegDATA>>8;
	Tx_Buf[_TxCount++] = _RegDATA;	
  return _TxCount;
}

/**
  * ��������: д�������ּĴ�������/д��
  * �������: _TxCount :���ͼ�����,_AddrOffset:��ַƫ����,_RegNum: д�����ݣ�_AddrAbs�����ּĴ�����ַ
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ���Tx_Buf
  */
static uint8_t MB_RSP_06H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _RegNum ,uint16_t *_AddrAbs)
{
	/*
		д���ּĴ�����ע��06ָ��ֻ�ܲ����������ּĴ�����10Hָ��������õ����������ּĴ���
		��������:
			01 �ӻ���ַ
			06 ������
			00 �Ĵ�����ַ���ֽ�
			10 �Ĵ�����ַ���ֽ�
			67 ����1���ֽ�
			4A ����1���ֽ�
			23 CRCУ����ֽ�
			C8 CRCУ����ֽ�

		�ӻ���Ӧ:
			01 �ӻ���ַ
			06 ������
			00 �Ĵ�����ַ���ֽ�
			10 �Ĵ�����ַ���ֽ�
			67 ����1���ֽ�
			4A ����1���ֽ�
			23 CRCУ����ֽ�
			C8 CRCУ����ֽ�

		����:
			���ͣ�	01 06 00 10 67 4A  23 C8    ---- ��0010��ַ�Ĵ�������Ϊ67 4A
			���أ�	01 06 00 10 67 4A  23 C8    ---- ����ͬ������
*/		
  /* ����ֵַ */
  Tx_Buf[_TxCount++] = _AddrOffset>>8;
  Tx_Buf[_TxCount++] = _AddrOffset;	

	/* ������д�뱣�ּĴ����� */	
	*_AddrAbs = _RegNum;
	
  /* ��䷵������ */
	Tx_Buf[_TxCount++] = PduData.Num>>8;
	Tx_Buf[_TxCount++] = PduData.Num;
  
  return _TxCount;	
}

/**
  * ��������: д������ּĴ�������/д��
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_RegNum:�ֽ�������_Datebuf:����
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ���Tx_Buf
  */
static uint8_t MB_RSP_10H(uint16_t _TxCount,uint16_t  _AddrOffset,uint16_t _RegNum, uint16_t *_AddrAbs, uint8_t* _Datebuf)
{
	/*
		��������:
			01 �ӻ���ַ
			10 ������
			00 �Ĵ�����ʼ��ַ���ֽ�
			10 �Ĵ�����ʼ��ַ���ֽ�
			00 �Ĵ����������ֽ�
			02 �Ĵ����������ֽ�
			04 �ֽ���
			12 ����1���ֽ�
			34 ����1���ֽ�
			02 ����2���ֽ�
			03 ����2���ֽ�
			F7 CRCУ����ֽ�
			74 CRCУ����ֽ�

		�ӻ���Ӧ:
			01 �ӻ���ַ
			10 ������
			00 �Ĵ�����ַ���ֽ�
			10 �Ĵ�����ַ���ֽ�
			00 �Ĵ����������ֽ�
			02 �Ĵ����������ֽ�
			40 CRCУ����ֽ�
			0D CRCУ����ֽ�

		����:
			���ͣ�	01 10 00 10 00 02 04 12 34 02 03 F7 74    ----  ��0010H~0011Hд��12 34 02 03 �ĸ��ֽ�����
			���أ�	01 10 00 10 00 02 40 0D                   ----  ��������

	*/		
  uint16_t i = 0;	
  uint16_t Value = 0;
  /* ����ֵַ */
  Tx_Buf[_TxCount++] = _AddrOffset>>8;
  Tx_Buf[_TxCount++] = _AddrOffset;
	
  /* д�������ּĴ��� */
  for(i=0;i<_RegNum;i++)
  {
    Value = (uint16_t)((*_Datebuf<<8 ) | (*(_Datebuf+1)));
    *_AddrAbs++ = Value ;
    _Datebuf+=2;
  }
	
	Tx_Buf[_TxCount++] = _RegNum>>8;
	Tx_Buf[_TxCount++] = _RegNum;	
  return _TxCount;
}

/**
  * ��������: �쳣��Ӧ
  * �������: _FunCode :�����쳣�Ĺ�����,_ExCode:�쳣��
  * �� �� ֵ: ��
  * ˵    ��: ��ͨ������֡�����쳣ʱ,�����쳣��Ӧ
  */
void MB_Exception_RSP(uint8_t _FunCode,uint8_t _ExCode)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = MB_SLAVEADDR;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = _FunCode|0x80;		  /* ������ + 0x80*/	
	Tx_Buf[TxCount++] = _ExCode ;	          /* �쳣��*/
	
  crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  UART_Tx((uint8_t*)Tx_Buf, TxCount);
}

/**
  * ��������: �жϵ�ַ�Ƿ����Э�鷶Χ
  * �������: _Addr:��ʼ��ַ,_RegNum:�Ĵ�������,_FunCode:������
  * �� �� ֵ: �쳣��:02H��NONE
  * ˵    ��: ��ַ��Χ��0x0000~0xFFFF,�ɲ����Ŀռ䷶Χ���ܳ����������
  */
uint8_t MB_JudgeAddr(uint16_t _Addr,uint16_t _RegNum)
{
  uint8_t Excode = EX_CODE_NONE;
  /* ��ַ+�Ĵ����������ܳ���0xFFFF */
  if( ((uint32_t)_RegNum+(uint32_t)_Addr) > (uint32_t)0xFFFF)
  {
    Excode = EX_CODE_02H;// �쳣�� 02H
  }
  return Excode;
}
/**
  * ��������: �жϲ������������Ƿ����Э�鷶Χ
  * �������: _RegNum:�Ĵ�������,_FunCode:������,_ByteNum:�ֽ�����
  * �� �� ֵ: �쳣��:03��NONE
  * ˵    ��: �Կɲ��������ڴ�ռ�Ĺ�������Ҫ��֤�����ĵ�ַ�Ƿ���Ϸ�Χ
  */
uint8_t MB_JudgeNum(uint16_t _RegNum,uint8_t _FunCode,uint16_t _ByteNum)
{
  uint8_t Excode = EX_CODE_NONE;
  uint16_t _CoilNum = _RegNum; // ��Ȧ(��ɢ��)������
  switch(_FunCode)
  {
    case FUN_CODE_01H: 
    case FUN_CODE_02H:
      if( (_CoilNum<0x0001) || (_CoilNum>0x07D0))
        Excode = EX_CODE_03H;// �쳣��03H;
      break;
    case FUN_CODE_03H:
    case FUN_CODE_04H:
      if( (_RegNum<0x0001) || (_RegNum>0x007D))
        Excode = EX_CODE_03H;// �쳣��03H;      
      break;
    case FUN_CODE_10H:
      if( (_RegNum<0x0001) || (_RegNum>0x007B))
        Excode = EX_CODE_03H;// �쳣��03H
      if( _ByteNum != (_RegNum<<1))
        Excode = EX_CODE_03H;// �쳣��03H
      break;
  }
  return Excode;
}


/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
