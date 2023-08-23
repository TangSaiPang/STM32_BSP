/**
  ******************************************************************************
  * 文件名程: bsp_MB_slave.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2018-09-01
  * 功    能: MODBUS从机
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-H7Multi使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/  
#include "bsp_MB_slave.h"
#include "bsp_RS485.h"
#include "bsp_GeneralTIM.h"
#include "bsp_led.h"
#include "bsp_debug_usart.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
PDUData_TypeDef PduData;
REG_VALUE R_value;

// CRC 高位字节值表
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
// CRC 低位字节值表
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


/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
static uint8_t MB_JudgeAddr(uint16_t _Addr,uint16_t _RegNum);
static uint8_t MB_JudgeNum(uint16_t _RegNum,uint8_t _FunCode,uint16_t _ByteNum);
static uint16_t MB_RSP_01H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _CoilNum );
static uint16_t MB_RSP_02H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _CoilNum);
static uint8_t MB_RSP_03H(uint16_t _TxCount,uint16_t *_AddrOffset,uint16_t _RegNum );
static uint8_t MB_RSP_04H(uint16_t _TxCount,uint16_t _AddrOffset,uint16_t _RegNum );
static uint8_t MB_RSP_05H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _RegDATA);
static uint8_t MB_RSP_06H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _RegNum ,uint16_t *_AddrAbs);
static uint8_t MB_RSP_10H(uint16_t _TxCount,uint16_t _AddrOffset,uint16_t _RegNum ,uint16_t *_AddrAbs ,uint8_t* _Datebuf);

/* 函数体 --------------------------------------------------------------------*/

/** 
  * 函数功能: Modbus CRC16 校验计算函数
  * 输入参数: pushMsg:待计算的数据首地址,usDataLen:数据长度
  * 返 回 值: CRC16 计算结果
  * 说    明: 计算结果是高位在前,需要转换才能发送
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

/* 提取数据帧,进行解析数据帧 */
void MB_Parse_Data()
{
  PduData.Code = Rx_Buf[1];                   // 功能码
  PduData.Addr = ((Rx_Buf[2]<<8) | Rx_Buf[3]);// 寄存器起始地址
  PduData.Num  = ((Rx_Buf[4]<<8) | Rx_Buf[5]);// 数量(Coil,Input,Holding Reg,Input Reg)
  PduData._CRC = MB_CRC16((uint8_t*)&Rx_Buf,RxCount-2);             // CRC校验码
  PduData.byteNums = Rx_Buf[6];                                     // 获得字节数
  PduData.ValueReg = (uint8_t*)&Rx_Buf[7];                          // 寄存器值起始地址
  PduData.PtrHoldingOffset = PduData.PtrHoldingbase + PduData.Addr; // 保持寄存器的起始地址
}

/** 
  * 函数功能: 对接收到的数据进行分析并执行
  * 输入参数: 无
  * 返 回 值: 异常码或0x00
  * 说    明: 判断功能码,验证地址是否正确.数值内容是否溢出,数据没错误就发送响应信号
  */
uint8_t MB_Analyze_Execute(void )
{
  uint16_t ExCode = EX_CODE_NONE;
  /* 校验功能码 */
  if( IS_NOT_FUNCODE(PduData.Code) ) // 不支持的功能码
  {
    /* Modbus异常响应 */
    ExCode = EX_CODE_01H;            // 异常码01H
    return ExCode;
  }
  /* 根据功能码分别做判断 */
  switch(PduData.Code)
  {
    /* 这里认为01H功能码和02功能码是一样的,其实也没什么不一样
     * 只是操作地址可能不一样,这一点结合具体来实现,可以在main函数
     * 申请单独的内存使用不同的功能码,在实际应用中必须加以区分使用
     * 不同的内存空间
     */
/* ---- 01H  02H 读取离散量输入(Coil Input)---------------------- */
    case FUN_CODE_01H:
    case FUN_CODE_02H:
      /* 判断线圈数量是否正确 */  
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,1);
      if(ExCode != EX_CODE_NONE )
        return ExCode;      
      
      /* 判断地址是否正确*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      break;
/* ---- 03H  04H 读取保持/输入寄存器---------------------- */
    case FUN_CODE_03H:
    case FUN_CODE_04H:
      /* 判断寄存器数量是否正确 */
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,PduData.byteNums);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      
      /* 判断地址是否正确*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);
      if(ExCode != EX_CODE_NONE )
        return ExCode;  
      break;
/* ---- 05H 写入单个离散量---------------------- */
    case FUN_CODE_05H:
      break;
/* ---- 06H 写单个保持寄存器 ---------------------- */
    case FUN_CODE_06H:     
      break;
/* ---- 10H 写多个保持寄存器 ---------------------- */
    case FUN_CODE_10H:
      /* 判断寄存器数量是否正确 */
      ExCode = MB_JudgeNum(PduData.Num,PduData.Code,PduData.byteNums);
      if(ExCode != EX_CODE_NONE )
        return ExCode;          
      /* 判断地址是否正确*/
      ExCode = MB_JudgeAddr( PduData.Addr,PduData.Num);		      		
      if(ExCode != EX_CODE_NONE )
        return ExCode;  			
      break;
  }
  /* 数据帧没有异常 */
  return ExCode; //   EX_CODE_NONE
}

/**
  * 函数功能: 正常响应
  * 输入参数: _FunCode :功能码
  * 返 回 值: 无
  * 说    明: 当通信数据帧没有异常时并且成功执行之后,发送响应数据帧
  */
void MB_RSP(uint8_t _FunCode)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;	
	Tx_Buf[TxCount++] = MB_SLAVEADDR;		 /* 从站地址 */
	Tx_Buf[TxCount++] = _FunCode;        /* 功能码   */	
  switch(_FunCode)
  {
    case FUN_CODE_01H:
			/* 读取线圈状态 */
			TxCount = MB_RSP_01H(TxCount,PduData.Addr,PduData.Num);
		  break;
    case FUN_CODE_02H:
			/* 读取离散输入 */
      TxCount = MB_RSP_02H(TxCount,PduData.Addr,PduData.Num);
      break;		 
    case FUN_CODE_03H:
			 /* 读取保持寄存器 */
			TxCount = MB_RSP_03H(TxCount,(uint16_t*)PduData.PtrHoldingOffset,PduData.Num);
	  	break;
    case FUN_CODE_04H:
			/* 读取输入寄存器 */
			TxCount =	MB_RSP_04H(TxCount,PduData.Addr,PduData.Num);      
      break;
    case FUN_CODE_05H:
			/* 写单个线圈 */
      TxCount = MB_RSP_05H(TxCount,PduData.Addr,PduData.Num);
      break;
    case FUN_CODE_06H:
			/* 写单个保持寄存器 */
      TxCount = MB_RSP_06H(TxCount,PduData.Addr,PduData.Num, (uint16_t*)PduData.PtrHoldingOffset);
      break;
    case FUN_CODE_10H:
			/* 写多个保持寄存器 */
      TxCount = MB_RSP_10H(TxCount,PduData.Addr,PduData.Num ,(uint16_t*)PduData.PtrHoldingOffset,(uint8_t*)PduData.ValueReg);
      break;
  }
  crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc 低字节 */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc 高字节 */
  UART_Tx((uint8_t*)Tx_Buf, TxCount);
}

/**
  * 函数功能: 读取线圈状态（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_CoilNum:线圈数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 读取离散输出,并且填充Tx_Buf
  */
static uint16_t MB_RSP_01H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _CoilNum )
{
	/*
		主机发送:
			01 从机地址
			01 功能码
			00 寄存器起始地址高字节
			02 寄存器起始地址低字节
			00 寄存器数量高字节
			08 寄存器数量低字节
			9C CRC校验高字节
			0C CRC校验低字节

		从机应答: 	1代表ON，0代表OFF（使用LED的状态来代替）。若返回的线圈数不为8的倍数，则在最后数据字节未尾使用0代替. BIT0对应第1个
			01 从机地址
			01 功能码
			01 返回字节数
			02 数据1(线圈0002H-线圈0011H)
			D0 CRC校验高字节
			49 CRC校验低字节

		例子1:
		发送：	01 01 00 02 00 08   9C 0C	  --- 查询D02开始的8个继电器状态
		返回：	01 01 01 01         90 48   --- 查询到8个状态为：0000 0001 第二个LED为亮
		
		例子2:
		发送：	01 01 00 01 00 10   6C 06	  --- 查询D01开始的16个继电器状态
		返回：	01 01 02 FF FF      B8 4C   --- 查询到两个字节数据为0xFFFF
				
	*/	
//  uint16_t i = 0;
//	uint16_t m;	
//	uint8_t status[10];	
//	
//  /* 计算返回字节数（_CoilNum变量是以位为单位） */
//  m = (_CoilNum+7)/8;
//  /* 返回字节数（数量）*/
//	Tx_Buf[_TxCount++] = m; 
//	if ((_AddrOffset >= COIL_D01) && (_CoilNum > 0))
//  {
//		/* 将获取的线圈状态首先清零 */
//		for (i = 0; i < m; i++)
//		{
//			status[i] = 0;
//		}		
//		/* 获取对应线圈状态，并将其写入status[] */
//		for (i = 0; i < _CoilNum; i++)
//		{
//			/* 读LED的状态，写入状态寄存器的每一位 */
//			if (Get_LEDx_State(i + 1 + _AddrOffset - COIL_D01))		
//			{  
//				status[i / 8] |= (1 << (i % 8));
//			}			
//		}    		
//	}
//	/* 填充发送内容 */
//	for (i = 0; i < m; i++)
//	{
//		Tx_Buf[_TxCount++] = status[i];	/* 继电器状态 */
//	}		
	
	/*----------------------------分割线----------------------------------*/
	Tx_Buf[_TxCount++] = 2; 
  /* 填充返回内容 */ 	
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
  * 函数功能: 读取离散输入（只读）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_CoilNum:线圈数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 读取离散输出,并且填充Tx_Buf
  */
static uint16_t MB_RSP_02H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _CoilNum)
{
	/*
		主机发送:
			01 从机地址
			02 功能码
			00 寄存器起始地址高字节
			01 寄存器起始地址低字节
			00 寄存器数量高字节
			08 寄存器数量低字节
			28 CRC校验高字节
			0C CRC校验低字节

		从机应答: 	1代表ON，0代表OFF（使用LED的状态来代替）。若返回的线圈数不为8的倍数，则在最后数据字节未尾使用0代替. BIT0对应第1个
			01 从机地址
			02 功能码
			01 返回字节数
			02 数据1(线圈0002H-线圈0011H)
			D0 CRC校验高字节
			49 CRC校验低字节

		例子:
		发送：	01 01 00 02 00 08   9C 0C	  --- 查询D02开始的8个继电器状态
		返回：	01 01 01 02         D0 49   --- 查询到8个状态为：0000 0010 第二个LED为亮
	*/	
  uint16_t i = 0;
	uint16_t m;	
	uint8_t status[10];	
	
  /* 计算返回字节数（_CoilNum变量是以位为单位） */
  m = (_CoilNum+7)/8;
  /* 返回字节数（数量）*/
	Tx_Buf[_TxCount++] = m; 
	  
	if ((_AddrOffset >= COIL_D01) && (_CoilNum > 0))
  {
		/* 将获取的线圈状态首先清零 */
		for (i = 0; i < m; i++)
		{
			status[i] = 0;
		}		
		/* 获取对应线圈状态，并将其写入status[] */
		for (i = 0; i < _CoilNum; i++)
		{
			/* 读LED的状态，写入状态寄存器的每一位 */
			if (Get_LEDx_State(i + 1 + _AddrOffset - COIL_D01))		
			{  
				status[i / 8] |= (1 << (i % 8));
			}			
		}
	}
	/* 填充发送内容 */
	for (i = 0; i < m; i++)
	{
		/* 继电器状态 */
		Tx_Buf[_TxCount++] = status[i];	
	}	
  return _TxCount;
}

/**
  * 函数功能: 读取保持寄存器（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_RegNum:寄存器数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 读取保持寄存器的内容,并且填充Tx_Buf
  */
static uint8_t MB_RSP_03H(uint16_t _TxCount,uint16_t *_AddrOffset,uint16_t _RegNum )
{
	/*
		从机地址为01H。保持寄存器的起始地址为0010H，结束地址为0011H。该次查询总共访问2个保持寄存器。
		主机发送:
			01 从机地址
			03 功能码
			00 寄存器地址高字节
			10 寄存器地址低字节
			00 寄存器数量高字节
			02 寄存器数量低字节
			C5 CRC高字节
			CE CRC低字节

		从机应答: 	保持寄存器的长度为2个字节。对于单个保持寄存器而言，寄存器高字节数据先被传输，
					低字节数据后被传输。保持寄存器之间，低地址寄存器先被传输，高地址寄存器后被传输。
			01 从机地址
			03 功能码
			04 字节数
			12 数据1高字节(0010H)
			34 数据1低字节(0010H)
			02 数据2高字节(0011H)
			03 数据2低字节(0100H)
			FF CRC高字节
			F4 CRC低字节

		读一个保持寄存器例子:
			发送：	01 03 00 10 00 01            85 CF ---- 读 0010H一个寄存器内容
			返回：	01 03 02 12 34               B5 33 ---- 返回10H功能码写入的内容（10H功能码会介绍）
	*/
	
  /* 填充返回寄存器数量 */
  Tx_Buf[_TxCount++] = _RegNum*2;
  /* 返回保持寄存器内的数据 */
  for(uint8_t i = 0;i< _RegNum;i++)
  {
    Tx_Buf[_TxCount++] = ((*_AddrOffset)>>8);
    Tx_Buf[_TxCount++] = *_AddrOffset++;
  }
  return _TxCount;
}

/**
  * 函数功能: 读取输入寄存器（与上述03H指令类似，返回内容也类似）（只读）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_RegNum:寄存器数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 读取保持寄存器的内容,并且填充Tx_Buf
  */
static uint8_t MB_RSP_04H(uint16_t _TxCount,uint16_t _AddrOffset,uint16_t _RegNum )
{
	/*
		主机发送:
			01 从机地址
			04 功能码
			00 寄存器起始地址高字节
			20 寄存器起始地址低字节
			00 寄存器个数高字节
			02 寄存器个数低字节
			70 CRC高字节
			01 CRC低字节

		从机应答:  输入寄存器长度为2个字节。对于单个输入寄存器而言，寄存器高字节数据先被传输，
				低字节数据后被传输。输入寄存器之间，低地址寄存器先被传输，高地址寄存器后被传输。
			01 从机地址
			04 功能码
			02 字节数
			02 数据1高字节(0020H)
			03 数据1低字节(0020H)
			00 数据2高字节(0021H)
			00 数据2低字节(0021H)
			82 CRC高字节
			3C CRC低字节

		例子:
			发送：	01 04 00 20 00 02      70 01  --- 读 0020H IN1 开始的2个字节数据内容
			返回：	01 04 02 02 03 00 00   82 3C  --- 返回：02 03 00 00 4个数据（按下KEY1按键后改变 R_value.IN1的值）
	*/	
	uint8_t i;
	uint16_t reg_value[64];
  /* 填充返回寄存器数量 */
  Tx_Buf[_TxCount++] = _RegNum;
	/* 读取保持寄存器内容 */
	for(i = 0; i < _RegNum; i++)
	{
		switch (_AddrOffset)
		{
			/* 测试参数 */
			case REG_IN1:
				reg_value[i] = R_value.IN1;
				break;
		
			default:
				reg_value[i] = 0;
				break;
		}
		_AddrOffset++;
	}
	
  /* 填充返回内容 */
  for(i = 0;i< _RegNum;i++)
  {
		
    Tx_Buf[_TxCount++] = reg_value[i]>>8;
    Tx_Buf[_TxCount++] = reg_value[i]& 0xFF;
  }
  return _TxCount;
}

/**
  * 函数功能: 写单个线圈（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_RegDATA:写入数据
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 填充Tx_Buf
  */
static uint8_t MB_RSP_05H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _RegDATA)
{
	/*
		主机发送: 写单个线圈寄存器。简单的01~03寄存器地址对应LED1~LED3,将数据存放于D01、D02、D03、D04等4个成员中
		05H指令设置单个线圈的状态
			01 从机地址
			05 功能码
			00 寄存器地址高字节
			01 寄存器地址低字节
			FF 数据1高字节
			FF 数据1低字节
			9D CRC校验高字节
			BA CRC校验低字节

		从机应答:
			01 从机地址
			05 功能码
			00 寄存器地址高字节
			01 寄存器地址低字节
			FF 寄存器1高字节
			FF 寄存器1低字节
			9D CRC校验高字节
			BA CRC校验低字节

		例子:
		发送：	01 05 00 04 FF FF   8D BB   -- 发送数据0xFFFF至0x04地址线圈中
		返回：	01 05 00 04 FF FF   8D BB   -- 返回原始数据
	*/	
	
  /* 填充地址值 */
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
  * 函数功能: 写单个保持寄存器（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset:地址偏移量,_RegNum: 写入数据，_AddrAbs：保持寄存器地址
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 填充Tx_Buf
  */
static uint8_t MB_RSP_06H(uint16_t _TxCount,uint16_t _AddrOffset ,uint16_t _RegNum ,uint16_t *_AddrAbs)
{
	/*
		写保持寄存器。注意06指令只能操作单个保持寄存器，10H指令可以设置单个或多个保持寄存器
		主机发送:
			01 从机地址
			06 功能码
			00 寄存器地址高字节
			10 寄存器地址低字节
			67 数据1高字节
			4A 数据1低字节
			23 CRC校验高字节
			C8 CRC校验低字节

		从机响应:
			01 从机地址
			06 功能码
			00 寄存器地址高字节
			10 寄存器地址低字节
			67 数据1高字节
			4A 数据1低字节
			23 CRC校验高字节
			C8 CRC校验低字节

		例子:
			发送：	01 06 00 10 67 4A  23 C8    ---- 将0010地址寄存器设置为67 4A
			返回：	01 06 00 10 67 4A  23 C8    ---- 返回同样数据
*/		
  /* 填充地址值 */
  Tx_Buf[_TxCount++] = _AddrOffset>>8;
  Tx_Buf[_TxCount++] = _AddrOffset;	

	/* 将数据写入保持寄存器内 */	
	*_AddrAbs = _RegNum;
	
  /* 填充返回内容 */
	Tx_Buf[_TxCount++] = PduData.Num>>8;
	Tx_Buf[_TxCount++] = PduData.Num;
  
  return _TxCount;	
}

/**
  * 函数功能: 写多个保持寄存器（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_RegNum:字节数量，_Datebuf:数据
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 填充Tx_Buf
  */
static uint8_t MB_RSP_10H(uint16_t _TxCount,uint16_t  _AddrOffset,uint16_t _RegNum, uint16_t *_AddrAbs, uint8_t* _Datebuf)
{
	/*
		主机发送:
			01 从机地址
			10 功能码
			00 寄存器起始地址高字节
			10 寄存器起始地址低字节
			00 寄存器数量高字节
			02 寄存器数量低字节
			04 字节数
			12 数据1高字节
			34 数据1低字节
			02 数据2高字节
			03 数据2低字节
			F7 CRC校验高字节
			74 CRC校验低字节

		从机响应:
			01 从机地址
			10 功能码
			00 寄存器地址高字节
			10 寄存器地址低字节
			00 寄存器数量高字节
			02 寄存器数量低字节
			40 CRC校验高字节
			0D CRC校验低字节

		例子:
			发送：	01 10 00 10 00 02 04 12 34 02 03 F7 74    ----  向0010H~0011H写入12 34 02 03 四个字节数据
			返回：	01 10 00 10 00 02 40 0D                   ----  返回内容

	*/		
  uint16_t i = 0;	
  uint16_t Value = 0;
  /* 填充地址值 */
  Tx_Buf[_TxCount++] = _AddrOffset>>8;
  Tx_Buf[_TxCount++] = _AddrOffset;
	
  /* 写入多个保持寄存器 */
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
  * 函数功能: 异常响应
  * 输入参数: _FunCode :发送异常的功能码,_ExCode:异常码
  * 返 回 值: 无
  * 说    明: 当通信数据帧发生异常时,发送异常响应
  */
void MB_Exception_RSP(uint8_t _FunCode,uint8_t _ExCode)
{
  uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = MB_SLAVEADDR;		    /* 从站地址 */
	Tx_Buf[TxCount++] = _FunCode|0x80;		  /* 功能码 + 0x80*/	
	Tx_Buf[TxCount++] = _ExCode ;	          /* 异常码*/
	
  crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc 低字节 */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc 高字节 */
  UART_Tx((uint8_t*)Tx_Buf, TxCount);
}

/**
  * 函数功能: 判断地址是否符合协议范围
  * 输入参数: _Addr:起始地址,_RegNum:寄存器数量,_FunCode:功能码
  * 返 回 值: 异常码:02H或NONE
  * 说    明: 地址范围是0x0000~0xFFFF,可操作的空间范围不能超过这个区域
  */
uint8_t MB_JudgeAddr(uint16_t _Addr,uint16_t _RegNum)
{
  uint8_t Excode = EX_CODE_NONE;
  /* 地址+寄存器数量不能超过0xFFFF */
  if( ((uint32_t)_RegNum+(uint32_t)_Addr) > (uint32_t)0xFFFF)
  {
    Excode = EX_CODE_02H;// 异常码 02H
  }
  return Excode;
}
/**
  * 函数功能: 判断操作的数据量是否符合协议范围
  * 输入参数: _RegNum:寄存器数量,_FunCode:功能码,_ByteNum:字节数量
  * 返 回 值: 异常码:03或NONE
  * 说    明: 对可操作连续内存空间的功能码需要验证操作的地址是否符合范围
  */
uint8_t MB_JudgeNum(uint16_t _RegNum,uint8_t _FunCode,uint16_t _ByteNum)
{
  uint8_t Excode = EX_CODE_NONE;
  uint16_t _CoilNum = _RegNum; // 线圈(离散量)的数量
  switch(_FunCode)
  {
    case FUN_CODE_01H: 
    case FUN_CODE_02H:
      if( (_CoilNum<0x0001) || (_CoilNum>0x07D0))
        Excode = EX_CODE_03H;// 异常码03H;
      break;
    case FUN_CODE_03H:
    case FUN_CODE_04H:
      if( (_RegNum<0x0001) || (_RegNum>0x007D))
        Excode = EX_CODE_03H;// 异常码03H;      
      break;
    case FUN_CODE_10H:
      if( (_RegNum<0x0001) || (_RegNum>0x007B))
        Excode = EX_CODE_03H;// 异常码03H
      if( _ByteNum != (_RegNum<<1))
        Excode = EX_CODE_03H;// 异常码03H
      break;
  }
  return Excode;
}


/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
