/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : notify.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
	* BEEP TIM3 CHANNEL1 PWM Gerente
	* LED is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dev.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "nrf24l01.h"

/* USER CODE END Includes */

SPI_HandleTypeDef * rf_spi_handle;
static unsigned char  tr_addr_g[5] = {INIT_ADDR};
/* delay */
static void rf_delay_ms(unsigned int t)
{
  //unsigned int tmp = t;
  /* delay */
  //while(tmp--);
}
/* spi write */
unsigned char hal_spi_rw_byte(unsigned char data)
{
	unsigned char d[2];
	/* structet */
	d[0] = data;
	/* send and receive */
	HAL_SPI_TransmitReceive(rf_spi_handle,&d[0],&d[1],1,0xffff);
	/* return data */
	return d[1];
}	

/**
  * @brief :NRF24L01读寄存器
  * @param :
           @Addr:寄存器地址
  * @note  :地址在设备中有效
  * @retval:读取的数据
  */
unsigned char NRF24L01_Read_Reg( unsigned char RegAddr )
{
    unsigned char btmp;
	
    RF24L01_SET_CS_LOW( );			//片选
	
    hal_spi_rw_byte( NRF_READ_REG | RegAddr );	//读命令 地址
    btmp = hal_spi_rw_byte( 0xFF );				//读数据
	
    RF24L01_SET_CS_HIGH( );			//取消片选
	
    return btmp;
}

/**
  * @brief :NRF24L01读指定长度的数据
  * @param :
  *			@reg:地址
  *			@pBuf:数据存放地址
  *			@len:数据长度
  * @note  :数据长度不超过255，地址在设备中有效
  * @retval:读取状态
  */
void NRF24L01_Read_Buf( unsigned char RegAddr, unsigned char *pBuf, unsigned char len )
{
    unsigned char btmp;
	
    RF24L01_SET_CS_LOW( );			//片选
	
    hal_spi_rw_byte( NRF_READ_REG | RegAddr );	//读命令 地址
    for( btmp = 0; btmp < len; btmp ++ )
    {
        *( pBuf + btmp ) = hal_spi_rw_byte( 0xFF );	//读数据
    }
    RF24L01_SET_CS_HIGH( );		//取消片选
}

/**
  * @brief :NRF24L01写寄存器
  * @param :无
  * @note  :地址在设备中有效
  * @retval:读写状态
  */
void NRF24L01_Write_Reg( unsigned char RegAddr, unsigned char Value )
{
    RF24L01_SET_CS_LOW( );		//片选
	
    hal_spi_rw_byte( NRF_WRITE_REG | RegAddr );	//写命令 地址
    hal_spi_rw_byte( Value );			//写数据
	
    RF24L01_SET_CS_HIGH( );		//取消片选
}

/**
  * @brief :NRF24L01写指定长度的数据
  * @param :
  *			@reg:地址
  *			@pBuf:写入的数据地址
  *			@len:数据长度
  * @note  :数据长度不超过255，地址在设备中有效
  * @retval:写状态
  */
void NRF24L01_Write_Buf( unsigned char RegAddr, unsigned char *pBuf, unsigned char len )
{
    unsigned char i;
	
    RF24L01_SET_CS_LOW( );		//片选
	
    hal_spi_rw_byte( NRF_WRITE_REG | RegAddr );	//写命令 地址
    for( i = 0; i < len; i ++ )
    {
        hal_spi_rw_byte( *( pBuf + i ) );		//写数据
    }
	
    RF24L01_SET_CS_HIGH( );		//取消片选

}

/**
  * @brief :清空TX缓冲区
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF24L01_Flush_Tx_Fifo ( void )
{
    RF24L01_SET_CS_LOW( );		//片选
	
    hal_spi_rw_byte( FLUSH_TX );	//清TX FIFO命令
	
    RF24L01_SET_CS_HIGH( );		//取消片选
}

/**
  * @brief :清空RX缓冲区
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF24L01_Flush_Rx_Fifo( void )
{
    RF24L01_SET_CS_LOW( );		//片选
	
    hal_spi_rw_byte( FLUSH_RX );	//清RX FIFO命令
	
    RF24L01_SET_CS_HIGH( );		//取消片选
}

/**
  * @brief :重新使用上一包数据
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF24L01_Reuse_Tx_Payload( void )
{
    RF24L01_SET_CS_LOW( );		//片选
	
    hal_spi_rw_byte( REUSE_TX_PL );		//重新使用上一包命令
	
    RF24L01_SET_CS_HIGH( );		//取消片选
}

/**
  * @brief :NRF24L01空操作
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF24L01_Nop( void )
{
    RF24L01_SET_CS_LOW( );		//片选
	
    hal_spi_rw_byte( NOP );		//空操作命令
	
    RF24L01_SET_CS_HIGH( );		//取消片选
}

/**
  * @brief :NRF24L01读状态寄存器
  * @param :无
  * @note  :无
  * @retval:RF24L01状态
  */
unsigned char NRF24L01_Read_Status_Register( void )
{
    unsigned char Status;
	
    RF24L01_SET_CS_LOW( );		//片选
	
    Status = hal_spi_rw_byte( NRF_READ_REG + STATUS );	//读状态寄存器
	
    RF24L01_SET_CS_HIGH( );		//取消片选
	
    return Status;
}

/**
  * @brief :NRF24L01清中断
  * @param :
           @IRQ_Source:中断源
  * @note  :无
  * @retval:清除后状态寄存器的值
  */
unsigned char NRF24L01_Clear_IRQ_Flag( unsigned char IRQ_Source )
{
    unsigned char btmp = 0;

    IRQ_Source &= ( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT );	//中断标志处理
    btmp = NRF24L01_Read_Status_Register( );			//读状态寄存器
			
    RF24L01_SET_CS_LOW( );			//片选
    hal_spi_rw_byte( NRF_WRITE_REG + STATUS );	//写状态寄存器命令
    hal_spi_rw_byte( IRQ_Source | btmp );		//清相应中断标志
    RF24L01_SET_CS_HIGH( );			//取消片选
	
    return ( NRF24L01_Read_Status_Register( ));			//返回状态寄存器状态
}

/**
  * @brief :读RF24L01中断状态
  * @param :无
  * @note  :无
  * @retval:中断状态
  */
unsigned char RF24L01_Read_IRQ_Status( void )
{
    return ( NRF24L01_Read_Status_Register( ) & (( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT )));	//返回中断状态
}
 
 /**
  * @brief :读FIFO中数据宽度
  * @param :无
  * @note  :无
  * @retval:数据宽度
  */
unsigned char NRF24L01_Read_Top_Fifo_Width( void )
{
    unsigned char btmp;
	
    RF24L01_SET_CS_LOW( );		//片选
	
    hal_spi_rw_byte( R_RX_PL_WID );	//读FIFO中数据宽度命令
    btmp = hal_spi_rw_byte( 0xFF );	//读数据
	
    RF24L01_SET_CS_HIGH( );		//取消片选
	
    return btmp;
}

 /**
  * @brief :读接收到的数据
  * @param :无
  * @note  :无
  * @retval:
           @pRxBuf:数据存放地址首地址
  */
unsigned char NRF24L01_Read_Rx_Payload( unsigned char *pRxBuf )
{
    unsigned char Width, PipeNum;
	
    PipeNum = ( NRF24L01_Read_Reg( STATUS ) >> 1 ) & 0x07;	//读接收状态
    Width = NRF24L01_Read_Top_Fifo_Width( );		//读接收数据个数

    RF24L01_SET_CS_LOW( );		//片选
    hal_spi_rw_byte( RD_RX_PLOAD );			//读有效数据命令
	
    for( PipeNum = 0; PipeNum < Width; PipeNum ++ )
    {
        *( pRxBuf + PipeNum ) = hal_spi_rw_byte( 0xFF );		//读数据
    }
    RF24L01_SET_CS_HIGH( );		//取消片选
    NRF24L01_Flush_Rx_Fifo( );	//清空RX FIFO
	
    return Width;
}

 /**
  * @brief :发送数据（带应答）
  * @param :
  *			@pTxBuf:发送数据地址
  *			@len:长度
  * @note  :一次不超过32个字节
  * @retval:无
  */
void NRF24L01_Write_Tx_Payload_Ack( unsigned char *pTxBuf, unsigned char len )
{
    unsigned char btmp;
    unsigned char length = ( len > 32 ) ? 32 : len;		//数据长达大约32 则只发送32个

    NRF24L01_Flush_Tx_Fifo( );		//清TX FIFO
	
    RF24L01_SET_CS_LOW( );			//片选
    hal_spi_rw_byte( WR_TX_PLOAD );	//发送命令
	
    for( btmp = 0; btmp < length; btmp ++ )
    {
        hal_spi_rw_byte( *( pTxBuf + btmp ) );	//发送数据
    }
    RF24L01_SET_CS_HIGH( );			//取消片选
}

 /**
  * @brief :发送数据（不带应答）
  * @param :
  *			@pTxBuf:发送数据地址
  *			@len:长度
  * @note  :一次不超过32个字节
  * @retval:无
  */
void NRF24L01_Write_Tx_Payload_NoAck( unsigned char *pTxBuf, unsigned char len )
{
    if( len > 32 || len == 0 )
    {
        return ;		//数据长度大于32 或者等于0 不执行
    }
	
    RF24L01_SET_CS_LOW( );	//片选
    hal_spi_rw_byte( WR_TX_PLOAD_NACK );	//发送命令
    while( len-- )
    {
        hal_spi_rw_byte( *pTxBuf );			//发送数据
		pTxBuf++;
    }
    RF24L01_SET_CS_HIGH( );		//取消片选
}

 /**
  * @brief :在接收模式下向TX FIFO写数据(带ACK)
  * @param :
  *			@pData:数据地址
  *			@len:长度
  * @note  :一次不超过32个字节
  * @retval:无
  */
void NRF24L01_Write_Tx_Payload_InAck( unsigned char *pData, unsigned char len )
{
    unsigned char btmp;
	
	len = ( len > 32 ) ? 32 : len;		//数据长度大于32个则只写32个字节

    RF24L01_SET_CS_LOW( );			//片选
    hal_spi_rw_byte( W_ACK_PLOAD );		//命令
    for( btmp = 0; btmp < len; btmp ++ )
    {
        hal_spi_rw_byte( *( pData + btmp ) );	//写数据
    }
    RF24L01_SET_CS_HIGH( );			//取消片选
}

 /**
  * @brief :设置发送地址
  * @param :
  *			@pAddr:地址存放地址
  *			@len:长度
  * @note  :无
  * @retval:无
  */
void NRF24L01_Set_TxAddr( unsigned char *pAddr, unsigned char len )
{
	len = ( len > 5 ) ? 5 : len;					//地址不能大于5个字节
    NRF24L01_Write_Buf( TX_ADDR, pAddr, len );	//写地址
}

 /**
  * @brief :设置接收通道地址
  * @param :
  *			@PipeNum:通道
  *			@pAddr:地址存肥着地址
  *			@Len:长度
  * @note  :通道不大于5 地址长度不大于5个字节
  * @retval:无
  */
void NRF24L01_Set_RxAddr( unsigned char PipeNum, unsigned char *pAddr, unsigned char Len )
{
    Len = ( Len > 5 ) ? 5 : Len;
    PipeNum = ( PipeNum > 5 ) ? 5 : PipeNum;		//通道不大于5 地址长度不大于5个字节

    NRF24L01_Write_Buf( RX_ADDR_P0 + PipeNum, pAddr, Len );	//写入地址
}

 /**
  * @brief :设置通信速度
  * @param :
  *			@Speed:速度
  * @note  :无
  * @retval:无
  */
void NRF24L01_Set_Speed( nRf24l01SpeedType Speed )
{
	unsigned char btmp = 0;
	
	btmp = NRF24L01_Read_Reg( RF_SETUP );
	btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	
	if( Speed == SPEED_250K )		//250K
	{
		btmp |= ( 1<<5 );
	}
	else if( Speed == SPEED_1M )   //1M
	{
   		btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	}
	else if( Speed == SPEED_2M )   //2M
	{
		btmp |= ( 1<<3 );
	}

	NRF24L01_Write_Reg( RF_SETUP, btmp );
}

 /**
  * @brief :设置功率
  * @param :
  *			@Speed:速度
  * @note  :无
  * @retval:无
  */
void NRF24L01_Set_Power( nRf24l01PowerType Power )
{
    unsigned char btmp;
	
	btmp = NRF24L01_Read_Reg( RF_SETUP ) & ~0x07;
    switch( Power )
    {
        case POWER_F18DBM:
            btmp |= PWR_18DB;
            break;
        case POWER_F12DBM:
            btmp |= PWR_12DB;
            break;
        case POWER_F6DBM:
            btmp |= PWR_6DB;
            break;
        case POWER_0DBM:
            btmp |= PWR_0DB;
            break;
        default:
            break;
    }
    NRF24L01_Write_Reg( RF_SETUP, btmp );
}

 /**
  * @brief :设置频率
  * @param :
  *			@FreqPoint:频率设置参数
  * @note  :值不大于127
  * @retval:无
  */
void RF24LL01_Write_Hopping_Point( unsigned char FreqPoint )
{
    NRF24L01_Write_Reg(  RF_CH, FreqPoint & 0x7F );
}

/**
  * @brief :NRF24L01检测
  * @param :无
  * @note  :无
  * @retval:无
  */ 
int NRF24L01_check( void )
{
	unsigned char i;
	unsigned char buf[5]={ 0XA5, 0XA5, 0XA5, 0XA5, 0XA5 };
	unsigned char read_buf[ 5 ] = { 0 };
	unsigned char try_time = 0;
	while( 1 )
	{
		NRF24L01_Write_Buf( TX_ADDR, buf, 5 );			//写入5个字节的地址
		NRF24L01_Read_Buf( TX_ADDR, read_buf, 5 );		//读出写入的地址  
		for( i = 0; i < 5; i++ )
		{
			if( buf[ i ] != read_buf[ i ] )
			{
				break;
			}	
		} 
		
		if( 5 == i )
		{
			return 0;
		}
		else
		{
		  try_time ++;
		}
		/* timeout */
		if( try_time > 5 )
		{
			return (-1);
		}
		/*----------*/
		rf_delay_ms( 2000 );
	}
}

 /**
  * @brief :设置模式
  * @param :
  *			@Mode:模式发送模式或接收模式
  * @note  :无
  * @retval:无
  */
void RF24L01_Set_Mode( nRf24l01ModeType Mode )
{
    unsigned char controlreg = 0;
	controlreg = NRF24L01_Read_Reg( CONFIG );
	
    if( Mode == MODE_TX )       
	{
		controlreg &= ~( 1<< PRIM_RX );
	}
    else 
	{
		if( Mode == MODE_RX )  
		{ 
			controlreg |= ( 1<< PRIM_RX ); 
		}
	}

    NRF24L01_Write_Reg( CONFIG, controlreg );
}

/**
  * @brief :NRF24L01发送一次数据
  * @param :
  *			@txbuf:待发送数据首地址
  *			@Length:发送数据长度
  * @note  :无
  * @retval:
  *			MAX_TX：达到最大重发次数
  *			TX_OK：发送完成
  *			0xFF:其他原因
  */  
unsigned char NRF24L01_TxPacket( unsigned char *txbuf, unsigned char Length ,unsigned char * addr)
{
	unsigned char l_Status = 0;
	uint16_t l_MsTimes = 0;
	
	RF24L01_SET_CS_LOW( );		//片选
	hal_spi_rw_byte( FLUSH_TX );
	RF24L01_SET_CS_HIGH( );
	
	RF24L01_SET_CE_LOW( );		
	NRF24L01_Write_Buf( WR_TX_PLOAD, txbuf, Length );	//写数据到TX BUF 32字节  TX_PLOAD_WIDTH
	RF24L01_SET_CE_HIGH( );			//启动发送
	while( 0 != RF24L01_GET_IRQ_STATUS( ))
	{
		rf_delay_ms( 1 );
		if( 500 == l_MsTimes++ )						//500ms还没有发送成功，重新初始化设备
		{
      RF24L01_Init(addr);
			RF24L01_Set_Mode( MODE_TX );
			break;
		}
	}
	l_Status = NRF24L01_Read_Reg(STATUS);						//读状态寄存器
	NRF24L01_Write_Reg( STATUS, l_Status );						//清除TX_DS或MAX_RT中断标志
	
	if( l_Status & MAX_TX )	//达到最大重发次数
	{
		NRF24L01_Write_Reg( FLUSH_TX,0xff );	//清除TX FIFO寄存器
		return MAX_TX; 
	}
	if( l_Status & TX_OK )	//发送完成
	{
		return TX_OK;
	}
	
	return 0xFF;	//其他原因发送失败
}

/**
  * @brief :NRF24L01接收数据
  * @param :
  *			@rxbuf:接收数据存放地址
  * @note  :无
  * @retval:接收到的数据个数
  */ 
unsigned char NRF24L01_RxPacket( unsigned char *rxbuf ,unsigned char *addr)
{
	unsigned char l_Status = 0, l_RxLength = 0;//, l_100MsTimes = 0;
	
	RF24L01_SET_CS_LOW( );		//片选
	hal_spi_rw_byte( FLUSH_RX );
	RF24L01_SET_CS_HIGH( );
	
	while( 0 != RF24L01_GET_IRQ_STATUS( ))
	{
//		rf_delay_ms( 100 );
//		
//		if( 30 == l_100MsTimes++ )		//3s没接收过数据，重新初始化模块
//		{
//			RF24L01_Init(addr );
//			RF24L01_Set_Mode( MODE_RX );
//			break;
//		}
	}
	
	l_Status = NRF24L01_Read_Reg( STATUS );		//读状态寄存器
	NRF24L01_Write_Reg( STATUS,l_Status );		//清中断标志
	if( l_Status & RX_OK)	//接收到数据
	{
		l_RxLength = NRF24L01_Read_Reg( R_RX_PL_WID );		//读取接收到的数据个数
		NRF24L01_Read_Buf( RD_RX_PLOAD,rxbuf,l_RxLength );	//接收到数据 
		NRF24L01_Write_Reg( FLUSH_RX,0xff );				//清除RX FIFO
		return l_RxLength; 
	}	
	
	return 0;				//没有收到数据	
}

 /**
  * @brief :RF24L01模块初始化
  * @param :无
  * @note  :无
  * @retval:无
  */
void RF24L01_Init(unsigned char * addr )
{
    /* addr */
    RF24L01_SET_CE_HIGH( );
    NRF24L01_Clear_IRQ_Flag( IRQ_ALL );
#if DYNAMIC_PACKET == 1

    NRF24L01_Write_Reg( DYNPD, ( 1 << 0 ) ); 	//ê1?üí¨μà1?ˉì?êy?Y3¤?è
    NRF24L01_Write_Reg( FEATRUE, 0x07 );
    NRF24L01_Read_Reg( DYNPD );
    NRF24L01_Read_Reg( FEATRUE );
	
#elif DYNAMIC_PACKET == 0
    
    L01_WriteSingleReg( L01REG_RX_PW_P0, FIXED_PACKET_LEN );	//1ì?¨êy?Y3¤?è
	
#endif	//DYNAMIC_PACKET

    NRF24L01_Write_Reg( CONFIG, /*( 1<<MASK_RX_DR ) |*/		//?óê??D??
                                      ( 1 << EN_CRC ) |     //ê1?üCRC 1??×??ú
                                      ( 1 << PWR_UP ) );    //?a??éè±?
    NRF24L01_Write_Reg( EN_AA, ( 1 << ENAA_P0 ) );   		//í¨μà0×??ˉó|′e
    NRF24L01_Write_Reg( EN_RXADDR, ( 1 << ERX_P0 ) );		//í¨μà0?óê?
    NRF24L01_Write_Reg( SETUP_AW, AW_5BYTES );     			//μ??·?í?è 5??×??ú
    NRF24L01_Write_Reg( SETUP_RETR, ARD_4000US |
                        ( REPEAT_CNT & 0x0F ) );         	//???′μè′yê±?? 250us
    NRF24L01_Write_Reg( RF_CH, 60 );             			//3?ê??ˉí¨μà
    NRF24L01_Write_Reg( RF_SETUP, 0x26 );

    NRF24L01_Set_TxAddr( addr, 5 );                      //éè??TXμ??·
    NRF24L01_Set_RxAddr( 0, addr, 5 );                   //éè??RXμ??·
		
    NRF24L01_Set_Speed(SPEED_1M);
}
/* set package */
int nrf_write(unsigned int addr,void * data , unsigned int len)
{
	/* set TX mode */
  RF24L01_Set_Mode( MODE_TX );
	/* send data */
  NRF24L01_TxPacket( (unsigned char *)data , len ,tr_addr_g );
  /* return */
	return 0;
}
/* nrf read */
int nrf_read(unsigned int addr,void * data , unsigned int len)
{
	RF24L01_Set_Mode( MODE_RX );
	/* read */
	unsigned char read_len = NRF24L01_RxPacket( (unsigned char *)data ,tr_addr_g);
	/* return */
	return read_len;
}
/* delay for a while , just for notify */
static void delay_ms_rf(unsigned int ms)
{
	unsigned int tick;
	/* get tick */
	tick = HAL_GetTick();
	/* wait */
	while( !((HAL_GetTick() - tick) > ms) );
}
/* process */
void key_process(unsigned int pm1,unsigned int pm2,unsigned int pm3,unsigned int pm4,unsigned int pm5)
{
		static unsigned int power_cnt = 0;
	  static unsigned int entrance_cnt = 0;
		static unsigned char release_flag = 0;
    dev_HandleTypeDef * fl = (dev_HandleTypeDef *)pm1;	
		/* check the key */
		if( HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_1) == 1 )
		{
			 if( release_flag )
			 {
					power_cnt ++ ;
					/* power off */
					if( power_cnt > 1000 )
					{
						HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);	
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
						/* infinite loop */
						while(1);
					}
			 }
			 else
			 {
				  if( ++entrance_cnt > 5000 )
					{
						 /* erase the flash zone */
					   fl->ioctrl(ERASE_FLASH,FEBASE_ADDR,0,0);
						 /* and reset system */
						 HAL_NVIC_SystemReset();
						 /* end of files */
				  }
			 }
		}
		else
		{
			/* clear */
			power_cnt = 0;
			/* enable the reciver */
			release_flag = 1;
		}
}
/* nrf 24l01 ioctrl */
int nrf_ioctrl(unsigned int cmd,unsigned int param,void * data,unsigned len)
{
	int ret = 0;
	rcs_HandleTypeDef * rcs;
	unsigned char * p;
	unsigned short tmp;
	/* get parse */
	switch(cmd)
	{
		case CREATE_DSM:
			/* rcs_HandleTypeDef */
		  rcs = (rcs_HandleTypeDef *)data;
		  p   = (unsigned char *)param;
		  /* transfer */
		  if( len != 16 )
			{
				return (-1); // cannot supply this format , dsm 
			}
		  tmp = rcs->channel[3] + 12;
		  tmp = (tmp&0x3ff)|(0x0<<10);  //channel 1 byte 0 
		  p[14] = tmp>>8;
		  p[15] = tmp&0xff;
		 
		  tmp =  rcs->channel[0] + 12;//yaw
		  tmp = (tmp&0x3ff)|(0x3<<10);  //channel 4 byte 3 
		  p[10] = tmp>>8;
		  p[11] = tmp&0xff;
		 
		  tmp =  rcs->channel[2] + 12;//roll
		  tmp = (tmp&0x3ff)|(0x1<<10);  //channel 2 byte 1 
		  p[2] = tmp>>8;
		  p[3] = tmp&0xff;
		 
		  tmp =  rcs->channel[1] + 12;//pitch
		  tmp = (tmp&0x3ff)|(0x2<<10);  //channel 3 byte 2 
		  p[6] = tmp>>8;
		  p[7] = tmp&0xff;
		 
		  tmp = ((rcs->channel567>>14) == 0x01)?171:(((rcs->channel567>>14)== 0x11)?512:853);
		  tmp = (tmp&0x3ff)|(0x5<<10);  //channel 6 byte 5 
		  p[4] = tmp>>8;
		  p[5] = tmp&0xff;
			
		  tmp = (rcs->channel567 & (1<<13))?15:512;
		  tmp = (tmp&0x3ff)|(0x4<<10);  //channel 6 byte 5 
		  p[8] = tmp>>8;
		  p[9] = tmp&0xff;		

		  tmp = (rcs->channel567 & (1<<12))?171:853;
		  tmp = (tmp&0x3ff)|(0x6<<10);  //channel 6 byte 5 
		  p[12] = tmp>>8;
		  p[13] = tmp&0xff;	
			/* head */
			p[0] = 0x00;
			p[1] = 0x02;
		  /* end if function */
			break;
		default:
			break;
	}
	/* return ret */
	return ret;
}
/* dev init */
int nrf24L01_Init( dev_HandleTypeDef * dev , void * spi_handle ,unsigned int pm)
{
	  /* transfer interface */
	  if( spi_handle != 0 && rf_spi_handle == 0 )
		{
			rf_spi_handle = spi_handle;
		}
		/* copy data */
		dev->write = nrf_write;
		dev->state = delay_ms_rf;
		dev->read  = nrf_read;
		dev->process = key_process;
		dev->ioctrl = nrf_ioctrl;
		/*---------*/
		if( NRF24L01_check() != 0 )
		{
			return (-1);
		}
		/* get and check pm */
		dev_HandleTypeDef * fs = (dev_HandleTypeDef *)pm;
		unsigned char unique_d[4];
		/* read from flash */
		fs->read(FEBASE_ADDR,unique_d,sizeof(unique_d));
		/* check it out */
		if( unique_d[2] == 0xAA && unique_d[0] != 0 && unique_d[1] != 0 &&
			(unsigned char)(unique_d[0]+unique_d[1]) == unique_d[3] )
		{
			/* got correct unique ID */
			tr_addr_g[0] = unique_d[0];
			tr_addr_g[1] = unique_d[1];
			/* set flag */
			dev->i_flag = 1;
			dev->i_pri  = unique_d[0] << 8 | unique_d[1]; 
		}				
		/* init rf */
		RF24L01_Init(tr_addr_g);
		/* ok */
		return 0;
}







