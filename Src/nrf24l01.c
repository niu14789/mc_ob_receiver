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
void hal_spi_write_byte(unsigned char data)
{
	unsigned char d = data;
	/* send */
	HAL_SPI_Transmit(rf_spi_handle,&d,1,0xffffffff);
}	
/* spi read */
unsigned char hal_spi_read_byte(unsigned char data)
{
	unsigned char d;
	/* send */
	HAL_SPI_Receive(rf_spi_handle,&d,1,0xffffffff);
	/* return */
	return d;
}	
/**
  * @brief :NRF24L01¶Á¼Ä´æÆ÷
  * @param :
           @Addr:¼Ä´æÆ÷µØÖ·
  * @note  :µØÖ·ÔÚÉè±¸ÖĞÓĞĞ§
  * @retval:¶ÁÈ¡µÄÊı¾İ
  */
unsigned char NRF24L01_Read_Reg( unsigned char RegAddr )
{
    unsigned char btmp;
	
    RF24L01_SET_CS_LOW( );			//Æ¬Ñ¡
	
    hal_spi_write_byte( NRF_READ_REG | RegAddr );	//¶ÁÃüÁî µØÖ·
    btmp = hal_spi_read_byte( 0xFF );				//¶ÁÊı¾İ
	
    RF24L01_SET_CS_HIGH( );			//È¡ÏûÆ¬Ñ¡
	
    return btmp;
}

/**
  * @brief :NRF24L01¶ÁÖ¸¶¨³¤¶ÈµÄÊı¾İ
  * @param :
  *			@reg:µØÖ·
  *			@pBuf:Êı¾İ´æ·ÅµØÖ·
  *			@len:Êı¾İ³¤¶È
  * @note  :Êı¾İ³¤¶È²»³¬¹ı255£¬µØÖ·ÔÚÉè±¸ÖĞÓĞĞ§
  * @retval:¶ÁÈ¡×´Ì¬
  */
void NRF24L01_Read_Buf( unsigned char RegAddr, unsigned char *pBuf, unsigned char len )
{
    unsigned char btmp;
	
    RF24L01_SET_CS_LOW( );			//Æ¬Ñ¡
	
    hal_spi_write_byte( NRF_READ_REG | RegAddr );	//¶ÁÃüÁî µØÖ·
    for( btmp = 0; btmp < len; btmp ++ )
    {
        *( pBuf + btmp ) = hal_spi_read_byte( 0xFF );	//¶ÁÊı¾İ
    }
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
}

/**
  * @brief :NRF24L01Ğ´¼Ä´æÆ÷
  * @param :ÎŞ
  * @note  :µØÖ·ÔÚÉè±¸ÖĞÓĞĞ§
  * @retval:¶ÁĞ´×´Ì¬
  */
void NRF24L01_Write_Reg( unsigned char RegAddr, unsigned char Value )
{
    RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
	
    hal_spi_write_byte( NRF_WRITE_REG | RegAddr );	//Ğ´ÃüÁî µØÖ·
    hal_spi_write_byte( Value );			//Ğ´Êı¾İ
	
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
}

/**
  * @brief :NRF24L01Ğ´Ö¸¶¨³¤¶ÈµÄÊı¾İ
  * @param :
  *			@reg:µØÖ·
  *			@pBuf:Ğ´ÈëµÄÊı¾İµØÖ·
  *			@len:Êı¾İ³¤¶È
  * @note  :Êı¾İ³¤¶È²»³¬¹ı255£¬µØÖ·ÔÚÉè±¸ÖĞÓĞĞ§
  * @retval:Ğ´×´Ì¬
  */
void NRF24L01_Write_Buf( unsigned char RegAddr, unsigned char *pBuf, unsigned char len )
{
    unsigned char i;
	
    RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
	
    hal_spi_write_byte( NRF_WRITE_REG | RegAddr );	//Ğ´ÃüÁî µØÖ·
    for( i = 0; i < len; i ++ )
    {
        hal_spi_write_byte( *( pBuf + i ) );		//Ğ´Êı¾İ
    }
	
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
}

/**
  * @brief :Çå¿ÕTX»º³åÇø
  * @param :ÎŞ
  * @note  :ÎŞ
  * @retval:ÎŞ
  */
void NRF24L01_Flush_Tx_Fifo ( void )
{
    RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
	
    hal_spi_write_byte( FLUSH_TX );	//ÇåTX FIFOÃüÁî
	
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
}

/**
  * @brief :Çå¿ÕRX»º³åÇø
  * @param :ÎŞ
  * @note  :ÎŞ
  * @retval:ÎŞ
  */
void NRF24L01_Flush_Rx_Fifo( void )
{
    RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
	
    hal_spi_write_byte( FLUSH_RX );	//ÇåRX FIFOÃüÁî
	
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
}

/**
  * @brief :ÖØĞÂÊ¹ÓÃÉÏÒ»°üÊı¾İ
  * @param :ÎŞ
  * @note  :ÎŞ
  * @retval:ÎŞ
  */
void NRF24L01_Reuse_Tx_Payload( void )
{
    RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
	
    hal_spi_write_byte( REUSE_TX_PL );		//ÖØĞÂÊ¹ÓÃÉÏÒ»°üÃüÁî
	
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
}

/**
  * @brief :NRF24L01¿Õ²Ù×÷
  * @param :ÎŞ
  * @note  :ÎŞ
  * @retval:ÎŞ
  */
void NRF24L01_Nop( void )
{
    RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
	
    hal_spi_write_byte( NOP );		//¿Õ²Ù×÷ÃüÁî
	
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
}

/**
  * @brief :NRF24L01¶Á×´Ì¬¼Ä´æÆ÷
  * @param :ÎŞ
  * @note  :ÎŞ
  * @retval:RF24L01×´Ì¬
  */
unsigned char NRF24L01_Read_Status_Register( void )
{
    unsigned char Status;
	
    RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
	
    Status = hal_spi_read_byte( NRF_READ_REG + STATUS );	//¶Á×´Ì¬¼Ä´æÆ÷
	
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
	
    return Status;
}

/**
  * @brief :NRF24L01ÇåÖĞ¶Ï
  * @param :
           @IRQ_Source:ÖĞ¶ÏÔ´
  * @note  :ÎŞ
  * @retval:Çå³ıºó×´Ì¬¼Ä´æÆ÷µÄÖµ
  */
unsigned char NRF24L01_Clear_IRQ_Flag( unsigned char IRQ_Source )
{
    unsigned char btmp = 0;

    IRQ_Source &= ( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT );	//ÖĞ¶Ï±êÖ¾´¦Àí
    btmp = NRF24L01_Read_Status_Register( );			//¶Á×´Ì¬¼Ä´æÆ÷
			
    RF24L01_SET_CS_LOW( );			//Æ¬Ñ¡
    hal_spi_write_byte( NRF_WRITE_REG + STATUS );	//Ğ´×´Ì¬¼Ä´æÆ÷ÃüÁî
    hal_spi_write_byte( IRQ_Source | btmp );		//ÇåÏàÓ¦ÖĞ¶Ï±êÖ¾
    RF24L01_SET_CS_HIGH( );			//È¡ÏûÆ¬Ñ¡
	
    return ( NRF24L01_Read_Status_Register( ));			//·µ»Ø×´Ì¬¼Ä´æÆ÷×´Ì¬
}

/**
  * @brief :¶ÁRF24L01ÖĞ¶Ï×´Ì¬
  * @param :ÎŞ
  * @note  :ÎŞ
  * @retval:ÖĞ¶Ï×´Ì¬
  */
unsigned char RF24L01_Read_IRQ_Status( void )
{
    return ( NRF24L01_Read_Status_Register( ) & (( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT )));	//·µ»ØÖĞ¶Ï×´Ì¬
}
 
 /**
  * @brief :¶ÁFIFOÖĞÊı¾İ¿í¶È
  * @param :ÎŞ
  * @note  :ÎŞ
  * @retval:Êı¾İ¿í¶È
  */
unsigned char NRF24L01_Read_Top_Fifo_Width( void )
{
    unsigned char btmp;
	
    RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
	
    hal_spi_write_byte( R_RX_PL_WID );	//¶ÁFIFOÖĞÊı¾İ¿í¶ÈÃüÁî
    btmp = hal_spi_read_byte( 0xFF );	//¶ÁÊı¾İ
	
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
	
    return btmp;
}

 /**
  * @brief :¶Á½ÓÊÕµ½µÄÊı¾İ
  * @param :ÎŞ
  * @note  :ÎŞ
  * @retval:
           @pRxBuf:Êı¾İ´æ·ÅµØÖ·Ê×µØÖ·
  */
unsigned char NRF24L01_Read_Rx_Payload( unsigned char *pRxBuf )
{
    unsigned char Width, PipeNum;
	
    PipeNum = ( NRF24L01_Read_Reg( STATUS ) >> 1 ) & 0x07;	//¶Á½ÓÊÕ×´Ì¬
    Width = NRF24L01_Read_Top_Fifo_Width( );		//¶Á½ÓÊÕÊı¾İ¸öÊı

    RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
    hal_spi_write_byte( RD_RX_PLOAD );			//¶ÁÓĞĞ§Êı¾İÃüÁî
	
    for( PipeNum = 0; PipeNum < Width; PipeNum ++ )
    {
        *( pRxBuf + PipeNum ) = hal_spi_read_byte( 0xFF );		//¶ÁÊı¾İ
    }
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
    NRF24L01_Flush_Rx_Fifo( );	//Çå¿ÕRX FIFO
	
    return Width;
}

 /**
  * @brief :·¢ËÍÊı¾İ£¨´øÓ¦´ğ£©
  * @param :
  *			@pTxBuf:·¢ËÍÊı¾İµØÖ·
  *			@len:³¤¶È
  * @note  :Ò»´Î²»³¬¹ı32¸ö×Ö½Ú
  * @retval:ÎŞ
  */
void NRF24L01_Write_Tx_Payload_Ack( unsigned char *pTxBuf, unsigned char len )
{
    unsigned char btmp;
    unsigned char length = ( len > 32 ) ? 32 : len;		//Êı¾İ³¤´ï´óÔ¼32 ÔòÖ»·¢ËÍ32¸ö

    NRF24L01_Flush_Tx_Fifo( );		//ÇåTX FIFO
	
    RF24L01_SET_CS_LOW( );			//Æ¬Ñ¡
    hal_spi_write_byte( WR_TX_PLOAD );	//·¢ËÍÃüÁî
	
    for( btmp = 0; btmp < length; btmp ++ )
    {
        hal_spi_write_byte( *( pTxBuf + btmp ) );	//·¢ËÍÊı¾İ
    }
    RF24L01_SET_CS_HIGH( );			//È¡ÏûÆ¬Ñ¡
}

 /**
  * @brief :·¢ËÍÊı¾İ£¨²»´øÓ¦´ğ£©
  * @param :
  *			@pTxBuf:·¢ËÍÊı¾İµØÖ·
  *			@len:³¤¶È
  * @note  :Ò»´Î²»³¬¹ı32¸ö×Ö½Ú
  * @retval:ÎŞ
  */
void NRF24L01_Write_Tx_Payload_NoAck( unsigned char *pTxBuf, unsigned char len )
{
    if( len > 32 || len == 0 )
    {
        return ;		//Êı¾İ³¤¶È´óÓÚ32 »òÕßµÈÓÚ0 ²»Ö´ĞĞ
    }
	
    RF24L01_SET_CS_LOW( );	//Æ¬Ñ¡
    hal_spi_write_byte( WR_TX_PLOAD_NACK );	//·¢ËÍÃüÁî
    while( len-- )
    {
        hal_spi_write_byte( *pTxBuf );			//·¢ËÍÊı¾İ
		pTxBuf++;
    }
    RF24L01_SET_CS_HIGH( );		//È¡ÏûÆ¬Ñ¡
}

 /**
  * @brief :ÔÚ½ÓÊÕÄ£Ê½ÏÂÏòTX FIFOĞ´Êı¾İ(´øACK)
  * @param :
  *			@pData:Êı¾İµØÖ·
  *			@len:³¤¶È
  * @note  :Ò»´Î²»³¬¹ı32¸ö×Ö½Ú
  * @retval:ÎŞ
  */
void NRF24L01_Write_Tx_Payload_InAck( unsigned char *pData, unsigned char len )
{
    unsigned char btmp;
	
	len = ( len > 32 ) ? 32 : len;		//Êı¾İ³¤¶È´óÓÚ32¸öÔòÖ»Ğ´32¸ö×Ö½Ú

    RF24L01_SET_CS_LOW( );			//Æ¬Ñ¡
    hal_spi_write_byte( W_ACK_PLOAD );		//ÃüÁî
    for( btmp = 0; btmp < len; btmp ++ )
    {
        hal_spi_write_byte( *( pData + btmp ) );	//Ğ´Êı¾İ
    }
    RF24L01_SET_CS_HIGH( );			//È¡ÏûÆ¬Ñ¡
}

 /**
  * @brief :ÉèÖÃ·¢ËÍµØÖ·
  * @param :
  *			@pAddr:µØÖ·´æ·ÅµØÖ·
  *			@len:³¤¶È
  * @note  :ÎŞ
  * @retval:ÎŞ
  */
void NRF24L01_Set_TxAddr( unsigned char *pAddr, unsigned char len )
{
	len = ( len > 5 ) ? 5 : len;					//µØÖ·²»ÄÜ´óÓÚ5¸ö×Ö½Ú
    NRF24L01_Write_Buf( TX_ADDR, pAddr, len );	//Ğ´µØÖ·
}

 /**
  * @brief :ÉèÖÃ½ÓÊÕÍ¨µÀµØÖ·
  * @param :
  *			@PipeNum:Í¨µÀ
  *			@pAddr:µØÖ·´æ·Ê×ÅµØÖ·
  *			@Len:³¤¶È
  * @note  :Í¨µÀ²»´óÓÚ5 µØÖ·³¤¶È²»´óÓÚ5¸ö×Ö½Ú
  * @retval:ÎŞ
  */
void NRF24L01_Set_RxAddr( unsigned char PipeNum, unsigned char *pAddr, unsigned char Len )
{
    Len = ( Len > 5 ) ? 5 : Len;
    PipeNum = ( PipeNum > 5 ) ? 5 : PipeNum;		//Í¨µÀ²»´óÓÚ5 µØÖ·³¤¶È²»´óÓÚ5¸ö×Ö½Ú

    NRF24L01_Write_Buf( RX_ADDR_P0 + PipeNum, pAddr, Len );	//Ğ´ÈëµØÖ·
}

 /**
  * @brief :ÉèÖÃÍ¨ĞÅËÙ¶È
  * @param :
  *			@Speed:ËÙ¶È
  * @note  :ÎŞ
  * @retval:ÎŞ
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
  * @brief :ÉèÖÃ¹¦ÂÊ
  * @param :
  *			@Speed:ËÙ¶È
  * @note  :ÎŞ
  * @retval:ÎŞ
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
  * @brief :ÉèÖÃÆµÂÊ
  * @param :
  *			@FreqPoint:ÆµÂÊÉèÖÃ²ÎÊı
  * @note  :Öµ²»´óÓÚ127
  * @retval:ÎŞ
  */
void RF24LL01_Write_Hopping_Point( unsigned char FreqPoint )
{
    NRF24L01_Write_Reg(  RF_CH, FreqPoint & 0x7F );
}

/**
  * @brief :NRF24L01¼ì²â
  * @param :ÎŞ
  * @note  :ÎŞ
  * @retval:ÎŞ
  */ 
int NRF24L01_check( void )
{
	unsigned char i;
	unsigned char buf[5]={ 0XA5, 0XA5, 0XA5, 0XA5, 0XA5 };
	unsigned char read_buf[ 5 ] = { 0 };
	unsigned char try_time = 0;
	while( 1 )
	{
		NRF24L01_Write_Buf( TX_ADDR, buf, 5 );			//Ğ´Èë5¸ö×Ö½ÚµÄµØÖ·
		NRF24L01_Read_Buf( TX_ADDR, read_buf, 5 );		//¶Á³öĞ´ÈëµÄµØÖ·  
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
  * @brief :ÉèÖÃÄ£Ê½
  * @param :
  *			@Mode:Ä£Ê½·¢ËÍÄ£Ê½»ò½ÓÊÕÄ£Ê½
  * @note  :ÎŞ
  * @retval:ÎŞ
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
  * @brief :NRF24L01·¢ËÍÒ»´ÎÊı¾İ
  * @param :
  *			@txbuf:´ı·¢ËÍÊı¾İÊ×µØÖ·
  *			@Length:·¢ËÍÊı¾İ³¤¶È
  * @note  :ÎŞ
  * @retval:
  *			MAX_TX£º´ïµ½×î´óÖØ·¢´ÎÊı
  *			TX_OK£º·¢ËÍÍê³É
  *			0xFF:ÆäËûÔ­Òò
  */  
unsigned char NRF24L01_TxPacket( unsigned char *txbuf, unsigned char Length ,unsigned char * addr)
{
	unsigned char l_Status = 0;
	uint16_t l_MsTimes = 0;
	
	RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
	hal_spi_write_byte( FLUSH_TX );
	RF24L01_SET_CS_HIGH( );
	
	RF24L01_SET_CE_LOW( );		
	NRF24L01_Write_Buf( WR_TX_PLOAD, txbuf, Length );	//Ğ´Êı¾İµ½TX BUF 32×Ö½Ú  TX_PLOAD_WIDTH
	RF24L01_SET_CE_HIGH( );			//Æô¶¯·¢ËÍ
	while( 0 != RF24L01_GET_IRQ_STATUS( ))
	{
		rf_delay_ms( 1 );
		if( 500 == l_MsTimes++ )						//500ms»¹Ã»ÓĞ·¢ËÍ³É¹¦£¬ÖØĞÂ³õÊ¼»¯Éè±¸
		{
      RF24L01_Init(addr);
			RF24L01_Set_Mode( MODE_TX );
			break;
		}
	}
	l_Status = NRF24L01_Read_Reg(STATUS);						//¶Á×´Ì¬¼Ä´æÆ÷
	NRF24L01_Write_Reg( STATUS, l_Status );						//Çå³ıTX_DS»òMAX_RTÖĞ¶Ï±êÖ¾
	
	if( l_Status & MAX_TX )	//´ïµ½×î´óÖØ·¢´ÎÊı
	{
		NRF24L01_Write_Reg( FLUSH_TX,0xff );	//Çå³ıTX FIFO¼Ä´æÆ÷
		return MAX_TX; 
	}
	if( l_Status & TX_OK )	//·¢ËÍÍê³É
	{
		return TX_OK;
	}
	
	return 0xFF;	//ÆäËûÔ­Òò·¢ËÍÊ§°Ü
}

/**
  * @brief :NRF24L01½ÓÊÕÊı¾İ
  * @param :
  *			@rxbuf:½ÓÊÕÊı¾İ´æ·ÅµØÖ·
  * @note  :ÎŞ
  * @retval:½ÓÊÕµ½µÄÊı¾İ¸öÊı
  */ 
unsigned char NRF24L01_RxPacket( unsigned char *rxbuf ,unsigned char *addr)
{
	unsigned char l_Status = 0, l_RxLength = 0;//, l_100MsTimes = 0;
	
	RF24L01_SET_CS_LOW( );		//Æ¬Ñ¡
	hal_spi_write_byte( FLUSH_RX );
	RF24L01_SET_CS_HIGH( );
	
	while( 0 != RF24L01_GET_IRQ_STATUS( ))
	{
//		rf_delay_ms( 100 );
//		
//		if( 30 == l_100MsTimes++ )		//3sÃ»½ÓÊÕ¹ıÊı¾İ£¬ÖØĞÂ³õÊ¼»¯Ä£¿é
//		{
//			RF24L01_Init(addr );
//			RF24L01_Set_Mode( MODE_RX );
//			break;
//		}
	}
	
	l_Status = NRF24L01_Read_Reg( STATUS );		//¶Á×´Ì¬¼Ä´æÆ÷
	NRF24L01_Write_Reg( STATUS,l_Status );		//ÇåÖĞ¶Ï±êÖ¾
	if( l_Status & RX_OK)	//½ÓÊÕµ½Êı¾İ
	{
		l_RxLength = NRF24L01_Read_Reg( R_RX_PL_WID );		//¶ÁÈ¡½ÓÊÕµ½µÄÊı¾İ¸öÊı
		NRF24L01_Read_Buf( RD_RX_PLOAD,rxbuf,l_RxLength );	//½ÓÊÕµ½Êı¾İ 
		NRF24L01_Write_Reg( FLUSH_RX,0xff );				//Çå³ıRX FIFO
		return l_RxLength; 
	}	
	
	return 0;				//Ã»ÓĞÊÕµ½Êı¾İ	
}

 /**
  * @brief :RF24L01Ä£¿é³õÊ¼»¯
  * @param :ÎŞ
  * @note  :ÎŞ
  * @retval:ÎŞ
  */
void RF24L01_Init(unsigned char * addr )
{
    /* addr */
    RF24L01_SET_CE_HIGH( );
    NRF24L01_Clear_IRQ_Flag( IRQ_ALL );
#if DYNAMIC_PACKET == 1

    NRF24L01_Write_Reg( DYNPD, ( 1 << 0 ) ); 	//¨º1?¨¹¨ª¡§¦Ì¨¤1?¡¥¨¬?¨ºy?Y3¡è?¨¨
    NRF24L01_Write_Reg( FEATRUE, 0x07 );
    NRF24L01_Read_Reg( DYNPD );
    NRF24L01_Read_Reg( FEATRUE );
	
#elif DYNAMIC_PACKET == 0
    
    L01_WriteSingleReg( L01REG_RX_PW_P0, FIXED_PACKET_LEN );	//1¨¬?¡§¨ºy?Y3¡è?¨¨
	
#endif	//DYNAMIC_PACKET

    NRF24L01_Write_Reg( CONFIG, /*( 1<<MASK_RX_DR ) |*/		//?¨®¨º??D??
                                      ( 1 << EN_CRC ) |     //¨º1?¨¹CRC 1??¡Á??¨²
                                      ( 1 << PWR_UP ) );    //?a??¨¦¨¨¡À?
    NRF24L01_Write_Reg( EN_AA, ( 1 << ENAA_P0 ) );   		//¨ª¡§¦Ì¨¤0¡Á??¡¥¨®|¡äe
    NRF24L01_Write_Reg( EN_RXADDR, ( 1 << ERX_P0 ) );		//¨ª¡§¦Ì¨¤0?¨®¨º?
    NRF24L01_Write_Reg( SETUP_AW, AW_5BYTES );     			//¦Ì??¡¤?¨ª?¨¨ 5??¡Á??¨²
    NRF24L01_Write_Reg( SETUP_RETR, ARD_4000US |
                        ( REPEAT_CNT & 0x0F ) );         	//???¡ä¦Ì¨¨¡äy¨º¡À?? 250us
    NRF24L01_Write_Reg( RF_CH, 60 );             			//3?¨º??¡¥¨ª¡§¦Ì¨¤
    NRF24L01_Write_Reg( RF_SETUP, 0x26 );

    NRF24L01_Set_TxAddr( addr, 5 );                      //¨¦¨¨??TX¦Ì??¡¤
    NRF24L01_Set_RxAddr( 0, addr, 5 );                   //¨¦¨¨??RX¦Ì??¡¤
		
    NRF24L01_Set_Speed(SPEED_1M);
}
/* set package */
int nrf_write(unsigned int addr,void * data , unsigned int len)
{
	/* set TX mode */
  RF24L01_Set_Mode( MODE_TX );
	/* send data */
  NRF24L01_TxPacket( (unsigned char *)&data , len ,tr_addr_g );
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
//RF24L01_Set_Mode( MODE_RX );		//½ÓÊÕÄ£Ê½
//			/* read data */
//			j = NRF24L01_RxPacket( (unsigned char *)RxBuffer );		//½ÓÊÕ×Ö½
/* delay for a while , just for notify */
static void delay_ms_rf(unsigned int ms)
{
	unsigned int tick;
	/* get tick */
	tick = HAL_GetTick();
	/* wait */
	while( !((HAL_GetTick() - tick) > ms) );
}
/* dev init */
int nrf24L01_Init( dev_HandleTypeDef * dev , void * spi_handle )
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
		/*---------*/
		if( NRF24L01_check() != 0 )
		{
			return (-1);
		}
		/*-------------*/
		RF24L01_Init(tr_addr_g);
		/* ok */
		return 0;
}







