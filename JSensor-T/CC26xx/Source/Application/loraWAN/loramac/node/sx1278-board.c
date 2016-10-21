/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "lboard.h"
#include "radio.h"
#include "sx1278/sx1278.h"
#include "sx1278-board.h"
#include "bsp_spi.h"
/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1278Init,
    SX1278GetStatus,
    SX1278SetModem,
    SX1278SetChannel,
    SX1278IsChannelFree,
    SX1278Random,
    SX1278SetRxConfig,
    SX1278SetTxConfig,
    SX1278CheckRfFrequency,
    SX1278GetTimeOnAir,
    SX1278Send,
    SX1278SetSleep,
    SX1278SetStby, 
    SX1278SetRx,
    SX1278StartCad,
    SX1278ReadRssi,
    SX1278Write,
    SX1278Read,
    SX1278WriteBuffer,
    SX1278ReadBuffer,
    SX1278SetMaxPayloadLength
};


void SX1278IoInit( void )
{
}

void SX1278IoDeInit( void )
{
}

uint8_t SX1278GetPaSelect( uint32_t channel )
{
    if( channel < RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1278SetAntSwLowPower( bool status )
{
    if( RadioIsActive != status )
    {
        RadioIsActive = status;
    
        if( status == false )
        {
            SX1278AntSwInit( );
        }
        else
        {
            SX1278AntSwDeInit( );
        }
    }
}

void SX1278AntSwInit( void )
{
}  

void SX1278AntSwDeInit( void )
{
}   

void SX1278SetAntSw( uint8_t rxTx )
{
    if( SX1278.RxTx == rxTx )
    {
 //       return;
    }

    SX1278.RxTx = rxTx;

    if( rxTx != 0 ) // 1: TX, 0: RX
    {
        SX1276WriteRxTx(1);
    }
    else
    {        
        SX1276WriteRxTx(0);        
    }
}

bool SX1278CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
