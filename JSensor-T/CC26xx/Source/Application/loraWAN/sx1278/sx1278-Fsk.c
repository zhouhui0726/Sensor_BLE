/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1276.c
 * \brief      SX1276 RF chip driver
 *
 * \version    2.0.B2 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "sx1278.h"

//#include "sx1278Regs-Fsk.h"
#include "sx1278-Fsk.h"
#include "bsp_spi.h"
#include "sx1278-FskMisc.h"
// Default settings
tFskSettings FskSettings = 
{
    470000000,      // RFFrequency
    9600,           // Bitrate
    0,          // Fdev
    20,             // Power
    100000,         // RxBw
    150000,         // RxBwAfc
    true,           // CrcOn
    true,           // AfcOn    
    255             // PayloadLength (set payload size to the maximum for variable mode, else set the exact payload length)
};

static uint8_t SX1276Regs[0x70];

/*!
 * SX1276 FSK registers variable
 */
tSX1278* SX1276;

/*!
 * Local RF buffer for communication support
 */
//static uint8_t RFBuffer[2];

/*!
 * Chunk size of data write in buffer 
 */
//static uint8_t DataChunkSize = 32;


/*!
 * RF state machine variable
 */
static uint8_t RFState = RF_STATE_IDLE;

/*!
 * Rx management support variables
 */

/*!
 * PacketTimeout holds the RF packet timeout
 * SyncSize = [0..8]
 * VariableSize = [0;1]
 * AddressSize = [0;1]
 * PayloadSize = [0..RF_BUFFER_SIZE]
 * CrcSize = [0;2]
 * PacketTimeout = ( ( 8 * ( VariableSize + AddressSize + PayloadSize + CrcSize ) / BR ) * 1000.0 ) + 1
 * Computed timeout is in miliseconds
 */
//static uint32_t PacketTimeout;

/*!
 * Preamble2SyncTimeout
 * Preamble2SyncTimeout = ( ( 8 * ( PremableSize + SyncSize ) / RFBitrate ) * 1000.0 ) + 1
 * Computed timeout is in miliseconds
 */
//static uint32_t Preamble2SyncTimeout;

//static bool PreambleDetected = false;
//static bool SyncWordDetected = false;
//static bool PacketDetected = false;
//static uint16_t RxPacketSize = 0;
//static uint8_t RxBytesRead = 0;
//static uint8_t TxBytesSent = 0;
//static double RxPacketRssiValue;
//static uint32_t RxPacketAfcValue;
//static uint8_t RxGain = 1;
//static uint32_t RxTimeoutTimer = 0;
//static uint32_t Preamble2SyncTimer = 0;

/*!
 * Tx management support variables
 */
//static uint16_t TxPacketSize = 0;
//static uint32_t TxTimeoutTimer = 0;

void SX1278FskInit( void )
{
    SX1276 = (tSX1278*)SX1276Regs;
    RFState = RF_STATE_IDLE;

    SX1278FskSetDefaults( );
    
    SX1278ReadBuffer( REG_OPMODE, SX1276Regs + 1, 0x70 - 1 );

    // Set the device in FSK mode and Sleep Mode
    SX1276->RegOpMode = RF_OPMODE_MODULATIONTYPE_FSK | RF_OPMODE_SLEEP;
    SX1278Write( REG_OPMODE, SX1276->RegOpMode );

    SX1276->RegOpMode = RF_OPMODE_MODULATIONTYPE_FSK | RF_OPMODE_SLEEP;
    SX1278Write( REG_OPMODE, SX1276->RegOpMode );

    SX1276->RegPaRamp = RF_PARAMP_MODULATIONSHAPING_01;
    SX1278Write( REG_PARAMP, SX1276->RegPaRamp );

    SX1276->RegLna = RF_LNA_GAIN_G1;
    SX1278Write( REG_LNA, SX1276->RegLna );

    if( FskSettings.AfcOn == true )
    {
        SX1276->RegRxConfig = RF_RXCONFIG_RESTARTRXONCOLLISION_OFF | RF_RXCONFIG_AFCAUTO_ON |
                              RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT;
    }
    else
    {
        SX1276->RegRxConfig = RF_RXCONFIG_RESTARTRXONCOLLISION_OFF | RF_RXCONFIG_AFCAUTO_OFF |
                              RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT;
    }

    SX1276->RegPreambleLsb = 8;
    
    SX1276->RegPreambleDetect = RF_PREAMBLEDETECT_DETECTOR_ON | RF_PREAMBLEDETECT_DETECTORSIZE_2 |
                                RF_PREAMBLEDETECT_DETECTORTOL_10;

    SX1276->RegRssiThresh = 0xFF;

    SX1276->RegSyncConfig = RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_ON | RF_SYNCCONFIG_PREAMBLEPOLARITY_AA |
                            RF_SYNCCONFIG_SYNC_ON |
                            RF_SYNCCONFIG_SYNCSIZE_4;

    SX1276->RegSyncValue1 = 0x69;
    SX1276->RegSyncValue2 = 0x81;
    SX1276->RegSyncValue3 = 0x7E;
    SX1276->RegSyncValue4 = 0x96;

    SX1276->RegPacketConfig1 = RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RF_PACKETCONFIG1_DCFREE_OFF |
                               ( FskSettings.CrcOn << 4 ) | RF_PACKETCONFIG1_CRCAUTOCLEAR_ON |
                               RF_PACKETCONFIG1_ADDRSFILTERING_OFF | RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT;
    SX1276->RegPacketConfig2 = 0;//��������ģʽ������ģʽ
    SX1278FskGetPacketCrcOn( ); // Update CrcOn on FskSettings

    SX1276->RegPayloadLength = FskSettings.PayloadLength;

    // we can now update the registers with our configuration
    SX1278WriteBuffer( REG_OPMODE, SX1276Regs + 1, 0x70 - 1 );

    // then we need to set the RF settings 
    SX1278FskSetRFFrequency( FskSettings.RFFrequency );
    SX1278FskSetBitrate( FskSettings.Bitrate );
    SX1278FskSetFdev( FskSettings.Fdev );
    SX1278FskSetPa20dBm( true );
    SX1278FskSetRFPower( FskSettings.Power );
//   SX1278FskSetDccBw( &SX1276->RegRxBw, 0, FskSettings.RxBw );
//   SX1278FskSetDccBw( &SX1276->RegAfcBw, 0, FskSettings.RxBwAfc );
    SX1278FskSetRssiOffset( 0 );

    SX1278FskSetOpMode( RF_OPMODE_STANDBY );
    SX1278FskSetOpMode( RF_OPMODE_TRANSMITTER );

    // Calibrate the HF
//    SX1278FskRxCalibrate( );
}

void SX1278FskSetDefaults( void )
{
    // REMARK: See SX1276 datasheet for modified default values.

    SX1276->RegVersion = SX1278Read( REG_VERSION );
}

void SX1278FskSetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RF_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;

    opModePrev = SX1276->RegOpMode & ~RF_OPMODE_MASK;

    if( opMode != opModePrev )
    {
        if( opMode == RF_OPMODE_TRANSMITTER )
        {
            antennaSwitchTxOn = true;            
        }
        else
        {
            antennaSwitchTxOn = false;            
        }
        if( antennaSwitchTxOn != antennaSwitchTxOnPrev )
        {
            antennaSwitchTxOnPrev = antennaSwitchTxOn;
            SX1276WriteRxTx( antennaSwitchTxOn ); // Antenna switch control
        }
        SX1276->RegOpMode = ( SX1276->RegOpMode & RF_OPMODE_MASK ) | opMode;

        SX1278Write( REG_OPMODE, SX1276->RegOpMode );        
    }
}

#if 0
uint8_t SX1278FskGetOpMode( void )
{
    SX1276->RegOpMode = SX1278Read( REG_OPMODE );
    
    return SX1276->RegOpMode & ~RF_OPMODE_MASK;
}

int32_t SX1278FskReadFei( void )
{
    SX1278ReadBuffer( REG_FEIMSB, &SX1276->RegFeiMsb, 2 );                          // Reads the FEI value

    return ( int32_t )( double )( ( ( uint16_t )SX1276->RegFeiMsb << 8 ) | ( uint16_t )SX1276->RegFeiLsb ) * ( double )FREQ_STEP;
}

int32_t SX1278FskReadAfc( void )
{
    SX1278ReadBuffer( REG_AFCMSB, &SX1276->RegAfcMsb, 2 );                            // Reads the AFC value
    return ( int32_t )( double )( ( ( uint16_t )SX1276->RegAfcMsb << 8 ) | ( uint16_t )SX1276->RegAfcLsb ) * ( double )FREQ_STEP;
}

uint8_t SX1278FskReadRxGain( void )
{
    SX1276->RegLna = SX1278Read( REG_LNA );
    return( SX1276->RegLna >> 5 ) & 0x07;
}

double SX1278FskReadRssi( void )
{
    SX1276->RegRssiValue = SX1278Read( REG_RSSIVALUE );                               // Reads the RSSI value

    return -( double )( ( double )SX1276->RegRssiValue / 2.0 );
}

uint8_t SX1278FskGetPacketRxGain( void )
{
    return RxGain;
}

double SX1278FskGetPacketRssi( void )
{
    return RxPacketRssiValue;
}

uint32_t SX1278FskGetPacketAfc( void )
{
    return RxPacketAfcValue;
}

void SX1278FskStartRx( void )
{
    SX1278FskSetRFState( RF_STATE_RX_INIT );
}

void SX1278FskGetRxPacket( void *buffer, uint16_t *size )
{
    *size = RxPacketSize;
    RxPacketSize = 0;
    memcpy( ( void * )buffer, ( void * )RFBuffer, ( size_t )*size );
}

void SX1278FskSetTxPacket( const void *buffer, uint16_t size )
{
    TxPacketSize = size;
    memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize ); 

    RFState = RF_STATE_TX_INIT;

     SX1276->RegDioMapping1 = RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_00 | RF_DIOMAPPING1_DIO3_01;
        //                           LowBat,                   Data
        SX1276->RegDioMapping2 = RF_DIOMAPPING2_DIO4_00 | RF_DIOMAPPING2_DIO5_10;
        SX1278WriteBuffer( REG_DIOMAPPING1, &SX1276->RegDioMapping1, 2 );

        SX1276->RegFifoThresh = RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY | 0x18; // 24 bytes of data
        SX1278Write( REG_FIFOTHRESH, SX1276->RegFifoThresh );

        SX1278FskSetOpMode( RF_OPMODE_TRANSMITTER );
        
    //    SX1278WriteFifo( ( uint8_t* )&TxPacketSize, 1 );
        
   //     SX1278WriteFifo( RFBuffer, TxPacketSize );
}

// Remark: SX1276 must be fully initialized before calling this function
uint16_t SX1278FskGetPacketPayloadSize( void )
{
    uint16_t syncSize;
    uint16_t variableSize;
    uint16_t addressSize;
    uint16_t payloadSize;
    uint16_t crcSize;

    syncSize = ( SX1276->RegSyncConfig & 0x07 ) + 1;
    variableSize = ( ( SX1276->RegPacketConfig1 & 0x80 ) == 0x80 ) ? 1 : 0;
    addressSize = ( ( SX1276->RegPacketConfig1 & 0x06 ) != 0x00 ) ? 1 : 0;
    payloadSize = SX1276->RegPayloadLength;
    crcSize = ( ( SX1276->RegPacketConfig1 & 0x10 ) == 0x10 ) ? 2 : 0;
    
    return syncSize + variableSize + addressSize + payloadSize + crcSize;
}

// Remark: SX1276 must be fully initialized before calling this function
uint16_t SX1278FskGetPacketHeaderSize( void )
{
    uint16_t preambleSize;
    uint16_t syncSize;

    preambleSize = ( ( uint16_t )SX1276->RegPreambleMsb << 8 ) | ( uint16_t )SX1276->RegPreambleLsb;
    syncSize = ( SX1276->RegSyncConfig & 0x07 ) + 1;
    
    return preambleSize + syncSize;
}

uint8_t SX1278FskGetRFState( void )
{
    return RFState;
}

void SX1278FskSetRFState( uint8_t state )
{
    RFState = state;
}
#endif
