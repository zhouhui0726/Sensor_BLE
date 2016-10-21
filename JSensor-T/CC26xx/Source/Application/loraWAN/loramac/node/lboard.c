/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "lboard.h"

/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1                                 ( 0x1FFF7A10)
#define         ID2                                 ( 0x1FFF7A14 )
#define         ID3                                 ( 0x1FFF7A18 )

extern uint8_t lora_ieee_addr[8];

/*!
 * Initializes the unused GPIO to a know status
 */
//static void BoardUnusedIoInit( void );

/*!
 * Flag to indicate if the MCU is Initialized
 */

uint32_t BoardGetRandomSeed( void )
{
    return 0x12345678;//( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
#if 0
    id[7] = 0x03;
    id[6] = 0x00;
    id[5] = 0x00;
    id[4] = 0x00;
    id[3] = 0x00;
    id[2] = 0x00;
    id[1] = 0x00;
    id[0] = 0x00;
#else
    memcpy(id, lora_ieee_addr, 8);
#endif

}

uint8_t BoardGetBatteryLevel( void ) 
{
    uint8_t batteryLevel = 0;  

    batteryLevel = 0xFE; 
    return batteryLevel;
}
