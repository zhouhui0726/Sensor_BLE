/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: End device comissioning parameters

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __LORA_COMISSIONING_H__
#define __LORA_COMISSIONING_H__

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     1

/*!
 * Indicates if the end-device is to be connected to a private or public network
 */
#define LORAWAN_PUBLIC_NETWORK                      true

#define LORAWAN_CLASSA_ENABLE                       true

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * Mote device IEEE EUI (big endian)
 *
 * \remark In this application the value is automatically generated by calling
 *         BoardGetUniqueId function
 */
//#define LORAWAN_DEVICE_EUI                          { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
#define LORAWAN_DEVICE_EUI                          { 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12 }

#if 1
/*!
 * Application IEEE EUI
 *
 * \remark must be written as a little endian value (reverse order of normal reading)
 */
#define LORAWAN_APPLICATION_EUI                     { 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01 }
/*!
* AES encryption/decryption cipher application key
*/
#define LORAWAN_APPLICATION_KEY                     { 0x22, 0x3D, 0x1F, 0x8C, 0x45, 0x25, 0xE3, 0xD2, 0x38, 0xE9, 0x48, 0x2C, 0x33, 0x6F, 0x11, 0xA2 }
#else
#define LORAWAN_APPLICATION_EUI                     { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
/*!
* AES encryption/decryption cipher application key
*/
#define LORAWAN_APPLICATION_KEY                     { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00 }
#endif
#else

/*!
 * Current network ID
 */
#define LORAWAN_NETWORK_ID                          ( uint32_t )0

/*!
 * Device address on the network (big endian)
 *
 * \remark In this application the value is automatically generated using
 *         a pseudo random generator seeded with a value derived from
 *         BoardUniqueId value
 */
#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0x00000000

/*!
 * AES encryption/decryption cipher network session key
 */
#define LORAWAN_NWKSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }

/*!
 * AES encryption/decryption cipher application session key
 */
#define LORAWAN_APPSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }

#endif

#endif // __LORA_COMISSIONING_H__
