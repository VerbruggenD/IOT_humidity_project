/*
 * resource_map.h
 *
 *  Created on: 22 Nov 2022
 *      Author: Dieter
 */

#ifndef RESOURCE_MAP_H_
#define RESOURCE_MAP_H_
#include "cybsp.h"

#if defined (TARGET_CY8CPROTO_063_BLE)
    #define KIT_SPI_MASTER_MOSI               (P9_0)
    #define KIT_SPI_MASTER_MISO               (P9_1)
    #define KIT_SPI_MASTER_SCLK               (P9_2)
    #define KIT_SPI_MASTER_SS                 (P9_3)
#elif defined (TARGET_CY8CPROTO_062S3_4343W)
    #define KIT_SPI_MASTER_MOSI               (P5_0)
    #define KIT_SPI_MASTER_MISO               (P5_1)
    #define KIT_SPI_MASTER_SCLK               (P11_2)
    #define KIT_SPI_MASTER_SS                 (P11_3)
#elif defined (TARGET_CY8CKIT_062_BLE) || defined (TARGET_CYW9P62S1_43438EVB_01) || \
    defined (TARGET_CY8CKIT_062_WIFI_BT) || defined (TARGET_CY8CKIT_062S2_43012) || \
    defined (TARGET_CYW9P62S1_43012EVB_01) || defined (TARGET_CY8CKIT_064B0S2_4343W) || \
    defined (TARGET_CYSBSYSKIT_01) || defined (TARGET_CYSBSYSKIT_DEV_01) || \
    defined (TARGET_CY8CKIT_062S4) || defined (TARGET_CY8CEVAL_062S2) || \
    defined (TARGET_CY8CEVAL_062S2_LAI_4373M2)
    #define KIT_SPI_MASTER_MOSI               CYBSP_SPI_MOSI
    #define KIT_SPI_MASTER_MISO               CYBSP_SPI_MISO
    #define KIT_SPI_MASTER_SCLK               CYBSP_SPI_CLK
    #define KIT_SPI_MASTER_SS                 CYBSP_SPI_CS
#elif  defined (TARGET_APP_CY8CPROTO_062_4343W)
    #define KIT_SPI_MASTER_MOSI               (P6_0)
    #define KIT_SPI_MASTER_MISO               (P6_1)
    #define KIT_SPI_MASTER_SCLK               (P6_2)
    #define KIT_SPI_MASTER_SS                 (P6_3)
#else
    #error Unsupported kit. Define pins for SPI master.
#endif

#if defined (TARGET_APP_CY8CPROTO_062_4343W) || (TARGET_CYSBSYSKIT_01) || defined (TARGET_CYSBSYSKIT_DEV_01)
    #define KIT_SPI_SLAVE_MOSI               (P9_0)
    #define KIT_SPI_SLAVE_MISO               (P9_1)
    #define KIT_SPI_SLAVE_SCLK               (P9_2)
    #define KIT_SPI_SLAVE_SS                 (P9_3)
#elif defined (TARGET_CY8CPROTO_062S3_4343W)
    #define KIT_SPI_SLAVE_MOSI               (P5_0)
    #define KIT_SPI_SLAVE_MISO               (P5_1)
    #define KIT_SPI_SLAVE_SCLK               (P11_2)
    #define KIT_SPI_SLAVE_SS                 (P11_3)
#elif defined (TARGET_CYW9P62S1_43012EVB_01) || defined (TARGET_CY8CKIT_062S4)
    #define KIT_SPI_SLAVE_MOSI               CYBSP_SPI_MOSI
    #define KIT_SPI_SLAVE_MISO               CYBSP_SPI_MISO
    #define KIT_SPI_SLAVE_SCLK               CYBSP_SPI_CLK
    #define KIT_SPI_SLAVE_SS                 CYBSP_SPI_CS
#elif defined (TARGET_CY8CKIT_062_BLE) || defined (TARGET_CY8CKIT_062_WIFI_BT) || \
    defined (TARGET_CY8CPROTO_063_BLE) || defined (TARGET_CY8CKIT_062S2_43012) || \
    defined (TARGET_CYW9P62S1_43438EVB_01) || defined (TARGET_CY8CKIT_064B0S2_4343W) || \
    defined (TARGET_CY8CEVAL_062S2) || defined (TARGET_CY8CEVAL_062S2_LAI_4373M2)
    #define KIT_SPI_SLAVE_MOSI               (P10_0)
    #define KIT_SPI_SLAVE_MISO               (P10_1)
    #define KIT_SPI_SLAVE_SCLK               (P10_2)
    #define KIT_SPI_SLAVE_SS                 (P10_3)
#else
    #error Unsupported kit. Define pins for SPI Slave.
#endif


#endif /* SOURCE_RESOURCE_MAP_H_ */
