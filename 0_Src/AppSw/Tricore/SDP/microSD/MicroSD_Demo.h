/*
 * MicroSD_Demo.h
 *
 *  Created on: 2020. 4. 14.
 *      Author: Hohyon_Choi
 */

#ifndef _MICROSD_DEMO_H_
#define _MICROSD_DEMO_H_


/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "HLD.h"
#include "microSD.h"
#include "ff.h"
#include "diskio.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------------Type Definitions------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
typedef union {
    struct{
        uint32_t reserve : 4;
        uint32_t ms : 10;
        uint32_t sec : 8;
        uint32_t min : 10;
    }__attribute__((aligned(1),packed)) time;
    uint32_t dataTime;
}recordTime_t;

extern recordTime_t recordTime;
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/

/*
 * MicroSD demo functions
 * more descriptions are in the functions
 */


IFX_EXTERN void MicroSD_Demo_initSD();
IFX_EXTERN void MicroSD_Demo_run();
IFX_EXTERN void MicroSD_Demo_stop();
IFX_EXTERN void MicroSD_Demo_addData(uint32_t ID, uint32_t data[2]);


/******************************************************************************/
/*---------------------Inline Function Implementations------------------------*/
/******************************************************************************/



#endif /* 0_SRC_APPSW_TRICORE_DEMO_MICROSD_MICROSD_DEMO_H_ */