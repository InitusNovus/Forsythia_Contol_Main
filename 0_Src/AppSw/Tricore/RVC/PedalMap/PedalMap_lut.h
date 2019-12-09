/*
 * PedalMap_lut.h
 *
 *  Created on: 2019. 5. 19.
 *      Author: mains
 */

#ifndef _PEDALMAP_LUT_H_
#define _PEDALMAP_LUT_H_


/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "HLD.h"
/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define SIZEOF_PMLUT	5
/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/
typedef enum{
	RH26_PedalMap_mode0,
	RH26_PedalMap_mode1,
	RH26_PedalMap_mode2,
	RH26_PedalMap_mode3,
	RH26_PedalMap_mode4
}RH26_PedalMap_lut_mode;
/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
typedef struct{
	float32		data[SIZEOF_PMLUT+1];
}RH26_PedalMap_lut_t;
/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
IFX_EXTERN RH26_PedalMap_lut_t		RH26_PedalMap_lut;
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
IFX_EXTERN void RH26_PedalMap_lut_setMode(RH26_PedalMap_lut_mode num);

IFX_INLINE void RH26_PedalMap_lut_setData(uint32 index, float32 data){
	if((index > 0)&&(index < SIZEOF_PMLUT)){
		RH26_PedalMap_lut.data[index] = data;
	}
}

IFX_EXTERN float32 RH26_PedalMap_lut_getResult(float32 input);


#endif /* 0_SRC_APPSW_TRICORE_RH26_PEDALMAP_PEDALMAP_LUT_H_ */