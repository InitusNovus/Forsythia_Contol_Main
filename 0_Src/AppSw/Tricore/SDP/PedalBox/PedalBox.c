 /*
 * PedalBox.c
 *
 *  Created on: 2019. 10. 6.
 *      Author: Dua
 */

#ifndef PedalBox_H
#define PedalBox_H
/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "SDP.h"
#include "PedalBox.h"
#include "GtmTim.h"
#include <math.h>

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

#if PPSMODE == PWM
/*FIXME Temporary values*/
#define A0END		27.6
#define A0STT		20.7
#define A0DDZ		5

#define A1END		23.3
#define A1STT		15.8
#define A1DDZ		5

#define A2END		25.3
#define A2STT		18.3
#define A2DDZ		5

#define B0STT		78.2
#define B0END		83.7
#define B0DDZ		5

#define B1STT		42.6
#define B1END		47.6
#define B1DDZ		5

#define V0_CONST_A 		-9.2980636458f
#define V0_CONST_B		100.7315829998f
#elif PPSMODE == ADC
#define A0STT		(1.50f)
#define A0END		(3.48f)

#define A1STT		(0.93f)
#define A1END		(2.22f)
#endif

#define PBERRORLIMIT	10


#if PPSMODE == PWM
	#define	APPS0		HLD_GtmTim.ch[0].data
	#define	APPS1		HLD_GtmTim.ch[1].data
	#define	APPS2		HLD_GtmTim.ch[2].data

	#define	BPPS0		HLD_GtmTim.ch[3].data
	#define	BPPS1		HLD_GtmTim.ch[4].data

#elif PPSMODE ==ADC
	AdcSensor APPS0;
	AdcSensor APPS1;

	AdcSensor BPPS0;
	AdcSensor BPPS1;
#endif





#define ERRLIM		SDP_PedalBox_errorLimit



/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/


/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
typedef struct
{
	float32 start;
	float32 end;
	float32 percentDeadzone;
	boolean reversed;
}SDP_PedalBox_sensorConfig_config;

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
float32				SDP_PedalBox_errorLimit = PBERRORLIMIT;

SDP_PedalBox_pps_t	SDP_PedalBox_pps;
SDP_PedalBox_t		SDP_PedalBox;

/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
IFX_STATIC float32 SDP_PedalBox_getPedalLinear(SDP_PedalBox_sensor_t *sensor, HLD_GtmTim_dataPulse_t *tim);
IFX_STATIC void SDP_PedalBox_initSensorConfig(SDP_PedalBox_sensorConfig_config* config);
IFX_STATIC void SDP_PedalBox_initSensor(SDP_PedalBox_sensor_t* sensor, SDP_PedalBox_sensorConfig_config* config);

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/


/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
void SDP_PedalBox_init(void)
{
	SDP_PedalBox_sensorConfig_config config;

	#if PPSMODE == PWM 
		SDP_PedalBox_initSensorConfig(&config);
		config.start = A0STT;
		config.end = A0END;
		config.percentDeadzone = A0DDZ;
		config.reversed = TRUE;
		SDP_PedalBox_initSensor(&SDP_PedalBox_pps.apps0, &config);

		SDP_PedalBox_initSensorConfig(&config);
		config.start = A1STT;
		config.end = A1END;
		config.percentDeadzone = A1DDZ;
		config.reversed = TRUE;
		SDP_PedalBox_initSensor(&SDP_PedalBox_pps.apps1, &config);

		SDP_PedalBox_initSensorConfig(&config);
		config.start = A2STT;
		config.end = A2END;
		config.percentDeadzone = A2DDZ;
		config.reversed = TRUE;
		SDP_PedalBox_initSensor(&SDP_PedalBox_pps.apps2, &config);
		
		
		SDP_PedalBox_initSensorConfig(&config);
		config.start = B0STT;
		config.end = B0END;
		config.percentDeadzone = B0DDZ;
		config.reversed = FALSE;
		SDP_PedalBox_initSensor(&SDP_PedalBox_pps.bpps0, &config);

		SDP_PedalBox_initSensorConfig(&config);
		config.start = B1STT;
		config.end = B1END;
		config.percentDeadzone = B1DDZ;
		config.reversed = FALSE;
		SDP_PedalBox_initSensor(&SDP_PedalBox_pps.bpps1, &config);
	#elif PPSMODE == ADC
		AdcSensor_Config config_adc;
		//APPS0
		config_adc.adcConfig.lpf.config.cutOffFrequency = 10000/(2.0*IFX_PI*0.05);		//FIXME: Adjust time constant
		config_adc.adcConfig.lpf.config.gain = 1;
		config_adc.adcConfig.lpf.config.samplingTime = 0.001;
		config_adc.adcConfig.lpf.activated = TRUE;

		config_adc.adcConfig.channelIn = &HLD_Vadc_P20_6_G2CH4_AD7;
		config_adc.tfConfig.a = 100.0f / (A0END - A0STT);
		config_adc.tfConfig.b = config_adc.tfConfig.a * (-A0STT);

		config_adc.isOvervoltageProtected = TRUE;

		AdcSensor_initSensor(&APPS0, &config_adc);
		HLD_AdcForceStart(APPS0.adcChannel.channel.group);

		//APPS1
		config_adc.adcConfig.channelIn = &HLD_Vadc_P23_4_G0CH0_AD11;
		config_adc.tfConfig.a = 100.0f / (A1END - A1STT);
		config_adc.tfConfig.b = config_adc.tfConfig.a * (-A1STT);
		AdcSensor_initSensor(&APPS1, &config_adc);
		HLD_AdcForceStart(APPS1.adcChannel.channel.group);
		
		//BPPS0
		config_adc.adcConfig.lpf.activated = TRUE;
		config_adc.adcConfig.lpf.config.gain = 1;
		config_adc.adcConfig.lpf.config.cutOffFrequency = 10000/(2*IFX_PI*(1e-2f));
		config_adc.adcConfig.lpf.config.samplingTime = 10.0e-3;
		config_adc.isOvervoltageProtected = FALSE;
		config_adc.linCalConfig.isAct = FALSE;
		config_adc.tfConfig.a = 22.5;
		config_adc.tfConfig.b = -6.25;
		
		config_adc.adcConfig.channelIn = &HLD_Vadc_P23_2_G4CH4_AD3;

		AdcSensor_initSensor(&BPPS0, &config_adc);
		HLD_AdcForceStart(BPPS0.adcChannel.channel.group);
		//BPPS1		
		config_adc.adcConfig.channelIn = &HLD_Vadc_P22_2_G0CH2_AD9;

		AdcSensor_initSensor(&BPPS1, &config_adc);
		HLD_AdcForceStart(BPPS1.adcChannel.channel.group);
	#endif

}

IFX_STATIC void SDP_PedalBox_initSensorConfig(SDP_PedalBox_sensorConfig_config* config)
{
	config->start = 0;
	config->end = 100;
	config->percentDeadzone = 0;
	config->reversed = FALSE;
}


IFX_STATIC void SDP_PedalBox_initSensor(SDP_PedalBox_sensor_t* sensor, SDP_PedalBox_sensorConfig_config* config)
{
	float32 start = config->start;
	float32 end = config->end;
	float32 deadzone = config->percentDeadzone;

	float32 len = end - start;

	float32 center;
	float32 opposite;

	if(len<0)
	{
		opposite = (end+start)/2;
		center = opposite + 50;
		if(center < 0)
		{
			center = center + 100;
		}
		end = end+100;
	}
	else
	{
		center = (end+start)/2;
		opposite = center + 50;
		if(opposite < 0)
		{
			opposite = opposite + 100;
		}
	}

	float32 dzstart = start+(len*deadzone/100);
	float32 dzend = end-(len*deadzone/100);
	float32 dzlen = dzend-dzstart;


	sensor->config.start = start;
	sensor->config.end = end;
	sensor->config.deadzone = deadzone;
	sensor->config.reversed = config->reversed;
	sensor->config.center = center;
	sensor->config.opposite = opposite;
	sensor->config.len = len;
	sensor->config.dzstart = dzstart;
	sensor->config.dzend = dzend;
	sensor->config.dzlen = dzlen;

}


IFX_STATIC float32 SDP_PedalBox_getPedalLinear(SDP_PedalBox_sensor_t *sensor, HLD_GtmTim_dataPulse_t *tim)
{
/*FIXME unnecessary local variables -> to data which are initialized once at first.*/
	float32 value = tim->dutyRatio_percent;

	float32 result;


	const float32 start = sensor->config.start;
	const float32 end = sensor->config.end;
	const float32 deadzone = sensor->config.deadzone;
	float32 len = sensor->config.len;
	float32 opposite = sensor->config.opposite;
	float32 center = sensor->config.center;


	if(len <0)
	{
		len = -len;
//		end = end+100;
		if(value < opposite)
		{
			value = value + 100;
		}
	}
	else if(opposite >= 100)
	{
		opposite = opposite - 100;
		if(value < opposite)
		{
			value = value + 100;
		}
	}
	else
	{
		if(value > opposite)
		{
			value = value - 100;
		}
	}

	float32 dzstart = sensor->config.dzstart;
	float32 dzend = sensor->config.dzend;
	float32 dzlen = sensor->config.dzlen;

	if((value > dzstart)&&(value < dzend))
	{
		result = (value-dzstart)/dzlen*100;
	}
	else if(value<dzstart)
	{
		result = 0;
	}
	else
	{
		result = 100;
	}
	return result;
}




IFX_STATIC void SDP_PedalBox_updatePPS(SDP_PedalBox_sensor_t *data_out, HLD_GtmTim_dataPulse_t *data_in)
{
	data_out->pedalPercent = data_out->config.reversed
			?(float32)100.0 - SDP_PedalBox_getPedalLinear(data_out, data_in)
					:SDP_PedalBox_getPedalLinear(data_out, data_in);
}

IFX_STATIC void SDP_PedalBox_updatePPS_AN(SDP_PedalBox_sensor_t *data_out, AdcSensor *data_in)
{
	AdcSensor_getData(data_in);
	data_out->pedalPercent = data_out->config.reversed
			?(float32)100.0 - data_in->value : data_in->value;
}

IFX_STATIC void SDP_PedalBox_checkErrorState_fromTwo(SDP_PedalBox_sensor_t *data1, SDP_PedalBox_sensor_t *data2)
{
	float32 diff = (data1->pedalPercent) - (data2->pedalPercent);
	float32 absDiff = fabs(diff);

	if (absDiff>ERRLIM)
	{
		data1 -> isValueOk = FALSE;
		data2 -> isValueOk = FALSE;
	}
	else
	{
		data1 -> isValueOk = TRUE;
		data2 -> isValueOk = TRUE;
	}
}


IFX_STATIC void SDP_PedalBox_checkErrorState_fromThree(SDP_PedalBox_sensor_t *data1, SDP_PedalBox_sensor_t *data2, SDP_PedalBox_sensor_t *data3)
{
	float32 diff12 = (data1->pedalPercent) - (data2->pedalPercent);
	float32 diff23 = (data2->pedalPercent) - (data3->pedalPercent);
	float32 diff31 = (data3->pedalPercent) - (data1->pedalPercent);

	float32 absDiff12 = fabs(diff12);
	float32 absDiff23 = fabs(diff23);
	float32 absDiff31 = fabs(diff31);

	if((absDiff12 < ERRLIM ) && (absDiff23 < ERRLIM) && (absDiff31 < ERRLIM))
	{
		data1 -> isValueOk = TRUE;
		data2 -> isValueOk = TRUE;
		data3 -> isValueOk = TRUE;
	}

	else if((absDiff12 < ERRLIM ) && (absDiff23 < ERRLIM) && (absDiff31 >= ERRLIM))
	{
		data1 -> isValueOk = TRUE;
		data2 -> isValueOk = TRUE;
		data3 -> isValueOk = TRUE;
	}
	else if((absDiff12 < ERRLIM ) && (absDiff23 >= ERRLIM) && (absDiff31 < ERRLIM))
	{
		data1 -> isValueOk = TRUE;
		data2 -> isValueOk = TRUE;
		data3 -> isValueOk = TRUE;
	}
	else if((absDiff12 >= ERRLIM ) && (absDiff23 < ERRLIM) && (absDiff31 < ERRLIM))
	{
		data1 -> isValueOk = TRUE;
		data2 -> isValueOk = TRUE;
		data3 -> isValueOk = TRUE;
	}
	else if((absDiff12 >= ERRLIM ) && (absDiff23 >= ERRLIM) && (absDiff31 < ERRLIM))
	{
		data1 -> isValueOk = TRUE;
		data2 -> isValueOk = FALSE;
		data3 -> isValueOk = TRUE;
	}
	else if((absDiff12 >= ERRLIM ) && (absDiff23 < ERRLIM) && (absDiff31 >= ERRLIM))
	{
		data1 -> isValueOk = FALSE;
		data2 -> isValueOk = TRUE;
		data3 -> isValueOk = TRUE;
	}
	else if((absDiff12 < ERRLIM ) && (absDiff23 >= ERRLIM) && (absDiff31 >= ERRLIM))
	{
		data1 -> isValueOk = TRUE;
		data2 -> isValueOk = TRUE;
		data3 -> isValueOk = FALSE;
	}
	else
	{
		data1 -> isValueOk = FALSE;
		data2 -> isValueOk = FALSE;
		data3 -> isValueOk = FALSE;
	}
}


IFX_STATIC void SDP_PedalBox_updateBPPS(void)
{
	uint8 	okCount = 0;
	float32 sum = 0;

	#if PPSMODE == PWM
	SDP_PedalBox_updatePPS(&SDP_PedalBox_pps.bpps0, &BPPS0);
	SDP_PedalBox_updatePPS(&SDP_PedalBox_pps.bpps1, &BPPS1);

	SDP_PedalBox_checkErrorState_fromTwo(&SDP_PedalBox_pps.bpps0, &SDP_PedalBox_pps.bpps1);

	#elif PPSMODE == ADC

	SDP_PedalBox_updatePPS_AN(&SDP_PedalBox_pps.bpps0, &BPPS0);
	SDP_PedalBox_updatePPS_AN(&SDP_PedalBox_pps.bpps1, &BPPS1);

	SDP_PedalBox_checkErrorState_fromTwo(&SDP_PedalBox_pps.bpps0, &SDP_PedalBox_pps.bpps1);
	#endif

	if(SDP_PedalBox_pps.bpps0.isValueOk)
	{
		sum += SDP_PedalBox_pps.bpps0.pedalPercent;
		okCount++;
	}
	if(SDP_PedalBox_pps.bpps1.isValueOk)
	{
		sum += SDP_PedalBox_pps.bpps1.pedalPercent;
		okCount++;
	}

	if(okCount>=2)
	{
		SDP_PedalBox.bpps.pps = (float32)sum/(float32)okCount;
		SDP_PedalBox.bpps.isValueOk = TRUE;
	}
	else
	{
		SDP_PedalBox.bpps.pps = 0;
		SDP_PedalBox.bpps.isValueOk = FALSE;
	}
}


IFX_STATIC void SDP_PedalBox_updateAPPS(void)
{
	uint8 	okCount = 0;
	float32 sum = 0;

	#if PPSMODE == PWM
	SDP_PedalBox_updatePPS(&SDP_PedalBox_pps.apps0, &APPS0);
	SDP_PedalBox_updatePPS(&SDP_PedalBox_pps.apps1, &APPS1);
	SDP_PedalBox_updatePPS(&SDP_PedalBox_pps.apps2, &APPS2);

	SDP_PedalBox_checkErrorState_fromThree(&SDP_PedalBox_pps.apps0, &SDP_PedalBox_pps.apps1, &SDP_PedalBox_pps.apps2);

	#elif PPSMODE == ADC

	SDP_PedalBox_updatePPS_AN(&SDP_PedalBox_pps.apps0, &APPS0);
	SDP_PedalBox_updatePPS_AN(&SDP_PedalBox_pps.apps1, &APPS1);

	SDP_PedalBox_checkErrorState_fromTwo(&SDP_PedalBox_pps.apps0, &SDP_PedalBox_pps.apps1);
	#endif



	if(SDP_PedalBox_pps.apps0.isValueOk)
	{
		sum += SDP_PedalBox_pps.apps0.pedalPercent;
		okCount++;
	}
	if(SDP_PedalBox_pps.apps1.isValueOk)
	{
		sum += SDP_PedalBox_pps.apps1.pedalPercent;
		okCount++;
	}
	if(SDP_PedalBox_pps.apps2.isValueOk)
	{
		sum += SDP_PedalBox_pps.apps2.pedalPercent;
		okCount++;
	}


	if(okCount>=2)
	{
		SDP_PedalBox.apps.pps = (float32)sum/(float32)okCount;
		SDP_PedalBox.apps.isValueOk = TRUE;
	}
	else
	{
		SDP_PedalBox.apps.pps = 0;
		SDP_PedalBox.apps.isValueOk = FALSE;
	}

	if(SDP_PedalBox.apps.pps < 0)
	{
		SDP_PedalBox.apps.pps = 0.0f;
	}
	else if(SDP_PedalBox.apps.pps > 100)
	{
		SDP_PedalBox.apps.pps = 100.0f;
	}
}


void SDP_PedalBox_run_1ms(void)
{
	SDP_PedalBox_updateBPPS();
	SDP_PedalBox_updateAPPS();
}


#endif