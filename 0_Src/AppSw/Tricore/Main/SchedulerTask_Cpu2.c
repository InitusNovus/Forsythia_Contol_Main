/*
 * SchedulerTask_Cpu2.c
 *
 *  Created on: 2019. 10. 30.
 *      Author: Dua
 */


/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "SchedulerTask_Cpu2.h"
#include "AmkInverter_can.h"
#include "OrionBms2.h"
#include "SteeringWheel.h"
#include "AdcSensor.h"
#include "PedalBox.h"
/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
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
Task_cpu2 Task_core2 =
{
	.flag = FALSE,
};
uint64 stm_buf_c2 = 0;
uint64 stm_buf_c2_delay = 0;
uint64 ticToc_1ms_c2 = 0;
uint64 delay_1ms_c2 = 0;
uint16_t task2_10ms_counter = 0;

extern AdcSensor APPS0;
uint16 tempPedal=0;
boolean RTD_flag = 0;
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/


/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/


/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
void Task_core2_primaryService(void)
{
	
}

void Task_core2_1ms(void)
{
	stm_buf_c2_delay = IfxStm_get(&MODULE_STM0);

	delay_1ms_c2 = (IfxStm_get(&MODULE_STM0) - stm_buf_c2_delay) * 1000000 / (IfxStm_getFrequency(&MODULE_STM0));
	stm_buf_c2 = IfxStm_get(&MODULE_STM0);
	AmkInverter_can_Run();
	// AccumulatorManager_master_run_1ms_c2();
	// kelly8080ips_can_run_1ms_c2();

	// OrionBms2_run_1ms_c2();
	// SteeringWheel_run_xms_c2();
	task2_10ms_counter+=1;
	writeMessage(tempPedal,tempPedal);
	writeMessage2(tempPedal,tempPedal);
	// if (task2_10ms_counter == 10){
		Task_core2_10ms_slot1();
		// task2_10ms_counter = 0;
	// }
	// else if (task2_10ms_counter ==15)
	if (RTD_flag)AmkInverterStart();
	

	ticToc_1ms_c2 = (IfxStm_get(&MODULE_STM0) - stm_buf_c2) * 1000000 / (IfxStm_getFrequency(&MODULE_STM0));
	
}

void Task_core2_10ms_slot1(void)
{
	/*
	FIXME:
	tempPedal to target torque value
	target torque value unit : 0.1 % of nominal torque 9.8Nm
	1000-> 100% of nominal torque
	2143 -> 214.3% of nominal torque 21Nm
	maby 21Nm is max by datasheet graph
	*/

}

void Task_core2_10ms_slot2(void){

}

void Task_core2_backgroundService(void)
{

}