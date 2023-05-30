/*
 * Cascadia_Inverter_can.h
 *
 * Created: Jan 8th 2023
 * Author: Terry
 * yoonsb@hanayng.ac.kr
 * blog.terrysyoon.com
 *
 * */

#include "Cascadia_Inverter_can.h"
#include "HLD.h"

PM100_ID_set Inverter_L_ID;
PM100_ID_set Inverter_R_ID;

CanCommunication_Message Rx_Inverter_L[8];
CanCommunication_Message Rx_Inverter_R[8];

CanCommunication_Message Tx_TC275_L[2];
CanCommunication_Message Tx_TC275_R[2];

PM100_Status_t Inverter_L_Status;
PM100_Status_t Inverter_R_Status;

PM100_Control_t Inverter_L_Control;
PM100_Control_t Inverter_R_Control;

PM100_RWParameter_t Inverter_L_RWParameter;
PM100_RWParameter_t Inverter_R_RWParameter;

void CascadiaInverter_SET_ID(PM100_ID_set* IN, int node);
void CascadiaInverter_can_init(void);
void CascadiaInverter_can_Run(void);

static void setReceiveMessage(PM100_ID_set* ID, CanCommunication_Message* Rm);
static void setTransmitMessage(PM100_ID_set* ID, CanCommunication_Message* Tm);
static void setInitialControl(PM100_Control_t* Control);
static void CascadiaInverter_enable();
//static void CascadiaInverter_disable();

void CascadiaInverter_SET_ID(PM100_ID_set* IN, int node)
{

	IN->node = node;

	/*Rx~*/
	IN->ID_PM100_Temperature  = 0x0A2;
	IN->ID_PM100_Position  = 0x0A5;
	IN->ID_PM100_Current  = 0x0A6;
	IN->ID_PM100_Voltage  = 0x0A7;
	IN->ID_PM100_InternalStates = 0x0AA;
	IN->ID_PM100_FaultCodes  = 0x0AB;
	IN->ID_PM100_HighSpeedMessage  = 0x0B0;
	/*~Rx*/

	/*Tx~*/
	IN->ID_PM100_Command  = 0x0C0;
	/*~Tx*/

	/*Parameter Messages~*/
	IN->ID_PM100_RWParameterCommand = 0x0C1;
	IN->ID_PM100_RWParameterResponse = 0x0C2;
	/*~Parameter Messages*/
}

void CascadiaInverter_can_init(void)
{
	CascadiaInverter_SET_ID(&Inverter_L_ID,2);
	CascadiaInverter_SET_ID(&Inverter_R_ID,1);

	setTransmitMessage(&Inverter_L_ID, Tx_TC275_L);
	setTransmitMessage(&Inverter_R_ID, Tx_TC275_R);

	setReceiveMessage(&Inverter_L_ID, Rx_Inverter_L);
	setReceiveMessage(&Inverter_R_ID, Rx_Inverter_R);

	setInitialControl(&Inverter_L_Control);
	setInitialControl(&Inverter_R_Control);
}

void CascadiaInverter_can_Run(void) //receive loop
{
	if(CanCommunication_receiveMessage(&Rx_Inverter_L[0]))
	{
		Inverter_L_Status.Temperature3.ReceivedData[0] = Rx_Inverter_L[0].msg.data[0];
		Inverter_L_Status.Temperature3.ReceivedData[1] = Rx_Inverter_L[0].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_L[1]))
	{
		Inverter_L_Status.Position.ReceivedData[0] = Rx_Inverter_L[1].msg.data[0];
		Inverter_L_Status.Position.ReceivedData[1] = Rx_Inverter_L[1].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_L[2]))
	{
		Inverter_L_Status.Current.ReceivedData[0] = Rx_Inverter_L[2].msg.data[0];
		Inverter_L_Status.Current.ReceivedData[1] = Rx_Inverter_L[2].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_L[3]))
	{
		Inverter_L_Status.Voltage.ReceivedData[0] = Rx_Inverter_L[3].msg.data[0];
		Inverter_L_Status.Voltage.ReceivedData[1] = Rx_Inverter_L[3].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_L[4]))
	{
		Inverter_L_Status.InternalStates.ReceivedData[0] = Rx_Inverter_L[4].msg.data[0];
		Inverter_L_Status.InternalStates.ReceivedData[1] = Rx_Inverter_L[4].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_L[5]))
	{
		Inverter_L_Status.FaultCodes.ReceivedData[0] = Rx_Inverter_L[5].msg.data[0];
		Inverter_L_Status.FaultCodes.ReceivedData[1] = Rx_Inverter_L[5].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_L[6]))
	{
		Inverter_L_Status.HighSpeedMessage.ReceivedData[0] = Rx_Inverter_L[6].msg.data[0];
		Inverter_L_Status.HighSpeedMessage.ReceivedData[1] = Rx_Inverter_L[6].msg.data[1];
	}





	if(CanCommunication_receiveMessage(&Rx_Inverter_R[0]))
	{
		Inverter_R_Status.Temperature3.ReceivedData[0] = Rx_Inverter_R[0].msg.data[0];
		Inverter_R_Status.Temperature3.ReceivedData[1] = Rx_Inverter_R[0].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_R[1]))
	{
		Inverter_R_Status.Position.ReceivedData[0] = Rx_Inverter_R[1].msg.data[0];
		Inverter_R_Status.Position.ReceivedData[1] = Rx_Inverter_R[1].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_R[2]))
	{
		Inverter_R_Status.Current.ReceivedData[0] = Rx_Inverter_R[2].msg.data[0];
		Inverter_R_Status.Current.ReceivedData[1] = Rx_Inverter_R[2].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_R[3]))
	{
		Inverter_R_Status.Voltage.ReceivedData[0] = Rx_Inverter_R[3].msg.data[0];
		Inverter_R_Status.Voltage.ReceivedData[1] = Rx_Inverter_R[3].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_R[4]))
	{
		Inverter_R_Status.InternalStates.ReceivedData[0] = Rx_Inverter_R[4].msg.data[0];
		Inverter_R_Status.InternalStates.ReceivedData[1] = Rx_Inverter_R[4].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_R[5]))
	{
		Inverter_R_Status.FaultCodes.ReceivedData[0] = Rx_Inverter_R[5].msg.data[0];
		Inverter_R_Status.FaultCodes.ReceivedData[1] = Rx_Inverter_R[5].msg.data[1];
	}
	if(CanCommunication_receiveMessage(&Rx_Inverter_R[6]))
	{
		Inverter_R_Status.HighSpeedMessage.ReceivedData[0] = Rx_Inverter_R[6].msg.data[0];
		Inverter_R_Status.HighSpeedMessage.ReceivedData[1] = Rx_Inverter_R[6].msg.data[1];
	}

	Tx_TC275_L[0].msg.data[0] = Inverter_L_Control.Command.TransmitData[0];
	Tx_TC275_L[0].msg.data[1] = Inverter_L_Control.Command.TransmitData[1];

	Tx_TC275_R[0].msg.data[0] = Inverter_R_Control.Command.TransmitData[0];
	Tx_TC275_R[0].msg.data[1] = Inverter_R_Control.Command.TransmitData[1];



	CanCommunication_transmitMessage(&Tx_TC275_L[0]);
	CanCommunication_transmitMessage(&Tx_TC275_R[0]);
}



static void setReceiveMessage(PM100_ID_set* ID, CanCommunication_Message* Rm)
{
	/*Config in common~*/
	IfxMultican_Can_Node* CanCommunication_canNodes[3] = {&CanCommunication_canNode0, &CanCommunication_canNode1, &CanCommunication_canNode2};
	CanCommunication_Message_Config config_Message_Receive;

	config_Message_Receive.frameType = IfxMultican_Frame_receive;
	config_Message_Receive.dataLen = IfxMultican_DataLengthCode_8;
	config_Message_Receive.isStandardId = FALSE;
	config_Message_Receive.node = CanCommunication_canNodes[ID->node];
	/*~Config in common*/

	config_Message_Receive.messageId = ID->ID_PM100_Temperature;
	CanCommunication_initMessage(&Rm[0], &config_Message_Receive);

	config_Message_Receive.messageId = ID->ID_PM100_Position;
	CanCommunication_initMessage(&Rm[1], &config_Message_Receive);

	config_Message_Receive.messageId = ID->ID_PM100_Current;
	CanCommunication_initMessage(&Rm[2], &config_Message_Receive);

	config_Message_Receive.messageId = ID->ID_PM100_Voltage;
	CanCommunication_initMessage(&Rm[3], &config_Message_Receive);

	config_Message_Receive.messageId = ID->ID_PM100_InternalStates;
	CanCommunication_initMessage(&Rm[4], &config_Message_Receive);

	config_Message_Receive.messageId = ID->ID_PM100_FaultCodes;
	CanCommunication_initMessage(&Rm[5], &config_Message_Receive);

	config_Message_Receive.messageId = ID->ID_PM100_HighSpeedMessage;
	CanCommunication_initMessage(&Rm[6], &config_Message_Receive);

	config_Message_Receive.messageId = ID->ID_PM100_RWParameterResponse;
	CanCommunication_initMessage(&Rm[7], &config_Message_Receive);
}

static void setTransmitMessage(PM100_ID_set* ID, CanCommunication_Message* Tm)
{
	/*Config in common~*/
	IfxMultican_Can_Node* CanCommunication_canNodes[3] = {&CanCommunication_canNode0, &CanCommunication_canNode1, &CanCommunication_canNode2};
	CanCommunication_Message_Config config_Message_Transmit;

	config_Message_Transmit.frameType = IfxMultican_Frame_transmit;
	config_Message_Transmit.dataLen = IfxMultican_DataLengthCode_8;
	config_Message_Transmit.isStandardId = FALSE;
	config_Message_Transmit.node = CanCommunication_canNodes[ID->node];
	/*~Config in common*/


	config_Message_Transmit.messageId = ID->ID_PM100_Command;
	CanCommunication_initMessage(&Tm[0], &config_Message_Transmit);

	config_Message_Transmit.messageId = ID->ID_PM100_RWParameterCommand;
	CanCommunication_initMessage(&Tm[1], &config_Message_Transmit);
}

static void setInitialControl(PM100_Control_t* Control)
{
	Control->Command.S.PM100_TorqueCommand = 0;

	//Debug
	//Control->Command.S.PM100_TorqueCommand_1 = 0;
	//Control->Command.S.PM100_TorqueCommand_2 = 0;
	//~Debug 02172022
	Control->Command.S.PM100_SpeedCommand = 0;
	Control->Command.S.PM100_DirectionCommand = 1;
	Control->Command.S.PM100_InverterEnable = 0;
	Control->Command.S.PM100_InverterDischarge = 0;
	Control->Command.S.PM100_SpeedModeEnable = 0;
	Control->Command.S.PM100_CommandedTorqueLimit = 0;
	//Control->Command.S.reservedBits = 0;
}

static void setInitialParameterMessage(PM100_RWParameter_t* Parameter) 
{
	Parameter->ParameterCommand.TransmitData[0] = 0;
	Parameter->ParameterCommand.TransmitData[1] = 0;
	Parameter->ParameterResponse.ReceivedData[0] = 0;
	Parameter->ParameterResponse.ReceivedData[1] = 0;
	Parameter->sentTick = 0;
	Parameter->receivedTick = 0;
	Parameter->RTT = 0;
}

//FIXME: Do this in the init step, not the main loop.
static void CascadiaInverter_enable()
{
	if(Inverter_L_Status.InternalStates.InverterEnableLockout || Inverter_R_Status.InternalStates.InverterEnableLockout) //If either of two interters cannot be enabled.
	{
		Inverter_L_Control.Command.S.PM100_InverterEnable = 0;
		Inverter_R_Control.Command.S.PM100_InverterEnable = 0;
	}
	else{
		Inverter_L_Control.Command.S.PM100_InverterEnable = 1;
		Inverter_R_Control.Command.S.PM100_InverterEnable = 1;
	}
}

void CascadiaInverter_disable() {
	Inverter_L_Control.Command.S.PM100_TorqueCommand = 0;
	Inverter_R_Control.Command.S.PM100_TorqueCommand = 0;
	Inverter_L_Control.Command.S.PM100_InverterEnable = 0;
	Inverter_R_Control.Command.S.PM100_InverterEnable = 0;
}

void CascadiaInverter_writeTorque(uint16 torque_L, uint16 torque_R){
	CascadiaInverter_enable();
	Inverter_L_Control.Command.S.PM100_TorqueCommand = torque_L;
	Inverter_R_Control.Command.S.PM100_TorqueCommand = torque_R;
}

