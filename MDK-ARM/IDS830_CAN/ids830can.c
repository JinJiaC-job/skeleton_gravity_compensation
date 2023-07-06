#include "ids830can.h"


#include "can.h"
#include "stdio.h"

void IDS830_can_send(uint8_t *buf,uint8_t id)
{
	uint8_t motorId = id;
	hCAN1_TxHeader.StdId = motorId;
	hCAN1_TxHeader.ExtId = 0x00;
	hCAN1_TxHeader.RTR = CAN_RTR_DATA;
	hCAN1_TxHeader.IDE = CAN_ID_STD;
	hCAN1_TxHeader.DLC = 8;
	static uint32_t txMailBox;
	HAL_CAN_AddTxMessage(&hcan, &hCAN1_TxHeader, buf, &txMailBox);
}

void writeData_pointTopoint(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 )
{
	uint8_t buf[LEN] = {group_id, 0x1A, reg1, 0x00, 0x00, reg2, 0x00, 0x00};
	buf[4] = *(uint8_t *)(& data1);
	buf[3] = *((uint8_t *)(& data1)+1);
	buf[7] = *((uint8_t *)(& data2));
	buf[6] = *((uint8_t *)(& data2)+1);
	IDS830_can_send(buf, id);
	ms_Delay(Linear_cmd_interval_time);
}

void readData_pointTopoint(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 )
{
	uint8_t buf[LEN] = {group_id, 0x2A, reg1, 0x00, 0x00, reg2, 0x00, 0x00};
	buf[4] = *(uint8_t *)(& data1);
	buf[3] = *((uint8_t *)(& data1)+1);
	buf[7] = *((uint8_t *)(& data2));
	buf[6] = *((uint8_t *)(& data2)+1);
	IDS830_can_send(buf, id);
	ms_Delay(Linear_cmd_interval_time);
}

void writeData_oneTomany(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 )
{
	uint8_t buf[LEN] = {0x00, 0x8A, reg1, 0x00, 0x00, reg2, 0x00, 0x00};
	buf[4] = *(uint8_t *)(& data1);
	buf[3] = *((uint8_t *)(& data1)+1);
	buf[7] = *((uint8_t *)(& data2));
	buf[6] = *((uint8_t *)(& data2)+1);	
	IDS830_can_send(buf, id);
	ms_Delay(Linear_cmd_interval_time);
}

void writeData_oneTomany_CorrNoRes(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 )
{
	uint8_t buf[LEN] = {0x00, 0x9A, reg1, 0x00, 0x00, reg2, 0x00, 0x00};
	buf[4] = *(uint8_t *)(& data1);
	buf[3] = *((uint8_t *)(& data1)+1);
	buf[7] = *((uint8_t *)(& data2));
	buf[6] = *((uint8_t *)(& data2)+1);		
	IDS830_can_send(buf, id);
	ms_Delay(Linear_cmd_interval_time);
}

void LinearActuator_startRun_targetSpeed(uint8_t id, uint16_t targetSpeed)
{
	writeData_pointTopoint(id, 0x00, 0x00, 0x0000, 0x06, targetSpeed);
	readData_pointTopoint(id, 0x00, 0xE8, 0x0000, 0xE9, 0x0000);
}

void LinearActuator_speedmode_runtime(uint8_t id, uint16_t runtime)
{
	writeData_pointTopoint(id, 0x00, 0x02, 0x00c4, 0x0a, runtime);
}

void LinearActuator_startRun_maxspeed_position(uint8_t id, float position, float maxspeed)
{
	position = position * 2500;
	maxspeed = maxspeed/3000*8192;
	writeData_pointTopoint(id, 0x00, 0x00, 0x0001, 0x1d, maxspeed);
	writeData_pointTopoint(id, 0x00, 0x50, ((uint32_t)position & 0xffff0000) >> 16, 0x05, (uint32_t)position & 0x0000ffff);
}

void LinearActuator_read_position(uint8_t id)
{
	uint32_t LinAcr_position = 0;
	readData_pointTopoint(id, 0x00, 0xe8, 0x0000, 0xe9, 0x0000);
	*(uint8_t *)(&LinAcr_position) = CAN_motor_data[7];
	*((uint8_t *)(&LinAcr_position)+1) = CAN_motor_data[6];
	*((uint8_t *)(&LinAcr_position)+2) = CAN_motor_data[4];
	*((uint8_t *)(&LinAcr_position)+3) = CAN_motor_data[3];
	float LinAcr_position_float = (float)LinAcr_position/2500;
//	return  LinAcr_position;
	printf("LinearActuator_position%d: %.3f mm\r\n", id, LinAcr_position_float);
}

void LinearActuator_read_CurrentandSpeed(uint8_t id)
{
	uint16_t LinAcr_current = 0;
	uint16_t LinAcr_speed = 0;
	
	readData_pointTopoint(id, 0x00, 0xe2, 0x0000, 0xe4, 0x0000);
	*(uint8_t *)(&LinAcr_current) = CAN_motor_data[4];
	*((uint8_t *)(&LinAcr_current)+1) = CAN_motor_data[3];
	float LinAcr_current_float = (float)LinAcr_current/100;
	printf("LinearActuator_current%d: %.3f mm\r\n", id, LinAcr_current_float);
	
	*(uint8_t *)(&LinAcr_speed) = CAN_motor_data[7];
	*((uint8_t *)(&LinAcr_speed)+1) = CAN_motor_data[6];
	float LinAcr_speed_float = (float)LinAcr_speed/8192*3000;
	printf("LinearActuator_speed%d: %.3f mm\r\n", id, LinAcr_speed_float);
}



