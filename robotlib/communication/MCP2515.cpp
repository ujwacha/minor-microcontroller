/*
		(c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
		software and any derivatives exclusively with Microchip products.

		THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
		EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
		WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
		PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
		WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

		IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
		INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
		WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
		BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
		FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
		ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
		THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

		MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
		TERMS.
*/

#include "MCP2515.hpp"

// public functions

MCP2515::MCP2515() : hspi(nullptr) {};

/* Entering Sleep Mode */
void MCP2515::sleep(void)
{
	/* Clear CAN bus wakeup interrupt */
	MCP2515_BitModify(MCP2515_CANINTF, 0x40, 0x00);

	/* Enable CAN bus activity wakeup */
	MCP2515_BitModify(MCP2515_CANINTE, 0x40, 0x40);

	MCP2515_SetSleepMode();
}

/* Initialize CAN */
bool MCP2515::init(void)
{
	RXF0 RXF0reg;
	RXF1 RXF1reg;
	RXF2 RXF2reg;
	RXF3 RXF3reg;
	RXF4 RXF4reg;
	RXF5 RXF5reg;
	RXM0 RXM0reg;
	RXM1 RXM1reg;

	/* Intialize Rx Mask values */
	RXM0reg.RXM0SIDH = 0x00;
	RXM0reg.RXM0SIDL = 0x00;
	RXM0reg.RXM0EID8 = 0x00;
	RXM0reg.RXM0EID0 = 0x00;

	RXM1reg.RXM1SIDH = 0x00;
	RXM1reg.RXM1SIDL = 0x00;
	RXM1reg.RXM1EID8 = 0x00;
	RXM1reg.RXM1EID0 = 0x00;

	/* Intialize Rx Filter values */
	RXF0reg.RXF0SIDH = 0x00;
	RXF0reg.RXF0SIDL = 0x00; // Starndard Filter
	RXF0reg.RXF0EID8 = 0x00;
	RXF0reg.RXF0EID0 = 0x00;

	RXF1reg.RXF1SIDH = 0x00;
	RXF1reg.RXF1SIDL = 0x08; // Exntended Filter
	RXF1reg.RXF1EID8 = 0x00;
	RXF1reg.RXF1EID0 = 0x00;

	RXF2reg.RXF2SIDH = 0x00;
	RXF2reg.RXF2SIDL = 0x00;
	RXF2reg.RXF2EID8 = 0x00;
	RXF2reg.RXF2EID0 = 0x00;

	RXF3reg.RXF3SIDH = 0x00;
	RXF3reg.RXF3SIDL = 0x00;
	RXF3reg.RXF3EID8 = 0x00;
	RXF3reg.RXF3EID0 = 0x00;

	RXF4reg.RXF4SIDH = 0x00;
	RXF4reg.RXF4SIDL = 0x00;
	RXF4reg.RXF4EID8 = 0x00;
	RXF4reg.RXF4EID0 = 0x00;

	RXF5reg.RXF5SIDH = 0x00;
	RXF5reg.RXF5SIDL = 0x08;
	RXF5reg.RXF5EID8 = 0x00;
	RXF5reg.RXF5EID0 = 0x00;

	/* Intialize MCP2515, check SPI */
	if (!MCP2515_Initialize())
	{
		return false;
	}

	/* Change mode as configuration mode */
	if (!MCP2515_SetConfigMode())
	{
		return false;
	}

	/* Configure filter & mask */
	MCP2515_WriteByteSequence(MCP2515_RXM0SIDH, MCP2515_RXM0EID0, &(RXM0reg.RXM0SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXM1SIDH, MCP2515_RXM1EID0, &(RXM1reg.RXM1SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF0SIDH, MCP2515_RXF0EID0, &(RXF0reg.RXF0SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF1SIDH, MCP2515_RXF1EID0, &(RXF1reg.RXF1SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF2SIDH, MCP2515_RXF2EID0, &(RXF2reg.RXF2SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF3SIDH, MCP2515_RXF3EID0, &(RXF3reg.RXF3SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF4SIDH, MCP2515_RXF4EID0, &(RXF4reg.RXF4SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF5SIDH, MCP2515_RXF5EID0, &(RXF5reg.RXF5SIDH));

	/* Accept All (Standard + Extended) */
	MCP2515_WriteByte(MCP2515_RXB0CTRL, 0x04); // Enable BUKT, Accept Filter 0
	MCP2515_WriteByte(MCP2515_RXB1CTRL, 0x01); // Accept Filter 1

	/*
	 * tq = 2 * (brp(0) + 1) / 16000000 = 0.125us
	 * tbit = (SYNC_SEG(1 fixed) + PROP_SEG + PS1 + PS2)
	 * tbit = 1tq + 5tq + 6tq + 4tq = 16tq
	 * 16tq = 2us = 500kbps
	 */

	/* 00(SJW 1tq) 000000 */
	MCP2515_WriteByte(MCP2515_CNF1, 0x00);

	/* 1 1 100(5tq) 101(6tq) */
	MCP2515_WriteByte(MCP2515_CNF2, 0x80);

	/* 1 0 000 011(4tq) */
	MCP2515_WriteByte(MCP2515_CNF3, 0x80);

	MCP2515_WriteByte(MCP2515_CANINTE, 0x03);

	/* Normal 모드로 설정 */
	if (!MCP2515_SetNormalMode())
		return false;

	return true;
}

uint8_t MCP2515::transmit(uCAN_MSG *tempCanMsg)
{
	uint8_t returnValue = 0;

	idReg.tempSIDH = 0;
	idReg.tempSIDL = 0;
	idReg.tempEID8 = 0;
	idReg.tempEID0 = 0;

	ctrlStatus.ctrl_status = MCP2515_ReadStatus();

	/* Finding empty buffer */
	if (ctrlStatus.TXB0REQ != 1)
	{
		/* convert CAN ID for register */
		convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);

		/* Load data to Tx Buffer */
		MCP2515_LoadTxSequence(MCP2515_LOAD_TXB0SIDH, &(idReg.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));

		/* Request to transmit */
		MCP2515_RequestToSend(MCP2515_RTS_TX0);

		returnValue = 1;
	}
	else if (ctrlStatus.TXB1REQ != 1)
	{
		convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);

		MCP2515_LoadTxSequence(MCP2515_LOAD_TXB1SIDH, &(idReg.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
		MCP2515_RequestToSend(MCP2515_RTS_TX1);

		returnValue = 1;
	}
	else if (ctrlStatus.TXB2REQ != 1)
	{
		convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);

		MCP2515_LoadTxSequence(MCP2515_LOAD_TXB2SIDH, &(idReg.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
		MCP2515_RequestToSend(MCP2515_RTS_TX2);

		returnValue = 1;
	}
	if (returnValue == 1)
		tx_count++;
	return (returnValue);
}

uint8_t MCP2515::receive(uCAN_MSG *tempCanMsg)
{
	uint8_t returnValue = 0;
	rx_reg_t rxReg;
	ctrl_rx_status_t rxStatus;

	rxStatus.ctrl_rx_status = MCP2515_GetRxStatus();

	/* Check receive buffer */
	if (rxStatus.rxBuffer != 0)
	{
		/* finding buffer which has a message */
		if ((rxStatus.rxBuffer == MSG_IN_RXB0) | (rxStatus.rxBuffer == MSG_IN_BOTH_BUFFERS))
		{
			MCP2515_ReadRxSequence(MCP2515_READ_RXB0SIDH, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));
		}
		else if (rxStatus.rxBuffer == MSG_IN_RXB1)
		{
			MCP2515_ReadRxSequence(MCP2515_READ_RXB1SIDH, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));
		}

		/* if the message is extended CAN type */
		if (rxStatus.msgType == dEXTENDED_CAN_MSG_ID_2_0B)
		{
			tempCanMsg->frame.idType = (uint8_t)dEXTENDED_CAN_MSG_ID_2_0B;
			tempCanMsg->frame.id = convertReg2ExtendedCANid(rxReg.RXBnEID8, rxReg.RXBnEID0, rxReg.RXBnSIDH, rxReg.RXBnSIDL);
		}
		else
		{
			/* Standard type */
			tempCanMsg->frame.idType = (uint8_t)dSTANDARD_CAN_MSG_ID_2_0B;
			tempCanMsg->frame.id = convertReg2StandardCANid(rxReg.RXBnSIDH, rxReg.RXBnSIDL);
		}

		tempCanMsg->frame.dlc = rxReg.RXBnDLC;
		tempCanMsg->frame.data0 = rxReg.RXBnD0;
		tempCanMsg->frame.data1 = rxReg.RXBnD1;
		tempCanMsg->frame.data2 = rxReg.RXBnD2;
		tempCanMsg->frame.data3 = rxReg.RXBnD3;
		tempCanMsg->frame.data4 = rxReg.RXBnD4;
		tempCanMsg->frame.data5 = rxReg.RXBnD5;
		tempCanMsg->frame.data6 = rxReg.RXBnD6;
		tempCanMsg->frame.data7 = rxReg.RXBnD7;

		returnValue = 1;
	}
	if (returnValue == 1)
		rx_count++;

	return (returnValue);
}

uint8_t MCP2515::messages_in_rx_buffer(void)
{
	uint8_t messageCount = 0;

	ctrlStatus.ctrl_status = MCP2515_ReadStatus();

	if (ctrlStatus.RX0IF != 0)
	{
		messageCount++;
	}

	if (ctrlStatus.RX1IF != 0)
	{
		messageCount++;
	}

	return (messageCount);
}

uint8_t MCP2515::messages_in_tx_buffer(void)
{
	uint8_t messageCount = 0;

	ctrlStatus.ctrl_status = MCP2515_ReadStatus();

	if (ctrlStatus.TX0IF != 0)
	{
		messageCount++;
	}

	if (ctrlStatus.TX1IF != 0)
	{
		messageCount++;
	}

	if (ctrlStatus.TX2IF != 0)
	{
		messageCount++;
	}

	return (messageCount);
}

uint8_t MCP2515::is_buss_off(void)
{
	uint8_t returnValue = 0;

	errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

	if (errorStatus.TXBO == 1)
	{
		returnValue = 1;
	}

	return (returnValue);
}

uint8_t MCP2515::is_rx_error_passive(void)
{
	uint8_t returnValue = 0;

	errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

	if (errorStatus.RXEP == 1)
	{
		returnValue = 1;
	}

	return (returnValue);
}

uint8_t MCP2515::is_tx_error_passive(void)
{
	uint8_t returnValue = 0;

	errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

	if (errorStatus.TXEP == 1)
	{
		returnValue = 1;
	}

	return (returnValue);
}

uint8_t MCP2515::get_tec(void)
{
	uint8_t to_ret;
	to_ret = MCP2515_ReadByte(MCP2515_TEC);
	return to_ret;
}

uint8_t MCP2515::get_rec(void)
{
	uint8_t to_ret;
	to_ret = MCP2515_ReadByte(MCP2515_REC);
	return to_ret;
}

void MCP2515::update_ctrl_status()
{
	/* Read the status of the controller */
	ctrlStatus.ctrl_status = MCP2515_ReadStatus();

	/* Update the error status */
	errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);
}

/**
 * Below are all the helper fucntions for the MCP2515 module
 */

/* initialize MCP2515 */
bool MCP2515::MCP2515_Initialize(void)
{
	MCP2515_CS_HIGH();

	uint8_t loop = 10;

	do
	{
		/* check SPI Ready */
		if (HAL_SPI_GetState(hspi) == HAL_SPI_STATE_READY)
			return true;

		loop--;
	} while (loop > 0);

	return false;
}

/* change mode as configuration mode */
bool MCP2515::MCP2515_SetConfigMode(void)
{
	/* configure CANCTRL Register */
	MCP2515_WriteByte(MCP2515_CANCTRL, 0x80);

	uint8_t loop = 10;

	do
	{
		/* confirm mode configuration */
		if ((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x80)
			return true;

		loop--;
	} while (loop > 0);

	return false;
}

/* change mode as normal mode */
bool MCP2515::MCP2515_SetNormalMode(void)
{
	/* configure CANCTRL Register */
	MCP2515_WriteByte(MCP2515_CANCTRL, 0x00);

	uint8_t loop = 10;

	do
	{
		/* confirm mode configuration */
		if ((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x00)
			return true;

		loop--;
	} while (loop > 0);

	return false;
}

/* Entering sleep mode */
bool MCP2515::MCP2515_SetSleepMode(void)
{
	/* configure CANCTRL Register */
	MCP2515_WriteByte(MCP2515_CANCTRL, 0x20);

	uint8_t loop = 10;

	do
	{
		/* confirm mode configuration */
		if ((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x20)
			return true;

		loop--;
	} while (loop > 0);

	return false;
}

/* MCP2515 SPI-Reset */
void MCP2515::MCP2515_Reset(void)
{
	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_RESET);

	MCP2515_CS_HIGH();
}

/* read single byte */
uint8_t MCP2515::MCP2515_ReadByte(uint8_t address)
{
	uint8_t retVal;

	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_READ);
	SPI_Tx(address);
	retVal = SPI_Rx();

	MCP2515_CS_HIGH();

	return retVal;
}

/* read buffer */
void MCP2515::MCP2515_ReadRxSequence(uint8_t instruction, uint8_t *data, uint8_t length)
{
	MCP2515_CS_LOW();

	SPI_Tx(instruction);
	SPI_RxBuffer(data, length);

	MCP2515_CS_HIGH();
}

/* write single byte */
void MCP2515::MCP2515_WriteByte(uint8_t address, uint8_t data)
{
	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_WRITE);
	SPI_Tx(address);
	SPI_Tx(data);

	MCP2515_CS_HIGH();
}

/* write buffer */
void MCP2515::MCP2515_WriteByteSequence(uint8_t startAddress, uint8_t endAddress, uint8_t *data)
{
	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_WRITE);
	SPI_Tx(startAddress);
	SPI_TxBuffer(data, (endAddress - startAddress + 1));

	MCP2515_CS_HIGH();
}

/* write to TxBuffer */
void MCP2515::MCP2515_LoadTxSequence(uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data)
{
	MCP2515_CS_LOW();

	SPI_Tx(instruction);
	SPI_TxBuffer(idReg, 4);
	SPI_Tx(dlc);
	uint8_t tempdata[8];
	memcpy(tempdata, data, 8);
	for (int i = 0; i < dlc; i++)
		SPI_Tx(tempdata[i]);

	MCP2515_CS_HIGH();
}

/* write to TxBuffer(1 byte) */
void MCP2515::MCP2515_LoadTxBuffer(uint8_t instruction, uint8_t data)
{
	MCP2515_CS_LOW();

	SPI_Tx(instruction);
	SPI_Tx(data);

	MCP2515_CS_HIGH();
}

/* request to send */
void MCP2515::MCP2515_RequestToSend(uint8_t instruction)
{
	MCP2515_CS_LOW();

	SPI_Tx(instruction);

	MCP2515_CS_HIGH();
}

/* read status */
uint8_t MCP2515::MCP2515_ReadStatus(void)
{
	uint8_t retVal;

	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_READ_STATUS);
	retVal = SPI_Rx();

	MCP2515_CS_HIGH();

	return retVal;
}

/* read RX STATUS register */
uint8_t MCP2515::MCP2515_GetRxStatus(void)
{
	uint8_t retVal;

	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_RX_STATUS);
	retVal = SPI_Rx();

	MCP2515_CS_HIGH();

	return retVal;
}

/* Use when changing register value */
void MCP2515::MCP2515_BitModify(uint8_t address, uint8_t mask, uint8_t data)
{
	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_BIT_MOD);
	SPI_Tx(address);
	SPI_Tx(mask);
	SPI_Tx(data);

	MCP2515_CS_HIGH();
}

/* SPI Tx wrapper function  */
inline void MCP2515::SPI_Tx(uint8_t data)
{
	HAL_SPI_Transmit_DMA(hspi, &data, 1);
}

/* SPI Tx wrapper function */
inline void MCP2515::SPI_TxBuffer(uint8_t *buffer, uint8_t length)
{
	HAL_SPI_Transmit_DMA(hspi, buffer, length);
}

/* SPI Rx wrapper function */
inline uint8_t MCP2515::SPI_Rx(void)
{
	uint8_t retVal;
	HAL_SPI_Receive_DMA(hspi, &retVal, 1);
	return retVal;
}

/* SPI Rx wrapper function */
inline void MCP2515::SPI_RxBuffer(uint8_t *buffer, uint8_t length)
{
	uint8_t recv[length];
	for (int i = 0; i < length; i++)
		HAL_SPI_Receive_DMA(hspi, &(recv[i]), 1);
	memcpy(buffer, &recv, length);
}

/* convert register value to extended CAN ID */
uint32_t MCP2515::convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL)
{
	uint32_t returnValue = 0;
	uint32_t ConvertedID = 0;
	uint8_t CAN_standardLo_ID_lo2bits;
	uint8_t CAN_standardLo_ID_hi3bits;

	CAN_standardLo_ID_lo2bits = (tempRXBn_SIDL & 0x03);
	CAN_standardLo_ID_hi3bits = (tempRXBn_SIDL >> 5);
	ConvertedID = (tempRXBn_SIDH << 3);
	ConvertedID = ConvertedID + CAN_standardLo_ID_hi3bits;
	ConvertedID = (ConvertedID << 2);
	ConvertedID = ConvertedID + CAN_standardLo_ID_lo2bits;
	ConvertedID = (ConvertedID << 8);
	ConvertedID = ConvertedID + tempRXBn_EIDH;
	ConvertedID = (ConvertedID << 8);
	ConvertedID = ConvertedID + tempRXBn_EIDL;
	returnValue = ConvertedID;
	return (returnValue);
}

/* convert register value to standard CAN ID */
uint32_t MCP2515::convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL)
{
	uint32_t returnValue = 0;
	uint32_t ConvertedID;

	ConvertedID = (tempRXBn_SIDH << 3);
	ConvertedID = ConvertedID + (tempRXBn_SIDL >> 5);
	returnValue = ConvertedID;

	return (returnValue);
}

/* convert CAN ID to register value */
void MCP2515::convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t *passedIdReg)
{
	uint8_t wipSIDL = 0;

	if (canIdType == dEXTENDED_CAN_MSG_ID_2_0B)
	{
		// EID0
		passedIdReg->tempEID0 = 0xFF & tempPassedInID;
		tempPassedInID = tempPassedInID >> 8;

		// EID8
		passedIdReg->tempEID8 = 0xFF & tempPassedInID;
		tempPassedInID = tempPassedInID >> 8;

		// SIDL
		wipSIDL = 0x03 & tempPassedInID;
		tempPassedInID = tempPassedInID << 3;
		wipSIDL = (0xE0 & tempPassedInID) + wipSIDL;
		wipSIDL = wipSIDL + 0x08;
		passedIdReg->tempSIDL = 0xEB & wipSIDL;

		// SIDH
		tempPassedInID = tempPassedInID >> 8;
		passedIdReg->tempSIDH = 0xFF & tempPassedInID;
	}
	else
	{
		passedIdReg->tempEID8 = 0;
		passedIdReg->tempEID0 = 0;
		tempPassedInID = tempPassedInID << 5;
		passedIdReg->tempSIDL = 0xFF & tempPassedInID;
		tempPassedInID = tempPassedInID >> 8;
		passedIdReg->tempSIDH = 0xFF & tempPassedInID;
	}
}
