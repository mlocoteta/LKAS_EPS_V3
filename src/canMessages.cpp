#ifndef BUILDCANMESSAGESH
#define BUILDCANMESSAGESH

#include "version.h"
#include "common.h"
#include "eXoCAN.h"

void txCanData(struct Status *status){

    txMotorTorque(427, 3, status);
    txSteerStatus(400, 3, status);

	if(status->can.txFirmwareVersion){
		txFirmwareVersion(520, 2, status);
		status->can.txFirmwareVersion = 0;
	}

	#ifdef RAW_LIN_DATA
    	txRawLKASData(523, 6, status);
    	txRawEPSData(524, 7, status);
	#endif

	status->counter++;
    if(status->counter > 3){
        status->counter = 0;
    }

}


void txFirmwareVersion(int id, int len, struct Status *status){ 
    CAN_msg_t msg;
    msg.id = id;
    msg.len = len;
	
	msg.buf[0]  =  VERSION_NUM;
	msg.buf[0] |=  VERSION_HW 						  << 4;
	msg.buf[1]  =  status->error.lateMsg			  << 0;
	msg.buf[1] |=  status->error.lateCanMsg 		  << 1;
	msg.buf[1] |=  status->error.invalidCounterCount  << 2;
	msg.buf[1] |=  status->error.invalidChecksumCount << 3;
	msg.buf[1] |= !status->lkasAllowed                << 4;
    
	sendCanMsg(&msg);
}

void txMotorTorque(int id, int len, struct Status *status){ 
    CAN_msg_t msg;
    msg.id = id;
    msg.len = len;


    msg.buf[1] =  ( status -> epsData[2] << 4 ) & B10000000; // push the last bit of the Big motor torque(3 bits) on the MSB (7th bit) of the first byte of the 10 bit signal
    msg.buf[1] |=   status -> epsData[3] & B01111111; // move the Small Motor Torque (7bits) into the rest of the first byte (bits 0-6)
    msg.buf[0] =  ( status -> epsData[2] >> 4 ) & B00000011; // move the 2 MSB of Big Steer (3 bit) into the LSB of the 2nd byte (bits 0 and 1) containing the 2 MSB of the signal 

    // msg.buf[0] |= ( status -> lkasData[0] >> 2 ) & B00000100; //LKAS B0 O4  into CAN B1 O2 // Don't think we care about this for now...
    // msg.buf[0] |= ( status -> epsData[1]  & B00100000); // EPS B1 O5 (EPS_LKAS_ON aka LKAS_ON_FROM_EPS)

    msg.buf[2] =  ( status -> counter << 4 ); 					// put in the counter
    msg.buf[2] |=   status -> epsData[0] & B01000000; 			//EPS  B0 O6  into CAN B2 O6
    msg.buf[2] |= ( status -> epsData[1] << 1 ) & B10000000; 	//EPS  B1 O6  into CAN B2 O7  //dont know if this does anything,keeping

    msg.buf[2] |= honda_compute_checksum(&msg.buf[0],3,(unsigned int)msg.id);
  
    sendCanMsg(&msg);
}

void txSteerStatus(int id, int len, struct Status *status){ //TODO: add to decclaration
	
    CAN_msg_t msg;
    msg.id = id;
    msg.len = len;
	
	msg.buf[0] = status -> driverAppliedSteer & B11111111;			// Break apart driver Torque
	msg.buf[1] = status -> driverAppliedSteer >> 8;

	msg.buf[1] |= !status->lkasAllowed << 1; 		        		// Record LKAS not allowed to separate out from B1 O5
	msg.buf[1] |= status->error.lateMsg << 2; 		        		// CAN B1 O2

	msg.buf[1] |= (status->epsData[0] << 3) & B10000000;    		//EPS B0 O4 into CAN B2 O3
	msg.buf[1] |= (status->epsData[2] << 4)	& B01110000;    		//EPS B2 O0-2 into CAN B2 O0-2
	msg.buf[1] |= (!status->lkasAllowed << 5) & B00010000;			//If LKAS isn't allowed for some reason, force temporary STEER_STATUS error

	msg.buf[2]  = (status->counter << 4 );				        // put in the counter
	msg.buf[2] |= honda_compute_checksum(&msg.buf[0], msg.len, (unsigned int) msg.id);
	msg.buf[2] |= (status -> steerBlended << 7);

	sendCanMsg(&msg);
}

void txRawLKASData(int id, int len, struct Status *status){
	
    CAN_msg_t msg;
    msg.id = id;
    msg.len = len;

	msg.buf[0] =  status -> lkasData[0];
	msg.buf[1] =  status -> lkasData[1];
	msg.buf[2] =  status -> lkasData[2];
	msg.buf[3] =  status -> lkasData[3];
	msg.buf[4] =  status -> lkasData[4];

	msg.buf[5]  = (status->counter << 4 ); // put in the counter
	msg.buf[5] |= honda_compute_checksum(&msg.buf[0],msg.len,(unsigned int) msg.id);

	sendCanMsg(&msg);
}

void txRawEPSData(int id, int len, struct Status *status){
	
    CAN_msg_t msg;
    msg.id = id;
    msg.len = len;

	msg.buf[0] =  status -> epsData[0];
	msg.buf[1] =  status -> epsData[1];
	msg.buf[2] =  status -> epsData[2];
	msg.buf[3] =  status -> epsData[3];
	msg.buf[4] =  status -> epsData[4];
	msg.buf[5] =  status -> epsData[5];

	msg.buf[6] = (status->counter << 4 ); 
	msg.buf[6] |= honda_compute_checksum(&msg.buf[0],msg.len,(unsigned int) msg.id);

	sendCanMsg(&msg);
}

void handleLkasFromCan(msgFrm canMsg, struct Status *status){

	if(canMsg.txMsgID != 228){
		return;
	}
	
	status->found0xE4 = 1; 											// Allow error state LED

	uint8_t steerMSB  = ( canMsg.txMsg.bytes[0] >> 4 ) & B00001000;
	steerMSB |= ( canMsg.txMsg.bytes[1] >> 5 ) & B00000111;
	uint8_t steerLSB = canMsg.txMsg.bytes[1] & B00011111;

	// Check counter validity
	uint8_t rxCanCounter= (canMsg.txMsg.bytes[4] >> 4) & B00000011;
	bool counterValid = verifyCounter(rxCanCounter, status->can.opCounter);
	if(!counterValid){
		status->error.invalidCounterCount++;
	} else {
		status->error.invalidCounterCount = 0;
	}
	status ->can.opCounter = rxCanCounter;

	// Check checksum validity
	bool checksumValid = honda_compute_checksum((uint8_t*) &canMsg.txMsg.bytes[0],5, 228U) == (canMsg.txMsg.bytes[4] & B00001111);
	if(!checksumValid){
		status->error.invalidChecksumCount++;
	} else {
		status->error.invalidChecksumCount = 0;
	}

	// Update steering values
	if(counterValid && checksumValid){
		status->can.lkasRequest = false;
		
		if((canMsg.txMsg.bytes[2] >> 7) == 1 ){ // Check for steer request
			status->can.lkasRequest = true;
		}
		
		status->can.steerB1 = steerMSB;
		status->can.steerB0 = steerLSB;
		uint16_t apply_steer = (steerMSB & B00000111) << 5; // Allow last 3 (real data)
		apply_steer |= steerLSB;

		if((steerMSB >> 3) == 1){							// Handle sign
			apply_steer |= 0xFF00;
		}

		if(apply_steer == 0){
			status->can.lkasRequest = false;
		}
	}
	status->timers.can100HzTimer = millis();
}

uint8_t getNextOpenTxMailbox(){
	if ((CAN1->TSR&CAN_TSR_TME0) == CAN_TSR_TME0_Msk) return 0;
	if ((CAN1->TSR&CAN_TSR_TME1) == CAN_TSR_TME1_Msk) return 1;
	if ((CAN1->TSR&CAN_TSR_TME2) == CAN_TSR_TME2_Msk) return 2;
	return 255;
} 

void sendCanMsg(CAN_msg_t *CAN_tx_msg){

    uint32_t out = (CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U;
	uint8_t mailbox = getNextOpenTxMailbox();
	
    if(mailbox > 3){
        return;
    } 
	
    CAN1->sTxMailBox[mailbox].TDTR &= ~(0xF);
	CAN1->sTxMailBox[mailbox].TDTR |= CAN_tx_msg->len & 0xFUL;
	CAN1->sTxMailBox[mailbox].TDLR  = 	(((uint32_t) CAN_tx_msg->buf[3] << 24) |
										((uint32_t) CAN_tx_msg->buf[2] << 16) |
										((uint32_t) CAN_tx_msg->buf[1] <<  8) |
										((uint32_t) CAN_tx_msg->buf[0]      ));
	CAN1->sTxMailBox[mailbox].TDHR  = 	(((uint32_t) CAN_tx_msg->buf[7] << 24)|
										((uint32_t) CAN_tx_msg->buf[6] << 16) |
										((uint32_t) CAN_tx_msg->buf[5] <<  8) |
										((uint32_t) CAN_tx_msg->buf[4]      ));
	CAN1->sTxMailBox[mailbox].TIR = out | STM32_CAN_TIR_TXRQ;

	return;
}

#endif
