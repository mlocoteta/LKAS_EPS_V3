#ifndef BUILDCANMESSAGESH
#define BUILDCANMESSAGESH

#include "version.h"
#include "common.h"
#include "eXoCAN.h"
#include "io.h"

void txCanData(struct Status *status){

    txMotorTorque(427, 3, status);
    txSteerStatus(400, 5, status);



	#ifdef RAW_LIN_DATA // Only 1 message can be added, otherwise mailbox is full
    	txRawLKASData(523, 6, status);
    	txRawEPSData(524, 7, status);
    	txRawTorqueBlend(525, 7, status);
	#endif

	if(status->can.txAllSerial){
		txAllSerialData(521, 8, status);

	}

	status->counter_100hz++;
    if(status->counter_100hz > 3){
        status->counter_100hz = 0;
    }

	if(status->can.txFirmwareVersion){
		txFirmwareVersion(520, 3, status);
		status->can.txFirmwareVersion = 0;
		status->counter_10hz++;
    	if(status->counter_10hz > 3){
            status->counter_10hz = 0;
        }
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
    msg.buf[2]  = (status->counter_10hz << 4 );				        // put in the counter
	msg.buf[2] |= honda_compute_checksum(&msg.buf[0], msg.len, (unsigned int) msg.id);
	sendCanMsg(&msg);
}

void txMotorTorque(int id, int len, struct Status *status){ 
    CAN_msg_t msg;
    msg.id = id;
    msg.len = len;

	// Comments are from old SSV2 DBC
    // msg.buf[1] =  ( status -> epsData[2] << 4 ) & B10000000; // push the last bit of the Big motor torque(3 bits) on the MSB (7th bit) of the first byte of the 10 bit signal
    // msg.buf[1] |=   status -> epsData[3] & B01111111; // move the Small Motor Torque (7bits) into the rest of the first byte (bits 0-6)
    // msg.buf[0] =  ( status -> epsData[2] >> 4 ) & B00000011; // move the 2 MSB of Big Steer (3 bit) into the LSB of the 2nd byte (bits 0 and 1) containing the 2 MSB of the signal 
    // msg.buf[0] |= ( status -> lkasData[0] >> 2 ) & B00000100; //LKAS B0 O4  into CAN B1 O2 // Don't think we care about this for now...
    // msg.buf[0] |= ( status -> epsData[1]  & B00100000); // EPS B1 O5 (EPS_LKAS_ON aka LKAS_ON_FROM_EPS)

	
	uint16_t motorTorque = (status -> epsData[2] & B00110000) << 4;
	motorTorque |= (status -> epsData[2] & B00001000) << 4;
	motorTorque |= (status -> epsData[3] & B01111111);
	
	if (motorTorque & 0x0200){
		motorTorque = ~(abs(motorTorque )) & 0x01FF;	// Convert to positive torque
	}
	
	msg.buf[0]  = motorTorque >> 8;        // Upper byte
	msg.buf[0] |= B10000000;			   // Configuration valid
	msg.buf[1]  = motorTorque & B11111111; // Lower byte of motor torque

    msg.buf[2] =  ( status -> counter_100hz << 4 ); 					// put in the counter
    msg.buf[2] |=   status -> epsData[0] & B01000000; 			//EPS  B0 O6  into CAN B2 O6
    msg.buf[2] |= ( status -> epsData[1] << 1 ) & B10000000; 	//EPS  B1 O6  into CAN B2 O7  //dont know if this does anything,keeping

    msg.buf[2] |= honda_compute_checksum(&msg.buf[0],3,(unsigned int)msg.id);
  
    sendCanMsg(&msg);
}

void txSteerStatus(int id, int len, struct Status *status){ //TODO: add to decclaration
	
    CAN_msg_t msg;
    msg.id = id;
    msg.len = len;
	
	msg.buf[0] = status->driverAppliedSteer & 0xFF;				// Break apart driver Torque
	msg.buf[1] = status->driverAppliedSteer >> 8;

	msg.buf[1] |= !status->lkasAllowed << 1; 		        		// Record LKAS not allowed to separate out from B1 O5
	msg.buf[1] |= status->error.lateMsg << 2; 		        		// CAN B1 O2

	msg.buf[1] |= (status->epsData[0] << 3) & B10000000;    		//EPS B0 O4 into CAN B2 O3
	msg.buf[1] |= (status->epsData[2] << 4)	& B01110000;    		//EPS B2 O0-2 into CAN B2 O0-2
	msg.buf[1] |= (!status->lkasAllowed << 5) & B00010000;			//If LKAS isn't allowed for some reason, force temporary STEER_STATUS error

	msg.buf[2]  = status->steerTorqueLast >> 8;			    		// 
	msg.buf[3]  = status->steerTorqueLast & 0xFF;			    		

	msg.buf[4]  = (status->counter_100hz << 4 );				        // put in the counter
	msg.buf[4] |= honda_compute_checksum(&msg.buf[0], msg.len, (unsigned int) msg.id);
	msg.buf[4] |= (status -> steerBlended << 7);

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

	msg.buf[5]  = (status->counter_100hz << 4 ); // put in the counter
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

	msg.buf[6] = (status->counter_100hz << 4 ); 
	msg.buf[6] |= honda_compute_checksum(&msg.buf[0],msg.len,(unsigned int) msg.id);

	sendCanMsg(&msg);
}

void txAllSerialData(int id, int len, struct Status *status){
	
    CAN_msg_t msg;
    msg.id = id;
    msg.len = len;

	msg.buf[0] =  status -> lkasData[0];
	msg.buf[1] =  status -> lkasData[1];
	msg.buf[2] =  status -> lkasData[2];
	msg.buf[3] =  status -> epsData[0];
	msg.buf[4] =  status -> epsData[1];
	msg.buf[5] =  status -> epsData[2];
	msg.buf[6] =  status -> epsData[3];

	msg.buf[7] = (status->counter_100hz << 4 ); 
	msg.buf[7] |= honda_compute_checksum(&msg.buf[0],msg.len,(unsigned int) msg.id);

	sendCanMsg(&msg);
}

// Diagnostic for torque blending
void txRawTorqueBlend(int id, int len, struct Status *status){
	
    CAN_msg_t msg;
    msg.id = id;
    msg.len = len;

	msg.buf[0] =  status -> steerTorqueLast >> 8;
	msg.buf[1] =  status -> steerTorqueLast & 0xFF;
	msg.buf[2] = 0;
	msg.buf[3] = 0;
	msg.buf[4] =  status -> driverAppliedSteer >> 8;
	msg.buf[5] =  status -> driverAppliedSteer & 0xFF;

	msg.buf[6] = (status->counter_100hz << 4 ); 
	msg.buf[6] |= honda_compute_checksum(&msg.buf[0],msg.len,(unsigned int) msg.id);

	sendCanMsg(&msg);
}


void handleLkasFromCan(msgFrm canMsg, struct Status *status){

	if(canMsg.txMsgID != 228){
		return;
	}
	
	status->found0xE4 = 1; 										// Allow error state LED
	// Start Blending 
	uint8_t steer_b0 = canMsg.txMsg.bytes[0];					// Upper word
	uint8_t steer_b1 = canMsg.txMsg.bytes[1];					// Lower word
	int16_t steerTorqueBlended;

	int16_t steerTorque = steer_b0 << 8 | steer_b1;				// Upper + Lower words  --> Total steer Torque
	status->steerTorqueIn = steerTorque;						// Upper + Lower words  --> Total steer Torque
	if (!status->can.torqueBlendDisable){
	steerTorqueBlended = torque_blend(steerTorque, status->steerTorqueLast, status->driverAppliedSteer);
	} else {
	steerTorqueBlended = steerTorque;
	}
	status->steerTorqueLast = steerTorqueBlended;

    uint8_t steerMSB_blend = steerTorqueBlended >> 8;			// Break back into upper/ lower
	uint8_t steerLSB_blend = steerTorqueBlended & 0xFF;
	// End blending

	uint8_t steerMSB  = ( steerMSB_blend >> 4 ) & B00001000; 	// Process for LKAS manipulation
			steerMSB |= ( steerLSB_blend >> 5 ) & B00000111;
	uint8_t steerLSB  =   steerLSB_blend & B00011111;        


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
		status->can.lkasRequestLED = false;
		
		if((canMsg.txMsg.bytes[2] >> 7) == 1 ){ // Check for steer request
			status->can.lkasRequestLED = true;
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
			status->can.lkasRequest = false;				// If we're blending we might still have a "LKAS REQUEST" which will error EPS on high driver torque
		}
	}

	// Update other from STEERING_CONTROL
	if (canMsg.txMsg.bytes[2] & 0x01){
		status->can.torqueBlendDisable = 1;
	} else {
		status->can.torqueBlendDisable = 0;
	}
	
	if (canMsg.txMsg.bytes[2] & 0x02){
		status->can.txAllSerial = 1;
	} else {
		status->can.txAllSerial = 0;
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

// Torque blend is WIP but we'd like to remove blending driver torque and requested torque on-device
int16_t torque_blend(int16_t applyTorque, int16_t applyTorqueLast, int16_t driverTorque){ // Source is from Openpilot

  int32_t apply_torque      = applyTorque;
  int32_t apply_torque_last = applyTorqueLast;		
  int8_t driver_torque     = driverTorque >> 1; 	// Don't care about LSB. Easier to process
  
  // limits due to driver torque
  int32_t driver_max_torque = steerMax + (steerDriverAllowance + driver_torque * steerDriverFactor) * steerDriverMultiplier;
  int32_t driver_min_torque = -steerMax + (-steerDriverAllowance + driver_torque * steerDriverFactor) * steerDriverMultiplier;
  int32_t max_steer_allowed = MAX(MIN(steerMax, driver_max_torque), 0);
  int32_t min_steer_allowed = MIN(MAX(-steerMax, driver_min_torque), 0);

  // slow rate if steer torque increases in magnitude
  if (apply_torque_last > 0){
    apply_torque = clip(apply_torque, MAX(apply_torque_last - steerDeltaDown, -steerDeltaUp),
                        apply_torque_last + steerDeltaUp);
  } else {
    apply_torque = clip(apply_torque, apply_torque_last - steerDeltaUp,
                        MIN(apply_torque_last + steerDeltaDown, steerDeltaUp));
  }
  int32_t min_torque = MIN(abs(max_steer_allowed), abs(min_steer_allowed));
  max_steer_allowed = min_torque;
  min_steer_allowed = -min_torque;
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed);
  int16_t final_torque = apply_torque;
  return final_torque;
}


// Math function required for torque blending
int16_t clip(int16_t x, int16_t lo, int16_t hi){
  return MAX(lo, MIN(hi, x));
}

#endif
