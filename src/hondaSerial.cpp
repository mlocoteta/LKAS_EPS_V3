#include "common.h"
#include "hondaSerial.h"

// Handles serial data from the camera to the power steering.
// There are three outputs to the power steering: 
// 1. Passthrough   	  --> This is the default output
// 2. "Fixed" array 	  --> Device which can transmit is detected but not requesting torque.
// 3. Manipulated message --> Transmits a requested torque from another device.
void LKAStoEPS(struct Status *status, Stream &serial){
    static struct serialLKAS lkasMsg;

    if(!serial.available()){             			 	// Exit if nothing availble
        return;
	}
	status->timers.steeringControlTime_LKAS = millis(); // Reset timer
    
	uint8_t rx = serial.read();           			 	// Get byte of LKAStoEPS 
    bool dataReady = deconstructLKAS(rx, &lkasMsg);  	// Update structure 

	if(dataReady && status->lkasAllowed){
        if(status->can.lkasRequest){
			serialSteerToEPS(&lkasMsg, status, serial); // Transmit steer message to EPS
		} else {
			uint8_t lkas_off_array[][4] = { {0x00,0x80,0xC0,0xC0}, {0x20,0x80,0xC0,0xA0} };  		  // LKAS is off array
			serialSteerToEPS(&lkasMsg, &lkas_off_array[lkasMsg.counterBit][0], serial);
			status->sendFrameDelay = 5;
		}
	} else {
        serial.write(rx);
		status->lkasData[lkasMsg.totalCounter] = rx;
	}
}

// Interprets and handles message from the camera system.
// Also helps us keep on the first byte of the 4 byte transmission
bool deconstructLKAS(uint8_t msg, struct serialLKAS *lkasMsg){
	bool dataAvailable = 0;
	
	if( (msg >> 4) < 4 ) {
		lkasMsg->totalCounter = 0;
		dataAvailable = 1;
	} else {
		lkasMsg->totalCounter++;
	}

	lkasMsg->data[lkasMsg->totalCounter] = msg;

	switch (lkasMsg->totalCounter){
		case 0:	
			lkasMsg->counterBit = msg >> 5;
			lkasMsg->steerB1 = msg & B00001111;  	    // Get MSB of steer
			break;
		case 1:
			lkasMsg-> steerB0 = msg & B00011111; 		// Get LSB of steer
			lkasMsg-> lkasOn = (msg >> 5) & B00000001;  // LKAS is active
			break;
		case 2:

			break;

		case 3:
			lkasMsg->checksum = msg;					// Get checksum
			break;
	}
	return dataAvailable;
}

// Transmits a message to the EPS based on an external torque request.
void serialSteerToEPS(struct serialLKAS *lkasMsg, struct Status *status, Stream &serial){
	uint8_t msg[4];
    
    uint8_t steerB1 = status->can.steerB1;
    uint8_t steerB0 = status->can.steerB0;						

	if(status->sendFrameDelay > 0){                             // LKAS start requires 5 cycles from off, can't TX steer data yet
        steerB0 = 0;
        steerB1 = 0;
        status->sendFrameDelay--;
		status->steerTorqueLast = 0;
	} else {
		if( (steerB1 > 0 || steerB0 > 0 )){
			steerB0 = steerB0 & B00011110;									// Preserve bits[5-1]
			steerB0 = steerB0 | ( (lkasMsg->steerLSB ^ 1 ) & B00000001);    // Last bit needs "wiggle". Without this the EPS will error out.
		}
	}

	msg[0] = (lkasMsg->counterBit << 5) | steerB1;              //  Byte 1 of steer (big endian)
	msg[1] = steerB0 | 0xA0;  						            //  Byte 0 of steer. 0xA0 = 1010 0000 & steerB0 = LKAS ON
	msg[2] = 0x80; 											    //  0x80 = 1000 0000 --> LKAS ON | 0xC0 = 1100 0000 --> LKAS OFF
	msg[2] |= lkasMsg -> ldw_enable;							//  When LDW--> & with B00110000. Needs more exploring
	msg[3] = chksm(msg[0], msg[1], msg[2]);

	for(int i=0; i <4;i++){										// Save entire message
		serial.write(msg[i]);
		lkasMsg->data[i] = msg[i]; 								// Save tx msg data
	}
	lkasMsg->steerLSB = steerB0 & B00000001;					// Save last bit for "wiggle" noted above
}

// Transmits a message to the EPS if there is no torque request.
void serialSteerToEPS(struct serialLKAS *lkasMsg, uint8_t *array, Stream &serial){ // Write predefined message
	for(int i=0; i<4; i++){
		serial.write(*(array+i));						
		lkasMsg->data[i] = *(array+i); 							// Save tx msg data
	}
}

// Message from the power steering to the camera.
// This contains torque information such as the driver torque applied to the wheel.
bool EPStoLKAS(struct Status *status, Stream &serial){
    static struct serialLKAS epsMsg;

    if(!serial.available()){       								// Exit if nothing availble
        return 0;
    }
	
	status->timers.steeringControlTime_EPS = millis(); 			// Reset timer

    uint8_t rx = serial.read();	     

    if((rx >> 4) < 4 ){					                       // Sync with first frame which has offset 4
        epsMsg.totalCounter = 0;
    }

    epsMsg.data[epsMsg.totalCounter] = rx;
    
	serial.write(rx);             							  //Forward data we just recieved
        
    epsMsg.totalCounter++;
    if(epsMsg.totalCounter < 5) {
        return 0;            
    } 
	   
    for(int i=0; i<5; i++){
		status->epsData[i] = epsMsg.data[i];				 // Save tx msg data
    }

    status->driverAppliedSteer = calculateSteerTorque(&epsMsg);  
    
	epsMsg.totalCounter = 0;
    return 1;
}

// Decodes the data from the EPS to generate one 16 bit word for driver torque.
int16_t calculateSteerTorque(struct serialLKAS *epsMsg){

    uint8_t lsb  =  (epsMsg->data[0] << 5 );					// 3 LSB of BigSteerTorque (4bit)
   			lsb |= epsMsg->data[1] & B00011111;					// all of smallSteerTorque
			lsb  = ~lsb;										// invert the whole message to make negative positive, positive negative.  OP wants left positive
	uint8_t msb  = ( ~( epsMsg->data[0] >> 3 ) )  & B00000001;  // 1st MSB of bigSteerTorque (4bit) ... added NOT (~) to invert the sign

    int16_t steerTorque = msb << 8 | lsb;						// Combine each byte
	return steerTorque;
} 
