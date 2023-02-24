#ifndef HONDASERIAL_H
#define HONDASERIAL_H

#include <Arduino.h>

bool EPStoLKAS(struct Status *status, Stream &serial); 									  // Data to TX ready when 1
void LKAStoEPS(struct Status *status, Stream &serial);
void serialSteerToEPS(struct serialLKAS *lkasMsg, struct Status *status, Stream &serial); // TX data from CAN
void serialSteerToEPS(struct serialLKAS *lkasMsg, uint8_t *array, Stream &serial);        // TX predefined message
bool deconstructLKAS(uint8_t msg, struct serialLKAS *lkasMsg);

int16_t calculateSteerTorque(struct serialLKAS *epsMsg);
uint8_t createSteerTorqueMSB(uint16_t steertorque);
uint8_t createSteerTorqueLSB(uint16_t steertorque);

uint16_t torque_blend(uint16_t applyTorque, uint16_t applyTorqueLast, uint16_t driverTorque);


struct serialLKAS {
	bool sent;
	uint8_t totalCounter;
	uint8_t data[5];
	uint8_t counterBit;
	uint8_t steerB1;
	uint8_t steerB0;
	bool	steerLSB;
	uint8_t lkasOn;
	uint8_t checksum;
	uint8_t ldw_enable;
};

#endif