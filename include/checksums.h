#ifndef CHECKSUMS_H 
#define CHECKSUMS_H 

#include <Arduino.h>
#include <eXoCAN.h>

bool verifyCounter(uint8_t newCounter, uint8_t oldCounter);
uint8_t honda_compute_checksum(uint8_t *steerTorqueAndMotorTorque, uint8_t size, unsigned int addr); // << From Comma.ai Panda safety_honda.h licensed unde MIT
uint8_t chksm(uint8_t , uint8_t , uint8_t );
uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte, uint8_t fourthByte);

#endif
