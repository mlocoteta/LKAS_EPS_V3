#include "common.h"

// Returns 1 if counter is valid
bool verifyCounter(uint8_t newCounter, uint8_t oldCounter){
	oldCounter = (oldCounter + 1U) & B00000011;
	if(newCounter == oldCounter){
		return 1;
	} else {
		return 0;
	}
}

uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte){
	uint8_t tot = firstByte + secondByte + thirdByte ;
	tot = 256- tot;
    tot %= 128;
    tot += 128;
	return tot;
}

// creates checksum of the EPStoLKAS frame, 4 bytes + 1 checksum byte... overloaded
uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte, uint8_t fourthByte){
	uint8_t tot = firstByte + secondByte + thirdByte + fourthByte;
	tot = 256- tot;
    tot %= 128;
    tot += 128;
	return tot;
}

uint8_t chksm(uint8_t* data, uint8_t len){
	uint8_t tot = 0;
	for(uint8_t zz =0; zz < len; zz ++) 
	{
		tot += *(data+zz);
	}
    tot = 256- tot;
    tot %= 128;
    tot += 128;
	return tot;
}

uint8_t chksm_old(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte){
	uint16_t local = firstByte + secondByte + thirdByte ;
	local = local % 512;
	local = 512 - local;
	return (uint8_t)(local % 256);
}

//code mostly taken from safety_honda.h  Credit Comma.ai MIT license on this function only
uint8_t honda_compute_checksum(uint8_t *steerTorqueAndMotorTorque, uint8_t len, unsigned int addr) {
//   int len = GET_LEN(to_push);
  uint8_t checksum = 0U;
  while (addr > 0U) {
    checksum += (addr & 0xFU); addr >>= 4;
  }
  for (int j = 0; j < len; j++) {
    uint8_t byte = *(steerTorqueAndMotorTorque + j);
    checksum += (byte & 0xFU) + (byte >> 4U);
    if (j == (len - 1)) {
      checksum -= (byte & 0xFU);  // remove checksum in message
    }
  }
  return (8U - checksum) & 0xFU;
}

// uint8_t honda_compute_checksum_CAN(CAN_message_t *themsg){
// 	return honda_compute_checksum(&(*themsg).buf[0],(*themsg).len, (unsigned int) (*themsg).id);
// }