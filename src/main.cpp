#include "common.h"
#include <SWOStream.h>

HardwareSerial EPStoLKAS_Serial(PA3,PA2); // rx, tx          
HardwareSerial LKAStoEPS_Serial(PA10,PA9);



#undef main

int main()
{
  init();

  eXoCAN can(STD_ID_LEN, BR500K, PORTB_8_9_XCVR); // Setup CAN at 500k
	msgFrm canMsg;								                	// Create CAN message
	can.begin(STD_ID_LEN, BR500K, PORTB_8_9_XCVR);
  can.filterList16Init(0,0xe4,0,0,0);
	canMsg.busConfig = PORTB_8_9_XCVR;				      // Update bus config
  int fltIdx;
  
  setupGPIO();
  	
  EPStoLKAS_Serial.begin(9600,SERIAL_8E1); 		// Setup EPS to LKAS UART
  LKAStoEPS_Serial.begin(9600,SERIAL_8E1); 		// Setup LKAS to EPS UART
  
  static struct Status globalStatus;
  globalStatus.counter = 0;
  static bool tx_can = 0;

  while (1)
  {
    updateStatus(&globalStatus);
    LKAStoEPS(&globalStatus, LKAStoEPS_Serial);
    tx_can = EPStoLKAS(&globalStatus, EPStoLKAS_Serial);

    if (tx_can){
      txCanData(&globalStatus);
    }

    if(can.receive(canMsg.txMsgID, fltIdx, canMsg.txMsg.bytes) > -1){      
	    handleLkasFromCan(canMsg, &globalStatus);
    }

  }
}

void updateStatus(struct Status *status) {

  bool tx = 1;
  status -> error.inError = 0;

  if (!status->can.lkasRequest){
    tx = 0;
  }

  if (status->error.invalidCounterCount > 2){
    status -> error.inError = 1;
  }

  if (status->error.invalidChecksumCount > 2){
    status -> error.inError = 1;
  }

  if (timeSince(&status->timers.steeringControlTime_LKAS, 50, false)){
    status->error.lateMsg = 1;
    status -> error.inError = 1;
  } else {
    status->error.lateMsg = 0;
  }

  if (timeSince(&status->timers.steeringControlTime_EPS, 50, false)){
    status->error.lateMsg = 1;
    status -> error.inError = 1;
  } else {
    status->error.lateMsg = 0;
  }

  if (timeSince(&status->timers.can100HzTimer, 50, false)){
    status->error.lateCanMsg = 1;
    status -> error.inError = 1;
  } else {
    status->error.lateCanMsg = 0;
  }

  if (status->error.inError){
    tx = 0;
  }

  if (timeSince(&status->timers.ledTimer, (1000 / (tx + 1)) , true)){
    digitalWrite(statusLED, !digitalRead(statusLED));
  }

  if(timeSince(&status->timers.versionTimer, 100, true)){
    status->can.txFirmwareVersion = 1;
  }

  digitalWrite(errorLED, tx);
  status->lkasAllowed = tx;
}


bool timeSince(uint32_t *timer, uint32_t compareTime, bool autoReset)
{
  bool flag = ((millis() - *timer) > compareTime);
  
  if (flag && autoReset) {
    *timer = millis();
  }
  return flag;
}

