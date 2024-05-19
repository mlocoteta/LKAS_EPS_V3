// Credit goes to reddn for which this is based off of 
// https://github.com/reddn/LINInterfaceV2
#include "common.h"
#include <SWOStream.h>
#include <IWatchdog.h>

HardwareSerial EPStoLKAS_Serial(PA3,PA2); // rx, tx          
HardwareSerial LKAStoEPS_Serial(PA10,PA9);


#undef main

int main()
{
  init();
  // delay(3000);
  IWatchdog.begin(12000);                         // 12ms watchdog
  
  eXoCAN can(STD_ID_LEN, BR500K, PORTB_8_9_XCVR); // Create CAN object 500k
	msgFrm canMsg;								                	// Create CAN message
	can.begin(STD_ID_LEN, BR500K, PORTB_8_9_XCVR);  // Start CAN bus
  can.filterList16Init(0,0xe4,0,0,0);             // Filter on 0xE4 
	canMsg.busConfig = PORTB_8_9_XCVR;				      // Update bus config
  int fltIdx;                                     // Required for filter to work.
  
  setupGPIO();                                    // Sets up LEDs
  	
  EPStoLKAS_Serial.begin(9600,SERIAL_8E1); 		// Setup EPS to LKAS UART
  LKAStoEPS_Serial.begin(9600,SERIAL_8E1); 		// Setup LKAS to EPS UART
  
  static struct Status globalStatus;          // Most global data is contained here
  static bool tx_can = 0;                     

  while (1)
  {
    updateStatus(&globalStatus);                          // Update status and determine if we can transmit requested torque
    LKAStoEPS(&globalStatus, LKAStoEPS_Serial);           // Captures and sends data from the camera to the power steering
    tx_can = EPStoLKAS(&globalStatus, EPStoLKAS_Serial);  // Captures and sends data from the power steering to the camera

    if(tx_can) {
      txCanData(&globalStatus);                           // Transmit CAN data once we have full messages from the EPS
    }

    if(can.receive(canMsg.txMsgID, fltIdx, canMsg.txMsg.bytes) > -1){      
	    handleLkasFromCan(canMsg, &globalStatus);           // Recieve CAN data for steering torque requestss
    }

  }
}

// Updates error states, LED state and determines if we can transmit requested data to the EPS
void updateStatus(struct Status *status) {
  IWatchdog.reload();
  bool tx = 1;

  if (status->error.invalidCounterCount > 2){
    tx = 0;
  }

  if (status->error.invalidChecksumCount > 2){
    tx = 0;
  }

  if (timeSince(&status->timers.steeringControlTime_LKAS, 50, false)){
    status->error.lateMsg = 1;
    tx = 0;
  } else {
    status->error.lateMsg = 0;
  }

  if (timeSince(&status->timers.steeringControlTime_EPS, 50, false)){
    status->error.lateMsg = 1;
    tx = 0;
  } else {
    status->error.lateMsg = 0;
  }

  if (timeSince(&status->timers.can100HzTimer, 50, false)){
    status->error.lateCanMsg = 1;
  } else {
    status->error.lateCanMsg = 0;
  }

// LED is ON if being manipulatd, otherwise blinks every ~10s for 20ms to indicate it's fine
  if (status->can.lkasRequestLED){
    digitalWrite(statusLED, HIGH);
    status->can.lkasRequestLastState = true;
  } else if (status->can.lkasRequestLastState && !status->can.lkasRequestLED) {
    status->can.lkasRequestLastState = false;
    digitalWrite(statusLED, LOW);
  }else if (timeSince(&status->timers.ledTimer, 10000, true)){
    digitalWrite(statusLED, LOW);
  } else if (timeSince(&status->timers.ledTimer, 10000-20, false)){
    digitalWrite(statusLED, HIGH);
  }

  if(timeSince(&status->timers.versionTimer, 100, true)){
    status->can.txFirmwareVersion = 1;
  }

  digitalWrite(errorLED, (!status->lkasAllowed & status->found0xE4));
  status->lkasAllowed = tx;
}

// Returns if the timer is expired (true/false). Also has self-reset function
bool timeSince(uint32_t *timer, uint32_t compareTime, bool autoReset)
{
  bool flag = ((millis() - *timer) > compareTime);
  
  if (flag && autoReset) {
    *timer = millis();
  }
  return flag;
}

