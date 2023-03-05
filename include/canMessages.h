#ifndef CANMESSAGES_H
#define CANMESSAGES_H
#include <Arduino.h>

#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK                 0x1FFFFFFFU
#define CAN_STD_ID_MASK                 0x000007FFU

typedef struct
{
  uint32_t id;             // 29 bit identifier                        
  uint8_t  buf[8] = {0};   // Data field
  uint8_t  len;            // Length of data field in bytes
  uint8_t  ch;             // Object channel(Not use)
  uint8_t  format = 0;     // 0 - STANDARD, 1- EXTENDED IDENTIFIER
  uint8_t  type = 0;       // 0 - DATA FRAME, 1 - REMOTE FRAME
} CAN_msg_t;

void handleLkasFromCan(msgFrm, struct Status *status);
void txCanData(struct Status *status);
void txMotorTorque(int id, int len, struct Status *status);
void txSteerStatus(int id, int len, struct Status *status);
void txRawLKASData(int id, int len, struct Status *status);
void txRawEPSData(int id, int len, struct Status *status);
void txAllSerialData(int id, int len, struct Status *status);
void txFirmwareVersion(int id, int len, struct Status *status);
int16_t torque_blend(int16_t applyTorque, int16_t applyTorqueLast, int16_t driverTorque);
void txRawTorqueBlend(int id, int len, struct Status *status);

uint8_t getNextOpenTxMailbox();
void sendCanMsg(CAN_msg_t *CAN_tx_msg);

#endif