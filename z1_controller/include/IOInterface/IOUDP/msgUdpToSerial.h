#ifndef MSG_UDP_TO_SERIAL_H
#define MSG_UDP_TO_SERIAL_H

#include <stdint.h>

#pragma pack(1)

// 20 Byte
struct JointCmd{
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
};

typedef struct{
    uint8_t reserved : 6 ;
    uint8_t state    : 2 ;//whether motor is connected; 0-ok, 1-disconnected, 2-CRC error
}Motor_ConnectedV2;

typedef struct{
    int8_t temperature;
    /* 0x01: phase current is too large
     * 0x02: phase leakage
     * 0x04: overheat(including the motor windings and the motor shell)
     * 0x20: jumped
     */
    uint8_t error;
    Motor_ConnectedV2 isConnected;
}Motor_StateV2;

struct JointStateV1{//no error state return
    float T;
    float W;
    float Acc;
    float Pos;
};

struct JointStateV2{
    float T;
    float W;
    float Acc;
    float Pos;
    Motor_StateV2 state[2];
};

//140bytes
union UDPSendCmd{
    uint8_t checkCmd;
    JointCmd jointCmd[7];
};

struct UDPRecvStateV1{
    JointStateV1 jointState[7];
};

struct UDPRecvStateV2{
    JointStateV2 jointState[7];
};

constexpr int JointCmd_LENGTH   = (sizeof(JointCmd));
constexpr int JointStateV2_LENGTH = (sizeof(JointStateV2));
constexpr int JointStateV1_LENGTH = (sizeof(JointStateV1));

#pragma pack()

#endif
