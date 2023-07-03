#ifndef IOUDP_H
#define IOUDP_H

#include "IOInterface/IOUDP/msgUdpToSerial.h"
#include "UnitreeArmModule/IOPort/UdpPort.h"
#include "IOInterface/IOInterface.h"

class IOUDP : public IOInterface
{
public:
  IOUDP(const char* toIP, uint ownPort);
  ~IOUDP(){};

  bool isConnected() override;
  bool init() override;
  bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) override;
  bool calibration() override;
private:
  void send(const LowLevelCmd *cmd);
  void recvV1(LowLevelState *state);
  void recvV2(LowLevelState *state);

  UdpPortPtr _udp;
  UDPSendCmd _cmd{};
  UDPRecvStateV2 _stateV2{};
  UDPRecvStateV1 _stateV1{};
  size_t _motorNum;
  size_t _jointNum;
  
  uint8_t _selfCheck[10];
  bool _checkCommunication;

  uint protocol_version;
};

#endif  // IOUDP_H