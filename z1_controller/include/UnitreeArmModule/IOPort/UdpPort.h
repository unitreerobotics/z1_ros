#pragma once

#include "UnitreeArmModule/IOPort/IOPort.h"

class UdpPort : public IOPort
{
public:
  /**
   * @brief Construct a new Udp Server object
   * 
   * @param name Mark of this communication
   * @param ownPort Local port for binding
   * @param timeout_ms Single UDP non-blocking receiving timeout, 
   * with an overall timeout default of 1s
   * @details No destination port is set, you need to receive first
   *  and start sending to that destination after the first message is received
   */
  UdpPort(std::string name, uint ownPort, size_t timeout_ms = 1000);
  
  /**
   * @brief Construct a new Udp Client object
   * 
   * @details Communicates with the assigned port
   * 
   * @param name Mark of this communication
   * @param ownPort Local port for binding
   * @param toIP the IP address of the target
   * @param toPort the target port
   * @param timeout_ms Single UDP non-blocking receiving timeout, 
   * with an overall timeout default of 1s
   */
  UdpPort(std::string name, uint ownPort, std::string toIP, uint toPort, size_t timeout_ms = 1000);
  ~UdpPort(){};

  /**
   * @brief UDP receive function. Always check length
   *        If CRC checksum is turned on, CRC checksum is forced
   */
  IOPortStatus recv(uint8_t *recvMsg, size_t recvLength) override;

  size_t recvLen(uint8_t* recvMsg, size_t recvLength) override;

  /**
   * @brief UDP send function
   * If CRC checksum is turned on, Add checksum to the end of message by default
   */
  IOPortStatus send(uint8_t *sendMsg, size_t sendLength) override;

private:
  bool isValidIPv4Address(const std::string& IP);

  /* socket setting*/
  int socket_fd_;
  sockaddr_in ownAddr_, targetAddr_, fromAddr_;
  socklen_t sockaddrSize_ = sizeof(struct sockaddr);
  int on_ = 1;
  bool isServer_{};

  std::array<uint8_t, 1500> _recvBuffTemp{};
};
typedef std::shared_ptr<UdpPort> UdpPortPtr;
