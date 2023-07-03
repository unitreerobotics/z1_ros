#pragma once

#include <arpa/inet.h>
#include <termios.h>
#include <string>
#include <string.h>
#include <vector>
#include <boost/crc.hpp>
#include "unitree_arm_sdk/loop.h"

/* Status of the communication */
enum class IOPortStatus{
	ok,
	err,
	timeout,
	msgbad,
	crcbad
};

inline std::ostream& operator<<(std::ostream& os, const IOPortStatus& status)
{
  switch (status) {
    case IOPortStatus::ok:        os << "ok"; break;
    case IOPortStatus::err:       os << "err"; break;
    case IOPortStatus::timeout:   os << "timeout"; break;
    case IOPortStatus::msgbad:    os << "msgbad"; break;
    case IOPortStatus::crcbad:    os << "crcbad"; break;
    default:                      os << "Unknown status"; break;
  }

  return os;
}

class IOPort
{
public:
  /**
   * @brief Construct a new IOPort object.
   * 
   * @param name Indicate what the IOPort amis to.
   * @param timeout_ms Disconnect when exceeding how long after the last receiving. 
   */
  IOPort(std::string name, size_t timeout_ms)
    :name_(name), timer_((double)timeout_ms/1000.0){}

  virtual ~IOPort(){}
  virtual IOPortStatus send(uint8_t *sendMsg, size_t sendLength) = 0;
  virtual IOPortStatus recv(uint8_t *recvMsg, size_t recvLength) = 0;
  
  /**
   * @brief Receive data of indeterminate length without checksum
   * 
   * @param recvMsg 
   * @return size_t 
   */
  virtual size_t recvLen(uint8_t *recvMsg, size_t recvLength){ return 0;};
  bool isConnected(){ return isConnect_; };

  /**
   * @brief Wheterh to turn on CRC checksum
   * 
   * @param status 
   * When it's true, the last field of the communication structure is a check byte by default,
   *  to which the calculated value is assigned directly
   */
  void setCRC32(bool status) { crc32_open_ = status; }

protected:
  bool check_crc32(uint8_t* data, size_t len)
  {
    boost::crc_32_type crc32;
    crc32.process_bytes(data, len-sizeof(uint32_t));
    uint32_t recv_crc = *reinterpret_cast<uint32_t*>(&data[len - sizeof(uint32_t)]);
    return crc32.checksum() == recv_crc;
  }

  void add_crc32(uint8_t* data, size_t len)
  {
    // add crc checksun to the end of data
    boost::crc_32_type crc32;
    crc32.process_bytes(data, len-sizeof(uint32_t));
    uint32_t crc_value = crc32.checksum();
    memcpy(data + len - sizeof(uint32_t), &crc_value, sizeof(uint32_t));
  }

  std::string name_;
  
  /* Check */
  Timer timer_;
  bool crc32_open_ = false; 
  bool isConnect_ = false;
  timeval timeout_saved_;// used by the select function for non-blocking receiving.
  std::array<uint8_t, 1500> _sendBuff{};
  std::array<uint8_t, 1500> _recvBuff{};
};
typedef std::shared_ptr<IOPort> IOPortPtr;


/**
 * @brief UpdPort
 * 
 * Maintain a udp connection.
 * Allow broadcast acceptance or communication with specified ports
 */
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
