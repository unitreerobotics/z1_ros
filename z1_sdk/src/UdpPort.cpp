#include "unitree_arm_sdk/UdpPort.h"
#include <regex>

bool UdpPort::isValidIPv4Address(const std::string& IP)
{
  // Regular expression pattern for validating an IPv4 address
  const std::regex pattern("^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$");

  // Check if the IP matches the pattern
  return std::regex_match(IP, pattern);
}


UdpPort::UdpPort(std::string name, uint ownPort, size_t timeout_ms)
  :IOPort(name, 1000)//UDP Overall disconnection set to 1s
{
  isServer_ = true;

  /* Set single non-blocking accept timeout */
  timeout_saved_.tv_sec = timeout_ms / 1000;
  timeout_saved_.tv_usec = (timeout_ms * 1000) % 1000000;

  /* Set local address, specify port */
  bzero(&ownAddr_, sizeof(sockaddr_in));
  bzero(&targetAddr_, sizeof(sockaddr_in));
  bzero(&fromAddr_, sizeof(sockaddr_in));
  ownAddr_.sin_family = AF_INET;// IPV4
  ownAddr_.sin_port = htons(ownPort);
  ownAddr_.sin_addr.s_addr = htonl(INADDR_ANY); // Bind to all local address

  /* Create socket file descriptor */
  if(socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0), socket_fd_ < 0) {
    perror("[ERROR] UDPPort::UDPPort, create socket failed.\n");
    exit(0);
  }

  /* Allow reusing local address */
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &on_, sizeof(on_));
  if(bind(socket_fd_, (struct sockaddr*)&ownAddr_, sockaddrSize_) < 0){
    perror("[ERROR] UDPPort::UDPPort, bind failed");
    exit(-1);
  }
}

UdpPort::UdpPort(std::string name, uint ownPort, std::string toIP, uint toPort, size_t timeout_ms)
  :IOPort(name, 1000)//UDP Overall disconnection set to 1s
{
  isServer_ = false;

  /* Check whether it is a valid IPv4 address */
  if(!isValidIPv4Address(toIP))
  {
    std::cerr << "[ERROR] Invalid IP address: "<< toIP << std::endl;
    exit(-1);
  }

  /* Set single non-blocking accept timeout */
  timeout_saved_.tv_sec = timeout_ms / 1000;
  timeout_saved_.tv_usec = (timeout_ms * 1000) % 1000000;

  /* Set local address, specify port */
  bzero(&ownAddr_, sizeof(sockaddr_in));
  bzero(&targetAddr_, sizeof(sockaddr_in));
  bzero(&fromAddr_, sizeof(sockaddr_in));
  ownAddr_.sin_family = AF_INET;// IPV4
  ownAddr_.sin_port = htons(ownPort);
  ownAddr_.sin_addr.s_addr = htonl(INADDR_ANY); // Bind to all local address

  /* Specify the destination address for sending */
  targetAddr_.sin_family = AF_INET;
  targetAddr_.sin_port = htons(toPort);
  targetAddr_.sin_addr.s_addr = inet_addr(toIP.c_str());

  /* Create socket file descriptor */
  if(socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0), socket_fd_ < 0) {
    perror("[ERROR] UDPPort::UDPPort, create socket failed.\n");
    exit(-1);
  }

  /* Allow reusing local address */
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &on_, sizeof(on_));
  if(bind(socket_fd_, (struct sockaddr*)&ownAddr_, sockaddrSize_) < 0){
    perror("[ERROR] UDPPort::UDPPort, bind failed");
    exit(-1);
  }
}

IOPortStatus UdpPort::send(uint8_t *sendMsg, size_t sendLength)
{
  size_t send_len = 0;

  if(crc32_open_) {
		add_crc32(sendMsg, sendLength);
  }

  /* Check if a destination address has been specified */
  if(ntohs(targetAddr_.sin_port) > 0) {
    // Regardless of how many bytes were sent and whether they were successful
    send_len = sendto(socket_fd_, sendMsg, sendLength, 0, (struct sockaddr*)&targetAddr_, sockaddrSize_);
  }else{
    return IOPortStatus::err;
  }

  if(send_len == sendLength) {
    return IOPortStatus::ok;
  }else{
		// std::cout << "[WARNING] UdpPort: "<< name_ << ". Send length imcompatible.\n";
  }
  return IOPortStatus::err;
}

size_t UdpPort::recvLen(uint8_t* recvMsg, size_t recvLength)
{
  return recvfrom(socket_fd_, recvMsg, recvLength, MSG_DONTWAIT, (struct sockaddr*)&fromAddr_, &sockaddrSize_);
}


IOPortStatus UdpPort::recv(uint8_t *recvMsg, size_t recvLength)
{
  /* non-blocking receiving */
  size_t received_len = 0;

  /* The _rSet will be modified and must be initialized each time */
  fd_set rset;
  FD_ZERO(&rset);
  FD_SET(socket_fd_, &rset);
  timeval timeout_select = timeout_saved_;
  switch (select(socket_fd_+1, &rset, NULL, NULL, &timeout_select))
  {
  case -1: // receive error
    return IOPortStatus::err;
  case 0: // timeout once
    if(isConnect_)
    {
      if(timer_.wait_time()< 0)
      {
        isConnect_ = false;
        std::cout << "[ERROR] Lose connection with UDP "<< name_ << std::endl;

        // Reset the destination port after disconnection
        // for switching between addresses on one server
        if(isServer_) {
          targetAddr_.sin_port = htons(0);
        }
      }else{
        // std::cout << "[WARNING] UDPPort::recv, connect with "<< name_ <<" wait time out" << std::endl;
      }
    }
    return IOPortStatus::timeout;
  default:
    timer_.start(); // Update connection
    
    /* Clear receiving cache */
    timeout_select.tv_sec = 0;
    timeout_select.tv_usec = 0;

    fd_set fdClearBuf;
    FD_ZERO(&fdClearBuf);
    FD_SET(socket_fd_, &fdClearBuf);
    while (1)
    {
      // Known data currently exists, keep reading until there is no more data, take the latest data
      if(select(socket_fd_+1, &rset, NULL, NULL, &timeout_select) <= 0) {
        break;
      }
      received_len = recvfrom(socket_fd_, _recvBuff.data(), recvLength, MSG_DONTWAIT,
                              (struct sockaddr*)&fromAddr_, &sockaddrSize_);
    }
    
    // Reconnect
    if(!isConnect_)
    {
      isConnect_ = true;
      std::cout << "[Report] Re-establish the connection with UDP "<< name_ << std::endl;
    }

    /* Check receiving length */
    if(received_len == recvLength)
    {
      // If CRC checksum has open and error detected, discard data, otherwise copy directly
      if(crc32_open_ && !check_crc32(_recvBuff.data(), received_len))
      {
        return IOPortStatus::crcbad;
      }else{
        memcpy(recvMsg, _recvBuff.data(), received_len);
        // If target port has not been initialized, the send to where data from
        if(ntohs(targetAddr_.sin_port) <= 0) {
          targetAddr_ = fromAddr_;
        }
        return IOPortStatus::ok;
      }
    }else{
      std::cout << "[WARNING] UDP "<< name_ <<" received " << received_len
              << " bytes, but not " << recvLength << " bytes."<< std::endl;
      return IOPortStatus::msgbad;
    }
    break;
  }
  return IOPortStatus::err;
}

