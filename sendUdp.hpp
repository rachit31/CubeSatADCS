/** 
 *  @file    sendUdp.hpp
 *  @author  Rachit Bhatia
 *  @date    02/23/2018
 *  @version 1.0.3
 *  
 *  @brief Send data using UDP Protocol
 *
 *  @section DESCRIPTION
 *  
 *  This class creates a socket and will send a specified number of doubles
 *  to the socket
 *
 */

#ifndef SENDUDP_HPP_
#define SENDUDP_HPP_

#include <arpa/inet.h> // sockaddr_in, in_addr, inet_pton, htons

/**
*  @brief Class to send UDP data
*/
class SendUdp
{
public:

  /** 
  *   @brief  Constructor which sets the socket file descriptor to uninitialized
  *
  */
  SendUdp();

  /** 
  *   @brief  Destructor which closes the socket if initialized
  *
  */
  ~SendUdp();

  /** 
  *   @brief  Initializes the socket to send UDP data to a specific IP and port
  *  
  *   @param  localIp is the IP address to send data from
  *   @param  remoteIp is the IP address to send data to
  *   @param  sendPort is the UDP port to send data to
  *   @return const char* error message on failure
  */
  const char* initialize(const char* localIp, const char* remoteIp, const uint16_t sendPort);

  /** 
  *   @brief  Sends the specified number of doubles
  *  
  *   @param  buffer is a pointer to a buffer where the data to send is located
  *   @param  number is the number of doubles to send
  *   @return const char* error message on failure
  */
  const char* sendDoubles(const double *buffer, const int number) const;

private:
  int m_socketFd;                   ///< File descriptor of the socket
  struct sockaddr_in m_sockaddr_in; ///< Socket structure setup to send data
  socklen_t m_len;                  ///< Lenght of IPv4 address
};

#endif