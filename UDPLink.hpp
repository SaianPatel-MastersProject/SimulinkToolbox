// Cross-platform class for transmitting/receiving structures over UDP with type identifier
//
// Copyright (c) 2014 Kangaloosh Ltd t/a rFpro, All Rights Reserved.
//
// NOTICE:  All information contained herein is, and remains the property of rFpro. The intellectual and technical concepts contained
// herein are proprietary to rFpro and may be covered by U.S. and foreign patents, patents in process, and are protected by trade secret or copyright law.
// Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained from rFpro.
//
// The copyright notice above does not evidence any actual or intended publication or disclosure of this source code, which includes information that is confidential
// and/or proprietary, and is a trade secret, of rFpro.  ANY REPRODUCTION, DISTRIBUTION, PUBLIC PERFORMANCE, OR PUBLIC DISPLAY OF THIS SOURCE CODE
// WITHOUT THE EXPRESS WRITTEN CONSENT OF RFPRO IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL TREATIES.
// THE RECEIPT OR POSSESSION OF THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS TO REPRODUCE,
// DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT MAY DESCRIBE, IN WHOLE OR IN PART.

#pragma once

#include <iostream>
#include <string>
#include <exception>
#include <cstring>
#include <cstdint>
#include <cassert>
#include <sys/types.h>
#include <ostream>

#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
inline int strerror_r(int errnum, char *buf, size_t buflen) { return strerror_s(buf, buflen, errnum); }
inline int close(SOCKET s) { return closesocket(s); }
inline int ioctl(int s, unsigned long r, char *o) { return ioctlsocket(s, r, reinterpret_cast<u_long *>(o)); }
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <netdb.h>
#endif

#include "ICom.hpp"

#ifndef SOCKET_ERROR
#define SOCKET_ERROR -1
#endif


namespace UDPLink
{
  typedef uint32_t msgType;
  const msgType MSG_NONE = 0;

  const int MAX_RECEIVE_BUFFER_SIZE = 128 * 1024; // bytes
  const int MAX_TRANSMIT_BUFFER_SIZE = 128 * 1024; // bytes
  const unsigned long DEFAULT_RECEIVE_TIMEOUT = 10; // ms

  // This value will be set absolute
  //const int SENDMESS_DEFAULT_TIMEOUT = 50;

  bool InitialiseSockets();
  bool ClearSockets();

  class Exception : public std::exception
  {
  public:
    explicit Exception(const std::string &userMessage) : mMessage(userMessage) {}
    ~Exception() throw() {}

    virtual const char* what() const throw()
    {
      return mMessage.c_str();
    }
  private:
    std::string mMessage;
  };

  class BaseUDP
  {
  public:
    BaseUDP()
    {
      if (mInstanceCount == 0) InitialiseSockets();
      mInstanceCount++;
      //std::cout << "Constructed BaseUDP " << mInstanceCount << std::endl;
    }

    virtual ~BaseUDP()
    {
      assert(mInstanceCount > 0);
      mInstanceCount--;
      if (mInstanceCount == 0) ClearSockets();
      //std::cout << "Destructed BaseUDP " << mInstanceCount << std::endl;
    }
  private:
    static size_t mInstanceCount;

  };

  class NullSender : public ICom::Sender
  {
  public:
    virtual void Send(const void *buffer, const size_t bufferSize) {};
    virtual void Send(const msgType msg_type, const void * const payload = 0, const size_t payloadSize = 0) {};
  };

  class Sender : public BaseUDP, public ICom::Sender
  {
  public:
    explicit Sender(const std::string &portDestination, const std::string &ipAddrDestination = "localhost");
    ~Sender();
    void SetSourcePort(const std::string &sourcePort);
    bool SetSourceAddress(const std::string &ipAddrSource, const std::string &sourcePort = "0");
    virtual void Send(const msgType msg_type, const void * const payload = 0, const size_t payloadSize = 0);
    virtual void Send(const void *buffer, const size_t bufferSize);

    size_t DatagramsSent(void) const { return mDatagramsSent; }
    size_t BytesSent(void) const { return mTotalBytesSent; }
    unsigned short GetDestPort() const { return mDestPort; }
    std::string GetDestAddress() const { return std::string(mDestIpAddress); }
    friend std::ostream& operator<< (std::ostream &out, Sender &objSender);

  private:
    int mSocketNumber;
    struct addrinfo *mDestInfo;
    struct addrinfo *mDestinationInfoLinkedList;
    char mBuffer[MAX_TRANSMIT_BUFFER_SIZE];
    size_t mDatagramsSent;
    size_t mTotalBytesSent;
    char mDestIpAddress[INET6_ADDRSTRLEN];
    unsigned short mDestPort;
    char mSourceIpAddress[INET6_ADDRSTRLEN];
    unsigned short mSourcePort;
  };

  class NullReceiver : public ICom::Receiver
  {
  public:
    virtual size_t GetNextMessage(void *structure = 0, const size_t structureSize = 0) { return 0; };
    virtual msgType MessageType() { return 0; };
    virtual size_t CopyMessageBody(void *structure, const size_t structureSize) { return 0; };
  };

  class Receiver : public BaseUDP, public ICom::Receiver
  {
  public:
    static void ReceiveTimeout(const unsigned long ms) { mReceiveTimeout = ms; }

    explicit Receiver(const std::string &port, const std::string &ip_addr = "0.0.0.0", const unsigned long timeout = mReceiveTimeout);
    ~Receiver();

    virtual size_t GetNextMessage(void *structure = 0, const size_t structureSize = 0);
    size_t GetNextMessageBlocking(void *structure = 0, const size_t structureSize = 0);
    virtual msgType MessageType();
    size_t MessageBody(void *structure, const size_t structureSize) { return CopyMessageBody(structure, structureSize); }
    virtual size_t CopyMessageBody(void *structure, const size_t structureSize);
    size_t CopyMessageComplete(void *structure, const size_t structureSize);
    virtual const char * const GetMessage(const size_t offset = 0) { return mBuffer + offset; }

    size_t DatagramsReceived(void) const { return mDatagramsReceived; }
    size_t BytesReceived(void) const { return mTotalBytesReceived; }
    unsigned short GetPort() const { return mPort; }
    std::string GetAddress() const { return std::string(mIpAddress); }
    bool ReusingAddress() const { return mReusingAddress; }
    int GetFd() const { return mSocketNumber; }
    friend std::ostream& operator<< (std::ostream &out, Receiver &objReceiver);
    std::string GetSender() const;

  private:
    virtual size_t NumBytesWaiting();
    virtual size_t GetNextMessagePrivate();

    struct sockaddr_storage mSockAddrSender;
    struct addrinfo *mAddressInfo;
    int mSocketNumber;
    char mBuffer[MAX_RECEIVE_BUFFER_SIZE];
    size_t mDatagramsReceived;
    size_t mTotalBytesReceived;
    size_t mNumBytesLastMessage;
    char mIpAddress[INET6_ADDRSTRLEN];
    unsigned short mPort;
    bool mReusingAddress;
    static unsigned long mReceiveTimeout;
  };
}
