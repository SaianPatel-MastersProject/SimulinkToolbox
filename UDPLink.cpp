//  UDPLink.cpp
//  Copyright (c) 2014-2016 rFpro. All rights reserved.

#include "UDPLink.hpp"
#include <algorithm>
#include <sstream>

namespace UDPLink
{
  size_t BaseUDP::mInstanceCount = 0;

#if _WIN32
  const BYTE WINSOCK_VER_LOW = 1;
  const BYTE WINSOCK_VER_HIGH = 1;
  bool SocketsInitialised = false;

  bool InitialiseSockets()
  {
    if (SocketsInitialised) return true;
    SocketsInitialised = true;

    WORD wRequestedVersion = MAKEWORD(WINSOCK_VER_LOW, WINSOCK_VER_HIGH);

    WSADATA wsaData;
    if (WSAStartup(wRequestedVersion, &wsaData) != 0)
    {
      throw UDPLink::Exception("WSAStartup");
    }

    /* Confirm that the WinSock DLL supports 1.1.*/
    /* Note that if the DLL supports versions greater    */
    /* than 1.1 in addition to 1.1, it will still return */
    /* 1.1 in wVersion since that is the version we      */
    /* requested.                                        */
    if (LOBYTE(wsaData.wVersion) != WINSOCK_VER_LOW ||
      HIBYTE(wsaData.wVersion) != WINSOCK_VER_HIGH)
    {
      WSACleanup();
      throw UDPLink::Exception("WinSock Version");
    }
    return true;
  }

  bool ClearSockets()
  {
    if (!SocketsInitialised) return true;
    SocketsInitialised = false;

    return (WSACleanup() != SOCKET_ERROR);
  }

#else

  bool InitialiseSockets() { return true; }
  bool ClearSockets() { return true; }

#endif // _WIN32


  //  Receiver.cpp

  unsigned long Receiver::mReceiveTimeout = DEFAULT_RECEIVE_TIMEOUT;

  Receiver::Receiver(const std::string &port, const std::string &ip_addr, const unsigned long timeout)
    : mDatagramsReceived(0), mTotalBytesReceived(0), mReusingAddress(false)
  {
    memset(&mBuffer, 0, sizeof mBuffer);

    struct addrinfo hints, *servinfo;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    if (getaddrinfo(ip_addr.c_str(), port.c_str(), &hints, &servinfo) != 0)
    {
      throw UDPLink::Exception("getaddrinfo - Could not interpret address");
    }

    // loop through all the results and bind to the first we can
    for (mAddressInfo = servinfo; mAddressInfo != NULL; mAddressInfo = mAddressInfo->ai_next)
    {
      if ((mSocketNumber = static_cast<int>(socket(mAddressInfo->ai_family, mAddressInfo->ai_socktype, mAddressInfo->ai_protocol))) == SOCKET_ERROR)
      {
        continue;
      }

#ifdef _WIN32
      const bool yes = true;
#else
      const int yes = 1;
#endif
      if (bind(mSocketNumber, mAddressInfo->ai_addr, static_cast<socklen_t>(mAddressInfo->ai_addrlen)) == SOCKET_ERROR)
      {
        if (setsockopt(mSocketNumber, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&yes), sizeof(yes)) == SOCKET_ERROR)
        {
          throw UDPLink::Exception("setsockopt(SO_REUSEADDR)");
        }

        if (bind(mSocketNumber, mAddressInfo->ai_addr, static_cast<socklen_t>(mAddressInfo->ai_addrlen)) == SOCKET_ERROR)
        {
          close(mSocketNumber);
          continue;
        }

        mReusingAddress = true;
      }

      break;
    }

    if (mAddressInfo == NULL)
    {
      throw UDPLink::Exception("bind " + ip_addr + ":" + port);
    }

    if (mAddressInfo->ai_family == AF_INET)
    {
      struct sockaddr_in *ipv4 = reinterpret_cast<struct sockaddr_in *>(mAddressInfo->ai_addr);
      inet_ntop(mAddressInfo->ai_family, &(ipv4->sin_addr), mIpAddress, sizeof mIpAddress);
      mPort = ntohs(ipv4->sin_port);
    }
    else
    {
      throw UDPLink::Exception("IPv6 Unsupported");
    }

    freeaddrinfo(servinfo);

#ifdef _WIN32
    DWORD tv = timeout;
#else
    struct timeval tv;
    tv.tv_sec = 0;
    if (timeout == DEFAULT_RECEIVE_TIMEOUT)
    {
      tv.tv_usec = (DEFAULT_RECEIVE_TIMEOUT * 1000) + 500000l;   // To match Win32 behaviour
    }
    else
    {
      tv.tv_usec = (timeout * 1000);
    }
#endif
    if (setsockopt(mSocketNumber, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&tv), sizeof tv) == SOCKET_ERROR)
    {
      throw UDPLink::Exception("setsockopt(SO_RCVTIMEO)");
    }

    if (setsockopt(mSocketNumber, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char *>(&MAX_RECEIVE_BUFFER_SIZE), sizeof MAX_RECEIVE_BUFFER_SIZE) == SOCKET_ERROR)
    {
      throw UDPLink::Exception("setsockopt(SO_RCVBUF)");
    }
  }

  Receiver::~Receiver()
  {
    if (mSocketNumber > 0)
    {
      close(mSocketNumber);
    }
  }

  size_t Receiver::GetNextMessagePrivate()
  {
    // Will wait for data for up to the socket timeout value (SO_RCVTIMEO)
    socklen_t addrLength = sizeof mSockAddrSender;
    int bytesReceived = recvfrom(mSocketNumber, mBuffer, sizeof mBuffer, 0, reinterpret_cast<struct sockaddr *>(&mSockAddrSender), &addrLength);

    if (bytesReceived > 0)
    {
      mDatagramsReceived++;
      mTotalBytesReceived += bytesReceived;
    }

    return bytesReceived > 0 ? bytesReceived : 0;
  }

  size_t Receiver::GetNextMessageBlocking(void *structure, const size_t structureSize)
  {
    mNumBytesLastMessage = GetNextMessagePrivate();
    size_t numBytes = std::min<size_t>(structureSize, mNumBytesLastMessage);
    if (structure) memcpy(structure, mBuffer, numBytes);
    return mNumBytesLastMessage;
  }

  size_t Receiver::GetNextMessage(void *structure, const size_t structureSize)
  {
    // Calling recvfrom when no data available returns an error, so avoid
    if (NumBytesWaiting() == 0) return 0;

    mNumBytesLastMessage = GetNextMessageBlocking(structure, structureSize);
    if (mNumBytesLastMessage < 0)
    {
      throw UDPLink::Exception("recvfrom");
    }
    return mNumBytesLastMessage;
  }

  msgType Receiver::MessageType()
  {
    return *reinterpret_cast<msgType *>(mBuffer);
  }

  size_t Receiver::CopyMessageBody(void *structure, const size_t structureSize)
  {
    size_t numBytes = std::min<size_t>(structureSize, mNumBytesLastMessage - sizeof(msgType));
    memcpy(structure, mBuffer + sizeof(msgType), numBytes);
    return numBytes;
  }

  size_t Receiver::CopyMessageComplete(void *structure, const size_t structureSize)
  {
    size_t numBytes = std::min<size_t>(structureSize, mNumBytesLastMessage);
    memcpy(structure, mBuffer, numBytes);
    return numBytes;
  }

  size_t Receiver::NumBytesWaiting()
  {
    u_long numBytesAvailable = 0;

    if (ioctl(mSocketNumber, FIONREAD, reinterpret_cast<char *>(&numBytesAvailable)) == SOCKET_ERROR)
    {
      throw UDPLink::Exception("ioctl");
    }

    return numBytesAvailable;
  }

  std::ostream& operator<< (std::ostream &out, Receiver &objReceiver)
  {
    out << objReceiver.GetAddress() << ":" << objReceiver.GetPort();
    return out;
  }

  std::string Receiver::GetSender() const
  {
    if (mSockAddrSender.ss_family == AF_INET)
    {
      const auto details = reinterpret_cast<const sockaddr_in*>(&mSockAddrSender);
      const size_t size = 30;
      char buf[size];
      auto copy = mSockAddrSender;  // Can't pass our const mSockAddrSender to inet_ntop which requires non const data.

      const size_t offsetToAddressBytes = 2;
#ifdef _WIN32
      const auto addressBytes = copy.__ss_pad1 + offsetToAddressBytes;
#else
      const auto addressBytes = copy.__ss_padding + offsetToAddressBytes;
#endif
      inet_ntop(mSockAddrSender.ss_family, addressBytes, buf, size);

      std::ostringstream stream;
      stream << std::string(buf) << ":" << details->sin_port;
      return stream.str();
    }
    return "";
  }

  //  Sender.cpp

  Sender::Sender(const std::string &portDestination, const std::string &ipAddrDestination) : mDatagramsSent(0), mTotalBytesSent(0)
  {
    memset(&mBuffer, 0, sizeof mBuffer);

    struct addrinfo hints;

    // first, load up address structs with getaddrinfo():
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    if (getaddrinfo(ipAddrDestination.c_str(), portDestination.c_str(), &hints, &mDestinationInfoLinkedList) != 0)
    {
      throw UDPLink::Exception("getaddrinfo - Could not interpret address");
    }

    for (mDestInfo = mDestinationInfoLinkedList; mDestInfo != NULL; mDestInfo = mDestInfo->ai_next)
    {
      if ((mSocketNumber = static_cast<int>(socket(mDestInfo->ai_family, mDestInfo->ai_socktype, mDestInfo->ai_protocol))) == SOCKET_ERROR)
      {
        continue;
      }

      break;
    }

    if (mDestInfo == NULL)
    {
      throw UDPLink::Exception("socket");
    }

    if (mDestInfo->ai_family == AF_INET)
    {
      struct sockaddr_in *ipv4 = reinterpret_cast<struct sockaddr_in *>(mDestInfo->ai_addr);
      inet_ntop(mDestInfo->ai_family, &(ipv4->sin_addr), mDestIpAddress, sizeof mDestIpAddress);
      mDestPort = ntohs(ipv4->sin_port);
    }
    else
    {
      throw UDPLink::Exception("IPv6 Unsupported");
    }

#ifdef _WIN32
    const bool broadcast = true;
#else
    const int broadcast = 1;
#endif
    if ((setsockopt(mSocketNumber, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char *>(&broadcast), sizeof broadcast)) == SOCKET_ERROR)
    {
      throw UDPLink::Exception("setsockopt(SO_BROADCAST)");
    }

    if (setsockopt(mSocketNumber, SOL_SOCKET, SO_SNDBUF, reinterpret_cast<const char *>(&MAX_TRANSMIT_BUFFER_SIZE), sizeof MAX_TRANSMIT_BUFFER_SIZE) == SOCKET_ERROR)
    {
      throw UDPLink::Exception("setsockopt(SO_SNDBUF)");
    }

    //strcpy(mSourceIpAddress, "");
  }

  Sender::~Sender()
  {
    if (mSocketNumber > 0)
    {
      close(mSocketNumber);
    }
    freeaddrinfo(mDestinationInfoLinkedList);
  }

  void Sender::SetSourcePort(const std::string &sourcePort)
  {
#ifdef _WIN32
    const bool yes = true;
#else
    const int yes = 1;
#endif
    if (setsockopt(mSocketNumber, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&yes), sizeof(yes)) == SOCKET_ERROR)
    {
      throw UDPLink::Exception("setsockopt(SO_REUSEADDR)");
    }

    {
      struct addrinfo hints, *res;

      // first, load up address structs with getaddrinfo():
      memset(&hints, 0, sizeof hints);
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_DGRAM;
      hints.ai_flags = AI_PASSIVE; // fill in my IP for me
      getaddrinfo(NULL, sourcePort.c_str(), &hints, &res);

      struct addrinfo *mSourceInfo;

      for (mSourceInfo = res; mSourceInfo != NULL; mSourceInfo = mSourceInfo->ai_next)
      {
        if (bind(mSocketNumber, mSourceInfo->ai_addr, static_cast<socklen_t>(mSourceInfo->ai_addrlen)) != SOCKET_ERROR) break;
      }

      if (mSourceInfo != NULL && mSourceInfo->ai_family == AF_INET)
      {
        //char mSourceIpAddress[INET6_ADDRSTRLEN];
        struct sockaddr_in *ipv4 = reinterpret_cast<struct sockaddr_in *>(mSourceInfo->ai_addr);
        mSourcePort = ntohs(ipv4->sin_port);
        //inet_ntop(mSourceInfo->ai_family, &(ipv4->sin_addr), mSourceIpAddress, sizeof mSourceIpAddress);
      }

      freeaddrinfo(res);
    }
  }

  bool Sender::SetSourceAddress(const std::string &ipAddrSource, const std::string &sourcePort)
  {
#ifdef _WIN32
    const bool yes = true;
#else
    const int yes = 1;
#endif
    if (setsockopt(mSocketNumber, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&yes), sizeof(yes)) == SOCKET_ERROR)
    {
      throw UDPLink::Exception("setsockopt(SO_REUSEADDR)");
    }

    {
      struct addrinfo hints, *res;

      // first, load up address structs with getaddrinfo():
      memset(&hints, 0, sizeof hints);
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_DGRAM;
      //hints.ai_flags = AI_PASSIVE; // fill in my IP for me
      getaddrinfo(ipAddrSource.c_str(), sourcePort.c_str(), &hints, &res);

      struct addrinfo *mSourceInfo;

      for (mSourceInfo = res; mSourceInfo != NULL; mSourceInfo = mSourceInfo->ai_next)
      {
        if (bind(mSocketNumber, mSourceInfo->ai_addr, static_cast<socklen_t>(mSourceInfo->ai_addrlen)) != SOCKET_ERROR) break;
      }

      if (mSourceInfo != NULL && mSourceInfo->ai_family == AF_INET)
      {
        //char mSourceIpAddress[INET6_ADDRSTRLEN];
        struct sockaddr_in *ipv4 = reinterpret_cast<struct sockaddr_in *>(mSourceInfo->ai_addr);
        mSourcePort = ntohs(ipv4->sin_port);
        //inet_ntop(mSourceInfo->ai_family, &(ipv4->sin_addr), mSourceIpAddress, sizeof mSourceIpAddress);
      }

      freeaddrinfo(res);

      return mSourceInfo != NULL;
    }
  }

  void Sender::Send(const msgType msg_type, const void * const payload, const size_t payloadSize)
  {
    if (payloadSize + sizeof msg_type > sizeof mBuffer)
    {
      throw UDPLink::Exception("sizeof payload > BUFLEN");
    }
    memcpy(mBuffer, reinterpret_cast<const char *>(&msg_type), sizeof msg_type);
    if (payload) memcpy(mBuffer + sizeof msg_type, payload, payloadSize);
    Send(&mBuffer, payloadSize + sizeof msg_type);
  }

  void Sender::Send(const void *buffer, const size_t bufferSize)
  {
    int bytesSent = sendto(mSocketNumber, reinterpret_cast<const char *>(buffer),
#ifdef _WIN32
      static_cast<int>(bufferSize),
#else
      bufferSize,
#endif
      0, mDestInfo->ai_addr, static_cast<socklen_t>(mDestInfo->ai_addrlen));

    assert(static_cast<size_t>(bytesSent) == bufferSize);

    if (bytesSent > 0)
    {
      mDatagramsSent++;
      mTotalBytesSent += bytesSent;
    }
  }

  std::ostream& operator<< (std::ostream &out, Sender &objSender)
  {
    out << objSender.GetDestAddress() << ":" << objSender.GetDestPort();
    return out;
  }
}
