//  Class for transmitting/receiving structures over UDP with type identifier
//  Copyright (c) 2014 rFactorPro. All rights reserved.

#pragma once

#include <cstdint>

namespace ICom
{
  typedef uint32_t msgType;

  class Sender
  {
  public:
    virtual void Send(const void *buffer, const size_t bufferSize) = 0;
    virtual void Send(const msgType msg_type, const void * const payload = 0, const size_t payloadSize = 0) = 0;
    virtual ~Sender(void) {};
  };

  class Receiver
  {
  public:

    virtual size_t GetNextMessage(void *structure = 0, const size_t structureSize = 0) = 0;
    virtual msgType MessageType() = 0;
    virtual size_t CopyMessageBody(void *structure, const size_t structureSize) = 0;
    virtual ~Receiver(void) {};

    virtual const char * const GetMessage(const size_t offset = 0) { return 0; } // TODO Remove after altering Traffic Vires
  };
}
