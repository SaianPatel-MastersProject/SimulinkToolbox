//
// Copyright (c) 2010-2020 rFpro Limited, All Rights Reserved.
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
// 

#pragma once

#ifndef _WIN32
#define __cdecl
#endif

#pragma pack( push, 4 )

enum PluginObjectType
{
  PO_INVALID = -1,
  //-------------------
  PO_GAMESTATS = 0,
  PO_NCPLUGIN = 1,
  PO_IVIBE = 2,
  PO_INTERNALS = 3,
  PO_RFONLINE = 4,
  //-------------------
  PO_MAXIMUM
};


// Interface used by plugin classes.

class PluginObject
{
private:

  class PluginInfo *mInfo;             // used by main executable to obtain info about the plugin that implements this object

public:

  void SetInfo(class PluginInfo *p) { mInfo = p; }        // used by main executable
  class PluginInfo *GetInfo() const { return mInfo; }  // used by main executable
};


// Typedefs for dll functions - easier to use a typedef than to type
// out the syntax for declaring and casting function pointers

typedef const char *      ( __cdecl *GETPLUGINNAME )();
typedef PluginObjectType  ( __cdecl *GETPLUGINTYPE )();
typedef int               ( __cdecl *GETPLUGINVERSION )();
typedef PluginObject *    ( __cdecl *CREATEPLUGINOBJECT )();
typedef void              ( __cdecl *DESTROYPLUGINOBJECT )( PluginObject *obj );


#pragma pack(pop)
