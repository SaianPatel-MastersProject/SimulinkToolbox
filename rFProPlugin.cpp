// Copyright (c) 2011-2020 rFpro Limited, All Rights Reserved.
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

#include <windows.h>
#include <stdio.h>        // for sample output  
#include <assert.h>         // for assert
#include <ctime>
#include <direct.h>
#include "rFProPlugin.hpp"
#include "rtwtypes.h"
#include <cstring>
#include <algorithm>
#include <map>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include "StringUtils.hpp"
#include "BoundingBoxControl.hpp"
#include "TelemOpponentControl.hpp"

extern "C" __declspec( dllexport )
  const char * __cdecl GetPluginName()           { return PLUGIN_NAME; }

extern "C" __declspec( dllexport )
  PluginObjectType __cdecl GetPluginType()         { return PO_INTERNALS; } 

extern "C" __declspec( dllexport )
  int __cdecl GetPluginVersion()             {  return rFProPlugin::API_VERSION; } // InternalsPluginV**Extras functionality

extern "C" __declspec( dllexport )
  PluginObject * __cdecl CreatePluginObject()      { return static_cast<PluginObject *>( new rFProPlugin ); }

extern "C" __declspec( dllexport )
  void __cdecl DestroyPluginObject( PluginObject *obj )  { delete static_cast<rFProPlugin *>( obj ); }

/* Macros to create model-name based structure to match Simulink Coder generated code */
#ifndef MODEL
# error "must define MODEL"
#endif
#define EXPAND_CONCAT(name1,name2) name1 ## name2
#define CONCAT(name1,name2) EXPAND_CONCAT(name1,name2)
#define RT_MODEL      CONCAT(MODEL,_rtModel)
#define MODELFCN(fcn) CONCAT(MODEL,fcn)

#ifdef __cplusplus
extern "C" {
#endif
  //extern RT_MODEL *MODEL(void);
  extern void MODELFCN(_initialize)(void);
  extern void MODELFCN(_step)(void);
  extern void MODELFCN(_terminate)(void);
#ifdef __cplusplus
}
#endif

/* Global data for data transfer between fFactor-Pro & Simulink */
double simulinkModelTime = -1;
double rFproSimTime = -1;
double rFproDeltaTime = -1;
PhysicsInputV01 simulinkInput;
PhysicsOutputV01 simulinkOutput;
StartingVehicleLocationV01 startLocn;
long currentViewedSlot = 0;
bool receivedStartLocation = false;

TelemInfoV01 simulinkInfo;
TelemInfoV01* vehicle_telemetry = nullptr; // pointer to working variable used in opponent telemetry tlc file
TelemetryOpponentList sl_TelemetryOpponents;

HDContactPatchV01 simulinkTyreContactInput[4];
bool wantsTelemetryUpdates = false;
bool wantsTyreContactUpdates = false;
TyreDefinitionInputV01 simulinkTyreDef;
TyreContactPointDefnV01 simulinkTyreContact[40];
long numContactPoints = 0;
bool simulinkOutputForceFeedbackValid = false;
double simulinkOutputForceFeedbackValue = 0.0;
bool wantsMotionStates = false;
MotionPlatformV01 simulinkMotionState;
std::vector<std::string> FMODParameterNames(7, "");
std::vector<float> FMODParameterValues(7, 0);

rFpro::TrafficDataModelV02 *simulinkTrafficInterface = 0; // Non-owning pointer (Don't call delete!)
rFpro::TrafficDataModelV03 *simulinkTrafficInterfaceV03 = 0; // Non-owning pointer (Don't call delete!)
rFpro::TrafficDataModelV04 *simulinkTrafficInterfaceV04 = 0; // Non-owning pointer (Don't call delete!)
std::unordered_set<std::string> mLiveLoadedActors;

std::map<uint32_t, NewVehicleDataV01> simulinkNewVehicles;
std::map<uint32_t, VehicleStateV01> simulinkVehicleUpdates;
bool wantsOpponentStates = false;

bool simulinkSceneLightsUpdated;
SceneLightStatesV01 simulinkSceneLights;
std::queue<LightControlV01> simulinkLightEvents;

std::vector<ProximityQueryPointDefnV01> simulinkProximityDef;
std::vector<ProximityQueryPointV01> simulinkProximityData;

// App checks for motion tracking once at start, so set true permanently, as in RMP
bool simulinkHeadTrackingActive = true;
MotionPlatformV01 simulinkHeadMotionInfo;

bool simulinkCameraControlInfoUpdated = false;
UDPCameraControlInfoV01 simulinkCameraControlInfo;

std::unordered_map<std::string, double> simulinkKeyPresses;

// Bounding Box (BB) data
bool simulinkWantsBoundingBoxes = false;
BoundingBoxControl simulinkBBControl;

// Animation commands
std::queue<AnimationActionV01> simulinkAnimationCommands;

// Global functions
// ----------------

void processModelOutput(const uint8_T* msg, const int msgSize = -1)
{
    uint32_t const msgType = *reinterpret_cast<const uint32_t *>(msg);

    switch (msgType)
    {
        case 0: //  = RFPRO_PAUSE
        {
            printf("[Simulink-Hosted] Msg number: %d", msgType);
            // Do nothing - Hosted models shouldn't be calling pause/resume?
            break;
        }
        case 1: //  = RFPRO_RESUME
        {
            printf("[Simulink-Hosted] Msg number: %d", msgType);
            // Do nothing - Hosted models shouldn't be calling pause/resume?
            break;
        }
        case 8: //  = RFPRO_KEY_PRESS
        {
            // Put key press in map structure, ready for checkHWcontrol to access
            char controlName[32] = "";
            double controlValue = 1.0;

            strncpy(controlName, reinterpret_cast<const char*>(msg[4]), 32);
            controlValue = *reinterpret_cast<const double*>(msg[36]);

            simulinkKeyPresses.insert(std::make_pair(controlName, controlValue));
            break;
        }
        case 101: //  = RFPRO_PHYSICS_OUTPUT
        {
          // Copy UDPPhysicsOutputV01 to PhysicsOutputV01 - NOT the same structure
          simulinkOutput.mET = *reinterpret_cast<const double*>(msg + 4);

          simulinkOutput.mSkipInternalPhysics = *reinterpret_cast<const bool*>(msg + 12);
          simulinkOutput.mUseInternalPhysics = *reinterpret_cast<const  bool*>(msg + 13);

          simulinkOutput.mPos[0] = *reinterpret_cast<const double*>(msg + 16);
          simulinkOutput.mPos[1] = *reinterpret_cast<const double*>(msg + 24);
          simulinkOutput.mPos[2] = *reinterpret_cast<const double*>(msg + 32);

          simulinkOutput.mOri[0][0] = *reinterpret_cast<const double*>(msg + 40);
          simulinkOutput.mOri[0][1] = *reinterpret_cast<const double*>(msg + 48);
          simulinkOutput.mOri[0][2] = *reinterpret_cast<const double*>(msg + 56);
          simulinkOutput.mOri[1][0] = *reinterpret_cast<const double*>(msg + 64);
          simulinkOutput.mOri[1][1] = *reinterpret_cast<const double*>(msg + 72);
          simulinkOutput.mOri[1][2] = *reinterpret_cast<const double*>(msg + 80);
          simulinkOutput.mOri[2][0] = *reinterpret_cast<const double*>(msg + 88);
          simulinkOutput.mOri[2][1] = *reinterpret_cast<const double*>(msg + 96);
          simulinkOutput.mOri[2][2] = *reinterpret_cast<const double*>(msg + 104);

          simulinkOutput.mVel[0] = *reinterpret_cast<const double*>(msg + 112);
          simulinkOutput.mVel[1] = *reinterpret_cast<const double*>(msg + 120);
          simulinkOutput.mVel[2] = *reinterpret_cast<const double*>(msg + 128);

          simulinkOutput.mRot[0] = *reinterpret_cast<const double*>(msg + 136);
          simulinkOutput.mRot[1] = *reinterpret_cast<const double*>(msg + 144);
          simulinkOutput.mRot[2] = *reinterpret_cast<const double*>(msg + 152);

          simulinkOutput.mAccelValid = *reinterpret_cast<const bool*>(msg + 161);
          simulinkOutput.mAccel[0] = *reinterpret_cast<const double*>(msg + 164);
          simulinkOutput.mAccel[1] = *reinterpret_cast<const double*>(msg + 172);
          simulinkOutput.mAccel[2] = *reinterpret_cast<const double*>(msg + 180);

          simulinkOutput.mRotAccelValid = *reinterpret_cast<const bool*>(msg + 922);
          simulinkOutput.mRotAccel[0] = *reinterpret_cast<const double*>(msg + 924);
          simulinkOutput.mRotAccel[1] = *reinterpret_cast<const double*>(msg + 932);
          simulinkOutput.mRotAccel[2] = *reinterpret_cast<const double*>(msg + 940);

          // wheel info (actual data originally copied by WheelOutputV01 in hosted)
          simulinkOutput.mWheelPosValid = *reinterpret_cast<const bool*>(msg + 188);
          simulinkOutput.mWheelOriValid = *reinterpret_cast<const unsigned char*>(msg + 189);
          simulinkOutput.mTireParamsValid = *reinterpret_cast<const bool*>(msg + 190);
          simulinkOutput.mWheelRotationValid = *reinterpret_cast<const bool*>(msg + 191);
          simulinkOutput.mWheelBrakeTempValid = *reinterpret_cast<const bool*>(msg + 192);
          simulinkOutput.mWheelTreadTempValid = *reinterpret_cast<const bool*>(msg + 193);
          simulinkOutput.mWheelAirTempValid = *reinterpret_cast<const bool*>(msg + 194);

          for (int wheelID = 0; wheelID < 4; ++wheelID)
          {
              simulinkOutput.mWheel[wheelID].mPos[0] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 196);
              simulinkOutput.mWheel[wheelID].mPos[1] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 204);
              simulinkOutput.mWheel[wheelID].mPos[2] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 212);

              simulinkOutput.mWheel[wheelID].mOri[0][0] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 220);
              simulinkOutput.mWheel[wheelID].mOri[0][1] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 228);
              simulinkOutput.mWheel[wheelID].mOri[0][2] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 236);
              simulinkOutput.mWheel[wheelID].mOri[1][0] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 244);
              simulinkOutput.mWheel[wheelID].mOri[1][1] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 252);
              simulinkOutput.mWheel[wheelID].mOri[1][2] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 260);
              simulinkOutput.mWheel[wheelID].mOri[2][0] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 268);
              simulinkOutput.mWheel[wheelID].mOri[2][1] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 276);
              simulinkOutput.mWheel[wheelID].mOri[2][2] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 284);

              simulinkOutput.mWheel[wheelID].mSlipAngle = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 292);
              simulinkOutput.mWheel[wheelID].mSlipRatio = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 300);
              simulinkOutput.mWheel[wheelID].mTireLoad  = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 308);
              simulinkOutput.mWheel[wheelID].mRotation  = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 316);
              simulinkOutput.mWheel[wheelID].mBrakeTemp = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 324);

              simulinkOutput.mWheel[wheelID].mTreadTemp[0] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 332);
              simulinkOutput.mWheel[wheelID].mTreadTemp[1] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 340);
              simulinkOutput.mWheel[wheelID].mTreadTemp[2] = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 348);

              simulinkOutput.mWheel[wheelID].mAirTemp = *reinterpret_cast<const double*>(msg + (wheelID * 168) + 356);
          }
          
          // engine, input, gear
          simulinkOutput.mGearValid   = *reinterpret_cast<const bool*>(msg + 868);
          simulinkOutput.mRPMValid    = *reinterpret_cast<const bool*>(msg + 869);
          simulinkOutput.mInputsValid = *reinterpret_cast<const bool*>(msg + 870);
          simulinkOutput.mGear      = *reinterpret_cast<const int32_t*>(msg + 872);
          simulinkOutput.mEngineRPM = *reinterpret_cast<const double*>(msg + 876);
          simulinkOutput.mThrottle  = *reinterpret_cast<const double*>(msg + 884);
          simulinkOutput.mBrake     = *reinterpret_cast<const double*>(msg + 892);
          simulinkOutput.mSteering  = *reinterpret_cast<const double*>(msg + 900);

          // steering torque (force feedback)
          simulinkOutputForceFeedbackValid = *reinterpret_cast<const bool*>(msg + 908);
          simulinkOutputForceFeedbackValue = *reinterpret_cast<const double*>(msg + 912);

          // Headlights
          simulinkOutput.mHeadlightsValid = *reinterpret_cast<const bool*>(msg + 920);
          simulinkOutput.mHeadlights= *reinterpret_cast<const char*>(msg + 921);
          break;
        }
        case 102: //  = RFPRO_REQUEST_START_LOCATION
        {
            // Do nothing - Hosted models acess start pos directly
            break;
        }
        case 105: //  = RFPRO_PROXIMITY_CONFIG
        {
            // The proximity block does the initialisation
            // in the .tlc start fcn
            break;
        }
        case 107: //  = RFPRO_CONTROL_INPUTS
        {
            // Not in simulink library blocks at present.
            break;
        }
        case 108: //  = RFPRO_ACTOR_INPUT
        {
            bool doPreLoad;
            rFpro::SpecialActorStateV03 actorState;
                      
            // Check to see if there is a traffic plugin first
            if (simulinkTrafficInterface != 0)
            {
              memset(&actorState, 0, sizeof(rFpro::SpecialActorStateV03));
              actorState.mET = *reinterpret_cast<const double*>(msg + 4);
              strncpy(reinterpret_cast<char *>(actorState.mName), reinterpret_cast<const char *>(msg + 12), sizeof(actorState.mName));
              actorState.mPos.x = *reinterpret_cast<const double*>(msg + 36);
              actorState.mPos.y = *reinterpret_cast<const double*>(msg + 44);
              actorState.mPos.z = *reinterpret_cast<const double*>(msg + 52);
              actorState.mHeading = *reinterpret_cast<const double*>(msg + 60);
              actorState.mHeadingValid = *reinterpret_cast<const bool*>(msg + 68);
              strncpy(reinterpret_cast<char *>(actorState.mType), reinterpret_cast<const char*>(msg + 70), sizeof(actorState.mType));
              actorState.mSpecialType = static_cast<AnimationDataSpecialV01::Type>(*reinterpret_cast<const char*>(msg + 69));
              actorState.mHeightSetting = *(msg + 95); // 0 - auto @ ground height, 1 = rel to ground, 2 = abs height
              
              if (msgSize >= 4 + 172) // sizeof(msgType) + sizeof(UDPSpecialActorStateV02)
              {
                actorState.mOriValid = *reinterpret_cast<const bool*>(msg + 97);
                actorState.mOri[0].x = *reinterpret_cast<const double*>(msg + 104);
                actorState.mOri[0].y = *reinterpret_cast<const double*>(msg + 112);
                actorState.mOri[0].z = *reinterpret_cast<const double*>(msg + 120);
                actorState.mOri[1].x = *reinterpret_cast<const double*>(msg + 128);
                actorState.mOri[1].y = *reinterpret_cast<const double*>(msg + 136);
                actorState.mOri[1].z = *reinterpret_cast<const double*>(msg + 144);
                actorState.mOri[2].x = *reinterpret_cast<const double*>(msg + 152);
                actorState.mOri[2].y = *reinterpret_cast<const double*>(msg + 160);
                actorState.mOri[2].z = *reinterpret_cast<const double*>(msg + 168);
              }
              
              if (msgSize >= 4 + 204) // sizeof(msgType) + sizeof(UDPSpecialActorStateV03)
              {
                const uint8_t animationControlFlags = *reinterpret_cast<const uint8_t*>(msg + 98);
                actorState.mManual = (animationControlFlags & 0x01) != 0;
                strncpy(actorState.mAnimationState, reinterpret_cast<const char *>(msg + 176), sizeof(actorState.mAnimationState));
                actorState.mOneShot = (animationControlFlags & 0x02) != 0;
                actorState.mUseDirectAnimationFrame = (animationControlFlags & 0x04) != 0;
                actorState.mPlaybackRate = *reinterpret_cast<const float*>(msg + 200);
                actorState.mDirectAnimationFrame = *reinterpret_cast<const int32_t*>(msg + 204);
              }
              
              doPreLoad = *reinterpret_cast<const bool*>(msg + 94);
              if (doPreLoad)
              {
                if (mLiveLoadedActors.find(actorState.mName) == mLiveLoadedActors.end())
                {
                  if (strcmp(actorState.mType, "") == 0)
                  {
                    strncpy(actorState.mType, "pedestrian01", sizeof("pedestrian01"));
                  }
                  // printf("[Simulink] Loading special actor: %s\n", actorState.mType);
                  simulinkTrafficInterface->LoadSpecialActors(actorState.mSpecialType, actorState.mType, 1);
                  mLiveLoadedActors.insert(actorState.mName);
                }
              }

              if (simulinkTrafficInterfaceV04 != nullptr)
              {
                simulinkTrafficInterfaceV04->SetSpecialActor(&actorState);
              }
              else if (simulinkTrafficInterfaceV03 != nullptr)
              {
                simulinkTrafficInterfaceV03->SetSpecialActor(&actorState);
              }
              else
              {
                simulinkTrafficInterface->SetSpecialActor(&actorState);
              }
            }
            break;
        }
        case 110: //  = RFPRO_SET_SCENE_LIGHTS
        {
            simulinkSceneLights.mStartLights = *reinterpret_cast<const int*>(msg + 4);
            simulinkSceneLights.mPitEntry = *reinterpret_cast<const int*>(msg + 8);
            simulinkSceneLights.mPitExit = *reinterpret_cast<const int*>(msg + 12);
            simulinkSceneLights.mStartAmber = *reinterpret_cast<const char*>(msg + 16);
            simulinkSceneLights.mStartGreen = *reinterpret_cast<const char*>(msg + 17);
            simulinkSceneLights.mFlagState = *reinterpret_cast<const char*>(msg + 18);
            simulinkSceneLights.mFlagPostId = *reinterpret_cast<const char*>(msg + 19);
            simulinkSceneLights.mPitCrew = *reinterpret_cast<const char*>(msg + 20);
            simulinkSceneLightsUpdated = true;
            break;
        }
        case 111: //  = RFPRO_SET_GENERAL_LIGHT
        {
            LightControlV01 state;
            memset(&state, 0, sizeof(LightControlV01));

            state.mSlotID = (*reinterpret_cast<const bool*>(msg + 4) ? 0 : -1);
            state.mExactTrueOrWildcardFalse = *reinterpret_cast<const bool*>(msg + 5);
            strncpy(&(state.mName[0]), reinterpret_cast<const char*>(msg + 6), sizeof(state.mName));
            state.mState = *reinterpret_cast<const int32_t*>(msg + 40);

            if (msgSize > 44) // UDPLightControlV01 is 44 bytes long
            {
                state.mPosValid = *reinterpret_cast<const bool*>(msg + 45);
                state.mDirValid = *reinterpret_cast<const bool*>(msg + 46);
                state.mOriValid = *reinterpret_cast<const bool*>(msg + 47);
                state.mIntensityValid = *reinterpret_cast<const bool*>(msg + 48);
                state.mColorValid = *reinterpret_cast<const bool*>(msg + 49);

                state.mPos[0] = *reinterpret_cast<const float*>(msg + 52);
                state.mPos[1] = *reinterpret_cast<const float*>(msg + 56);
                state.mPos[2] = *reinterpret_cast<const float*>(msg + 60);

                state.mDir[0] = *reinterpret_cast<const float*>(msg + 64);
                state.mDir[1] = *reinterpret_cast<const float*>(msg + 68);
                state.mDir[2] = *reinterpret_cast<const float*>(msg + 72);

                state.mOri[0] = *reinterpret_cast<const float*>(msg + 76);
                state.mOri[1] = *reinterpret_cast<const float*>(msg + 80);
                state.mOri[2] = *reinterpret_cast<const float*>(msg + 84);

                state.mIntensity = *reinterpret_cast<const float*>(msg + 88);

                long tmpCol = *reinterpret_cast<const long*>(msg + 92);
                state.mColor = (tmpCol << 16) & 0xFF0000;
                tmpCol = *reinterpret_cast<const long*>(msg + 93);
                state.mColor += (tmpCol << 8) & 0xFF00;
                tmpCol = *reinterpret_cast<const long*>(msg + 94);
                state.mColor += tmpCol & 0xFF;
                state.mSlotID = (*reinterpret_cast<const bool*>(msg + 4) ? *reinterpret_cast<const int8_T*>(msg + 95): -1);
            }

            simulinkLightEvents.push(state);
            break;
        }
        case 120: // = RFPRO_SET_ANIM_VISIBILITY
        {
          AnimationActionV01 action;
          action.mCommand = ACMD_SETINSTANCEVISIBILITY;
          action.mSetInstance.mVisible = *reinterpret_cast<const bool*>(msg + 4);
          strncpy(action.mSetInstance.mParent, reinterpret_cast<const char*>(msg + 8), sizeof(action.mSetInstance.mParent));
          strncpy(action.mSetInstance.mInstance, reinterpret_cast<const char*>(msg + 72), sizeof(action.mSetInstance.mInstance));
          simulinkAnimationCommands.push(action);
        }
        case 150: //  = RFPRO_MOTION_PLATFORM_INFO 
        {
            wantsMotionStates = true;
            
            // This is a definite candidate for a single memcopy call
            simulinkMotionState.mET = *reinterpret_cast<const double*>(msg + 4);

            simulinkMotionState.mPos[0] = *reinterpret_cast<const double*>(msg + 12);
            simulinkMotionState.mPos[1] = *reinterpret_cast<const double*>(msg + 20);
            simulinkMotionState.mPos[2] = *reinterpret_cast<const double*>(msg + 28);

            simulinkMotionState.mOri[0][0] = *reinterpret_cast<const double*>(msg + 36);
            simulinkMotionState.mOri[0][1] = *reinterpret_cast<const double*>(msg + 44);
            simulinkMotionState.mOri[0][2] = *reinterpret_cast<const double*>(msg + 52);
            simulinkMotionState.mOri[1][0] = *reinterpret_cast<const double*>(msg + 60);
            simulinkMotionState.mOri[1][1] = *reinterpret_cast<const double*>(msg + 68);
            simulinkMotionState.mOri[1][2] = *reinterpret_cast<const double*>(msg + 76);
            simulinkMotionState.mOri[2][0] = *reinterpret_cast<const double*>(msg + 84);
            simulinkMotionState.mOri[2][1] = *reinterpret_cast<const double*>(msg + 92);
            simulinkMotionState.mOri[2][2] = *reinterpret_cast<const double*>(msg + 100);

            simulinkMotionState.mVel[0] = *reinterpret_cast<const double*>(msg + 108);
            simulinkMotionState.mVel[1] = *reinterpret_cast<const double*>(msg + 116);
            simulinkMotionState.mVel[2] = *reinterpret_cast<const double*>(msg + 124);

            simulinkMotionState.mRot[0] = *reinterpret_cast<const double*>(msg + 132);
            simulinkMotionState.mRot[1] = *reinterpret_cast<const double*>(msg + 140);
            simulinkMotionState.mRot[2] = *reinterpret_cast<const double*>(msg + 148);

            break;
        }
        case 151: //  = RFPRO_HEAD_TRACKING_INFO
        {
          // This is a definite candidate for a single memcopy call
          simulinkHeadMotionInfo.mET = *reinterpret_cast<const double*>(msg + 4);
          simulinkHeadMotionInfo.mPos.x = *reinterpret_cast<const double*>(msg + 12);
          simulinkHeadMotionInfo.mPos.y = *reinterpret_cast<const double*>(msg + 20); 
          simulinkHeadMotionInfo.mPos.z = *reinterpret_cast<const double*>(msg + 28);

          simulinkHeadMotionInfo.mOri[0][0] = *reinterpret_cast<const double*>(msg + 36);
          simulinkHeadMotionInfo.mOri[0][1] = *reinterpret_cast<const double*>(msg + 44);
          simulinkHeadMotionInfo.mOri[0][2] = *reinterpret_cast<const double*>(msg + 52);
          simulinkHeadMotionInfo.mOri[1][0] = *reinterpret_cast<const double*>(msg + 60);
          simulinkHeadMotionInfo.mOri[1][1] = *reinterpret_cast<const double*>(msg + 68);
          simulinkHeadMotionInfo.mOri[1][2] = *reinterpret_cast<const double*>(msg + 76);
          simulinkHeadMotionInfo.mOri[2][0] = *reinterpret_cast<const double*>(msg + 84);
          simulinkHeadMotionInfo.mOri[2][1] = *reinterpret_cast<const double*>(msg + 92);
          simulinkHeadMotionInfo.mOri[2][2] = *reinterpret_cast<const double*>(msg + 100);

          simulinkHeadMotionInfo.mVel.x = *reinterpret_cast<const double*>(msg + 108);
          simulinkHeadMotionInfo.mVel.y = *reinterpret_cast<const double*>(msg + 116);
          simulinkHeadMotionInfo.mVel.z = *reinterpret_cast<const double*>(msg + 124);

          simulinkHeadMotionInfo.mRot.x = *reinterpret_cast<const double*>(msg + 132);
          simulinkHeadMotionInfo.mRot.y = *reinterpret_cast<const double*>(msg + 140);
          simulinkHeadMotionInfo.mRot.z = *reinterpret_cast<const double*>(msg + 148);
          break;
        }
        case 159: //  = RFPRO_REMOVE_ADDITIONAL_VEHICLE
        {
            // No option to remove vehicles from a Simulink model at present
            break;
        }
        case 160: //  = RFPRO_ADD_ADDITIONAL_VEHICLE
        {
            // Send registration details from UDPNewVehicleRegistrationV01 to map
            NewVehicleDataV01 newVeh;
            memset(&newVeh, 0, sizeof(NewVehicleDataV01));
            strncpy(reinterpret_cast<char *>((newVeh.mVehFile)), reinterpret_cast<const char*>(msg + 4), sizeof(newVeh.mVehFile));
            strncpy(reinterpret_cast<char *>((newVeh.mDriverName)), reinterpret_cast<const char*>(msg + 36), sizeof(newVeh.mDriverName));
            strncpy(reinterpret_cast<char *>((newVeh.mSkin)), reinterpret_cast<const char*>(msg + 76), sizeof(newVeh.mSkin));
            strncpy(reinterpret_cast<char *>((newVeh.mHelmet)), reinterpret_cast<const char*>(msg + 108), sizeof(newVeh.mHelmet));
            simulinkNewVehicles.insert(std::make_pair(*reinterpret_cast<const uint32_t*>(msg + 140), newVeh));
            break;
        }
        case 161: //  = RFPRO_UPDATE_ADDITIONAL_VEHICLE
        {
            // Update vehicle data from UDPVehicleStateV01
            // Note: In hosted - vehicle ID in msg is Simulink ID, it's not updated to the rFpro ID
            // If the rFpro ID is needed in the model, a new map of local to rFpro IDs is required
            int32_t SLID = *reinterpret_cast<const int32_t*>(msg + 4);
            // Send update details to map
            auto Veh_iter = simulinkVehicleUpdates.find(SLID);
            if (Veh_iter != simulinkVehicleUpdates.end())
              {
              VehicleStateV01* Vehicle = &(Veh_iter->second);
              // Retain the (positive) rFpro vehicle ID
              int rFproVehID = abs(Vehicle->mID);

              Vehicle->Clear();
              Vehicle->mID = rFproVehID; // positive indicates update to do
                
              Vehicle->mET = *reinterpret_cast<const double*>(msg + 8);

              Vehicle->mPos.x = *reinterpret_cast<const double*>(msg + 40);
              Vehicle->mPos.y = *reinterpret_cast<const double*>(msg + 48);
              Vehicle->mPos.z = *reinterpret_cast<const double*>(msg + 56);

              Vehicle->mCollidableVsAI = *reinterpret_cast<const bool*>(msg + 17);
              Vehicle->mCollidableVsPlayer = *reinterpret_cast<const bool*>(msg + 17);
              Vehicle->mCollidableVsStatic = *reinterpret_cast<const bool*>(msg + 17);

              Vehicle->mTransparentProximity = *reinterpret_cast<const double*>(msg + 20);
              Vehicle->mManualTransparency = *reinterpret_cast<const double*>(msg + 28);

              Vehicle->mPosYValid = *reinterpret_cast<const bool*>(msg + 36);
              Vehicle->mOriValid = *reinterpret_cast<const bool*>(msg + 37);
              Vehicle->mHeadingValid = *reinterpret_cast<const bool*>(msg + 38);

              Vehicle->mOri[0][0] = *reinterpret_cast<const double*>(msg + 64);
              Vehicle->mOri[0][1] = *reinterpret_cast<const double*>(msg + 72);
              Vehicle->mOri[0][2] = *reinterpret_cast<const double*>(msg + 80);
              Vehicle->mOri[1][0] = *reinterpret_cast<const double*>(msg + 88);
              Vehicle->mOri[1][1] = *reinterpret_cast<const double*>(msg + 96);
              Vehicle->mOri[1][2] = *reinterpret_cast<const double*>(msg + 104);
              Vehicle->mOri[2][0] = *reinterpret_cast<const double*>(msg + 112);
              Vehicle->mOri[2][1] = *reinterpret_cast<const double*>(msg + 120);
              Vehicle->mOri[2][2] = *reinterpret_cast<const double*>(msg + 128);

              Vehicle->mHeading = *reinterpret_cast<const double*>(msg + 136);

              Vehicle->mVelValid = *reinterpret_cast<const bool*>(msg + 144);
              Vehicle->mRotValid = *reinterpret_cast<const bool*>(msg + 145);

              Vehicle->mVel.x =  *reinterpret_cast<const double*>(msg + 148);
              Vehicle->mVel.y =  *reinterpret_cast<const double*>(msg + 156);
              Vehicle->mVel.z =  *reinterpret_cast<const double*>(msg + 164);

              Vehicle->mRot.x =  *reinterpret_cast<const double*>(msg + 172);
              Vehicle->mRot.y =  *reinterpret_cast<const double*>(msg + 180);
              Vehicle->mRot.z =  *reinterpret_cast<const double*>(msg + 188);

              Vehicle->mGearValid = *reinterpret_cast<const bool*>(msg + 196);
              Vehicle->mRPMValid = *reinterpret_cast<const bool*>(msg + 197);
              Vehicle->mInputsValid = *reinterpret_cast<const bool*>(msg + 198);

              Vehicle->mGear = *reinterpret_cast<const int32_t*>(msg + 200);
              Vehicle->mEngineRPM = *reinterpret_cast<const double*>(msg + 204);
              Vehicle->mThrottle = *reinterpret_cast<const double*>(msg + 212); 
              Vehicle->mBrake =  *reinterpret_cast<const double*>(msg + 220);
              Vehicle->mSteering = *reinterpret_cast<const double*>(msg + 228);

              Vehicle->mWheelRotationsValid = *reinterpret_cast<const bool*>(msg + 236);
              Vehicle->mWheelBrakeTempsValid = *reinterpret_cast<const bool*>(msg + 237);
              Vehicle->mWheelYLocationsValid = *reinterpret_cast<const bool*>(msg + 238);

              // WheelStateV01
              Vehicle->mWheel[0].mRotation =  *reinterpret_cast<const double*>(msg + 240);
              Vehicle->mWheel[0].mBrakeTemp =  *reinterpret_cast<const double*>(msg + 248);
              Vehicle->mWheel[0].mYLocation =  *reinterpret_cast<const double*>(msg + 256);

              Vehicle->mWheel[1].mRotation =  *reinterpret_cast<const double*>(msg + 264);
              Vehicle->mWheel[1].mBrakeTemp = *reinterpret_cast<const double*>(msg + 272);
              Vehicle->mWheel[1].mYLocation = *reinterpret_cast<const double*>(msg + 280);

              Vehicle->mWheel[2].mRotation =  *reinterpret_cast<const double*>(msg + 288);
              Vehicle->mWheel[2].mBrakeTemp = *reinterpret_cast<const double*>(msg + 296);
              Vehicle->mWheel[2].mYLocation = *reinterpret_cast<const double*>(msg + 304);

              Vehicle->mWheel[3].mRotation =  *reinterpret_cast<const double*>(msg + 312);
              Vehicle->mWheel[3].mBrakeTemp = *reinterpret_cast<const double*>(msg + 320);
              Vehicle->mWheel[3].mYLocation = *reinterpret_cast<const double*>(msg + 328);

              Vehicle->mBrakelights = *reinterpret_cast<const char*>(msg + 336);
              Vehicle->mHeadlights = *reinterpret_cast<const char*>(msg + 337);
            }
            break;
        }
        // 162 = RFPRO_NEW_VEHICLE_ID - not used in hosted, Simulink vehicle # used
        case 163: //  = RFPRO_EXT_RESET_VEHICLE
        {
            // Not part of the library at present.
            break;
        }
        case 170: //  = RFPRO_FMOD_REGISTER_PARAM_UPDATER
        {
            // Not part of the library at present.
            break;
        }
        case 172: //  = RFPRO_FMOD_UPDATE_PARAM
        {
            char paramName[50];
            float paramValue;
            int idx = 0;
            paramValue = *reinterpret_cast<const float*>(msg + 8);
            strncpy(&(paramName[0]), reinterpret_cast<const char*>(msg + 12), 50);
            
            // TODO this code can be tidied up, perhaps use a map too?
            for (std::vector<std::string>::iterator it = FMODParameterNames.begin(); it != FMODParameterNames.end(); ++it)
            {
              if (strcmp((*it).c_str(), paramName) == 0)
              {
                FMODParameterValues[idx] = paramValue;
              }
            ++idx;
            }
            break;
        }
        case 180: //  = RFPRO_CAMERA_CONTROL_INFO
        {
            uint32_t buffer;
            buffer = *reinterpret_cast<const int*>(msg + 4);
            if (buffer != simulinkCameraControlInfo.mID)
            {
                simulinkCameraControlInfo.mID = buffer;
                simulinkCameraControlInfoUpdated = true;
            }
            
            buffer = *reinterpret_cast<const int*>(msg + 8);
            if (buffer != simulinkCameraControlInfo.mCameraType)
            {
                simulinkCameraControlInfo.mCameraType = buffer;
                simulinkCameraControlInfoUpdated = true;
            }
            break;
        }
        case 190: //  = RFPRO_BOUNDING_BOX_CONFIG
        {
            // Hosted mode sets up the list and IDs during model startup
            // Bounding box structures are defined in rFproTypes.h
            const UDPBoundBoxListV02 header = *reinterpret_cast<const UDPBoundBoxListV02*>(msg);
            const UDPBoundBoxConfigItemV01* config = reinterpret_cast<UDPBoundBoxConfigItemV01*>((char*)msg + sizeof(UDPBoundBoxListV02));
            std::vector<UDPBoundBoxConfigItemV01> config_items;
            config_items.assign(config, config + header.mLength);
            
            simulinkBBControl.add_new_group(header.mMeshGroup, config_items);
            break;
        }
        default:
            break;
    }
}

#ifdef SENSOR
double mediaTime;
const SensorImageInfoV01 *sensorImageInfo;
int sensorImageFormat = SENSOR_FORMAT_NONE;
int overrideViewMatrix = 0;
float overridenViewMatrix[4][4];
#endif

#ifdef ADD_PHYSICS
PhysicsAdditiveV01 simulinkPhysicsAdditive;
#endif

#ifdef COMPUTE_DIFF
DifferentialInitV01 simulinkDifferentialInit;
DifferentialInputV01 simulinkDifferentialInput;
DifferentialOutputV01 simulinkDifferentialOutput;
#endif

#ifdef COMPUTE_TYRE
ISOTyreInitV01 simulinkISOTyreInit;
ISOTyreInputV01 simulinkISOTyreInput;
ISOTyreOutputV01 simulinkISOTyreOutput;
#endif

// rFProPlugin class
rFProPlugin::rFProPlugin()
{
}

rFProPlugin::~rFProPlugin() 
{
}

void rFProPlugin::Startup(long version)
{
  printf("[Simulink] Starting model: ");
  MODELFCN(_initialize)();
  printf("Done.\n");
}

void rFProPlugin::Shutdown()
{
  printf("[Simulink] Stopping model: ");
  MODELFCN(_terminate)();
  printf("Done\n");
}

void rFProPlugin::StartSession()
{
  // Called once per track load, when track is loaded

  /*
  * Initialise all outputs as invalid, then Simulink can
  * optionally make parts valid as it writes to them.
  */
  simulinkOutput.mSkipInternalPhysics = true;
  simulinkOutput.mUseInternalPhysics = true;
  simulinkOutput.mAccelValid = false;
  simulinkOutput.mWheelPosValid = false;       // whether wheel positions are valid
  simulinkOutput.mWheelOriValid = 0;         // 0=invalid, 1=spindle axis only (stored in mOri[0]; rFpro will spin wheel), 2=full orientation (rFpro will *not* spin wheel other than for graphical extrapolation)
  simulinkOutput.mTireParamsValid = false;       // slip angle, slip ratio, and tire load
  simulinkOutput.mWheelRotationValid = false;    // whether rotations are valid
  simulinkOutput.mWheelBrakeTempValid = false;     // whether brake temps are valid
  simulinkOutput.mWheelTreadTempValid = false;     // whether tread temps are valid
  simulinkOutput.mWheelAirTempValid = false;     // whether air temps are valid

  // engine, input, gear
  simulinkOutput.mGearValid = false;         // whether gear is valid
  simulinkOutput.mRPMValid = false;          // whether engine RPM is valid
  simulinkOutput.mInputsValid = false;         // whether throttle, brake, and steering are valid
}

void rFProPlugin::EndSession()
{
  // Called once per track load, when track is UNloaded
}

void rFProPlugin::EnterRealtime()
{
  // Called each time we enter "Real time"
}

void rFProPlugin::ExitRealtime()
{
  // Called each time we exit "Real time"
}

void rFProPlugin::InitVehicle(const VehicleAndPhysicsV01 &data)
{
    // InitVehicle is only called for the main physics vehicle
    // => the ID will be correct
    sl_TelemetryOpponents.setEgoID(data.mID);
}

bool rFProPlugin::WantsGraphicsUpdates() { return wantsTelemetryUpdates; }

void rFProPlugin::UpdateGraphics( const GraphicsInfoV02 &info )
{
   currentViewedSlot = info.mID;
}

long rFProPlugin::WantsTelemetryUpdates()
{
  return wantsTelemetryUpdates ? 2 : 0;
}

void rFProPlugin::UpdateTelemetry( const TelemInfoV01 &info )
{
  if (info.mID == currentViewedSlot) memcpy(&simulinkInfo, &info, sizeof(TelemInfoV01));
  // Note: the added telemetry should be cleared after a model step, 
  //       => either RunPhysics or SetSimulationTiming
  if (sl_TelemetryOpponents.active())
  {
      if (info.mID == sl_TelemetryOpponents.getEgoID())
          sl_TelemetryOpponents.setEgoPosition(info.mPos);
      else
          sl_TelemetryOpponents.addTelemetry(info);
  }
}

long rFProPlugin::WantsTyreContactUpdates( long maxNumPoints, TyreContactPointDefnV01 *points )
{
  if (numContactPoints <= maxNumPoints)
  {
    memcpy(points, &simulinkTyreContact, (sizeof TyreContactPointDefnV01) * numContactPoints);
    return wantsTyreContactUpdates ? numContactPoints : 0;
  }
  else return 0;
}

void rFProPlugin::UpdateTyreContact( const TyreContactInputV01 &input )
{
  FILE *fid;
  fid = fopen("UpdateTyreContact.txt","a");
  fprintf(fid,"UpdateTyreContact SHOULD NEVER GET CALLED\n");
  fprintf(fid,"heights %f %f %f %f\n", input.mContactPatch[0].mHeight, input.mContactPatch[1].mHeight, input.mContactPatch[2].mHeight, input.mContactPatch[3].mHeight);
  fclose(fid);
}

bool rFProPlugin::WantsHDTyreContactUpdates( TyreDefinitionInputV01 &tyreDef )
{
  memcpy(&tyreDef, &simulinkTyreDef, sizeof tyreDef);
  return wantsTyreContactUpdates;
}

void rFProPlugin::UpdateHDTyreContact( HDContactPatchV01 contactPatch[4] )
{
  memcpy(&simulinkTyreContactInput[0], &contactPatch[0], sizeof(HDContactPatchV01)*4);
}

void rFProPlugin::SetVehicleLocation( StartingVehicleLocationV01 &data )
{
  startLocn = data;
  receivedStartLocation = true;
}

bool rFProPlugin::WantsMotionStates()
{
  return wantsMotionStates;
} 

void rFProPlugin::UpdateMotionState(MotionPlatformV01 &motionInfo)
{
  memcpy(&motionInfo, &simulinkMotionState, sizeof(MotionPlatformV01));
}

// Set the FMOD sound parameters
// This tells the plugin how many params FMOD understands, and what their names are
void rFProPlugin::FMOD_ValidParameters( long numParams, const char **paramNames )
{
  FMODParameterNames.resize(numParams);
  FMODParameterValues.resize(numParams);
  
  std::vector<std::string> nameList(paramNames, paramNames + numParams);
  for (int idx = 0; idx < numParams; ++idx)
  {
    FMODParameterNames[idx] =  nameList[idx];
    FMODParameterValues[idx] = 0;
  }
}

// Given the array (as defined in FMOD_ValidParameters) update it with the latest values
void rFProPlugin::FMOD_UpdateNew( long numParams, float *paramValues )
{
  for (int idx = 0; idx < numParams; ++idx)
  {
    paramValues[idx] = FMODParameterValues[idx];
  }
}

// Special Actor functions
// Specify the plugin to be used
void rFProPlugin::WantsDataModel(rFpro::DataModelProvider * provider)
{
  simulinkTrafficInterface = static_cast<rFpro::TrafficDataModelV02 *>(provider->GetDataModelConnection(rFpro::Traffic, 2));
  simulinkTrafficInterfaceV03 = dynamic_cast<rFpro::TrafficDataModelV03 *>(provider->GetDataModelConnection(rFpro::Traffic, 3));
  simulinkTrafficInterfaceV04 = dynamic_cast<rFpro::TrafficDataModelV04 *>(provider->GetDataModelConnection(rFpro::Traffic, 4));
}

// Animation commands
bool rFProPlugin::GetAnimation(AnimationActionV01& info)
{
  if (!simulinkAnimationCommands.empty())
  {
    info = simulinkAnimationCommands.front();
    simulinkAnimationCommands.pop();
    return true;
  }
  return false;
}

// Add vehicle functions
// ---------------------
bool rFProPlugin::WantsToAddVehicle( const long id, NewVehicleDataV01 &data )
{
  // We only register a new vehicle if it is in the new vehicles map container
  if (simulinkNewVehicles.size() == 0) {return false;}
  
  // Take first value in new vehicles map, load the data
  auto newVeh = simulinkNewVehicles.begin();
  memcpy(&data, &(newVeh->second), sizeof(NewVehicleDataV01));

  // generate a new map entry with the simulink ID as key and vehicle state as value.
  // The mID field in the vehicle state is set -ve, indicating no update data currently
  VehicleStateV01 newVehState;
  newVehState.mID = -id;
  simulinkVehicleUpdates.insert(std::make_pair(newVeh->first, newVehState));

  // With vehicle registered, we can remove it from the new vehicles container
  simulinkNewVehicles.erase(newVeh);
  return true;
}

bool rFProPlugin::GetVehicleState( VehicleStateV01 &data )
{
  // Find an entry with a +ve mID field and use that for the update
  for (auto iter = simulinkVehicleUpdates.begin(); iter != simulinkVehicleUpdates.end(); ++iter)
  {
    if (iter->second.mID > 0)
    {
      memcpy(&data, &(iter->second), sizeof(VehicleStateV01));
      iter->second.mID = -data.mID;
      return true;
    }
  }
  // No update records have +ve mID fields so we are done for now
  return false;
}


// Lighting functions
// ------------------
// whether plugin wants light info and control
bool rFProPlugin::WantsLightControl()
{
  return simulinkSceneLightsUpdated || !simulinkLightEvents.empty();
}

// return true for rFpro to read new states
bool rFProPlugin::UpdateSceneLights( SceneLightStatesV01 &state )
{
  if (simulinkSceneLightsUpdated)
  {
    state = simulinkSceneLights;
    simulinkSceneLightsUpdated = false;
    return true;
  }
  else
  {
    return false;
  }
}

bool rFProPlugin::GetLights( LightControlV01 &lightControl )
{
  if (!simulinkLightEvents.empty())
  {
    memcpy(&lightControl, &(simulinkLightEvents.front()), sizeof(LightControlV01));
    simulinkLightEvents.pop();
    return true;
  }
  else
  {
    return false;
  }
}

// Object Proximity Functions
// --------------------------
long rFProPlugin::WantsObjectProximityUpdates( long maxNumPoints, ProximityQueryPointDefnV01 *points )
{
  const long simNum = static_cast<long>(simulinkProximityDef.size());
  const long num = maxNumPoints < simNum ? maxNumPoints : simNum;
  if (num < simNum)
      printf("[SL Library] Limiting number of proximity points to %d.\n", num);

  for (auto i = 0; i < num; ++i)
  {
    points[i].mPos.x = simulinkProximityDef[i].mPos.x;
    points[i].mPos.y = simulinkProximityDef[i].mPos.y;
    points[i].mPos.z = simulinkProximityDef[i].mPos.z;
    
    points[i].mRay.x = simulinkProximityDef[i].mRay.x;
    points[i].mRay.y = simulinkProximityDef[i].mRay.y;
    points[i].mRay.z = simulinkProximityDef[i].mRay.z;
  }
  return num;
}

void rFProPlugin::UpdateObjectProximity( const ProximityQueryInputV01 &input )
{
  simulinkProximityData.assign(input.mProximityQueryPoint, input.mProximityQueryPoint + input.mNumPoints);
}

// Head Tracking Functions
// -----------------------
bool rFProPlugin::WantsToProvideHeadTracking()
{
  return simulinkHeadTrackingActive;
}

void rFProPlugin::UpdateHeadTrackingState(MotionPlatformV01 &motionInfo)
{
  memcpy(&motionInfo, &simulinkHeadMotionInfo, sizeof(MotionPlatformV01));
}

// Camera Function
// ---------------
unsigned char rFProPlugin::WantsToViewVehicle(CameraControlInfoV01& camControl)
{
  if (simulinkCameraControlInfoUpdated)
  {
    camControl.mID = simulinkCameraControlInfo.mID;
    camControl.mCameraType = simulinkCameraControlInfo.mCameraType;
    simulinkCameraControlInfoUpdated = false;
    return 1;
  }
  else
  {
    return 0;
  }
}

// Bounding Boxes
// --------------
bool rFProPlugin::WantsBoundingBoxes()
{
    return simulinkWantsBoundingBoxes;
}

bool rFProPlugin::WantsBoundingBox(const QueryInstance& instance)
{
  // Just add all the bounding boxes to the list reference since the
  // model could add/remove instances during simulation
  simulinkBBControl.add_new_mesh(instance, false); // ignore children by default
  return true;
}

void rFProPlugin::SetBoundingBoxes(long num, const BoundingBoxV01* boundingBoxArray)
{
    // Loop through all the bounding box groups and generate vectors
    // for each in the simulinkBBControl object for the blocks to reference
    std::vector<uint16_t> groups = simulinkBBControl.get_group_ids();
    for (uint16_t idx = 0; idx < groups.size(); ++idx)
    {
        simulinkBBControl.generate_group_vector(groups[idx], num, boundingBoxArray);
    }
}


#ifdef RUN_PHYSICS
void rFProPlugin::RunPhysics( long id, PhysicsInputV01 &input )
{
  memcpy(&simulinkInput, &input, sizeof(PhysicsInputV01));
  
  // Opponent telemetry data needs to be in distance to ego order
  sl_TelemetryOpponents.prepare();
  
  while (simulinkOutput.mET < simulinkInput.mET)
  {
    MODELFCN(_step)();
  }
}

void rFProPlugin::GetPhysicsState( long id, PhysicsOutputV01 &output )
{
  // All RFPRO_KEY_PRESS msgs should be done in one frame
  // leaving only invalid key press strings - so wipe these
  if (!simulinkKeyPresses.empty())
  {
    simulinkKeyPresses.clear();
  }
  
  memcpy(&output, &simulinkOutput, sizeof(PhysicsOutputV01));
}

bool rFProPlugin::ForceFeedback( double &forceValue )
{
  forceValue = simulinkOutputForceFeedbackValue;
  return simulinkOutputForceFeedbackValid;
}

bool rFProPlugin::CheckHWControl(const char * const controlName, double &fRetVal)
{
  // Note: invalid strings are wiped at start of GetPhysicsState
  if (!simulinkKeyPresses.empty())
  {
    auto it = simulinkKeyPresses.find(controlName);
    if (it != simulinkKeyPresses.end())
    {
      fRetVal = it->second;
      simulinkKeyPresses.erase(controlName);
      return true;
    }
  }
  return false;
}

#endif


#ifdef TRAFFIC
void rFProPlugin::SetSimulationTiming(const ThreadTimingV01 &info)
{
  // Opponent telemetry data needs to be in distance to ego order
  sl_TelemetryOpponents.prepare();
    
  while (simulinkModelTime < info.mElapsedTime)
  {
    MODELFCN(_step)();
    if (simulinkModelTime < 0)
        break; // If we are not getting updates break out to prevent inf loop
  }
}
#endif


#ifdef SENSOR
bool rFProPlugin::GetSensorImageSettings( SensorImageSettingsV01 &settings )
{
  settings.mFormat = sensorImageFormat;
  return true;
}

void rFProPlugin::SetMultimediaTiming(const ThreadTimingV01 &info)
{
  mediaTime = info.mElapsedTime;
}

void rFProPlugin::SetSensorImage( const SensorImageInfoV01 &info )
{
  sensorImageInfo = &info;

  // Call Simulink Model
  MODELFCN(_step)();
}

long rFProPlugin::GetPreDrawParams( void *pData )
{
  if (overrideViewMatrix == 0)
  {
    return 0;
  }
  
  ImageWarpDynDataV01 *data = (ImageWarpDynDataV01 *)pData;
  
  memcpy(data->mViewMatrix, overridenViewMatrix, sizeof(float)*16);
  
  data->mAppUseViewMatrix = true;
  data->mAppViewMatrixPre = false;
  
  return 1;
}
#endif

#ifdef ADD_PHYSICS
bool rFProPlugin::AddPhysics( const long id, double et, PhysicsAdditiveV01 &add )
{
  if (id != 0) return false;

  // Call Simulink Model
  MODELFCN(_step)();

  // Copy data from Simulink to rFpro return value
  memcpy(&add, &simulinkPhysicsAdditive, sizeof(PhysicsAdditiveV01));

  return true;
}
#endif

#ifdef COMPUTE_DIFF
void rFProPlugin::InitDifferential( long id, DifferentialInitV01 &init )
{
  if (id != 0) return;

  // Copy data from rFpro to Simulink
  memcpy(&simulinkDifferentialInit, &init, sizeof(DifferentialInitV01));
}

bool rFProPlugin::ComputeDifferentialTransfer( long id, DifferentialInputV01 &input, DifferentialOutputV01 &output )
{
  if (id != 0) return false;
  
  // Copy data from rFpro to Simulink
  memcpy(&simulinkDifferentialInput, &input, sizeof(DifferentialInputV01));

  // Call Simulink Model
  MODELFCN(_step)();

  // Copy data from Simulink to rFpro return value
  memcpy(&output, &simulinkDifferentialOutput, sizeof(DifferentialOutputV01));

  return true;
}
#endif

#ifdef COMPUTE_TYRE
void rFProPlugin::InitISOTyre( long id, ISOTyreInitV01 &init )
{
  if (id != 0) return;

  // Copy data from rFpro to Simulink
  memcpy(&simulinkISOTyreInit, &init, sizeof(ISOTyreInitV01));
}

bool rFProPlugin::ComputeISOTyreForces( long id, ISOTyreInputV01 &input, ISOTyreOutputV01 &output )
{
  if (id != 0) return false;
  
  // Copy data from rFpro to Simulink
  memcpy(&simulinkISOTyreInput, &input, sizeof(ISOTyreInputV01));

  // Call Simulink Model
  MODELFCN(_step)();

  // Copy data from Simulink to rFpro return value
  memcpy(&output, &simulinkISOTyreOutput, sizeof(ISOTyreOutputV01));

  return true;
}
#endif
