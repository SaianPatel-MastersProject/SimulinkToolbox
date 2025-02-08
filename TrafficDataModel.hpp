//
// Copyright (c) 2017-2020 rFpro Limited, All Rights Reserved.
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

#include "InternalsPluginExtras.hpp"

// rF currently uses 4-byte packing ... whatever the current packing is will
// be restored at the end of this include with another #pragma.
#pragma pack( push, 4 )

namespace rFpro
{
    struct PedestrianStateV01
    {
        double mET;
        char mName[24];
        TelemVect3 mPos;
        double mHeading;
        bool mHeadingValid;
        char mHeightSetting; // 0 - auto @ ground height, 1 = rel to ground, 2 = abs height
        char spareFlags[7];
        double spare[3];

        PedestrianStateV01()
            : mHeadingValid(false), mHeightSetting(0)
        {
            spareFlags[0] = 0;
            spareFlags[1] = 0;
            spareFlags[2] = 0;
            spareFlags[3] = 0;
            spareFlags[4] = 0;
            spareFlags[5] = 0;
            spareFlags[6] = 0;
            spare[0] = 0;
            spare[1] = 0;
            spare[2] = 0;
        }
    };

    struct SpecialActorStateV01 : public PedestrianStateV01
    {
        char mType[24];
        AnimationDataSpecialV01::Type mSpecialType;

        SpecialActorStateV01()
            :mSpecialType( AnimationDataSpecialV01::PEDESTRIAN )
        {
            mType[0] = 0;
        }
    };

    struct SpecialActorStateV02 : public SpecialActorStateV01
    {
        TelemVect3 mOri[3];
        bool mOriValid;

        SpecialActorStateV02()
            : mOriValid(false)
        {
        }
    };

    struct SpecialActorStateV03 : public SpecialActorStateV02
    {
        bool mManual;  // false: automatic animation control based on the actor's speed  true: manual animation control
        char mAnimationState[24];  // Name of the animation state
        bool mOneShot;  // false: play the animation in loop  true: stay at the last frame when reaching the end of animation
        bool mUseDirectAnimationFrame;  // false: animation frame is automatically calculated using mPlaybackRate  true: animation frame is manually specified using mDirectAnimationFrame
        float mPlaybackRate;  // 1.0: normal speed  0.0: pause  <0.0: reverse
        int32_t mDirectAnimationFrame;  // Animation frame

        SpecialActorStateV03()
            : mManual(false), mAnimationState(), mOneShot(false),
            mUseDirectAnimationFrame(false), mPlaybackRate(1.0f),
            mDirectAnimationFrame(0)
        {
        }
    };


    class TrafficDataModelV01 : public SimControlDataModel
    {
    public:
        virtual void SetTrafficLight(const char *name, const char *state) = 0;
        virtual void SetPedestrian(const PedestrianStateV01 *state) = 0;
    };


    class TrafficDataModelV02 : public TrafficDataModelV01
    {
    public:
        virtual void LoadPedestrians(const char *actorFile, const size_t quantity, const char *typeName = nullptr) = 0;
        virtual void LoadSpecialActors(const AnimationDataSpecialV01::Type specialType, const char *specialActorFile, const size_t quantity, const char *typeName = nullptr) = 0;
        virtual void SetSpecialActor(const SpecialActorStateV01 *state) = 0;

        virtual void LoadVehicles(const char *vehicleFile, const size_t quantity) = 0;
        virtual void AllowLoadingMidSession(const bool allow) = 0;
    };


    class TrafficDataModelV03 : public TrafficDataModelV02
    {
    public:
        virtual void SetSpecialActor(const SpecialActorStateV02 *state) = 0;
    };


    class TrafficDataModelV04 : public TrafficDataModelV03
    {
    public:
        virtual void SetSpecialActor(const SpecialActorStateV03* state) = 0;
    };
}

// See #pragma at top of file
#pragma pack( pop )
