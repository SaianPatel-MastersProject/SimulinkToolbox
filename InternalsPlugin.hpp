//
// Copyright (c) 2010-2022 rFpro Limited, All Rights Reserved.
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

#ifndef _INTERNALS_PLUGIN_HPP_
#define _INTERNALS_PLUGIN_HPP_

#include "PluginObjects.hpp"     // base class for plugin objects to derive from
#include "TelemVect3.hpp"


#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 
#endif
#include <windows.h>         // for HWND
#else
    typedef void* HWND;
#endif

#include <cmath>                 // for sqrt()
#include <cstring>               // for memset()
#include <cstdint>

// rF and plugins must agree on structure packing, so set it explicitly here ... whatever the current
// packing is will be restored at the end of this include with another #pragma.
#pragma pack( push, 4 )

//==========================================================================
// Version01 Structures
//==========================================================================

struct TelemQuat
{
  double w, x, y, z;

  // Convert this quaternion to a matrix
  void ConvertQuatToMat( TelemVect3 ori[3] ) const
  {
    const double x2 = x + x;
    const double xx = x * x2;
    const double y2 = y + y;
    const double yy = y * y2;
    const double z2 = z + z;
    const double zz = z * z2;
    const double xz = x * z2;
    const double xy = x * y2;
    const double wy = w * y2;
    const double wx = w * x2;
    const double wz = w * z2;
    const double yz = y * z2;
    ori[0][0] = (double) 1.0 - ( yy + zz );
    ori[0][1] = xy - wz;
    ori[0][2] = xz + wy;
    ori[1][0] = xy + wz;
    ori[1][1] = (double) 1.0 - ( xx + zz );
    ori[1][2] = yz - wx;
    ori[2][0] = xz - wy;
    ori[2][1] = yz + wx;
    ori[2][2] = (double) 1.0 - ( xx + yy );
  }

  // Convert a matrix to this quaternion
  void ConvertMatToQuat( const TelemVect3 ori[3] )
  {
    const double trace = ori[0][0] + ori[1][1] + ori[2][2] + (double) 1.0;
    if( trace > 0.0625f )
    {
      const double sqrtTrace = sqrt( trace );
      const double s = (double) 0.5 / sqrtTrace;
      w = (double) 0.5 * sqrtTrace;
      x = ( ori[2][1] - ori[1][2] ) * s;
      y = ( ori[0][2] - ori[2][0] ) * s;
      z = ( ori[1][0] - ori[0][1] ) * s;
    }
    else if( ( ori[0][0] > ori[1][1] ) && ( ori[0][0] > ori[2][2] ) )
    {
      const double sqrtTrace = sqrt( (double) 1.0 + ori[0][0] - ori[1][1] - ori[2][2] );
      const double s = (double) 0.5 / sqrtTrace;
      w = ( ori[2][1] - ori[1][2] ) * s;
      x = (double) 0.5 * sqrtTrace;
      y = ( ori[0][1] + ori[1][0] ) * s;
      z = ( ori[0][2] + ori[2][0] ) * s;
    }
    else if( ori[1][1] > ori[2][2] )
    {
      const double sqrtTrace = sqrt( (double) 1.0 + ori[1][1] - ori[0][0] - ori[2][2] );
      const double s = (double) 0.5 / sqrtTrace;
      w = ( ori[0][2] - ori[2][0] ) * s;
      x = ( ori[0][1] + ori[1][0] ) * s;
      y = (double) 0.5 * sqrtTrace;
      z = ( ori[1][2] + ori[2][1] ) * s;
    }
    else
    {
      const double sqrtTrace = sqrt( (double) 1.0 + ori[2][2] - ori[0][0] - ori[1][1] );
      const double s = (double) 0.5 / sqrtTrace;
      w = ( ori[1][0] - ori[0][1] ) * s;
      x = ( ori[0][2] + ori[2][0] ) * s;
      y = ( ori[1][2] + ori[2][1] ) * s;
      z = (double) 0.5 * sqrtTrace;
    }
  }
};

#ifndef MAX_LENGTH_TERRAIN_NAME
#define MAX_LENGTH_TERRAIN_NAME (16)
#endif

struct TelemWheelV01
{
  double mSuspensionDeflection;  // meters
  double mRideHeight;            // meters
  double mSuspForce;             // pushrod load in Newtons
  double mBrakeTemp;             // Celsius
  double mBrakePressure;         // currently 0.0-1.0, depending on driver input and brake balance; will convert to true brake pressure (kPa) in future

  double mRotation;              // radians/sec
  double mLateralPatchVel;       // lateral velocity at contact patch
  double mLongitudinalPatchVel;  // longitudinal velocity at contact patch
  double mLateralGroundVel;      // lateral velocity at contact patch
  double mLongitudinalGroundVel; // longitudinal velocity at contact patch
  double mCamber;                // radians (positive is left for left-side wheels, right for right-side wheels)
  double mLateralForce;          // Newtons
  double mLongitudinalForce;     // Newtons
  double mTireLoad;              // Newtons

  double mGripFract;             // an approximation of what fraction of the contact patch is sliding
  double mPressure;              // kPa (tire pressure)
  double mTemperature[3];        // Kelvin (subtract 273.15 to get Celsius), left/center/right (not to be confused with inside/center/outside!)
  double mWear;                  // wear (0.0-1.0, fraction of maximum) ... this is not necessarily proportional with grip loss
  char mTerrainName[MAX_LENGTH_TERRAIN_NAME]; // the material prefixes from the TDF file
  unsigned char mSurfaceType;    // 0=dry, 1=wet, 2=grass, 3=dirt, 4=gravel, 5=rumblestrip, 6=special, 7=runoff, 8=astroturf, 9=kerb, 10=pitlane, 11=metal, 12=concrete, 13=paint, 14-23=spare0-spare9
  bool mFlat;                    // whether tire is flat
  bool mDetached;                // whether wheel is detached
  unsigned char mStaticUndeflectedRadius; // tire radius in centimeters

  double mVerticalTireDeflection;// how much is tire deflected from its (speed-sensitive) radius
  double mWheelYLocation;        // wheel's y location relative to vehicle y location
  double mToe;                   // current toe angle w.r.t. the vehicle

  TelemVect3 mWorldPos;          // world position of wheel
  float mOri[2][3];              // For space reasons these are the first two rows of the orientation matrix in single-precision. You can get the third row by doing a cross product of the first two
  double mPitch;                 // angle difference around x-axis from spindle/hub to wheel (mOri is the wheel's orientation, so you would need to rotate by -mPitch if you need the spindle's orientation)

//  unsigned char mExpansion[ 0 ]; // for future use
};


// Telemetry is usually in rFpro axes or ISO axes.
// The rFpro axis system is as follows:
//    Axis   Linear      Angular
//     +x     Left       Pitch up
//     +y      Up      Yaw to right
//     +z   Backwards  Roll to right
// The ISO axis system is as follows:
//    Axis   Linear      Angular
//     +x   Forwards   Roll to right
//     +y     Left      Pitch down
//     +z      Up       Yaw to left
// Refer to the appendix of the Sim Control Plugin documentation for further
// information, including interpretation of orientation matrices.

struct TelemInfoV01
{
  // Time
  int32_t mID;                   // slot ID (note that it can be re-used in multiplayer after someone leaves)
  double mDeltaTime;             // time since last update (seconds)
  double mElapsedTime;           // game session time
  int32_t mLapNumber;            // current lap number
  double mLapStartET;            // time this lap was started
  char mVehicleName[64];         // current vehicle name
  char mTrackName[64];           // current track name

  // Position and derivatives
  TelemVect3 mPos;               // world position in meters
  TelemVect3 mLocalVel;          // velocity (meters/sec) in local vehicle coordinates
  TelemVect3 mLocalAccel;        // acceleration (meters/sec^2) in local vehicle coordinates

  // Orientation and derivatives
  TelemVect3 mOri[3];            // rows of orientation matrix (use TelemQuat conversions if desired), also converts local
                                 // vehicle vectors into world X, Y, or Z using dot product of rows 0, 1, or 2 respectively
  TelemVect3 mLocalRot;          // angular velocity (radians/sec) in local vehicle coordinates
  TelemVect3 mLocalRotAccel;     // angular acceleration (radians/sec^2) in local vehicle coordinates

  // Vehicle status
  int32_t mGear;                 // -1=reverse, 0=neutral, 1+=forward gears
  double mEngineRPM;             // engine RPM
  double mEngineWaterTemp;       // Celsius
  double mEngineOilTemp;         // Celsius
  double mClutchRPM;             // clutch RPM

  // Driver input
  double mUnfilteredThrottle;    // ranges  0.0-1.0
  double mUnfilteredBrake;       // ranges  0.0-1.0
  double mUnfilteredSteering;    // ranges -1.0-1.0 (left to right)
  double mUnfilteredClutch;      // ranges  0.0-1.0

  // Filtered input (various adjustments for rev or speed limiting, TC, ABS?, speed sensitive steering, clutch work for semi-automatic shifting, etc.)
  double mFilteredThrottle;      // ranges  0.0-1.0
  double mFilteredBrake;         // ranges  0.0-1.0
  double mFilteredSteering;      // ranges -1.0-1.0 (left to right)
  double mFilteredClutch;        // ranges  0.0-1.0

  // Misc
  double mSteeringArmForce;      // force on steering arms
  double mFront3rdDeflection;    // deflection at front 3rd spring
  double mRear3rdDeflection;     // deflection at rear 3rd spring

  // Aerodynamics
  double mFrontWingHeight;       // front wing height
  double mFrontRideHeight;       // front ride height
  double mRearRideHeight;        // rear ride height
  double mDrag;                  // drag
  double mFrontDownforce;        // front downforce
  double mRearDownforce;         // rear downforce

  // State/damage info
  double mFuel;                  // amount of fuel (liters)
  double mEngineMaxRPM;          // rev limit
  unsigned char mScheduledStops; // number of scheduled pitstops
  bool  mOverheating;            // whether overheating icon is shown
  bool  mDetached;               // whether any parts (besides wheels) have been detached
  unsigned char mHeadlights;     // status of headlights
  unsigned char mDentSeverity[8];// dent severity at 8 locations around the car (0=none, 1=some, 2=more)
  double mLastImpactET;          // time of last impact
  double mLastImpactMagnitude;   // magnitude of last impact
  TelemVect3 mLastImpactPos;     // location of last impact

  // Expanded
  double mEngineTq;              // current engine torque (including additive torque)
  int32_t mCurrentSector;        // the current sector (zero-based) with the pitlane stored in the sign bit (example: entering pits from third sector gives 0x80000002)
  unsigned char mSpeedLimiter;   // whether speed limiter is on
  unsigned char mMaxGears;       // maximum forward gears
  unsigned char mFrontTireCompoundIndex;   // index within brand
  unsigned char mRearTireCompoundIndex;    // index within brand
  double mFuelCapacity;          // capacity in litres
  unsigned char mFrontFlapActivated;       // whether front flap is activated
  unsigned char mRearFlapActivated;        // whether rear flap is activated
  unsigned char mRearFlapLegalStatus;      // 0=disallowed, 1=criteria detected but not allowed quite yet, 2=allowed
  unsigned char mIgnitionStarter;          // 0=off 1=ignition 2=ignition+starter

  char mFrontTireCompoundName[18];         // name of front tire compound
  char mRearTireCompoundName[18];          // name of rear tire compound

  unsigned char mSpeedLimiterAvailable;    // whether speed limiter is available
  unsigned char mAntiStallActivated;       // whether (hard) anti-stall is activated
  unsigned char mUnused[2];                //
  float mVisualSteeringWheelRange;         // the *visual* steering wheel range

  double mRearBrakeBias;                   // fraction of brakes on rear
  double mTurboBoostPressure;              // current turbo boost pressure if available
  float mPhysicsToGraphicsOffset[3];       // offset from static CG to graphical center
  float mPhysicalSteeringWheelRange;       // the *physical* steering wheel range
  float mStandardLocation[3];              // offset used for starting locations (center of nose at ground level)
  float mBrakeTorque[4];                   // brake torque at each wheel

  bool mBeacon[6];                         // 3 sectors, 3 spare
  bool mUnused2[2];                        //

  // Future use
  unsigned char mExpansion[96];  // for future use (note that the slot ID has been moved to mID above)

  float mSLapRef;                // sLapRef - distance vehicle has travelled along the reference line defined by AIW way points (metres)
  double mLapDistanceTravelled;  // True distance vehicle has travelled in this lap (metres)
  double mDistanceToGrooveCentre;// horizontal distance from mPos to centre of track 'groove' (metres)

  // keeping this at the end of the structure to make it easier to replace in future versions
  TelemWheelV01 mWheel[4];       // wheel info (front left, front right, rear left, rear right)
};


struct GraphicsInfoV01
{
  TelemVect3 mCamPos;            // Camera position (rFpro axes, world coordinate system)
  TelemVect3 mCamOri[3];         // Rows of orientation matrix (rFpro axes, world coordinate system)
  HWND mHWND;                    // App handle

  double mAmbientRed;
  double mAmbientGreen;
  double mAmbientBlue;
};


struct GraphicsInfoV02 : public GraphicsInfoV01
{
  int32_t mID;                   // slot ID being viewed (-1 if invalid)

  // Camera types (some of these may only be used for *setting* the camera type in WantsToViewVehicle())
  //    0  = TV cockpit
  //    1  = cockpit
  //    2  = nosecam
  //    3  = swingman
  //    4  = trackside (nearest)
  //    5  = onboard000
  //       :
  //       :
  // 1004  = onboard999
  // 1005+ = (currently unsupported, in the future may be able to set/get specific trackside camera)
  int32_t mCameraType;           // see above comments for possible values

  // Reverb parameters. Most parameters are only valid for cameras attached to a
  // vehicle, but the reverb levels are valid for trackside cameras.
  float mLeftColl;               // approximate guess of left collision corridor (relative to path where current vehicle is)
  float mRightColl;              // approximate guess of right collision corridor (relative to path where current vehicle is)
  float mLeftReverb;             // calculated left reverb level (relative to path where current vehicle is)
  float mRightReverb;            // calculated right reverb level (relative to path where current vehicle is)
  float mLocalRightX;            // which way right is, relative to the camera
  float mLocalRightY;            // which way right is, relative to the camera
  float mLocalRightZ;            // which way right is, relative to the camera

  bool mLoadingScreenActive;     // whether loading screen is being displayed
  int8_t mFisheyeProfile;        // 0 = no fisheye (rectilinear projection). non-zero = fisheye enabled and current fisheye profile is (mFisheyeProfile-1)
  unsigned char mUnused[2];      //

  enum FisheyeProjectionMode
  {
    FISHEYE_OTHER = 0,       // Non-standard fisheye projection
    FISHEYE_EQUIDISTANT,     // R=f.theta, also known as Linear projection
    FISHEYE_EQUISOLID,       // R=2f.sin(theta/2)
    FISHEYE_STEREOGRAPHIC,   // R=2f.tan(theta/2)
    FISHEYE_ORTHOGRAPHIC,    // R=f.sin(theta)

    FISHEYE_FORCEDWORD = 0x7fffffffL,
  };

  union
  {
    float mProjMatrix[4][4];     // Camera projection matrix. Used for rectilinear projection in non-fisheye mode.
    struct
    {
      // Projection parameters specific to fisheye mode.
      // See the rFpro Sensor IG User Guide for more information

      // All fisheye parameters available as key-value pairs.
      // Actual content will vary depending on projection mode.
      // Some examples of expected values:
      //   "ProjectionMode"     = same as mFisheyeProjectionMode. See enum above for values
      //   "Projection=***"     = Name of the projection mode is key.substr(value)
      //   "RadialFieldOfView"  = Nominal radial field of view (degrees)
      //   "SizeX"              = Scale factor applied in the horizontal image direction
      //   "ApertureMask"       = A value of 1.0 masks the image beyond the FOV. A value less than 1.0 reduces the size of the viewport. A value greater than 1.0 increases it.
      //   "EdgeBlend"          = Magnitude of the vignette effect at the edge of the graphical area. A value of 0.0 is a hard edge.
      // NB: Not all values will be available for all projection modes.

      FisheyeProjectionMode mFisheyeProjectionMode;       // See enum above for values
      uint32_t mFisheyeNumValues;       // Size of the mFisheyeValueNames and mFisheyeValues arrays
      union
      {
        const char** mFisheyeValueNames;// Array of null-terminated parameter names
        char ptrA64[8];                 // Ensure same space used regardless of platform
      };
      union
      {
        const float* mFisheyeValues;    // Array of parameter values
        char ptrB64[8];                 // Ensure same space used regardless of platform
      };
    };
  };

  unsigned char mExpansion[32];  // for future use (possibly camera name)
};


struct CameraControlInfoV01
{
  // Cameras
  int32_t mID;                   // slot ID to view
  int32_t mCameraType;           // see GraphicsInfoV02 comments for values

  // Replays (note that these are asynchronous)
  bool mReplayActive;            // This variable is an *input* filled with whether the replay is currently active (as opposed to realtime).
  bool mReplayUnused;            //
  unsigned char mReplayCommand;  // 0=do nothing, 1=begin, 2=end, 3=rewind, 4=fast backwards, 5=backwards, 6=slow backwards, 7=stop, 8=slow play, 9=play, 10=fast play, 11=fast forward

  bool mReplaySetTime;           // Whether to skip to the following replay time:
  float mReplaySeconds;          // The replay time in seconds to skip to (note: the current replay maximum ET is passed into this variable in case you need it)

  //
  bool mApplyMultiChannel;       // whether to apply to all other channels
  char mUnused[3];               //

  //
  unsigned char mExpansion[116]; // for future use (possibly camera name & positions/orientations)
};


struct MessageInfoV01
{
  char mText[128];               // message to display

  unsigned char mExpansion[128]; // for future use (possibly what color, what font, whether to attempt translation, and seconds to display)
};


struct VehicleScoringInfoV01
{
  int32_t mID;                   // slot ID (note that it can be re-used in multiplayer after someone leaves)
  char mDriverName[32];          // driver name
  char mVehicleName[64];         // vehicle name
  short mTotalLaps;              // laps completed
  signed char mSector;           // 0=sector3, 1=sector1, 2=sector2 (don't ask why)
  signed char mFinishStatus;     // 0=none, 1=finished, 2=dnf, 3=dq
  double mLapDist;               // current distance around track
  double mPathLateral;           // lateral position with respect to *very approximate* "center" path
  double mTrackEdge;             // track edge (w.r.t. "center" path) on same side of track as vehicle

  double mBestSector1;           // best sector 1
  double mBestSector2;           // best sector 2 (plus sector 1)
  double mBestLapTime;           // best lap time
  double mLastSector1;           // last sector 1
  double mLastSector2;           // last sector 2 (plus sector 1)
  double mLastLapTime;           // last lap time
  double mCurSector1;            // current sector 1 if valid
  double mCurSector2;            // current sector 2 (plus sector 1) if valid
  // no current laptime because it instantly becomes "last"

  short mNumPitstops;            // number of pitstops made
  short mNumPenalties;           // number of outstanding penalties
  bool mIsPlayer;                // is this the player's vehicle

  signed char mControl;          // who's in control: -1=nobody (shouldn't get this), 0=local player, 1=local AI, 2=remote, 3=replay (shouldn't get this)
  bool mInPits;                  // between pit entrance and pit exit (not always accurate for remote vehicles)
  unsigned char mPlace;          // 1-based position
  char mVehicleClass[32];        // vehicle class

  // Dash Indicators
  double mTimeBehindNext;        // time behind vehicle in next higher place
  int32_t mLapsBehindNext;       // laps behind vehicle in next higher place
  double mTimeBehindLeader;      // time behind leader
  int32_t mLapsBehindLeader;     // laps behind leader
  double mLapStartET;            // time this lap was started

  // Position and derivatives
  TelemVect3 mPos;               // world position in meters
  TelemVect3 mLocalVel;          // velocity (meters/sec) in local vehicle coordinates
  TelemVect3 mLocalAccel;        // acceleration (meters/sec^2) in local vehicle coordinates

  // Orientation and derivatives
  TelemVect3 mOri[3];            // rows of orientation matrix (use TelemQuat conversions if desired), also converts local
                                 // vehicle vectors into world X, Y, or Z using dot product of rows 0, 1, or 2 respectively
  TelemVect3 mLocalRot;          // angular velocity (radians/sec) in local vehicle coordinates
  TelemVect3 mLocalRotAccel;     // angular acceleration (radians/sec^2) in local vehicle coordinates

  unsigned char mHeadlights;     // status of headlights
  unsigned char mPitState;       // 0=none, 1=request, 2=entering, 3=stopped, 4=exiting
  unsigned char mServerScored;   // whether this vehicle is being scored by server (could be off in qualifying or racing heats)
  unsigned char mIndividualPhase;// game phases (described below) plus 9=after formation, 10=under yellow, 11=under blue (not used)

  int32_t mQualification;        // 1-based, can be -1 when invalid

  double mTimeIntoLap;           // estimated time into lap
  double mEstimatedLapTime;      // estimated laptime used for 'time behind' and 'time into lap' (note: this may changed based on vehicle and setup)

  char mPitGroup[24];            // pit group (same as team name unless pit is shared)
  unsigned char mFlag;           // primary flag being shown to vehicle (currently only 0=green or 6=blue)
  bool mUnderYellow;             // whether this car has taken a full-course caution flag at the start/finish line
  unsigned char mCountLapFlag;   // 0 = do not count lap or time, 1 = count lap but not time, 2 = count lap and time
  bool mInGarageStall;           // appears to be within the correct garage stall

  unsigned char mUpgradePack[16];  // Coded upgrades
  float mPitLapDist;             // location of pit in terms of lap distance

  float mBestLapSector1;         // sector 1 time from best lap (not necessarily the best sector 1 time)
  float mBestLapSector2;         // sector 2 time from best lap (not necessarily the best sector 2 time)

  char mVehicleCategory[32];     // Vehicle's category (as read from VEH file).

  // Future use
  unsigned char mExpansion[16];  // for future use
};


struct ScoringInfoV01
{
  char mTrackName[64];           // current track name
  int32_t mSession;              // current session (0=testday 1-4=practice 5-8=qual 9=warmup 10-13=race)
  double mCurrentET;             // current time
  double mEndET;                 // ending time
  int32_t  mMaxLaps;             // maximum laps
  double mLapDist;               // distance around track
  char *mResultsStream;          // results stream additions since last update (newline-delimited and NULL-terminated)

  int32_t mNumVehicles;          // current number of vehicles

  // Game phases:
  // 0 Before session has begun
  // 1 Reconnaissance laps (race only)
  // 2 Grid walk-through (race only)
  // 3 Formation lap (race only)
  // 4 Starting-light countdown has begun (race only)
  // 5 Green flag
  // 6 Full course yellow / safety car
  // 7 Session stopped
  // 8 Session over
  // 9 Paused
  unsigned char mGamePhase;   

  // Yellow flag states (applies to full-course only)
  // -1 Invalid
  //  0 None
  //  1 Pending
  //  2 Pits closed
  //  3 Pit lead lap
  //  4 Pits open
  //  5 Last lap
  //  6 Resume
  //  7 Race halt (not currently used)
  signed char mYellowFlagState;

  signed char mSectorFlag[3];      // whether there are any local yellows at the moment in each sector (not sure if sector 0 is first or last, so test)
  unsigned char mStartLight;       // start light frame (number depends on track)
  unsigned char mNumRedLights;     // number of red lights in start sequence
  bool mInRealtime;                // in realtime as opposed to at the monitor
  char mPlayerName[32];            // player name (including possible multiplayer override)
  char mPlrFileName[64];           // may be encoded to be a legal filename

  // weather
  double mDarkCloud;               // cloud darkness? 0.0-1.0
  double mRaining;                 // raining severity 0.0-1.0
  double mAmbientTemp;             // temperature (Celsius)
  double mTrackTemp;               // temperature (Celsius)
  TelemVect3 mWind;                // wind speed
  double mOnPathWetness;           // on main path 0.0-1.0
  double mOffPathWetness;          // on main path 0.0-1.0

  // Future use
  unsigned char mExpansion[256];

  // keeping this at the end of the structure to make it easier to replace in future versions
  VehicleScoringInfoV01 *mVehicle; // array of vehicle scoring info's
};


struct CommentaryRequestInfoV01
{
  char mName[32];                  // one of the event names in the commentary INI file
  double mInput1;                  // first value to pass in (if any)
  double mInput2;                  // first value to pass in (if any)
  double mInput3;                  // first value to pass in (if any)
  bool mSkipChecks;                // ignores commentary detail and random probability of event

  // constructor (for noobs, this just helps make sure everything is initialized to something reasonable)
  CommentaryRequestInfoV01()       { mName[0] = 0; mInput1 = 0.0; mInput2 = 0.0; mInput3 = 0.0; mSkipChecks = false; }
};


//==========================================================================
// Version02 Structures
//==========================================================================

struct NewVehicleDataV01
{
  char mVehFile[32];               // Name of the VEH file (no path, with or without extension .VEH)
  char mDriverName[32];            // If empty, will use default from VEH file
  unsigned char mUpgradePack[8];   // Coded upgrades (recommend to set to all zeroes if unknown)
  char mSkin[32];                  // Skin name (set empty for default)
  char mHelmet[32];                // Helmet name (set empty for default)
};


struct VehicleAndPhysicsV01 : public NewVehicleDataV01
{
  int32_t mID;                     // slot ID (note that it can be re-used in multiplayer after someone leaves)
  bool mIsPlayer;                  // is this the player's vehicle
  signed char mControl;            // who's in control: -1=nobody (shouldn't get this), 0=local player, 1=local AI, 2=remote, 3=replay (shouldn't get this)
  char mPhysicsFile[256];          // relative path to physics file
  char mSetupFile[256];            // relative path to vehicle setup file
};

struct StartingVehicleLocationV01
{
  int32_t mID;                     // slot ID (note that it can be re-used in multiplayer after someone leaves)
  TelemVect3 mPos;                 // Position (specifies where the *front* of the vehicle should be at ground level)
  TelemVect3 mOri[3];              // Orientation matrix (use conversion from TelemQuat if desired)
};

enum PluginControlRequest
{
  PCR_NONE = 0,                    // No control desired over given vehicle
  PCR_STATE,                       // State control (substitution in AI physics of x&z location and optionally much more)
  PCR_PHYSICS,                     // Is capable of physics control
  //-------------
  PCR_MAXIMUM
};


struct WheelStateV01
{
  double mRotation;                // radians/sec
  double mBrakeTemp;               // Celsius
  double mYLocation;               // wheel's y location relative to vehicle y location
};

struct VehicleStateV01
{
  int32_t mID;                     // slot ID (note that it can be re-used in multiplayer after someone leaves)

  // this is the time at which this vehicle state is valid
  double mET;                      // elapsed time

  // special
  bool mPurgeFlag;                 // flag to remove any previously stored states (could be used to start a new lap, for example)
  bool mCollidableVsAI;            // can this vehicle collide with other AIs? (this basically means other locally-controlled vehicles in multiplayer)
  bool mCollidableVsPlayer;        // can this vehicle collide with player vehicles? (this basically means remotely-controlled vehicles in multiplayer)
  bool mCollidableVsStatic;        // can this vehicle collide against the static database?
  double mTransparentProximity;    // for automatic transparency: 0.0 = off, 2.0-62.0 = fully visible at this distance from the vehicle that camera is focusing on, and totally invisible at 1/4th of that distance
  double mManualTransparency;      // for manual transparency: 0.0 = visible, 1.0 = invisible, can be used to indicate confidence in data

  // location
  // Please see note above struct TelemInfoV01 for rFactor coordinate system and conversion info!
  bool mPosYValid;                 // whether the y (height) coordinate is valid (if invalid, will use HAT)
  bool mOriValid;                  // whether the orientation matrix is valid (if invalid, will use AI simulation)
  bool mHeadingValid;              // only if full orientation is invalid, we'll try to use simple heading instead
  TelemVect3 mPos;                 // Position (x and z values must *always* be valid)
  TelemVect3 mOri[3];              // Orientation matrix (use conversion from TelemQuat if desired)
  double mHeading;                 // Heading angle, in radians

  // motion
  bool mVelValid;                  // whether velocity is valid (if invalid, will use AI simulation)
  bool mRotValid;                  // whether rotation is valid (if invalid, will use AI simulation)
  TelemVect3 mVel;                 // World linear velocity (sorry, telemetry reports local velocity but we can't convert if optional orientation is invalid!)
  TelemVect3 mRot;                 // World angular velocity (sorry, telemetry reports local velocity but we can't convert if optional orientation is invalid!)

  // engine, input, gear
  bool mGearValid;                 // whether gear is valid
  bool mRPMValid;                  // whether engine RPM is valid
  bool mInputsValid;               // whether throttle, brake, and steering are valid
  int32_t mGear;                   // -1=reverse, 0=neutral, 1+=forward gears
  double mEngineRPM;               // engine RPM (rotations per minute)
  double mThrottle;                // ranges  0.0-1.0
  double mBrake;                   // ranges  0.0-1.0
  double mSteering;                // ranges -1.0-1.0 (left to right)

  // wheel info
  bool mWheelRotationsValid;       // whether wheel rotations are valid
  bool mWheelBrakeTempsValid;      // whether wheel brake temps are valid
  bool mWheelYLocationsValid;      // whether wheel y locations are valid
  WheelStateV01 mWheel[4];         // wheel information

  // tag.2009.01.21 - added this expansion area ... currently rFactor is clearing this memory
  // due to backwards-compatibility concerns, but we will eventually stop doing that when all
  // current plugins are re-compiled.
  unsigned char mBrakelights;       // brakelight override: 0=invalid (rF control), 1=force off, 2=force on
  unsigned char mHeadlights;        // headlight override: 0=invalid (rF control), 1=force off, 2=force on

  bool mInvulnerable;              // whether this vehicle is invulnerable to damage (similarly to the player's driving aid)
  bool mSpecialEffects;            // whether this vehicle emits special effects (tyre smoke & skids, etc.)
  bool mRequestSleep;              // request that this vehicle's physics sleep to save CPU
  bool mUnusedB;

  unsigned char mExpansion[ 250 ];

  // This structure is *not* initialized upon the call to GetVehicleState(), so if you plan to
  // modify the vehicle state and return true, we *highly* recommend that you first call this
  // Clear() function before modifying the vehicle state; it will initialize all "valid flags"
  // to false and set everything else to zero (in a hacky but efficient manner).  If you do not
  // call it, make sure all "valid flags" are set properly!
  void Clear()                     { for( size_t i = 0; i < sizeof( VehicleStateV01 ); ++i ) reinterpret_cast<char *>( this )[ i ] = 0; } // TODO use memset instead?
};


struct PhysicsAdditiveV01
{
  TelemVect3 mLocalForce;          // will be added to the standardized CG (vehicle minus fluids, wheels in default locations)
  TelemVect3 mLocalTorque;         // will be added to body's other torques
  double mEngineTorque;            // positive for power, negative for coast (engine braking)
  double mRearBrakeBias;           // rear brake bias fraction (0.0-1.0); please set to -1.0 if invalid
};


struct ISOTyreInitV01
{
  int32_t mTyreID;                 // 0=FrontLeft, 1=FrontRight, 2=RearLeft, 3=RearRight
  bool mThermalMode;               // false = NonThermal, true = Thermal (from HDV file [PLUGIN] section: TyreThermalMode=<0 or 1>)
  double mPressurePSI;             // Tyre pressure
  union
  {
    double mCarcassTemp;           // OBSOLETE: will be removed at future date when existing plugins are updated
    double mSurfaceTemp;           // initial surface temp in Celsius, used for Thermal mode only
  };
  double mBulkTemp;                // mean bulk tread in Celsius, used for Thermal mode
};


struct ISOTyreInputV01
{
  int32_t mTyreID;                 // 0=FrontLeft, 1=FrontRight, 2=RearLeft, 3=RearRight
  double mDT;                      // delta time to integrate through
  double mET;                      // elapsed time (at *end* of physics frame, not beginning)

  // Note: ISO definitions used here
  double mTyreLoad;                // Fz, or tyre normal load in Newtons (positive for upward force on tyre)
  double mSlipAngle;               // In radians, positive ISO lateral velocity (left)
  double mSlipRatio;               // Unitless, positive for traction. 0.0=0% slip, 1.0=100% slip
  double mPressurePSI;             // Tyre pressure
  double mForwardVelocity;         // VxWc_t, wheel center forward velocity in stationary tyre axes (different to car Vx for front steered wheel)
  double mTrackTemp;               // Track surface temperature in Celsius
  double mAmbientTemp;             // Ambient temperature in Celsius
  double mRollAngle;               // Positive about ISO x axis (neg camber on left wheel)

  double mSurfaceTemp;             // tread surface temperature in Celsius (input for Thermal mode only)
  double mBulkTemp;                // mean bulk tread in Celsius (input for Thermal mode only)

  // surface properties (can use either look-up or simple gain)
  char mTerrainName[16];           // the material prefixes from the TDF file
  unsigned char mSurfaceType;      // 0=dry, 1=wet, 2=grass, 3=dirt, 4=gravel, 5=rumblestrip, 6=special, 7=runoff, 8=astroturf, 9=kerb, 10=pitlane, 11=metal, 12=concrete, 13=paint, 14-23=spare0-spare9
  double mSurfaceGain;             // friction multiplier should be exactly 1.0 for 'average' asphalt
  double mSurfaceProperty[3];      // not currently defined
};


struct ISOTyreOutputV01
{
  // Note: ISO definitions used here
  // The forces applied at the intersection of three planes: the ground plane, the wheel plane, and
  // a third plane perpendicular to the first two and containing the wheel center.
  double mLongitudinalForce;       // FxTy_t
  double mLateralForce;            // FyTy_t
  double mAligningMoment;          // MzTy_t
  double mPressurePSI;             // Tyre pressure
  double mSurfaceTemp;             // tread surface temperature in Celsius, used for Thermal mode only
  double mBulkTemp;                // mean bulk tread in Celsius, used for Thermal mode only
  double mCarcassTemp;             // mean tread/carcass in Celsius (used for Thermal mode only?)
  double mRollingResistance;       // FxRolRes
};


struct DifferentialInitV01
{
  double mRollingRadius;           // Average of driven wheels
  double mInitialTorque;           // Initial torque
};


struct DifferentialInputV01
{
  double mDT;                      // delta time to integrate through
  double mET;                      // elapsed time (at *end* of physics frame, not beginning)

  double mEngineRPM;               // engine speed
  double mThrottle;                // throttle position 0.0 - 1.0
  double mGearRatio;               // current gear ratio, estimated by ( engine RPM / diff input shaft RPM )

  double mFinalDriveInputRPM;      // rotational speed of input shaft
  TelemVect3 mLocalAccel;          // acceleration (meters/sec^2) in local vehicle coordinates (x is left, z is rearward)
  double mFinalDriveInputTorque;   // approximate input torque
  double mDifferentialWheelRPM;    // positive means right wheel is spinning faster (in forward direction)

  double mLapDistance;             // distance around track in meters
  double mSteering;                // steering wheel (-1.0 = left, 1.0 = right)
  TelemVect3 mLocalRot;            // rotation (radians/sec) in local vehicle coordinates (y is yaw rate, positive to right, I think)
};


struct DifferentialOutputV01
{
  double mTorque;                  // amount of torque transferred (should be positive unless differential is magical)
};


struct PhysicsTriangleV01
{
  TelemVect3 mVertex[3];           // three verts in clockwise
  char mTerrainName[16];           // the material prefixes from the TDF file
  unsigned char mSurfaceType;      // 0=dry, 1=wet, 2=grass, 3=dirt, 4=gravel, 5=rumblestrip, 6=special, 7=runoff, 8=astroturf, 9=kerb, 10=pitlane, 11=metal, 12=concrete, 13=paint, 14-23=spare0-spare9
  double mSurfaceGain;             // friction multiplier should be exactly 1.0 for 'average' asphalt
  double mSurfaceProperty[3];      // not currently defined
};


struct TrackGeometryV01
{
  int32_t mNumTriangles;           // number of relevant triangles
  PhysicsTriangleV01 *mTriangle;   // array of triangles
};


struct DriverInputV01
{
  // 'standard' inputs
  double mSteering;
  double mThrottle;
  double mHandbrake;               // rear (front is below)
  double mBrakes;
  double mClutch;
  double mPowerDemand;             // KERS or other

  int32_t mDirectManualShift;      // -2="no selection"/invalid, -1=reverse, 0=neutral, 1+=forward gear
  bool mShiftUp;
  bool mShiftDown;
  bool mShiftToNeutral;
  bool mTCOverride;
  bool mLaunchControl;

  double mHandfrontbrake;          // 'handfrontbrake' (the other one above is for the rear)
  double mUnusedFloat[5];          // future use
  double mSteeringVel;             // rate of change of steering angle
  double mSteeringAcc;             // acceleration of steering angle

  int32_t mUnusedInt[16];          // future use

  bool mFrontFlap;                 // controls any front aerodynamic device
  bool mRearFlap;                  // controls any rear aerodynamic device
  bool mTemporaryBoost;
  bool mUnusedBool[10];            // future use
  bool mSteeringRatesValid;        // if true, mSteering derivates are valid
  bool mSteeringAbsolute;          // if true, mSteering and derivates are in radians, otherwise -1..+1
  bool mSteerByTorque;             // if true, mSteering is steering column torque (Nm)
};


struct SlotStateV01                // location and motion of a slot
{
  // Please see note above struct TelemInfoV01 for rFactor coordinate system and conversion info!
  int32_t mID;                     // slot ID (note that it can be re-used in multiplayer after someone leaves)
  TelemVect3 mPos;                 // position
  TelemVect3 mOri[3];              // orientation matrix (use conversion from TelemQuat if desired)
  TelemVect3 mVel;                 // linear velocity (in world coordinates, not local!)
  TelemVect3 mRot;                 // angular velocity (in world coordinates, not local!)
};


struct PhysicsInputV01 : DriverInputV01
{
  // timing info
  double mDT;                      // delta time to integrate through
  double mET;                      // elapsed time (at *end* of physics frame, not beginning)

  // weather
  double mAirDensity;              // current air density (which can be derived from: track altitude, ambient temp, humidity, and pressure)
  double mHumidity;                // humidity (0.0-1.0)
  double mPressure;                // air pressure (kPa)
  double mAmbientTemp;             // temperature (Celsius)
  double mTrackTemp;               // temperature (Celsius)
  TelemVect3 mWind;                // wind speed

  // Track geometry can be optionally provided at load time or in realtime (see interface).
  // track geometry (for whole vehicle in order to calculate aerodynamics and bottoming-out)
  TrackGeometryV01 mTrackGeometryForVehicle;
  // track geometry (for each wheel - smaller sets than for vehicle in order to calculate contact more quickly)
  TrackGeometryV01 mTrackGeometryForWheel[4];

  // miscellaneous
  int32_t mPrivate[8];             // for private use
  bool mStartSim;                  // 1 for start (running?), 0 to stop?
  bool mBeacon[6];                 // 3 sectors, 3 spare
  bool mPitting;                   // in pitlane
  bool mCollision;                 // collision detected by internal rFactor physics (data below)

  TelemVect3 mCollisionExitVector; // this is an optional position change to be applied in order to exit the collision, in world coordinates
  TelemVect3 mCollisionForce;      // collision force to be applied to main vehicle body, in world coordinates
  TelemVect3 mCollisionTorque;     // collision torque to be applied to main vehicle body, in world coordinates

  // location and motion data
  SlotStateV01 mInternalState;     // location and motion of this vehicle according to internal rFactor physics

  int32_t mNumOpponents;           // number of other vehicles
  SlotStateV01 *mOpponentState;    // location and motion states
};


struct WheelOutputV01
{
  TelemVect3 mPos;                 // center of wheel, in world coordinates
  TelemVect3 mOri[3];              // orientation matrix (use conversion from TelemQuat if desired)
  double mSlipAngle;               // radians
  double mSlipRatio;               // Unitless, positive for traction. 0.0=0% slip, 1.0=100% slip
  double mTireLoad;                // load on tire in Newtons
  double mRotation;                // radians/sec
  double mBrakeTemp;               // Celsius
  double mTreadTemp[3];            // left/center/right
  double mAirTemp;                 // Celsius

  unsigned char mExpansion[64];    // future use
};


struct PhysicsOutputV01
{
  // timing and state info
  double mET;                      // elapsed time (at which this output state is valid ... rFactor may extrapolate briefly to synch with graphics)
  bool mSkipInternalPhysics;       // request internal physics to be skipped for performance reasons; note that most valid flags probably need to be true for this to work well
  bool mUseInternalPhysics;        // this indicates that plugin physics are temporarily invalid; rFactor should use its own internal calculations
  int32_t mPrivate[8];             // for private use

  // location and motion are required
  // Please see note above struct TelemInfoV01 for rFactor coordinate system and conversion info!
  TelemVect3 mPos;                 // position
  TelemVect3 mOri[3];              // orientation matrix (use conversion from TelemQuat if desired)
  TelemVect3 mVel;                 // linear velocity (in world coordinates, not local!)
  TelemVect3 mRot;                 // angular velocity (radians/sec, in world coordinates, not local!)

  bool mAccelValid;                // whether acceleration is valid
  TelemVect3 mAccel;               // linear acceleration (in world coordinates)

  // wheel info
  bool mWheelPosValid;             // whether wheel positions are valid
  unsigned char mWheelOriValid;    // 0=invalid, 1 or 2=full orientation (rFactor will *not* spin wheel other than for graphical extrapolation) 3=spindle axis only (stored in mOri[0]; rFactor will spin wheel)
  bool mTireParamsValid;           // slip angle, slip ratio, and tire load
  bool mWheelRotationValid;        // whether rotations are valid
  bool mWheelBrakeTempValid;       // whether brake temps are valid
  bool mWheelTreadTempValid;       // whether tread temps are valid
  bool mWheelAirTempValid;         // whether air temps are valid
  WheelOutputV01 mWheel[4];        // wheel info

  // engine, input, gear
  bool mGearValid;                 // whether gear is valid
  bool mRPMValid;                  // whether engine RPM is valid
  bool mInputsValid;               // whether throttle, brake, and steering are valid
  int32_t mGear;                   // -1=reverse, 0=neutral, 1+=forward gears
  double mEngineRPM;               // engine RPM (rotations per minute)
  double mThrottle;                // ranges  0.0-1.0
  double mBrake;                   // ranges  0.0-1.0
  double mSteering;                // ranges -1.0-1.0 (left to right)
  double mClutch;                  // ranges  0.0-1.0

  bool mAdjustVerticalPosition;    // if true, overrides mPos.y so that at least one tire contacts the terrain appropriately
  bool mAdjustRollAndPitch;        // if true, adjusts vehicle roll and pitch so that the tires conform better to the terrain
  unsigned char mUnused[2];        // 

  unsigned char mExpansion[216];   // future use

  bool mRotAccelValid;             // whether angular acceleration is valid
  bool mHeadlightsValid;
  char mHeadlights;
  TelemVect3 mRotAccel;            // angular acceleration (radians/sec^2, in world coordinates)

  // For efficiency's sake, this structure is *not* initialized upon the call to GetPhysicsState().
  // Please make sure to either call the Clear() function before modifying the structure, or fill
  // in all required data plus the "valid flags".
  void Clear()                     { for( size_t i = 0; i < sizeof( PhysicsOutputV01 ); ++i ) reinterpret_cast<char *>( this )[ i ] = 0; } // TODO use memset instead?
};


struct PhysicsOptionsV01
{
  unsigned char mTractionControl;  // 0 (off) - 3 (high)
  unsigned char mAntiLockBrakes;   // 0 (off) - 2 (high)
  unsigned char mStabilityControl; // 0 (off) - 2 (high)
  unsigned char mAutoShift;        // 0 (off), 1 (upshifts), 2 (downshifts), 3 (all)
  unsigned char mAutoClutch;       // 0 (off), 1 (on)
  unsigned char mInvulnerable;     // 0 (off), 1 (on)
  unsigned char mOppositeLock;     // 0 (off), 1 (on)
  unsigned char mSteeringHelp;     // 0 (off) - 3 (high)
  unsigned char mBrakingHelp;      // 0 (off) - 2 (high)
  unsigned char mSpinRecovery;     // 0 (off), 1 (on)
  unsigned char mAutoPit;          // 0 (off), 1 (on)
  unsigned char mAutoLift;         // 0 (off), 1 (on)
  unsigned char mAutoBlip;         // 0 (off), 1 (on)

  unsigned char mFuelMult;         // fuel multiplier (0x-7x)
  unsigned char mTireMult;         // tire wear multiplier (0x-7x)
  unsigned char mMechFail;         // mechanical failure setting; 0 (off), 1 (normal), 2 (timescaled)
  unsigned char mAllowPitcrewPush; // 0 (off), 1 (on)
  unsigned char mRepeatShifts;     // accidental repeat shift prevention (0-5; see PLR file)
  unsigned char mHoldClutch;       // for auto-shifters at start of race: 0 (off), 1 (on)
  unsigned char mAutoReverse;      // 0 (off), 1 (on)
  unsigned char mAlternateNeutral; // Whether shifting up and down simultaneously equals neutral

  unsigned char mAIControl;        // Whether player vehicle is currently under AI control
  unsigned char mUnused1;          //
  unsigned char mUnused2;          //

  float mManualShiftOverrideTime;  // time before auto-shifting can resume after recent manual shift
  float mAutoShiftOverrideTime;    // time before manual shifting can resume after recent auto shift
  float mSpeedSensitiveSteering;   // 0.0 (off) - 1.0
  float mSteerRatioSpeed;          // speed (m/s) under which lock gets expanded to full
};

namespace EnvironmentInfo
{
    const size_t CWD = 3;
    const size_t SCN_file = 4;
    const size_t AIW_file = 5;
    const size_t VEH_file = 6;
    const size_t HDV_file = 7;
    const size_t Command_Line = 15;
}

struct EnvironmentInfoV01
{
  // TEMPORARY buffers (you should copy them if needed for later use) containing various paths that may be needed.  Each of these
  // could be relative ("UserData\") or full ("C:\BlahBlah\rFpro\UserData\").
  // mPath[ 0 ] points to the UserData directory.
  // mPath[ 1 ] points to the CustomPluginOptions.ini filename.
  // mPath[ 2 ] points to the latest results file
  // mPath[ 3 ] points to current working directory (absolute path)
  // mPath[ 4 ] points to the SCN file (absolute path)
  // mPath[ 5 ] points to the AIW file (absolute path)
  // mPath[ 6 ] points to the player's VEH file (absolute path)
  // mPath[ 7 ] points to the player's HDV file (absolute path)
  // mPath[ 8 ] points to the Locations directory
  // mPath[ 9 ] points to the Libraries directory
  // mPath[ 10 ] points to the Vehicles directory
  // mPath[ 15 ] points to the command-line parameters passed into rFpro
  const char *mPath[ 16 ];
  unsigned char mExpansion[256];   // future use
};


struct ScreenInfoV01
{
  HWND mAppWindow;                      // Application window handle
  void *mDevice;                        // Cast type to LPDIRECT3DDEVICE9
  void *mRenderTarget;                  // Cast type to LPDIRECT3DTEXTURE9
  int32_t mDriver;                      // Current video driver index

  int32_t mWidth;                       // Screen width
  int32_t mHeight;                      // Screen height
  int32_t mPixelFormat;                 // Pixel format
  int32_t mRefreshRate;                 // Refresh rate
  int32_t mWindowed;                    // Really just a boolean whether we are in windowed mode

  int32_t mOptionsWidth;                // Width dimension of screen portion used by UI
  int32_t mOptionsHeight;               // Height dimension of screen portion used by UI
  int32_t mOptionsLeft;                 // Horizontal starting coordinate of screen portion used by UI
  int32_t mOptionsUpper;                // Vertical starting coordinate of screen portion used by UI

  unsigned char mOptionsLocation;       // 0=main UI, 1=track loading, 2=monitor, 3=on track
  char mOptionsPage[ 31 ];              // the name of the options page

  unsigned char mExpansion[ 224 ];      // future use
};

#ifndef NO_WEATHER_CONTROL

struct WeatherControlInfoV01
{
  // The current conditions are passed in with the API call. The following ET (Elapsed Time) value should typically be far
  // enough in the future that it can be interpolated smoothly, and allow clouds time to roll in before rain starts. In
  // other words you probably shouldn't have mCloudiness and mRaining suddenly change from 0.0 to 1.0 and expect that
  // to happen in a few seconds without looking crazy.
  double mET;                           // when you want this weather to take effect

  // mRaining[1][1] is at the origin (and currently the only implemented node), while the others
  // are spaced at <trackNodeSize> meters where <trackNodeSize> is the maximum absolute value of a track vertex
  // coordinate (and is passed into the API call).
  double mRaining[ 3 ][ 3 ];            // rain (0.0-1.0) at different nodes

  double mCloudiness;                   // general cloudiness (0.0=clear to 1.0=dark), will be automatically overridden to help ensure clouds exist over rainy areas
  double mAmbientTempK;                 // ambient temperature (Kelvin)
  double mWindMaxSpeed;                 // maximum speed of wind (ground speed, but it affects how fast the clouds move, too)

  bool mApplyCloudinessInstantly;       // preferably we roll the new clouds in, but you can instantly change them now
  unsigned char mFogMode;               // currently active fog mode: 0 = none, 1 = constant, 2 = linear, 3 = exponential, 4 = exponential2
  bool mAccumulateNodes;                // whether to add to existing list of these weather control info structures
  bool mUnused3;                        //

  // Rain effects are now passed in. Set to -1.0 to return to default, or set any of them to non-negative values in order to override the internal calculations.
  float mOverrideRainWind;              // 0.0-20.0 meters/second
  float mOverrideRainWidth;             // 0.00-0.50 meters
  float mOverrideRainHeight;            // 0.0-5.0 meters
  float mOverrideRainSpeed;             // 0.0-20.0 meters/second
  float mOverrideRainRadius;            // 0.0-40.0 meters
  float mOverrideRainAlpha;             // 0.0-1.0
  float mOverrideRainPasses;            // 0-5, will be rounded to closest integer
  float mOverrideRainDensity;           // 0.0-20.0
  float mOverrideRainParticles;         // 0-2500, will be rounded to closest integer

  // Fog effects are now passed in. Set to -1.0 to return to default, or set any of them to non-negative values in order to override the internal calculations.
  // NOTE: There are two special combinations of the union-ized variables mOverrideFogDensity/mOverrideLinearFogIn and mOverrideLienarFogOut/mOverrideFogDensity2:
  //   If both values are negative, the fog mode and settings will continue as they were (either using latest overrides or, if no overrides have been applied, performing the normal internal calculations based on the original SCN file values).
  //   If both values are zero, the fog mode and settings will return to performing the normal internal calculations based on the original SCN file values.
  float mOverrideSkyFogLevel;           // internal value equals 'fake humidity' (0.0-1.0) which is average of rain level and off-path track wetness
  union
  {
    float mOverrideFogDensity;          // internal value comes from scene file + ( 0.00159 * 'fake humidity' ), sets fog to 'EXP' if not negative and mOverrideLinearFogOut is negative (see above for more info)
    float mOverrideLinearFogIn;         // internal value comes from scene file, sets fog to 'LINEAR' if both mOverrideLinearFogOut and this are not negative (see above for more info)
  };
  float mOverrideTurbidity;             // internal value is 2.2 + ( 0.25 * 'fake humidity' )
  float mOverrideHazeDepth;             // internal value is 0.0 + ( 1000.0 * 'fake humidity' )
  float mOverrideFogColorRed;           // internal value is calculated by sky simulation
  float mOverrideFogColorGreen;         // internal value is calculated by sky simulation
  float mOverrideFogColorBlue;          // internal value is calculated by sky simulation
  union
  {
    float mOverrideLinearFogOut;        // internal value comes from scene file, sets fog to 'LINEAR' if this is positive and mOverrideLinearFogIn is non-negative (see above for more info)
    float mOverrideFogDensity2;         // internal value comes from scene file + ( 0.00159 * 'fake humidity' ), sets fog to 'EXP2' if not negative and mOverrideLinearFogIn is negative (see above for more info)
  };

  // More new weather overrides (these features are generally not supported internally, but are initialized to reasonable values).
  double mOverrideWindDirection;        // 0 is north, 1 is northeast, 2 is east, etc. (note that mWindMaxSpeed is located earlier in this structure)
  double mOverrideAirDensity;           // kg/m^3
  double mOverrideHumidity;             // 0.0-1.0
  double mOverridePressure;             // kPa
  double mOverrideTrackTempK;           // average track temperature (note that mAmbientTempK is located earlier in this structure)

  // Road effect is now passed in. Set to -1.0 to return to default, or set any of them to non-negative values in order to override the internal calculations.
  float mOverrideRoadWetness;           // 0.0(dry) - 1.0(full wet)
  float mOverrideDiffusionBias;         // 0.0-1.0 (controls how fog affects sun occlusion)

  // These are the currently active fog conditions (coupled with mFogMode above) which are passed into the plugin
  float mCurrentFogDensity;             // valid if mFogMode==3 (exponential) or mFogMode==4 (exponential2)
  float mCurrentLinearFogIn;            // valid if mFogMode==2 (linear)
  float mCurrentLinearFogOut;           // valid if mFogMode==2 (linear)

  //
  unsigned char mExpansion[ 380 ];      // future use (humidity, pressure, air density, etc.)
};

#endif

struct LightInfoV01
{
  // Identifying name of light
  char mName[ 32 ];

  // Number of states that the light has (for example: headlights off/on/high-beam, or the many stages of a racing start light)
  int32_t mNumStates;

  // Type is one of:
  enum Type
  {
    INVALID = -1,          // not a valid type of light
    VEHICLE_HEADLIGHTS,    // headlamps including high beam
    VEHICLE_LEFTTURN,      // left turn signal
    VEHICLE_RIGHTTURN,     // right turn signal
    VEHICLE_REVERSE,       // reverse lights
    VEHICLE_BRAKELIGHTS,   // brakes
    VEHICLE_WARNINGLIGHTS, // as for a safety car, police car, ambulance, fire truck, etc.
    VEHICLE_RAINLIGHT,     // rainlight (as in some open-wheel racing series)
    VEHICLE_MISCELLANEOUS, // some sort of light or animation detected but of an unknown type
    SCENE_STARTLIGHTS,     // racing start lights
    SCENE_PITENTRY,        // pit entry lights
    SCENE_PITEXIT,         // pit exit lights
    SCENE_STREETLIGHTS,    // a.k.a. "NIGHTLIGHT"s
    SCENE_TRAFFICSIGNAL,   // a.k.a. "TRAFFIC"s
    SCENE_MISCELLANEOUS    // some sort of light or animation detected but of an unknown type
  };
  Type mType;

  // For future use:
  unsigned char mExpansion[ 88 ];
};


struct LightControlV01
{
  // Vehicle slot that this light belongs to, or -1 for a scene light
  int32_t mSlotID;

  // Whether to use the exact name given below, or to apply to any light that contains the name (like a wildcard)
  bool mExactTrueOrWildcardFalse;

  // Name of light to control (or part of the name if it is a wildcard)
  char mName[32];

  // Desired state of light, or -1 to return control to rFactor
  int32_t mState;

  // Which parameters to update
  bool mPosValid;
  bool mDirValid;
  bool mOriValid;
  bool mIntensityValid;
  bool mColorValid;
  char mblank[3];

  // Omni light parameters
  float mPos[3];
  float mDir[3];
  float mOri[3];
  float mIntensity;
  long int mColor;

  // For future use:
  unsigned char mExpansion[32];
};

struct ThreadTimingV01
{
  // Note that there are situations (for example during synchronization) when mElapsedTime isn't simply the previous mElapsedTime plus mDeltaTime.
  double mDeltaTime;                    // time since last frame
  double mElapsedTime;                  // elapsed time since start of session

  unsigned char mExpansion[ 128 ];      // for future use
};


struct BeaconContact
{
  int32_t mSlotID;                      // slot ID of vehicle making contact
  char mInstance[ 64 ];                 // name of instance contacted with
  double mET;                           // elapsed time of contact

  unsigned char mExpansion[ 128 ];      // for future use
};


struct QueryBoneAnim
{
  char *mName;                          // null-terminated bone animation name

  float mTime;                          // length of animation in seconds
  int32_t mFrames;                      // number of frames

  unsigned char mExpansion[ 64 ];       // for future use

  QueryBoneAnim()                       { mName = 0; }
  ~QueryBoneAnim()                      { delete [] mName; }

private:
  // deleted C++98 style
  QueryBoneAnim(const QueryBoneAnim&);
  QueryBoneAnim& operator=(const QueryBoneAnim&);
};

struct QueryTexture
{
  char *mName;                          // null-terminated material name
  void *mD2DInterface;                  // pointer to ID2DIRenderTarget interface if available (only if render target, only if DX11)

  unsigned char mExpansion[ 64 ];       // for future use

  QueryTexture()                        { mName = 0; mD2DInterface = 0; }
  ~QueryTexture()                       { delete [] mName; }

private:
  // deleted C++98 style
  QueryTexture(const QueryTexture&);
  QueryTexture& operator=(const QueryTexture&);
};

struct QueryMesh
{
  char *mName;                          // null-terminated mesh name

  bool mIsDeformable;                   // whether mesh is deformable
  bool mUnused[ 3 ];                    //

  int32_t mNumBoneAnims;                // number of bone animations available
  QueryBoneAnim *mBoneAnim;             // array of bone animations

  uint32_t mID;                         // unique ID for this mesh

  int32_t mNumBoneNames;                // number of bones in mesh
  union
  {
    char **mBoneName;                   // array of null-terminated bone names
    double mSpacerA;                    // guarantees same size usage for 32 or 64 bit
  };

  int32_t mNumTextures;
  union
  {
    QueryTexture *mTexture;             // array of null-terminated textures
    double mSpacerB;                    // guarantees same size usage for 32 or 64 bit
  };

  unsigned char mExpansion[ 36 ];       // for future use

  QueryMesh()                           { mName = 0; mBoneAnim = 0; mID = 0xffffffff; mNumBoneNames = 0; mBoneName = 0; mNumTextures = 0; mTexture = 0; }
  ~QueryMesh()                          { delete [] mName; delete [] mBoneAnim; for( int32_t i = 0; i < mNumBoneNames; ++i ) delete [] mBoneName[ i ]; delete [] mBoneName; delete [] mTexture; }

private:
  // deleted C++98 style
  QueryMesh(const QueryMesh&);
  QueryMesh& operator=(const QueryMesh&);
};

struct QueryInstance
{
  char *mName;                          // null-terminated instance name
  char *mParent;                        // null-terminated parent name (the pointer itself will be null if there is no parent, so check that first)

  bool mIsMoveable;                     // whether instance is moveable
  bool mIsFunctional;                   // whether instance is Functional Ground Truth
  bool mUnused[ 2 ];                    //

  int32_t mNumMeshes;                   // number of meshes in the instance
  QueryMesh *mMesh;                     // array of meshes

  unsigned char mExpansion[ 64 ];       // for future use

  QueryInstance()                       { mName = 0; mParent = 0; mMesh = 0; mIsFunctional = false; }
  ~QueryInstance()                      { delete [] mName; delete [] mParent; delete [] mMesh; }

private:
  // deleted C++98 style
  QueryInstance(const QueryInstance&);
  QueryInstance& operator=(const QueryInstance&);
};

//--------------------------------------------------------------------------

enum AnimationCommandV01                //
{
  ACMD_READSCENE = 0,                   // read a scene file (add to a scene)
  ACMD_LOADINSTANCE,                    // load an instance (from a mesh name)
  ACMD_UNLOADINSTANCE,                  // unload an instance (by name)

  ACMD_SETINSTANCELOCATION,             // set the location of an instance
  ACMD_SETINSTANCEVISIBILITY,           // set whether an instance is visible or not

  ACMD_RESETANIMATION,                  // resets animation
  ACMD_STARTANIMATION,                  // starts animation
  ACMD_STOPANIMATION,                   // stops animation
  ACMD_PAUSEANIMATION,                  // pauses animation
  ACMD_SETANIMATIONET,                  // sets animation to a specific ET
  ACMD_SETANIMATIONFRAME,               // sets animation to a specific frame
  ACMD_LOADANIMATION,                   // loads an animation
  ACMD_UNLOADANIMATION,                 // *not* currently implemented
  ACMD_SETBONENAMELOCATION,             // sets a bone location by name
  ACMD_SETBONEINDEXLOCATION,            // sets a bone location by index

  ACMD_SPECIAL,                         // automatically set the full location and motion for preset types (bikes, trains? etc.) based on limited info, e.g. x-z position only

  ACMD_EXPOSUREBIAS,                    // set the tonemapper exposure bias
  ACMD_SETLIGHTPROPERTY,                // set the direction of a light (for lights with profile)

  // future commands go here ...

  //------------------
  ACMD_MAXIMUM                          // should be last
};

struct AnimationDataReadSceneV01        // ACMD_READSCENE
{
  char mFileName[ 512 ];                // filename
  char mPathOrMAS[ 512 ];               // search path or MAS file to load from (if string ends with ".mas"); either way needs to be relative to the configurable tracks directory
  unsigned char mExpansion[ 512 ];      // for future use
};

struct AnimationDataLoadInstanceV01     // ACMD_LOADINSTANCE
{
  char mName[ 64 ];                     // name to give instance
  char mMesh[ 64 ];                     // mesh name
  char mPathOrMAS[ 512 ];               // search path or MAS file to load from (if string ends with ".mas"); either way needs to be relative to the configurable tracks directory
  bool mMoveable;                       // whether to make instance moveable
  bool mUnused[ 3 ];                    //
  int32_t mMeshFlags;                   // Flags to add: 0x00000200 (shadow object), 0x00200000 (texture shadow), 0x00800000 (shadow caster), 0x10000000 (shadow active), more available upon request
  unsigned char mExpansion[ 508 ];      // for future use
};

struct AnimationDataUnloadInstanceV01   // ACMD_UNLOADINSTANCE
{
  char mParent[ 64 ];                   // name of instance parent (if necessary for proper identification, otherwise empty)
  char mInstance[ 64 ];                 // name of instance to unload
  unsigned char mExpansion[ 256 ];      // for future use
};

struct AnimationDataSetInstanceV01      // ACMD_SETINSTANCELOCATION, ACMD_SETINSTANCEVISIBILITY
{
  char mParent[ 64 ];                   // name of instance parent (if necessary for proper identification, otherwise empty)
  char mInstance[ 64 ];                 // name of instance to set
  unsigned char mExpansion1[ 128 ];     // for future use
  union
  {
    struct                              // only used for ACMD_SETINSTANCELOCATION (once)
    {
      TelemVect3 mPos;                  // world position in meters
      TelemVect3 mOri[3];               // rows of orientation matrix
    };
    struct                              // only used for ACMD_SETINSTANCELOCATION (range of time)
    {
      TelemVect3 mStartPos;             // world position in meters at session time mStartElapsedTime
      TelemVect3 mStartOri[3];          // rows of orientation matrix at session time mStartElapsedTime
      double mStartElapsedTime;         // this is the session time at which to set the animation to mStartAnimET/mStartAnimFrame (the special value 0.0 means "right now")

      TelemVect3 mEndPos;               // world position in meters at session time mEndElapsedTime
      TelemVect3 mEndOri[3];            // rows of orientation matrix at session time mEndElapsedTime
      double mEndElapsedTime;           // this is the session time at which to set the end location (0.0 means "invalid", i.e. we are only setting the instance location once; otherwise it should be a value slightly bigger than mStartElapsedTime, even if mStartElapsedTime == 0.0)
    };
    bool mVisible;                      // only used for ACMD_SETINSTANCEVISIBILITY
  };
  int32_t mAbsoluteOrRelativeTo;        // only used for ACMD_SETINSTANCELOCATION; The given location(s) are: 0=absolute 1=relative to current camera, 2=relative to current vehicle, all other values reserved for possible future use
  unsigned char mExpansion2[ 12 ];      // for future use
};

struct AnimationDataControlBonesV01     // ACMD_RESETANIMATION, ACMD_STARTANIMATION, ACMD_STOPANIMATION, ACMD_PAUSEANIMATION, ACMD_SETANIMATIONET, ACMD_SETANIMATIONFRAME, ACMD_LOADANIMATION, ACMD_UNLOADANIMATION, ACMD_SETBONENAMELOCATION, ACMD_SETBONEINDEXLOCATION
{
  enum Flags : uint32_t
  {
    FLAG_NONE = 0x0,
    FLAG_ONE_SHOT = 0x1                 // ACMD_STARTANIMATION: stay at the last frame when reaching the end of animation. Otherwise, play in loop.
  };

  char mParent[ 64 ];                   // name of instance parent (if necessary for proper identification, otherwise empty)
  char mInstance[ 64 ];                 // name of instance
  char mMesh[ 64 ];                     // name of mesh
  char mAnim[ 64 ];                     // name of animation (*not* used for ACMD_SETBONENAMELOCATION or ACMD_SETBONEINDEXLOCATION)
  union
  {
    char mBoneName[ 64 ];               // only used for ACMD_SETBONENAMELOCATION
    int32_t mBoneIndex;                 // only used for ACMD_SETBONEINDEXLOCATION
  };
  unsigned char mExpansion1[ 64 ];      // for future use
  union
  {
    // setting animation ET or frame, and doing so either once or over a smoothly-interpolated range of time
    float mET;                          // only used for ACMD_SETANIMATIONET (once)
    int32_t mFrame;                     // only used for ACMD_SETANIMATIONFRAME (once)
    struct                              // used for either ACMD_SETANIMATIONET (range of time) or ACMD_SETANIMATIONFRAME (range of time)
    {
      union
      {
        float mStartAnimET;             // ACMD_SETANIMATIONET: animation ET to set at the session time of mStartElapsedTime
        int32_t mStartAnimFrame;        // ACMD_SETANIMATIONFRAME: animation frame to set at the session time of mStartElapsedTime
      };
      union
      {
        float mEndAnimET;               // ACMD_SETANIMATIONET: animation ET to set at the session time of mEndElapsedTime  (and only applicable if mEndElapsedTime != 0.0)
        int32_t mEndAnimFrame;          // ACMD_SETANIMATIONFRAME: animation frame to set at the session time of mEndElapsedTime (and only applicable if mEndElapsedTime != 0.0)
      };
      double mStartElapsedTime;         // ACMD_SETANIMATIONET/ACMD_SETANIMATIONFRAME/ACMD_SETBONENAMELOCATION/ACMD_SETBONEINDEXLOCATION: this is the session time at which to set the animation to its start state (the special value 0.0 means "right now")
      double mEndElapsedTime;           // ACMD_SETANIMATIONET/ACMD_SETANIMATIONFRAME/ACMD_SETBONENAMELOCATION/ACMD_SETBONEINDEXLOCATION: this is the session time at which to set the animation to its end state (0.0 means "invalid", i.e. we are only setting the animation once; otherwise it should be a value slightly bigger than mStartElapsedTime, even if mStartElapsedTime == 0.0)
    };
  };
  uint32_t mFlags;                      // combination of bitmask. See enum Flags.
  unsigned char mExpansion2[ 104 ];     // for future use
  union
  {
    float mBoneMat[ 4 ][ 4 ];           // only used for ACMD_SETBONENAMELOCATION and ACMD_SETBONEINDEXLOCATION (once)
    struct
    {
      float mStartBoneMat[ 4 ][ 4 ];    // only used for ACMD_SETBONENAMELOCATION and ACMD_SETBONEINDEXLOCATION (range of time; see mStartElapsedTime above)
      float mEndBoneMat[ 4 ][ 4 ];      // only used for ACMD_SETBONENAMELOCATION and ACMD_SETBONEINDEXLOCATION (range of time; see mEndElapsedTime above)
      int32_t mAbsoluteOrRelativeTo;    // ACMD_SETBONENAMELOCATION/ACMD_SETBONEINDEXLOCATION; The given location(s) are: 0=absolute 1=relative to current camera, 2=relative to current vehicle, all other values reserved for possible future use
    };
    char mPathOrMAS[ 512 ];             // only used for ACMD_LOADANIMATION; search path or MAS file to load from (if string ends with ".mas"); either way needs to be relative to the configurable tracks directory
  };
};

struct AnimationDataSpecialV01          // ACMD_SPECIAL
{
  enum Type
  {
    PEDESTRIAN,                         // pedestrian with one contact point, always upright
    BICYCLE,                            // two contact points, can automatically lean with lateral acceleration
    MOTORCYCLE,                         // two contact points, can automatically lean with lateral acceleration
    ANIMAL,                             // four contact points
    FLYER,                              // no contact points, automatically adjusts orientation according to path

    // future types go here ...

    //---------
    MAXIMUM                             // should always be the last entry
  };

  char mParent[ 64 ];                   // name of instance parent (if necessary for proper identification, otherwise empty)
  char mInstance[ 64 ];                 // name of instance to set
  unsigned char mExpansion1[ 256 ];     // for future use

  Type mType;                           // our special type of object
  int32_t mAbsoluteOrRelativeTo;        // The given location(s) are: 0=absolute 1=relative to current camera, 2=relative to current vehicle, all other values reserved for possible future use

  // timestamp
  double mElapsedTime;                  // session time for the following state data

  // linear position, velocity, and acceleration
  bool mPosYValid;                      // whether the y (height) coordinate is valid (if invalid, we will automatically attempt to use HAT database)
  bool mVelValid;                       // whether velocity is valid (if invalid, we will automatically attempt to derive from multiple samples)
  bool mAccValid;                       // whether acceleration is valid (if invalid, we will automatically attempt to derive from multiple samples)
  TelemVect3 mPos;                      // position (x and z values must *always* be valid)
  TelemVect3 mVel;                      // world linear velocity
  TelemVect3 mAcc;                      // world linear acceleration

  // orientation, rotation
  bool mOriValid;                       // whether the orientation matrix is valid (if invalid, we will attempt to estimate from other information, including type)
  bool mHeadingValid;                   // only if full orientation is invalid, we'll try to use this simple heading if provided
  bool mRotValid;                       // whether angular velocity is valid (if invalid, we will automatically attempt to derive from multiple samples)
  TelemVect3 mOri[ 3 ];                 // orientation matrix (use conversion from TelemQuat if desired)
  double mHeading;                      // heading angle, in radians
  TelemVect3 mRot;                      // world angular velocity

  // TODO: automatically perform mesh animation as well?

  double mCenterHeightAboveGround;      // *** ALL TYPES *** - required for vertical placement using HAT if !mPosYValid
  double mWheelBase;                    // BICYCLE/MOTORCYCLE/ANIMAL (paw wheelbase) - required for calculating orientation if !mOriValid

  //
  unsigned char mExpansion2[ 384 ];     // for future use

  // Internal data only (possibly made obsolete in future builds)
  bool mInternalValid;
  bool mInternalPrevValid;
  bool mInternalUnused[ 6 ];
  double mInternalET;
  TelemVect3 mInternalPos;
  TelemVect3 mInternalVel;
  TelemVect3 mInternalOri[ 3 ];
  TelemVect3 mInternalRot;

  //
  unsigned char mExpansion3[ 64 ];      // for future use
};

struct AnimationDataVariableV01         // ACMD_EXPOSUREBIAS
{
  unsigned char mApplyMultiChannel;     // whether to pass this variable out on the network to other multi-channel nodes (i.e. physics, IGs, servers); 0=no, 1=yes, 2=any channel spectating mSpectateSlotID
  bool mUnused[ 3 ];                    //
  int32_t mIndex;                       // index (for exposure bias, this is the tonemapper index if known, otherwise use -1 for default); -1 = default or invalid
  int32_t mProfile;                     // profile (for exposure bias, this is the tonemapper profile if known, otherwise use -1 for default); -1 = default or invalid
  float mFloatValue;                    // the value to apply
  int32_t mSpectateSlotID;              // if mApplyMultiChannel==2, this setting is applied to any channel that is spectating this slot ID (you can set this ID to -1 to mean "whatever slot this node is viewing")

  //
  unsigned char mExpansion[ 492 ];      // for future use
};

struct AnimationDataLightV01            // ACMD_SETLIGHTPROPERTY
{
  char mParent[ 64 ];                   // name of light parent (if necessary for proper identification, otherwise empty)
  char mName[ 64 ];                     // name of light

  enum LightInstruction
  {
    SetDirectionByEulerAngles,          // pitch, yaw, roll
    SetDirectionByOriMatrix,            // 3x3 matrix
    SetDirectionByDirVector,            // x, y, z (assumes no roll); this method is not recommended
  };
  LightInstruction mInstruction;

  union
  {
    TelemVect3 mStartEuler;             // pitch, yaw, roll
    TelemVect3 mStartOri[3];            // 3x3 matrix
    TelemVect3 mStartDir;               // x, y, z (assumes no roll)
  };
  union
  {
    TelemVect3 mEndEuler;               // pitch, yaw, roll
    TelemVect3 mEndOri[3];              // 3x3 matrix
    TelemVect3 mEndDir;                 // x, y, z (assumes no roll)
  };
  double mStartElapsedTime;             // this is the session time at which to set the light to its start state (the special value 0.0 means "right now")
  double mEndElapsedTime;               // this is the session time at which to set the light to its end state (0.0 means "invalid", i.e. we are only setting the light once; otherwise it should be a value slightly bigger than mStartElapsedTime, even if mStartElapsedTime == 0.0)

  unsigned char mExpansion[ 128 ];      // for future use
};

struct AnimationActionV01               //
{
  AnimationCommandV01 mCommand;         // what action to perform

  // command-specific data
  union
  {
    AnimationDataReadSceneV01           mReadScene;         // ACMD_READSCENE
    AnimationDataLoadInstanceV01        mLoadInstance;      // ACMD_LOADINSTANCE
    AnimationDataUnloadInstanceV01      mUnloadInstance;    // ACMD_UNLOADINSTANCE
    AnimationDataSetInstanceV01         mSetInstance;       // ACMD_SETINSTANCELOCATION, ACMD_SETINSTANCEVISIBILITY
    AnimationDataControlBonesV01        mControlBones;      // ACMD_RESETANIMATION, ACMD_STARTANIMATION, ACMD_STOPANIMATION, ACMD_PAUSEANIMATION, ACMD_SETANIMATIONET, ACMD_SETANIMATIONFRAME, ACMD_LOADANIMATION, ACMD_UNLOADANIMATION, ACMD_SETBONENAMELOCATION, ACMD_SETBONEINDEXLOCATION
    AnimationDataSpecialV01             mSpecial;           // ACMD_SPECIAL
    AnimationDataVariableV01            mVariable;          // ACMD_EXPOSUREBIAS
    AnimationDataLightV01               mLight;             // ACMD_SETLIGHTPROPERTY
  };

  AnimationActionV01() { memset(this, 0, sizeof(AnimationActionV01)); }
};

struct GuideLinePointsV01
{
    TelemVect3 mPos;        // Position of the point
    double mAcceleration;   // Acceleration to be used at this point
    double mBrake;          // Brake level to be used at this point

    char mExpansion[36];
};

struct BoundingBoxV01
{
  // Mesh identification
  int32_t mMeshID;
  char mInstanceName[64];

  // 3D extents
  bool m3DExtentsValid;
  double mWorld3D[8][3];       // 8 corners of an oriented bounding box in world coordinates
  double mEye3D[8][3];         // 8 corners of an oriented bounding box in eye coordinates
  
  // 2D extents
  bool mPixelExtentsValid;
  int32_t mPixelScreenMin[2];  // minimum screen x/y coordinates
  int32_t mPixelScreenMax[2];  // maximum screen x/y coordinates

  // 2D extents from projecting the 3D bounding box onto the viewport
  bool mViewExtentsValid;
  int32_t mViewScreenMin[2];   // minimum screen x/y coordinates
  int32_t mViewScreenMax[2];   // maximum screen x/y coordinates

  uint32_t mSegmentationColor; // BGRA byte order. Can be cast directly to a SensorPixel_COLOR

  char mExpansion[252];
};

struct SplineNodeInfoV01
{
  float    mWorldPos[3];  // 3D world coordinates
  int32_t  mScreenPos[2]; // pixel coordinates
  uint32_t mFunctionType; // function type of the segment the node is on
  uint32_t mIndex;        // index of the node along the owning spline
  uint32_t mSplineID;     // ID of the owning spline
  bool     mOccluded;     // true if the node is occluded by another object in the scene

  char     mExpansion[35];
};

//--------------------------------------------------------------------------
struct RayVec3
{
  float x;
  float y;
  float z;
};

struct RayInput
{
  uint32_t  mFlags;           // Flags (for future expansion)
  RayVec3*  mOrigin;          // Ray origin
  RayVec3*  mDirection;       // Ray direction
  float*    mDistanceMin;     // (Optional) Minimum distance along ray (0 if null)
  float*    mDistanceMax;     // (Optional) Maximum distance along ray (+inf if null)

  char      mExpansion[256];

  RayInput()
  : mFlags(0),
    mOrigin(nullptr),
    mDirection(nullptr),
    mDistanceMin(nullptr),
    mDistanceMax(nullptr)
  {
    memset(mExpansion, 0, sizeof(mExpansion));
  }
};

struct RayOutput
{
  bool*     mHit;                // Did the ray hit anything?
  float*    mDistance;           // (Optional) Distance to the hit point
  RayVec3*  mPosition;           // (Optional) Position of hit
  RayVec3*  mShadingNormal;      // (Optional) Interpolated vertex normal at hit point
  RayVec3*  mGeometricNormal;    // (Optional) Triangle normal at hit point
  uint32_t* mMeshID;             // (Optional) ID of hit mesh
  uint32_t* mMaterialID;         // (Optional) ID of material at hit point (note: different values from mPerPixelMaterialID)
  uint32_t* mPerPixelMaterialID; // (Optional) per-pixel ID of material at hit point (note: different values from mMaterialID)

  bool*     mFunctionalGroundTruthHit;                // (Optional) Did the ray hit any functional ground truth geometry?
  uint32_t* mFunctionalGroundTruthMeshID;             // (Optional) ID of hit functional ground truth mesh

  char      mExpansion[232];

  RayOutput()
  : mHit(nullptr),
    mDistance(nullptr),
    mPosition(nullptr),
    mShadingNormal(nullptr),
    mGeometricNormal(nullptr),
    mMeshID(nullptr),
    mMaterialID(nullptr),
    mPerPixelMaterialID(nullptr),
    mFunctionalGroundTruthHit(nullptr),
    mFunctionalGroundTruthMeshID(nullptr)
  {
    memset(mExpansion, 0, sizeof(mExpansion));
  }
};

class RayCastInterface
{
public:
  virtual ~RayCastInterface() {}
  virtual void CastRays(uint32_t num, const RayInput& in, const RayOutput& out) = 0;
};

struct MeshInfoV01
{
  const char*         mName;          // Mesh name
  uint32_t            mID;            // Mesh ID

  char mExpansion[256];
};

struct InstanceInfoV01
{
  const char*         mName;          // Instance name
  uint32_t            mNumMeshes;     // Number of meshes that make up the instance
  const MeshInfoV01*  mMeshes;        // Information about each mesh

  char mExpansion[256];
};

struct MaterialInfoV01
{
  const char*         mName;          // Material name
  uint32_t            mID;            // Material ID

  char mExpansion[256];
};

struct RayCastInfoV01
{
  float mCameraPos[3];                // Camera position

  float mCameraRight[3];              // Camera orientation
  float mCameraUp[3];
  float mCameraForward[3];

  float mProjMatrix[4][4];            // Projection matrix

  uint32_t mWidth;                    // Dimensions of rendered image in pixels
  uint32_t mHeight;

  char mExpansion[1024];
};

//旼컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴커
// Specifically for Image Warp plugins
//읕컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴켸

#ifndef _IMAGEWARP_PLUGIN_H
 #define MHWDEVICE void*
 #define MHWSWAPCHAIN void*
 #define MHWTEXTURE void*
#else
 #ifdef GMOTOR_30
  #define MHWDEVICE ID3D11Device *
  #define MHWSWAPCHAIN IDXGISwapChain *         // Actually a IDXGISwapChain1. Cast if necessary.
  #define MHWTEXTURE ID3D11Texture2D *
 #else
  #define MHWDEVICE LPDIRECT3DDEVICE9
  #define MHWSWAPCHAIN LPDIRECT3DSWAPCHAIN9
  #define MHWTEXTURE LPDIRECT3DTEXTURE9
 #endif
#endif

// define some operator overloads for enums
#define NOTOVERLOAD(type)        inline type operator ~  (type e1) { const type e3 = (type)~(uint32_t)e1; return e3; }
#define OROVERLOAD(type)         inline type  operator |  (type e1,  type e2) { return ((type)((uint32_t)e1 | (uint32_t)e2)); }
#define ANDOVERLOAD(type)        inline type  operator &  (type e1,  type e2) { return ((type)((uint32_t)e1 & (uint32_t)e2)); }
#define OREQUALSOVERLOAD(type)   inline type &operator |= (type &e1, type e2) { e1 = (type)((uint32_t)e1 | (uint32_t)e2); return e1; }
#define ANDEQUALSOVERLOAD(type)  inline type &operator &= (type &e1, type e2) { e1 = (type)((uint32_t)e1 & (uint32_t)e2); return e1; }

enum ImageWarpFlagsV01
{
  IWARPV01_DEFAULT            = 0x0000L,
  IWARPV01_USEALTRENDERTARGET = 0x0001L,  // if set, use offscreen rendertarget
  IWARPV01_SWAPRENDERTARGETS  = 0x0002L,  // if set, swap rendertarget before PostDraw (required for gMotors double buffered post-processing)
  IWARPV01_UPDATERENDERTARGET = 0x0004L,  // if set, set rendertarget to "back buffer" every frame (should do in case current RT is not "back buffer"
  IWARPV01_ENDSCENEBEFOREWARP = 0x0008L,  // if set, call EndScene before PostDraw
  IWARPV01_OVERRIDEVIEWMATRIX = 0x0010L,  // if set, override view matrix instead of multiply

  IWARPV01_FORCEDWORD         = 0x7fffffffL,
};

// instantiate operator overloads for this enum
OROVERLOAD (ImageWarpFlagsV01); ANDOVERLOAD (ImageWarpFlagsV01); OREQUALSOVERLOAD (ImageWarpFlagsV01); ANDEQUALSOVERLOAD (ImageWarpFlagsV01);

// controls which targets to download to the CPU
enum ImageWarpFlagsV02
{
  IWARPV02_DOWNLOAD_COLOR_LDR_DATA = 0x0001L,       // final color output, after tonemapping. this is the image displayed on screen.
  IWARPV02_DOWNLOAD_COLOR_HDR_DATA = 0x0002L,       // primary render result, before tonemapping. it is the input to the tonemapper.
  IWARPV02_DOWNLOAD_RANGE_DATA = 0x0004L,           // each pixel contains distances to nearest surface.
  IWARPV02_DOWNLOAD_NORMAL_DATA = 0x0008L,          // each pixel contains surface normals.
  IWARPV02_DOWNLOAD_MESHID_DATA = 0x0010L,          // each pixel contains a unique identifier for different meshes.
  IWARPV02_DOWNLOAD_LIGHT_DATA = 0x0020L,           // each pixel contains light data.
  IWARPV02_DOWNLOAD_MATERIALID_DATA = 0x0040L,      // each pixel contains a unique identifier for different materials.
  IWARPV02_DOWNLOAD_FUNCTGROUNDTRUTH_DATA = 0x0080L,// each pixel contains a unique identifier for different functional ground truth meshes.

  IWARPV02_DOWNLOAD_FORCEDWORD = 0x7fffffffL,
};

// instantiate operator overloads for this enum
OROVERLOAD(ImageWarpFlagsV02); ANDOVERLOAD(ImageWarpFlagsV02); OREQUALSOVERLOAD(ImageWarpFlagsV02); ANDEQUALSOVERLOAD(ImageWarpFlagsV02);

// controls which targets to export device memory for
enum ImageWarpFlagsV03
{
  IWARPV03_EXPORT_DEVICE_MEM_LDR        = 0x0001L,
  IWARPV03_EXPORT_DEVICE_MEM_HDR        = 0x0002L,
  IWARPV03_EXPORT_DEVICE_MEM_RANGE      = 0x0004L,
  IWARPV03_EXPORT_DEVICE_MEM_MESHID     = 0x0008L,
  IWARPV03_EXPORT_DEVICE_MEM_NORMAL     = 0x0010L,

  IWARPV03_EXPORT_DEVICE_MEM_FORCEDWORD = 0x7fffffffL,
};

// instantiate operator overloads for this enum
OROVERLOAD(ImageWarpFlagsV03); ANDOVERLOAD(ImageWarpFlagsV03); OREQUALSOVERLOAD(ImageWarpFlagsV03); ANDEQUALSOVERLOAD(ImageWarpFlagsV03);

// undef these operator overloads 
#undef NOTOVERLOAD
#undef OROVERLOAD
#undef ANDOVERLOAD
#undef OREQUALSOVERLOAD
#undef ANDEQUALSOVERLOAD

// how target data is laid out in host/device memory
enum ImageWarpTargetFormatV02
{
  IWARPV02_TARGET_FORMAT_UNKNOWN,
  IWARPV02_TARGET_FORMAT_BGRA_UINT8,
  IWARPV02_TARGET_FORMAT_RGBA_UINT8,
  IWARPV02_TARGET_FORMAT_BGRA_UINT8_SRGB,
  IWARPV02_TARGET_FORMAT_RGBA_UINT8_SRGB,
  IWARPV02_TARGET_FORMAT_RGBA_FLOAT16,
  IWARPV02_TARGET_FORMAT_RGBA_FLOAT32,
  IWARPV02_TARGET_FORMAT_R_FLOAT32,
  IWARPV02_TARGET_FORMAT_RG_UINT32,
  IWARPV02_TARGET_FORMAT_RGB_UINT8,
  IWARPV02_TARGET_FORMAT_RGB_FLOAT32,
  IWARPV02_TARGET_FORMAT_R_UINT32,
  IWARPV02_TARGET_FORMAT_NO_DATA,

  IWARPV02_TARGET_FORMAT_FORCEDWORD = 0x7fffffffL
};

// whether target data is allocated on the host (CPU) or the device (GPU)
enum ImageWarpTargetMemoryV01
{
  IWARP_TARGET_MEMORY_HOST,
  IWARP_TARGET_MEMORY_DEVICE,

  IWARP_TARGET_MEMORY_FORCEDWORD = 0x7fffffffL
};

class ImageWarpInitDataV01
{
 public:

  char mServerIP[32];
  int32_t mServerPort;
  char mChannelName[256];
  double mChannelOffset;
  int32_t mTargetHeight;
  int32_t mTargetWidth;

  MHWDEVICE mD3DDevice;
  MHWTEXTURE mRenderTarget;

  ImageWarpInitDataV01() { memset( this, 0, sizeof( ImageWarpInitDataV01 ) ); }
};

class ImageWarpInitDataV02 : public ImageWarpInitDataV01
{
 public:

  MHWSWAPCHAIN mD3DSwapChain;

  ImageWarpInitDataV02() { memset( this, 0, sizeof( ImageWarpInitDataV02 ) ); }
};

class ImageWarpDynDataV01
{
 public:

  ImageWarpFlagsV01 mFlags;

  bool  mAppUseViewParams; // if true, app will use the plugin view parameters
  bool  mAppUseViewMatrix; // if true, app will use the plugin view matrix
  bool  mAppViewMatrixPre; // if true, app will pre-multiply the plugin view matrix, else post-multiply

  bool  mAppUseProjParams; // if true, app will use the plugin proj parametes
  bool  mAppUseProjMatrix; // if true, app will use the plugin proj matrix

  bool  mAppUseWrldMatrix; // if true, app will use the plugin wrld matrix

  bool  mPlgUseViewMatrix; // if true, plugin will init with the app view matrix
  bool  mPlgUseProjMatrix; // if true, plugin will init with the app proj matrix
  bool  mPlgUseWrldMatrix; // if true, plugin will init with the app wrld matrix

  // view parameters
  double mYaw;
  double mPitch;
  double mRoll;

  double mLeftAngle;
  double mRightAngle;
  double mBottomAngle;
  double mTopAngle;

  // proj parameters
  double mLeftClip;
  double mRightClip;
  double mBottomClip;
  double mTopClip;
  double mNearClip;
  double mFarClip;

  // view / proj matrix
  float mViewMatrix[4][4];
  float mProjMatrix[4][4];

  union
  {
    float mWorldOffset[4][4];
    float mSpares[16];
  };

  MHWTEXTURE mRenderTarget;

  ImageWarpDynDataV01() { memset( this, 0, sizeof( ImageWarpDynDataV01 ) ); }
};

// Device memory is exported as a HANDLE on Windows or an int File Descriptor (FD) on Linux
#ifdef _WIN32
typedef HANDLE DeviceMemoryHandle; // Windows handle
#else
typedef int DeviceMemoryHandle;    // Linux File Descriptor (FD)
#endif

// Memory allocated on a device (clients assume ownership of the handle)
union DeviceMemoryV01
{
  DeviceMemoryHandle mHandle;
  uint8_t            bits[8];  // so that it's always 8 bytes
};

struct ImageWarpTargetDataV02
{
  union
  {
    void*                  mPtr;              // target data downloaded to the CPU
    DeviceMemoryV01        mDeviceMemory;     // exported image memory allocated on a device (clients assume ownership of the handle)
    unsigned char          pad[8];            // pad this addition so it's always 8 bytes
  };                                          
  ImageWarpTargetFormatV02 mFormat;           // how the target data is laid out in memory
  int32_t                  mBytesPerRow;      // aka 'pitch'
  ImageWarpTargetMemoryV01 mMemory;           // whether data is allocated on the host (mPtr) or the device (mDeviceMemory)
  uint64_t                 mDeviceMemorySize; // Allocation size of device memory in bytes (only set if mMemory is IWARP_TARGET_MEMORY_DEVICE)

  unsigned char mExpansion[116];
};

class ImageWarpDynDataV02 : public ImageWarpDynDataV01
{
public:

  MHWTEXTURE mDepthTarget;
  // from here, all data must add up to 2048 bytes
  union
  {
    MHWTEXTURE mVelocityTarget;
    unsigned char ptrA64[8];   // pad this addition so it's always 8 bytes
  };
  union
  {
    MHWSWAPCHAIN mD3DSwapChain;
    unsigned char ptrB64[8];   // pad this addition so it's always 8 bytes
  };
  union
  {
    MHWTEXTURE mFinalTarget;
    unsigned char ptrC64[8];   // pad this addition so it's always 8 bytes
  };
  union
  {
    MHWTEXTURE mZBuffer;
    unsigned char ptrD64[8];   // pad this addition so it's always 8 bytes
  };
  union
  {
    bool mAppUseStereoProjMatrix;
    unsigned char boolA64[4];  // pad this addition so it's always 4 bytes
  };
  float mLeftProjMatrix[4][4];
  float mRightProjMatrix[4][4];
  union
  {
    MHWTEXTURE mNormalsTarget;
    unsigned char ptrE64[8];   // pad this addition so it's always 8 bytes
  };
  float mStereoIPD;            // interpupillary distance for active stereo 3D
  float mStereoEyeDist;        // distance eye to screen for active stereo 3D
  union
  {
    MHWTEXTURE mMeshIDTarget;
    unsigned char ptrF64[8];   // pad this addition so it's always 8 bytes
  };

  // Render ouputs already exported (device) or downloaded to the CPU (host)
  ImageWarpFlagsV02 mFlagsV02;
  ImageWarpTargetDataV02 mColorLDRData;
  ImageWarpTargetDataV02 mColorHDRData;
  ImageWarpTargetDataV02 mRangeData;
  ImageWarpTargetDataV02 mNormalData;
  ImageWarpTargetDataV02 mMeshIDData;
  ImageWarpTargetDataV02 mLightData;
  ImageWarpTargetDataV02 mMaterialIDData;

  // Dimensions of downloaded images
  int32_t mOutputWidth;
  int32_t mOutputHeight;

  // Device memory flags
  ImageWarpFlagsV03 mFlagsV03;

  // Additional render outputs
  ImageWarpTargetDataV02 mFunctGroundTruthData;

  unsigned char mExpansion[692];

  ImageWarpDynDataV02() { memset(this, 0, sizeof(ImageWarpDynDataV02)); }
};


//==========================================================================
// Plugin classes used to access internals
//==========================================================================

// Note: use class InternalsPluginV01 and have exported function GetPluginVersion() return 1, or
//       use class InternalsPluginV02 and have exported function GetPluginVersion() return 2, etc.
class InternalsPlugin : public PluginObject
{
 public:

  // General internals methods
  InternalsPlugin() {}
  virtual ~InternalsPlugin() {}

  // GAME FLOW NOTIFICATIONS
  virtual void Startup( long version ) {}                      // sim startup with version * 1000
  virtual void Shutdown() {}                                   // sim shutdown

  virtual void Load() {}                                       // scene/track load
  virtual void Unload() {}                                     // scene/track unload

  virtual void StartSession() {}                               // session started
  virtual void EndSession() {}                                 // session ended

  virtual void EnterRealtime() {}                              // entering realtime (where the vehicle can be driven)
  virtual void ExitRealtime() {}                               // exiting realtime

  // SCORING OUTPUT
  virtual bool WantsScoringUpdates() { return( false ); }      // whether we want scoring updates
  virtual void UpdateScoring( const ScoringInfoV01 &info ) {}  // update plugin with scoring info (approximately twice per second)

  // GAME OUTPUT
  // Possible return values of WantsTelemetryUpdates()
  //   0=no
  //   1=player-only
  //   2=all vehicles
  //   3=only vehicle being viewed
  //   4=all vehicles with detailed physics
  //   5=only vehicle being viewed with detailed physics
  virtual long WantsTelemetryUpdates() { return( 0 ); }        // whether we want telemetry updates (see above for possible return values)
  virtual void UpdateTelemetry( const TelemInfoV01 &info ) {}  // update plugin with telemetry info

  virtual bool WantsGraphicsUpdates() { return( false ); }     // whether we want graphics updates
  virtual void UpdateGraphics( const GraphicsInfoV01 &info ) {}// update plugin with graphics info

  // COMMENTARY INPUT
  virtual bool RequestCommentary( CommentaryRequestInfoV01 &info ) { return( false ); } // to use our commentary event system, fill in data and return true

  // GAME INPUT
  virtual bool HasHardwareInputs() { return( false ); }        // whether plugin has hardware plugins
  virtual void UpdateHardware( const double fDT ) {}           // update the hardware with the time between frames
  virtual void EnableHardware() {}                             // message from game to enable hardware
  virtual void DisableHardware() {}                            // message from game to disable hardware

  // See if the plugin wants to take over a hardware control.  If the plugin takes over the
  // control, this method returns true and sets the value of the double pointed to by the
  // second arg.  Otherwise, it returns false and leaves the double unmodified.
  virtual bool CheckHWControl( const char * const controlName, double &fRetVal ) { return false; }

  virtual bool ForceFeedback( double &forceValue ) { return( false ); } // alternate force feedback computation - return true if editing the value

  // VIDEO EXPORT [DEPRECATED - DO NOT USE]
  virtual bool WantsVideoOutput() { return( false ); }         // whether we want to export video
  virtual bool VideoOpen( const char * const szFilename, double fQuality, unsigned short usFPS, unsigned long fBPS,
                          unsigned short usWidth, unsigned short usHeight, char *cpCodec = NULL ) { return( false ); } // open video output file
  virtual void VideoClose() {}                                 // close video output file
  virtual void VideoWriteAudio( const short *pAudio, unsigned int uNumFrames ) {} // write some audio info
  virtual void VideoWriteImage( const unsigned char *pImage ) {} // write video image

  // ERROR FEEDBACK
  virtual void Error( const char * const msg ) {} // Called with explanation message if there was some sort of error in a plugin callback
};


class InternalsPluginV01 : public InternalsPlugin  // Version 01 is the exact same as the original
{
  // REMINDER: exported function GetPluginVersion() should return 1 if you are deriving from this InternalsPluginV01!
};


class InternalsPluginV02 : public InternalsPluginV01  // V02 contains everything from V01 plus the following:
{
  // REMINDER: exported function GetPluginVersion() should return 2 if you are deriving from this InternalsPluginV02!

 public:

  // VEHICLE CONTROL (PHYSICS/REPLAY)
  // To add vehicle, fill in data and return true.  You probably want to store the given ID so you can
  // correlate it with future callbacks.
  virtual bool WantsToAddVehicle( const long id, NewVehicleDataV01 &data ) { return( false ); }

  // InitVehicle() and ResetVehicle() are similar.  InitVehicle() is called only once upon vehicle load, while ResetVehicle()
  // is called immediately after InitVehicle() and then also whenever the vehicle is "reset" (user escapes back to the garage, or
  // the session changes, etc.).  ResetVehicle() must return a value indicating if the plugin wants to have some sort of control
  // over the vehicle.  These function also includes varied vehicle information which the plugin may wish to store, regardless of
  // whether it wants to control it.
  virtual void                  InitVehicle( const VehicleAndPhysicsV01 &data ) {}                       // called when vehicle is loaded
  virtual PluginControlRequest ResetVehicle( const VehicleAndPhysicsV01 &data ) { return( PCR_NONE ); }  // called when vehicle is reset
  virtual void UninitVehicle( const long id )                             {}                       // called when vehicle is unloaded

  // This function is called every time that the sim wants to set or reset the vehicle location (and stop vehicle).  This typically
  // happens right after the ResetVehicle() call, but it's possible to happen at other times.
  virtual void SetVehicleLocation( StartingVehicleLocationV01 &data ) {}

  // This essentially indicates that the engine starting procedure begin (the "instant" flag can be ignored; it is an aesthetic preference)
  virtual void StartVehicle( const long id, const bool instant ) {}

  // This function is called occasionally
  virtual void SetPhysicsOptions( PhysicsOptionsV01 &options ) {}

  // This is used to gather vehicle state override info for AI-controlled vehicles.  We will repeatedly call
  // this function, gathering any data points available for any vehicle, until it returns false.
  virtual bool GetVehicleState( VehicleStateV01 &data ) { return( false ); }

  // Use this function to apply additive forces to the default rFactor physics.  The force will be
  // applied at the vehicle's standardized CG (no fluids, wheels in default .PM positions), causing
  // very little or no torque.  The torque will be added to the body's other torques and the
  // engineTorque will be added directly to the current engine output.
  virtual bool AddPhysics( const long id, double et, PhysicsAdditiveV01 &add ) { return( false ); }

  // Tyre physics (only replaces tyre model in default rFactor physics)
  virtual void InitISOTyre( long id, ISOTyreInitV01 &init )                                       {}
  virtual bool ComputeISOTyreForces( long id, ISOTyreInputV01 &input, ISOTyreOutputV01 &output )  { return( false ); }

  // Differential physics (only replaces differential model in default rFactor physics)
  virtual void InitDifferential( long id, DifferentialInitV01 &init )                                              {}
  virtual bool ComputeDifferentialTransfer( long id, DifferentialInputV01 &input, DifferentialOutputV01 &output )  { return( false ); }

  // Only one plugin can receive RunPhysics() calls; rFactor arbitrarily chooses
  // one of the plugins that return the highest value for GetPhysicsRate().
  // Use Operator Console to set rate between 400Hz - 12kHz
  virtual long GetPhysicsRate( long id )                             { return( 0 ); } // now only used to activate RunPhysics() calls (and prioritize if multiple plugins return positive values); use 0 to disable plugin physics
  virtual void RunPhysics( long id, PhysicsInputV01 &input )         {} // run physics!
  virtual void GetPhysicsState( long id, PhysicsOutputV01 &output )  {} // output physics data
};


class InternalsPluginV03 : public InternalsPluginV02  // V03 contains everything from V02 plus the following:
{
  // REMINDER: exported function GetPluginVersion() should return 3 if you are deriving from this InternalsPluginV03!

 public:

  // RENDER CONTROL (Pre/Post draw)
  virtual long Init( void *pData ) { return( 0 ); }
  virtual long Post() { return( 0 ); }

  virtual long GetPreDrawParams( void *pData ) { return( 0 ); }
  virtual long SetPreDrawParams( void *pData ) { return( 0 ); }
  virtual long PreDraw() { return( 0 ); }

  virtual long GetPostDrawParams( void *pData ) { return( 0 ); }
  virtual long SetPostDrawParams( void *pData ) { return( 0 ); }
  virtual long PostDraw() { return( 1 ); }

  // EXTENDED VEHICLE CONTROL (PHYSICS/REPLAY)
  virtual bool WantsToDeleteVehicle( long &id )                       { return( false ); } // set ID and return true
  virtual unsigned char WantsToViewVehicle( CameraControlInfoV01 &camControl ) { return( 0 ); } // return values: 0=do nothing, 1=set ID and camera type, 2=replay controls, 3=both

  // EXTENDED GAME OUTPUT
  virtual void UpdateGraphics( const GraphicsInfoV02 &info )          {} // update plugin with extended graphics info

  // MESSAGE BOX INPUT
  virtual bool WantsToDisplayMessage( MessageInfoV01 &msgInfo )       { return( false ); } // set message and return true
};


//旼컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴커
//읕컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴켸

// See #pragma at top of file
#pragma pack( pop )

#endif // _INTERNALS_PLUGIN_HPP_

