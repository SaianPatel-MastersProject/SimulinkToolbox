#pragma once

#include "InternalsPluginExtras.hpp"
#include <cstdint>
#include <cstddef>

// NOTE: UDP packets will be expected with *Windows* byte ordering

// Compact the UDP structures
#pragma pack( push, 4 )

//
// Type IDs for packets
//
typedef uint32_t rFProMsgType;

const rFProMsgType RFPRO_PAUSE                       = 0;
const rFProMsgType RFPRO_RESUME                      = 1;
const rFProMsgType RFPRO_ALTESCAPE                   = 2;
const rFProMsgType RFPRO_ENTEREDREALTIME             = 3;
const rFProMsgType RFPRO_EXIT_REALTIME               = 4;
const rFProMsgType RFPRO_START_SESSION               = 5;
const rFProMsgType RFPRO_END_SESSION                 = 6;
const rFProMsgType RFPRO_ONLINE                      = 7;
const rFProMsgType RFPRO_KEY_PRESS                   = 8;

const rFProMsgType RFPRO_TELEMETRY                   = 0x32; // 50
const rFProMsgType RFPRO_TELEMETRY_OPPONENT          = 0x34; // 52
const rFProMsgType RFPRO_TELEMETRY_OPPONENT_CONFIG   = 0x35; // 53

const rFProMsgType RFPRO_PHYSICS_INPUT               = 0x64; // 100
const rFProMsgType RFPRO_PHYSICS_OUTPUT              = 0x65;
const rFProMsgType RFPRO_PHYSICS_SET_START_LOCATION  = 0x66;
const rFProMsgType RFPRO_REQUEST_START_LOCATION      = 0x67;
const rFProMsgType RFPRO_OPPONENT_STATES             = 0x68;
const rFProMsgType RFPRO_PROXIMITY_CONFIG            = 0x69;
const rFProMsgType RFPRO_PROXIMITY_STATES            = 0x6A;
const rFProMsgType RFPRO_CONTROL_INPUTS              = 0x6B;
const rFProMsgType RFPRO_ACTOR_INPUT                 = 0x6C;

const rFProMsgType RFPRO_SET_SCENE_LIGHTS            = 0x6E; // 110
const rFProMsgType RFPRO_SET_GENERAL_LIGHT           = 0x6F;

const rFProMsgType RFPRO_SET_ANIM_VISIBILITY         = 0x78; // 120

const rFProMsgType RFPRO_MOTION_PLATFORM_INFO        = 0x96; // 150
const rFProMsgType RFPRO_HEAD_TRACKING_INFO          = 0x97;

const rFProMsgType RFPRO_REMOVE_ADDITIONAL_VEHICLE   = 0x9F; // 159
const rFProMsgType RFPRO_ADD_ADDITIONAL_VEHICLE      = 0xA0;
const rFProMsgType RFPRO_UPDATE_ADDITIONAL_VEHICLE   = 0xA1;
const rFProMsgType RFPRO_NEW_VEHICLE_ID              = 0xA2;
const rFProMsgType RFPRO_EXT_RESET_VEHICLE           = 0xA3; // Input signal. Triggers a vehicle reset

const rFProMsgType RFPRO_FMOD_REGISTER_PARAM_UPDATER = 0xAA; // 170
const rFProMsgType RFPRO_FMOD_PARAM_ID               = 0xAB;
const rFProMsgType RFPRO_FMOD_UPDATE_PARAM           = 0xAC;

const rFProMsgType RFPRO_CAMERA_CONTROL_INFO         = 0xB4; // 180

const rFProMsgType RFPRO_BOUNDING_BOX_CONFIG         = 0xBE; // 190
const rFProMsgType RFPRO_BOUNDING_BOX                = 0xBF;

// Extract the Type ID from a packet
static inline const rFProMsgType rFPro_GET_MSG_TYPE( void *msg ) { return (*static_cast<rFProMsgType *>(msg)) ; }
static inline const void * rFPro_GET_PAYLOAD_PTR( const void *msg ) { return static_cast<const char *>(msg) + sizeof(rFProMsgType); }


//
// Basic types
//
struct UDPrFVector3
{
    double x ;
    double y ;
    double z ;
};

struct UDPDriverInputV01
{
    double mSteering;                // (-1.0-1.0)
    double mThrottle;                // (0.0-1.0)
    double mHandbrake;               // (0.0-1.0)
    double mBrakes;                  // (0.0-1.0)
    double mClutch;                  // (0.0-1.0)
    double mPowerDemand;             // KERS or other

    int32_t mDirectManualShift;         // -2="no selection"/invalid, -1=reverse, 0=neutral, 1+=forward gear
    bool mShiftUp;
    bool mShiftDown;
    bool mShiftToNeutral;
    bool mTCOverride;
    bool mLaunchControl;
};

struct UDPWheelOutputV01
{
    UDPrFVector3 mPos;               // Center of wheel, in world coordinates
    UDPrFVector3 mOri[3];            // Orientation matrix
    double mSlipAngle;               // Radians
    double mSlipRatio;               // Unitless, positive for traction. 0.0=0% slip, 1.0=100% slip
    double mTireLoad;                // Load on tire in Newtons
    double mRotation;                // Radians/sec
    double mBrakeTemp;               // Celsius
    double mTreadTemp[3];            // Left/center/right, Celsius
    double mAirTemp;                 // Celsius
};


//
// UDP packet structures
//
struct UDPGeneralMessageV01
{
    rFProMsgType messageType;
};      // 4 bytes

// Physics information from rFpro to Vehicle Model
struct UDPPhysicsInputMessageV01
{
    rFProMsgType messageType;        // =RFPRO_PHYSICS_INPUT

    UDPDriverInputV01 mDriverInputs; // Control input states

    // Timing - Use Operator Console to set rate in Hz
    double mDT;                      // Delta time to integrate through
    double mET;                      // Elapsed time (at *end* of physics frame, not beginning)

    // Environmental
    double mAirDensity;              // Current air density (which can be derived from: track altitude, ambient temp, humidity, and pressure)
    double mHumidity;                // Humidity (0.0-1.0)
    double mPressure;                // Air pressure (kPa)
    double mAmbientTemp;             // Temperature (Celsius)
    double mTrackTemp;               // Temperature (Celsius)
    UDPrFVector3 mWind;              // Wind velocity vector (m/s)

    // Collisions
    bool mCollision;                 // Collision detected by internal rFpro physics (data below)
    UDPrFVector3 mCollisionExitVector;  // An optional position change to be applied in order to exit the collision
    UDPrFVector3 mCollisionForce;    // Collision force to be applied to main vehicle body
    UDPrFVector3 mCollisionTorque;   // Collision torque to be applied to main vehicle body
};      // 220 bytes

struct UDPTyreContactV01
{
    double mSurfaceGain;                // friction multiplier. should be exactly 1.0 for 'average' asphalt
    double mHeight;                     // vertical position of the contact patch in world coordinates
    UDPrFVector3 mNormal;               // unit vector at normal to road surface
    unsigned char mSurfaceType ;        // 0=dry, 1=wet, 2=grass, 3=dirt, 4=gravel, 5=rumble strip (under the centre of the tyre)
    bool mHDvalid ;                     // TRUE=valid HD contact data, all data members valid; FALSE=mPatchCentre is NOT valid
    bool mSDvalid ;                     // TRUE=valid SD contact data; FALSE=beyond the world boundaries
    UDPrFVector3 mPatchCentre ;         // The position (in world coordinates) of the centre of the contact patch
};

struct UDPTyreContactExtras
{
    char mTerrainName[16];              // the material prefixes from the TDF file
};

struct UDPPhysicsInputMessageV02 : public UDPPhysicsInputMessageV01
{
    UDPTyreContactV01 mTyreContact[4];  // Tyre ground contact info (front left, front right, rear left, rear right)

    // Vehicle location state flags (bit fields)
    uint32_t mLocationFlags;         // bit 1: in pits. bits 2-32: unused.
    uint32_t mBeaconFlags;           // bits 1-6: beacon flags. bits 7-32: unused.

    UDPTyreContactExtras mTyreContactExtras[4];  // Extra tyre ground contact info (front left, front right, rear left, rear right)
};      // 564 bytes

struct UDPSlotStateV01                  // location and motion of a slot
{
  int32_t mID;                          // slot ID (note that it can be re-used in multiplayer after someone leaves)
  double mDistanceSquared;              // Square of distance to opponent (m^2)
  UDPrFVector3 mPos;                    // position
  UDPrFVector3 mOri[3];                 // orientation matrix (use conversion from TelemQuat if desired)
  UDPrFVector3 mVel;                    // velocity (in world coordinates, not local!)
  UDPrFVector3 mRot;                    // rotation (in world coordinates, not local!)
}; // 156 bytes

struct UDPOpponentStateMessageV01
{
    rFProMsgType messageType;           // =RFPRO_OPPONENT_STATES
    double mDistanceToNearest;
    uint32_t mNumNearOpponents; // Number of active/live opponents (Will be <= mLengthOpponents)
    uint32_t mLengthOpponents; // Number of UDPSlotStateV01 blocks in message
    // When message is constructed by rFpro, it will contain the array:
    // UDPSlotStateV01 mOpponentState[mLengthOpponents];
}; // 20 bytes + mLengthOpponents * UDPSlotStateV01

struct UDPTelemetryStreamMessageV01
{
    rFProMsgType messageType;           // =RFPRO_TELEMETRY
    uint16_t mNumParameters;
    uint16_t mParameterFlags;
    // When message is constructed by rFpro, it will contain the array:
    // double parameter[mNumParameters];
}; // 8 bytes + mNumParameters * 8

struct UDPTelemetryOpponentConfigV01
{
  rFProMsgType messageType;    // = RFPRO_TELEMETRY_OPPONENT_CONFIG
  char tag[8];                 // = [rFpro-]
  uint8_t version;             // version of packet
  bool reset;                  // if FALSE requested config is added to existing config
  bool config_str_NOT_mask;    // mask defintion is string (TRUE) or a bit mask (FALSE)
  bool number_NOT_id;          // opponent number is request for nearest n (TRUE) or a specific id (FALSE)
  int32_t opponent_number;     // requested vehicle(s)
  char mask_def[32];           // requested telemetry
};

struct UDPTelemetryOpponentV01
{
  rFProMsgType messageType;    // = RFPRO_TELEMETRY_OPPONENT
  char tag[8];                 // = "[rFpro-]" Note: The dash can be replaced to give a version number.
  uint16_t messageSize;        // = size of packet
  uint16_t numOpponents;       // = number of vehicles with specified data
  uint8_t bitMask[28];         // = bit mask indicating which telemetry is included for each vehicle
  uint16_t offsetFloat;        // = distance from end of header to float data (also = size of int32 data)
  uint16_t offsetDouble;       // = distance from end of header to double data 
  uint16_t offfsetByte;        // = distance from end of header to uint8 data
  uint16_t telemSize;          // = size of data for each vehicle (also = distance from end of header to 2nd vehicle)
}; // 52 bytes + blockSize*numOpponents for complete opponent telemetry message

// Physics information from Vehicle Model to rFpro
struct UDPPhysicsOutputMessageV01
{
    rFProMsgType messageType;        // =RFPRO_PHYSICS_OUTPUT

    // Timing and state info
    double mET;                      // Elapsed time (at which this output state is valid ... rFpro may extrapolate briefly to synch with graphics)
    bool mSkipInternalPhysics;       // Request internal physics to be skipped for performance reasons; note that most valid flags probably need to be true for this to work well
    bool mUseInternalPhysics;        // This indicates that plugin physics are temporarily invalid; rFpro should use its own internal calculations

    // Location and motion are required
    UDPrFVector3 mPos;               // Position (m)
    UDPrFVector3 mOri[3];            // Orientation matrix
    UDPrFVector3 mVel;               // Velocity (m/s) (in world coordinates, not local!)
    UDPrFVector3 mRot;               // Rotation (rad/s) (in world coordinates, not local!)

    bool mAccelValid;                // Whether mAccel is valid
    UDPrFVector3 mAccel;             // Linear acceleration (m/s2) (in world coordinates)

    // Wheel info
    bool mWheelPosValid;             // Whether wheel positions are valid
    unsigned char mWheelOriValid;    // 0=invalid, 1 or 2=full orientation (rFpro will *not* spin wheel other than for graphical extrapolation) 3=spindle axis only (stored in mOri[0]; rFpro will spin wheel)
    bool mTireParamsValid;           // Whether slip angle, slip ratio, and tire load are valid
    bool mWheelRotationValid;        // Whether rotations are valid
    bool mWheelBrakeTempValid;       // Whether brake temps are valid
    bool mWheelTreadTempValid;       // Whether tread temps are valid
    bool mWheelAirTempValid;         // Whether air temps are valid
    UDPWheelOutputV01 mWheel[4];     // Wheel info (front left, front right, rear left, rear right)

    // Engine, Inputs, Gear
    bool mGearValid;                 // whether gear is valid
    bool mRPMValid;                  // whether engine RPM is valid
    bool mInputsValid;               // whether throttle, brake, and steering are valid
    int32_t mGear;                      // -1=reverse, 0=neutral, 1+=forward gears
    double mEngineRPM;               // engine RPM (rotations per minute)
    double mThrottle;                // ranges  0.0-1.0
    double mBrake;                   // ranges  0.0-1.0
    double mSteering;                // ranges -1.0-1.0 (left to right)
    bool mSteerTorqueValid;          // whether steer torque is valid
    double mSteerTorque;             // the torque on the steering column (Nm)
    bool mHeadlightsValid ;          // whether HeadlightsOn is valid
    char mHeadlightsOn ;             // whether headlights are on (true) or off (false)

    bool mRotAccelValid;             // whether angular acceleration is valid
    UDPrFVector3 mRotAccel;          // rotational acceleration (radians/sec^2, in world coordinates)
};      // 948 bytes


// Startup information from rFpro to Vehicle Model
struct UDPStartingLocationMessageV01
{
    rFProMsgType messageType;       // =RFPRO_PHYSICS_SET_START_LOCATION
    UDPrFVector3 mPos;              // Position (see mPosOrigin for reference location)
    UDPrFVector3 mOri[3];           // Orientation matrix
    int32_t mPosOrigin;             // 0 = mPos specifies where the *front* of the vehicle should be at ground level; 1 = mPos specifies directly where the vehicle mPos should be
};      // 104 bytes


// Motion platform information from Vehicle Model to rFpro
struct UDPMotionPlatformInfoMessageV01
{
    rFProMsgType messageType;       // =RFPRO_MOTION_PLATFORM_INFO
    double mET;                     // Elapsed time (at which this output state is valid)
    UDPrFVector3 mPos;              // Position (relative to origin)
    UDPrFVector3 mOri[3];           // Orientation matrix
    UDPrFVector3 mVel;              // Velocity
    UDPrFVector3 mRot;              // Rotation
};      // 156 bytes

// Additional Vehicle information from Vehicle Model to rFpro
struct UDPNewVehicleDataV01
{
  char mVehFile[32];               // Name of the VEH file (no path, with or without extension .VEH)
  char mDriverName[32];            // If empty, will use default from VEH file
  unsigned char mUpgradePack[8];   // Coded upgrades (recommend to set to all zeroes if unknown)
  char mSkin[32];                  // Skin name (set empty for default)
  char mHelmet[32];                // Helmet name (set empty for default)
  uint32_t mLocalId;
};      // 140 bytes

struct UDPNewVehicleRegistrationV01
{
    uint32_t mLocalId;
    int32_t mRfPId;
};

struct UDPWheelStateV01
{
  double mRotation;                // radians/sec
  double mBrakeTemp;               // Celsius
  double mYLocation;               // wheel's y location relative to vehicle y location
};      // 24 bytes

struct UDPVehicleStateV01
{
  int32_t mID;                        // slot ID (note that it can be re-used in multiplayer after someone leaves)

  // this is the time at which this vehicle state is valid
  double mET;                      // elapsed time

  // special
  bool mPurgeFlag;                 // flag to remove any previously stored states (could be used to start a new lap, for example)
  bool mCollidable;                // can this vehicle be collided with?
  double mTransparentProximity;    // for automatic transparency: 0.0 = off, 2.0-62.0 = fully visible at this distance from the vehicle that camera is focusing on, and totally invisible at 1/4th of that distance
  double mManualTransparency;      // for manual transparency: 0.0 = visible, 1.0 = invisible, can be used to indicate confidence in data

  // location
  // Please see note above struct TelemInfoV01 for rFpro coordinate system and conversion info!
  bool mPosYValid;                 // whether the y (height) coordinate is valid (if invalid, will use HAT)
  bool mOriValid;                  // whether the orientation matrix is valid (if invalid, will use AI simulation)
  bool mHeadingValid;              // only if full orientation is invalid, we'll try to use simple heading instead
  UDPrFVector3 mPos;             // Position (x and z values must *always* be valid)
  UDPrFVector3 mOri[3];          // Orientation matrix (use conversion from TelemQuat if desired)
  double mHeading;                 // Heading angle, in radians

  // motion
  bool mVelValid;                  // whether velocity is valid (if invalid, will use AI simulation)
  bool mRotValid;                  // whether rotation is valid (if invalid, will use AI simulation)
  UDPrFVector3 mVel;             // World velocity (sorry, telemetry reports local velocity but we can't convert if optional orientation is invalid!)
  UDPrFVector3 mRot;             // World rotation (sorry, telemetry reports local rotation but we can't convert if optional orientation is invalid!)

  // engine, input, gear
  bool mGearValid;                 // whether gear is valid
  bool mRPMValid;                  // whether engine RPM is valid
  bool mInputsValid;               // whether throttle, brake, and steering are valid
  int32_t mGear;                      // -1=reverse, 0=neutral, 1+=forward gears
  double mEngineRPM;               // engine RPM (rotations per minute)
  double mThrottle;                // ranges  0.0-1.0
  double mBrake;                   // ranges  0.0-1.0
  double mSteering;                // ranges -1.0-1.0 (left to right)

  // wheel info
  bool mWheelRotationsValid;       // whether wheel rotations are valid
  bool mWheelBrakeTempsValid;      // whether wheel brake temps are valid
  bool mWheelYLocationsValid;      // whether wheel y locations are valid
  UDPWheelStateV01 mWheel[4];         // wheel information

  char mBrakelights; // brake light override: 0=invalid (rF control), 1=force off, 2=force on
  char mHeadlights; // headlight override: 0=invalid (rF control), 1=force off, 2=force on
};      // 336 bytes

static const size_t MAX_FMOD_PARAM_NAME_LENGTH = 50;

struct UDPFmodParameterRegistrationV01
{
    int32_t mId;
    char mName[MAX_FMOD_PARAM_NAME_LENGTH];
};

struct UDPFmodParameterUpdateV01
{
    int32_t mId;
    float mNewValue;
};

typedef struct UDPFmodParameterUpdateV02 : public UDPFmodParameterUpdateV01
{
    char mName[MAX_FMOD_PARAM_NAME_LENGTH];
} UDPFmodParameterUpdateV02;


struct UDPLightControlV01
{
    //rFProMsgType messageType;        // =RFPRO_SET_GENERAL_LIGHT
    bool mOnVehicle;
    bool mExactTrueOrWildcardFalse;
    char mName[32];
    int32_t mState;
}; // 40 Bytes + 4 when messageType is included

struct UDPLightControlV02 : public UDPLightControlV01
{
    char mVersion;
    bool mPosValid;
    bool mDirValid;
    bool mOriValid;
    bool mIntensityValid;
    bool mColorValid;

    float mPos[3];
    float mDir[3];
    float mOri[3];
    float mIntensity;
    char mColor[3];
    char mSlotID;
};

struct UDPSceneLightStates
{
    //rFProMsgType messageType;        // =RFPRO_SET_SCENE_LIGHTS
    int32_t mStartLights;
    int32_t mPitEntry;
    int32_t mPitExit;
    int8_t mStartAmber;
    int8_t mStartGreen;
    int8_t mFlagState;
    int8_t mFlagPostId; // Unimplemented
    int8_t mPitCrew;
    int8_t mSpare1;
    int8_t mSpare2;
    int8_t mSpare3;
    UDPSceneLightStates() : mPitCrew(SceneLightStatesV01::SLS_IGNORE), mSpare1(0), mSpare2(0), mSpare3(0) {}
};

struct UDPProximityQueryConfigsV01
{
    rFProMsgType messageType;           // =RFPRO_PROXIMITY_CONFIG
    uint32_t mLength;                   // Number of UDPProximityQueryConfigV01 blocks in message
};  // 8 bytes + mLength * sizeof(UDPProximityQueryConfigV01)

struct UDPProximityQueryConfigV01
{
    UDPrFVector3 mStart;
    UDPrFVector3 mRay;
};  // 48 bytes

struct UDPProximityQueryResultsV01
{
    rFProMsgType messageType;           // =RFPRO_PROXIMITY_STATES
    uint32_t mLength;                   // Number of UDPProximityQueryResultV01 blocks in message
};  // 8 bytes + mLength * sizeof(UDPProximityQueryResultV01)

struct UDPProximityQueryResultV01
{
    bool mValid;                        // Whether any object was detected within range. If false, no other members are valid.
    double mRange;                      // Distance from query point to surface of closest object (metres)
    UDPrFVector3 mNormal;               // Unit vector at normal to object surface (world coordinates)
    char mInstanceName[20];             // Name of the object instance (possibly truncated)
    char mMaterialName[20];             // Name of the surface material (possibly truncated)
};  // 76 bytes

struct UDPControlInputsV01
{
    double mThrottleTravel;          // 0..1
    double mBrakePressure[2];        // front | rear 0..1
    double mClutchTravel;            // 0..1
    uint32_t mGear;                  // -1..9 (set outside this range to use the sequential shift booleans instead)
    double mSteerAngle;              // -1..1 (left = -1)
    bool mNeutral;
    bool mUpshift;
    bool mDownshift;
};

struct UDPSpecialActorStateV01
{
    double mET;
    char mName[24];
    TelemVect3 mPos;
    double mHeading;
    bool mHeadingValid;
    char mSpecialType;
    char mType[24];
    bool mLoadActorFile;
    char mHeightSetting;  // 0 - auto @ ground height, 1 = rel to ground, 2 = abs height

    UDPSpecialActorStateV01()
    {
        mName[0] = 0;
        mType[0] = 0;
    }
};

struct UDPSpecialActorStateV02 : public UDPSpecialActorStateV01
{
    uint8_t mVersion;
    bool mOriValid;
    uint8_t mAnimationControlFlags;  // Combination of bitmask. See UDPSpecialActorStateV03
    uint8_t mSpare[5];
    UDPrFVector3 mOri[3];
};

struct UDPSpecialActorStateV03 : public UDPSpecialActorStateV02
{
    // Manual animation control. Otherwise automatic animation control based on the actor's speed.
    static constexpr uint8_t ANIMATION_CONTROL_MANUAL = 0x01;
    // Stay at the last frame when reaching the end of animation. Otherwise play the animation in loop.
    static constexpr uint8_t ANIMATION_CONTROL_ONE_SHOT = 0x02;
    // Animation frame is manually specified using mDirectAnimationFrame. Otherwise automatically calculated using mPlaybackRate.
    static constexpr uint8_t ANIMATION_CONTROL_DIRECT_FRAME = 0x04;

    char mAnimationState[24];
    float mPlaybackRate;  // 1.0: normal speed  0.0: pause  <0.0: reverse
    int32_t mDirectAnimationFrame;  // Animation frame
};

struct UDPAnimationControlV01
{
  // UDP packet will include:
  // rFProMsgType messageType;  = RFPRO_SET_ANIM_VISIBILITY (120) at present
  uint8_t mVisible;
  uint8_t mUnused[3];
  char mParent[64];
  char mInstance[64];
};

struct UDPCameraControlInfoV01
{
  // Cameras
  uint32_t mID;                  // slot ID to view
  uint32_t mCameraType;          // see GraphicsInfoV02 comments for values
  // Replays (note that these are asynchronous)
  bool mReplayActive;            // This variable is an *input* filled with whether the replay is currently active (as opposed to realtime).
  bool mReplayUnused;            //
  unsigned char mReplayCommand;  // 0=do nothing, 1=begin, 2=end, 3=rewind, 4=fast backwards, 5=backwards, 6=slow backwards, 7=stop, 8=slow play, 9=play, 10=fast play, 11=fast forward
  bool mReplaySetTime;           // Whether to skip to the following replay time:
  double mReplaySeconds;          // The replay time in seconds to skip to (note: the current replay maximum ET is passed into this variable in case you need it)
  bool mApplyMultiChannel;       // whether to apply to all other channels
  char mUnused[3];               //
};

struct UDPRemoteKeyPress
{
  char mControlName[30];
};

struct UDPRemoteKeyPressV02
{
  char mControlName[32];
  double mControlValue;
};

struct UDPBoundBoxListV01
{
  // Header
  rFProMsgType messageType;   // = RFPRO_BOUNDING_BOX_CONFIG  or  RFPRO_BOUNDING_BOX
  uint32_t mLength;
}; // 8 byte header to be followed by 64 or 104 byte elements

struct UDPBoundBoxListV02 : UDPBoundBoxListV01
{
  uint16_t mVersion;     // Future proofing (version 0 => mesh group is always zero)
  uint16_t mMeshGroup;   // defines group number in a config message
  uint8_t mUnused[8];    // Space for expansion
}; // 8 + 12 = 20 byte header to be followed by 64 or 104 byte elements

struct UDPBoundBoxConfigItemV01
{
  char mInstanceName[64];
};

struct UDPBoundBoxV01
{
  int32_t mMeshID;
  char mUnused[3];          // Potentially include valid flags for pixel and view extents if data added to end
  bool m3DExtentsValid;     // Does this packet have 3D data
  double mWorld3D[4][3];    // 4 corners of an oriented bounding box in world coordinates
}; // 104 bytes

// See #pragma at top of file
#pragma pack( pop )
