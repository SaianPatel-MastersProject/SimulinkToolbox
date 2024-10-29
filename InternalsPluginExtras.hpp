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

#ifndef _InternalsPluginEXTRAS_H
#define _InternalsPluginEXTRAS_H

#include "InternalsPlugin.hpp"
#include <string>
#include <vector>
#include <cstdint>

// rF currently uses 4-byte packing ... whatever the current packing is will
// be restored at the end of this include with another #pragma.
#pragma pack( push, 4 )

struct MotionPlatformV01
{
    double mET;                     // time at which this state was valid
    TelemVect3 mPos;                // position (relative to origin)
    TelemVect3 mOri[3];             // orientation matrix
    TelemVect3 mVel;                // linear velocity
    TelemVect3 mRot;                // angular velocity (instantaneous about each axis)
    void Clear()
    {
        for (uint32_t i = 0; i < sizeof(MotionPlatformV01); ++i)
            ((char *) this)[i] = 0;
        mOri[0][0] = mOri[1][1] = mOri[2][2] = 1;
    }

};

struct ControlInputsV01
{
    double throttleTravel;          // 0..1
    double brakePressure[2];        // front | rear 0..1
    double clutchTravel;            // 0..1
    int32_t gear;                   // -1..9 (set outside this range to use the sequential shift booleans instead)
    double steerAngle;              // -1..1 (left = -1)
    bool neutral;
    bool upshift;
    bool downshift;

    bool spare;                     // reserved for future use
    void Clear() { for( uint32_t i = 0; i < sizeof( ControlInputsV01 ); ++i ) ( (char *) this )[ i ] = 0; }
};

struct ControlInputsV02 : ControlInputsV01
{
    double mSteerVel;
    double mSteerAcc;
    double mSteerTorque;            // Nm (only used if mSteerByTorque is true)

    bool mSteeringRatesValid;       // if true, steering derivates are valid
    bool mSteeringAbsolute;         // if true, steering and derivates are in radians, otherwise -1..+1
    bool mSteerByTorque;            // if true, mSteerTorque is valid. steerAngle value is ignored
    bool mPedalsInvalid;            // NOTE: *IN*valid. if true, throttle, brake, clutch values are ignored (allows steering-only control plugins)
    bool mSteerAngleInvalid;        // NOTE: *IN*valid. if true, steerAngle value is ignored (allows pedal-only control plugins)

    char mExpansion[115];
    void Clear() { for( uint32_t i = 0; i < sizeof( ControlInputsV02 ); ++i ) ( (char *) this )[ i ] = 0; }
};

struct SimOptionsV01 
{
    int32_t driverID;
	  int32_t vehicleID;
	  int32_t trackID;
	  double muGain;
    bool runExternalPhysics;
};

struct ChassisContactPointDefnV01
{
    TelemVect3 mPos;                    // Offset from vehicle mPos in local vehicle coordinates
};

struct TyreContactPointDefnV01
{
    int32_t mTyreID;                    // Which wheel this is for
    double mWeighting;                  // The weighting of this point relative to the centre point
    TelemVect3 mPos;                    // Offset from centre of wheel contact patch in local wheel coordinates
};

struct ContactPatchV01
{
    double mSurfaceGain;                // friction multiplier. should be exactly 1.0 for 'average' asphalt
    double mHeight;                     // vertical position of the contact patch in world coordinates
    TelemVect3 mNormal;                 // unit vector at normal to road surface
};

struct ContactPatchV02 : ContactPatchV01
{
    double mLocalHeight;                // clearance between contact point definition and ground surface. Negative value for collision.
};

typedef enum
{
  HD_CONTACT_PATCH_POINT_SOURCE_MISSING = 0,
  HD_CONTACT_PATCH_POINT_SOURCE_LIDAR = 1,
  HD_CONTACT_PATCH_POINT_SOURCE_INTERPOLATED = 2,
  HD_CONTACT_PATCH_POINT_SOURCE_SDHD = 3
} HDContactPatchPointSource_t;

struct HDContactPatchV01 : ContactPatchV01
{
    bool mHDvalid;                      // TRUE=tyre nominal cylinder in contact with the HD surface; FALSE=not in contact with HD surface or no HD surface data present
    bool mSDvalid;                      // TRUE=valid SD contact data; FALSE=beyond the world boundaries
    TelemVect3 mPatchCentre;            // The position (in world coordinates) of the centre of the contact patch

    // Surface type information
    // This is the surface under the majority of tyre contact/sample points
    unsigned char mSurfaceType;         // 0=dry, 1=wet, 2=grass, 3=dirt, 4=gravel, 5=rumblestrip, 6=special, 7=runoff, 8=astroturf, 9=kerb, 10-19=spare0-spare9
    char mTerrainName[MAX_LENGTH_TERRAIN_NAME]; // the material prefix from the TDF file

    uint8_t mPointSourceProportions[8];  // The proportions out of 255 for each point source type used in the HD calculation. Indexed by HDContactPatchPointSource_t above.

    unsigned char mExpansion[39];       // for future use
};

struct TyreContactInputV01
{
    ContactPatchV01 mContactPatch[4];   // one for each wheel
};

class ChassisContactInputV01
{
public:
    uint32_t mNumPoints ;               // number of items in the array (=number of chassis contact points)
    ContactPatchV02 *mContactPoint;     // one for each chassis contact point

    ChassisContactInputV01() : mNumPoints(0), mContactPoint(NULL) {}
};

struct TyreDefinitionV01
{
    double mNomRadius ;                 // Nominal radius (m) of the tyre
    double mNomWidth ;                  // Nominal width (m) of the tyre

    unsigned char mExpansion[128] ;      // for future use
};

struct TyreDefinitionInputV01
{
    TyreDefinitionV01 mTyreDef[4];      // One for each wheel
};


typedef enum
{
    FMOD_EVT_UNKNOWN = -1,

    FMOD_EVT_SHIFT_UP = 0,
    FMOD_EVT_SHIFT_DOWN = 1,
    FMOD_EVT_SHIFT_NEUTRAL = 2,

    FMOD_EVT_MAX = 3
} FMOD_EventTrigger ;

struct FMOD_TelemetryInputs
{
    float rpm ;                 // Engine speed in revolutions per minute
    float throttle ;            // Throttle position varying from 0.0 to 1.0
    float brake ;               // Brake position varying from 0.0 to 1.0
    float clutch ;              // Clutch position varying from 0.0 to 1.0
    float velocity ;            // Longitudinal car speed in metres per second (+ve is forwards)
    float lap_distance ;        // Distance around the track in metres (zero = start/finish line)
    float edge_distance ;       // Distance between car mPos and the closest edge of the track in metres

    FMOD_TelemetryInputs()
        : rpm(0.0), throttle(0.0), brake(0.0), clutch(0.0), velocity(0.0), lap_distance(0.0), edge_distance(1000.0)
    {}
};

typedef enum
{
    FMOD_REVERB_OFF = 0 ,       // Turn off global reverb
    FMOD_REVERB_CUSTOM = 1      // Use FMOD_ReverbParams values for global reverb
} FMOD_ReverbPreset ;

struct FMOD_ReverbParams
{
    // Set Preset to FMOD_REVERB_OFF (and ignore other values) to turn global reverb off.
    // Set Preset to FMOD_REVERB_CUSTOM to turn on global reverb using the following values for settings.
    FMOD_ReverbPreset Preset ;

    // The information in brackets after each of these values are the units, minimum and maximum values.
    int     Room ;              // Room effect level at mid frequency (mB, -10000, 0)
    int     RoomHF ;            // Relative room effect level at high frequency (mB, -10000, 0)
    int     RoomLF ;            // Relative room effect level at low frequency (mB, -10000, 0)
    float   DecayTime ;         // Reverberation decay time at mid frequency (s, 0.1, 20.0)
    float   DecayHFRatio ;      // High frequency to mid frequency decay time ratio (ratio, 0.1, 2.0)
    int     Reflections ;       // Early reflections level relative to room effect (mB, -10000, 1000)
    float   ReflectionsDelay ;  // Early reflections delay time relative to room effect (s, 0.0, 0.3)
    int     Reverb ;            // Late reverberation level relative to room effect (mB, -10000, 2000)
    float   ReverbDelay ;       // Late reverberation delay time relative to intial reflection(s, 0.0, 0.1)
    float   HFReference ;       // Reference high frequency (Hz, 1000.0, 20000.0)
    float   LFReference ;       // Reference low frequency (Hz, 20.0, 1000.0)
    float   Diffusion ;         // Echo density in the late reverberation decay (%, 0.0, 100.0)
    float   Density ;           // Modal density in the late reverberation decay (%, 0.0, 100.0)

    FMOD_ReverbParams()
        : Preset( FMOD_REVERB_OFF )
    {}
} ;

struct SceneLightStatesV01
{
    int32_t mStartLights;
    int32_t mPitEntry;
    int32_t mPitExit;
    int8_t mStartAmber;
    int8_t mStartGreen;
    int8_t mFlagState;
    int8_t mFlagPostId; // Unimplemented
    int8_t mPitCrew;
    int8_t mSpare2a;
    int8_t mSpare2b;
    int8_t mSpare2c;
    int32_t mSpare3;
#ifdef __cplusplus
    SceneLightStatesV01()
        : mStartLights(SLS_IGNORE),
        mPitEntry(SLS_IGNORE),
        mPitExit(SLS_IGNORE),
        mStartAmber(SLS_IGNORE),
        mStartGreen(SLS_IGNORE),
        mFlagState(SLS_IGNORE),
        mFlagPostId(SLS_IGNORE),
        mPitCrew(SLS_IGNORE),
        mSpare2a(0), mSpare2b(0), mSpare2c(0),
        mSpare3(0)
    {}
#endif
    enum
    {
      SLS_RFP_CONTROL = -1,
      SLS_IGNORE = -2,
    };
};

const uint32_t SENSOR_FORMAT_NONE             = 0x00000000; // No output.
const uint32_t SENSOR_FORMAT_COLOR            = 0x00000001; // SensorPixel_COLOR: Screen-space color, 8-bits per color channel. This is the tone-mapped output
const uint32_t SENSOR_FORMAT_COLOR_HDR        = 0x00000002; // SensorPixel_COLOR_HDR: Full range HDR color, 32 bits per color channel, usually upscaled from 16 bits per color channel
const uint32_t SENSOR_FORMAT_RANGE            = 0x00000004; // SensorPixel_RANGE: Distance in metres from the camera (eye-point) to the object rendered at that pixel
const uint32_t SENSOR_FORMAT_MESHID           = 0x00000008; // SensorPixel_ID: Unique ID for mesh rendered at that pixel. See QueryMesh struct and SetAnimation callback.
const uint32_t SENSOR_FORMAT_COLOR_SEGMENTED  = 0x00000010; // SensorPixel_COLOR: Segmented color. Uses the [ColorFilterByName] color values.
const uint32_t SENSOR_FORMAT_NORMAL           = 0x00000020; // SensorPixel_VECTOR: Surface normal of each pixel
const uint32_t SENSOR_FORMAT_VELOCITY         = 0x00000040; // SensorPixel_VECTOR: Velocity of each pixel
const uint32_t SENSOR_FORMAT_LIGHT            = 0x00000080; // SensorPixel_LIGHT: RGB intensity, 32 bits per color channel usually upscaled from 16 bits per color channel, followed by number of light sources (32 bits float storing an int value)
const uint32_t SENSOR_FORMAT_MATERIALID       = 0x00000100; // SensorPixel_ID: Unique ID for material used by mesh rendered at that pixel.
const uint32_t SENSOR_FORMAT_FUNCTGROUNDTRUTH = 0x00000200; // SensorPixel_ID: Unique ID for functional ground truth mesh.
const uint32_t SENSOR_FORMAT_SEPARATE_DATA    = 0x10000000; // Each requested image will be provided in separate calls to SetSensorImage(). See SensorImageInfoV01 struct for more details.

// Flags for SensorImageSettingsV01.mDeviceFormat that control which images we want to export device memory for
// Value of SensorImageInfoV01.mDeviceImageFormat that identifies which type of sensor image is stored
const uint32_t SENSOR_DEVICE_FORMAT_NONE      = 0x00000000;
const uint32_t SENSOR_DEVICE_FORMAT_COLOR     = 0x00000001; // SensorPixel_COLOR_RGBA_UINT8
const uint32_t SENSOR_DEVICE_FORMAT_COLOR_HDR = 0x00000002; // SensorPixel_COLOR_HDR
const uint32_t SENSOR_DEVICE_FORMAT_RANGE     = 0x00000004; // SensorPixel_RANGE
const uint32_t SENSOR_DEVICE_FORMAT_MESHID    = 0x00000008; // SensorPixel_ID
const uint32_t SENSOR_DEVICE_FORMAT_NORMAL    = 0x00000010; // SensorPixel_VECTOR

struct SensorPixel_COLOR
{
    unsigned char blue;
    unsigned char green;
    unsigned char red;
    unsigned char unused;
};

struct SensorPixel_COLOR_RGBA_UINT8
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t unused;
};

struct SensorPixel_COLOR_HDR
{
    float red;
    float green;
    float blue;
    float unused;
};

struct SensorPixel_RANGE
{
    float range;
};

struct SensorPixel_ID
{
    uint32_t id;
};

struct SensorPixel_VECTOR
{
    float x;
    float y;
    float z;
    float extra;
};

struct SensorPixel_LIGHT
{
  float red;
  float green;
  float blue;
  float numLights;
};

struct SensorImageSettingsV01
{
    // Format to output sensor image data in.
    // Add together all required formats. SensorPixel_* structs will be packed into a single buffer. See SensorImageInfoV01.
    // No packing is done on exported device mem
    uint32_t mFormat;
    uint32_t mDeviceFormat; // Which images we should export device memory for

    unsigned char mExpansion[ 252 ];    // future use
};

struct SensorImageInfoV01
{
    union
    {
        unsigned char  *mData;          // Pointer to start of image memory location. Pixels are in row-major order, starting with top-left corner.
                                        // If SENSOR_FORMAT_SEPARATE_DATA is set, contains data for only a single type of sensor: each pixel contains a single SensorPixel_* struct.
                                        // In this case, SetSensorImage() will be called multiple times, each time with a different sensor data. Check mSeparateImageFormat to determine the kind of sensor data.
                                        // 
                                        // If SENSOR_FORMAT_SEPARATE_DATA is NOT set, each pixel contains set of SensorPixel_* structs. The packing order is the numerical order of the SENSOR_FORMAT_* values.
                                        // For example if mFormat is (SENSOR_FORMAT_RANGE|SENSOR_FORMAT_COLOR) then each pixel can be cast to:  struct { SensorPixel_COLOR color; SensorPixel_RANGE range; };
        
        DeviceMemoryV01 mDeviceMemory;  // Exported image memory allocated on a device (clients assume ownership of the handle)
    };
    

    int32_t mBytesPerPixel;             // Number of bytes for each pixel
    int32_t mBytesPerRow;               // Number of bytes for each pixel row (Note: row may be padded)
    int32_t mWidth;                     // Image pixel width
    int32_t mHeight;                    // Image pixel height
                                              
    uint32_t mSeparateImageFormat;      // Identifies which type of sensor image is stored in mData (not set if using mDeviceMemory)
                                        // Only used when SENSOR_FORMAT_SEPARATE_DATA is set
                                              
    uint32_t mDeviceImageFormat;        // Identifies which type of sensor image is stored in device memory (SENSOR_DEVICE_FORMAT_NONE if using mData)
    uint64_t mDeviceMemorySize;         // Allocation size of device memory in bytes (not set if using mData)
                                              
    unsigned char mExpansion[ 240 ];    // future use
};

struct ProximityQueryPointDefnV01
{
    TelemVect3 mPos;                    // Offset from vehicle mPos in local vehicle coordinates
    TelemVect3 mRay;                    // Vector (direction and range) to scan for objects in local vehicle coordinates

    unsigned char mExpansion[ 128 ];    // future use
};

struct ProximityQueryPointV01
{
    bool mValid;                        // Whether any object was detected within range. If false, no other members are valid.
    double mRange;                      // Distance from query point to surface of closest object
    TelemVect3 mNormal;                 // Unit vector at normal to object surface (world coordinates)
    char mInstanceName[20];             // Name of the object instance (possibly truncated)
    char mMaterialName[20];             // Name of the surface material (possibly truncated)

    unsigned char mExpansion[ 256 ];    // future use
};

struct ProximityQueryInputV01
{
    uint32_t mNumPoints ;               // number of items in the array (=number of proximity query points)
    ProximityQueryPointV01 *mProximityQueryPoint;     // one for each proximity query point

    unsigned char mExpansion[ 128 ];    // future use
};


namespace rFpro
{
    enum DataModelType
    {
        Base = 0,
        Traffic = 1,
        Sound = 2
    };

    class SimControlDataModel
    {
    public:
        SimControlDataModel(const DataModelType type = Base, const unsigned ver = 0) : mType(type), mVersion(ver) {}

        virtual ~SimControlDataModel() {}

        unsigned Version() const { return mVersion; }
        DataModelType Type() const { return mType; }

    private:
        const DataModelType mType;
        const int mVersion;
    };

    class DataModelProvider
    {
    public:
        virtual ~DataModelProvider() {}
        virtual SimControlDataModel * GetDataModelConnection(const DataModelType requiredType, const int requiredVersion) = 0;
    };
}

//==========================================================================



class InternalsPluginV03Extras : public InternalsPluginV03			// Return a 3 when creating the plugin in your .cpp file
{
public:
    /* To provide motion states return true to WantsMotionStates() and complete the struct
    in UpdateMotionState() it is acceptable to return the same state more than once 
	if this function runs faster than your motion state is updated */
    virtual bool WantsMotionStates() { return( false ); } 
    virtual void UpdateMotionState(MotionPlatformV01 &motionInfo) {}

    /* To override the control inputs return true to WantsToOverrideControlInputs() and complete
    the struct in OverrideControlInputs(). Cast the parameter to ControlInputsV02 to use the expanded fields */
    virtual bool WantsToOverrideControlInputs() { return( false ); }
    virtual void OverrideControlInputs(ControlInputsV01 &controlInfo) {}

    /* We make this call at startup, having already received it from Admin console via IP  */
    virtual void SetSimOptions(SimOptionsV01 &info) {}

    /* Pause() is called when SimControl receives a pause request via UDP. RequestResume will be polled
    when the simulation is paused, return true if you are ready to re-enter real-time.  If you return false
	the call will be repeated until you return true.  The function passes as a parameter the last (paused) vehicle state
	so that you may use the position and orientation for re-initialization of the model if required.*/
    virtual void Pause() {}
    virtual bool RequestResume(PhysicsOutputV01 &pausedState) { return( true ); }

};

class InternalsPluginV04Extras : public InternalsPluginV03Extras	// Return a 4 when creating the plugin in your .cpp file
{
public:
	/* Track Utility functions */
	virtual bool WantsLocalGeometry(char licenceCode[128] )							{ return( true );  }	// wants geometry local to vehicle in realtime (presumably returns the opposite value of WantsFullGeometry())
	virtual bool WantsFullGeometry( unsigned long checksum, char licenceCode[128] ) { return( false ); }	// wants full array of track geometry at loading time (checksum provided in case data is stored offline)
	virtual void SetFullGeometry( long numTriangles, PhysicsTriangleV01 *triangle ) {}						// provides one massive array of triangles (plugin is responsible for realtime searching)
};

class InternalsPluginV05Extras : public InternalsPluginV04Extras    // Return a 5 when creating the plugin in your .cpp file
{
public:
    /* Road surface pre-processing
    Used to get track geometry/contact patch surface data for input into the physics model without passing the mesh around.
    If WantsTyreContactUpdates has returned non-zero, UpdateTyreContact() will be called immediately before RunPhysics() with
    data calculated from that frame's mTrackGeometryForVehicle.
    !! NOTE !!: In addition to WantsTyreContactUpdates(), the output from GetPhysicsState must include valid wheel positions.
                If valid wheel positions are provided, but not valid wheel orientations then the vehicle body orientation will be used for each wheel.
    If WantsChassisContactUpdates has returned non-zero, UpdateChassisContact() will be called immediately before RunPhysics() with
    data calculated from that frame's mTrackGeometryForVehicle. Valid wheel positions are not required for chassis points.
    */
    virtual long WantsTyreContactUpdates( long maxNumPoints, TyreContactPointDefnV01 *points ) { return( 0 ); }
                // maxNumPoints is the maximum possible number of points available (and indicates the size of the points array).
                // Return the number of points populated in the array and to be passed back in the UpdateTyreContact function.
                // The points array is pre-initialised with the centre points of each of the 4 tyres. Simply return 4 to use this single-point-sample definition.
                // Return 0 to disable this functionality.
                // !! NOTE !!: See above that the output from GetPhysicsState must include valid wheel positions.
    virtual void UpdateTyreContact( const TyreContactInputV01 &input )  {}
                // Provides the plugin with the information requested in WantsTyreContactUpdates.
                // Called immediately before RunPhysics().
    virtual long WantsChassisContactUpdates( long maxNumPoints, ChassisContactPointDefnV01 *points ) { return( 0 ); }
                // maxNumPoints is the maximum possible number of points available (and indicates the size of the points array).
                // Return the number of points populated in the array and to be passed back in the UpdateChassisContact function.
                // Return 0 to disable this functionality
    virtual void UpdateChassisContact( const ChassisContactInputV01 &input )  {}
                // Provides the plugin with the information requested in WantsChassisContactUpdates.
                // Called immediately before RunPhysics().

};

class InternalsPluginV06Extras : public InternalsPluginV05Extras    // Return a 6 when creating the plugin in your .cpp file
{
public:
    /* FMOD Audio Control API
    To enable the sub-plugin to have control over the FMOD audio sub-system. */
    virtual bool FMOD_Update( FMOD_TelemetryInputs &telemetry_params ) { return false ; }
                // Called at UpdateTelemetry rate immediately after sub-plugin call to UpdateTelemetry.
                // The structure is pre-populated with the values that will be sent to FMOD.
                // Note that the structure is only guaranteed to be populated with parameters that are used by the FMOD sounds defined in FMODplugin.ini.
                // Adjust the structure values to control the parameter values before being passed to FMOD audio sub-system.
                // Return true to use the modified values. Return false to use original values.
                // Example application: to set the audio engine rpm separately to the physics model's engine rpm.

    virtual FMOD_EventTrigger FMOD_Play() { return FMOD_EVT_UNKNOWN ; }
                // Called at UpdateTelemetry rate immediately after sub-plugin call to UpdateTelemetry.
                // Return a value between FMOD_EVT_UNKNOWN and FMOD_EVT_MAX to trigger the event.
                // All FMOD sounds with the matching event trigger will start (see FMODplugin.ini).
                // NOTE: To enable multiple events to be triggered per time step, this function is called repeatedly
                //       per time step until an FMOD_EVT_UNKNOWN or FMOD_EVT_MAX value is returned.
                // Example application: gear shift sounds when running an external gearbox model.

    virtual bool FMOD_Play( char sound_name[128] ) { return false ; }
    virtual bool FMOD_Stop( char sound_name[128] ) { return false ; }
                // Called at UpdateTelemetry rate immediately after sub-plugin call to UpdateTelemetry.
                // Set sound_name and return true to start or stop the sound with the matching name.
                // The name must match a section name in the FMODplugin.ini file.
                // The sound_name has a maximum length of 127 characters and must be null-terminated (making 128 characters in total).
                // NOTE: To enable multiple sounds to be controlled per time step, this function is called repeatedly
                //       per time step until false is returned.
                // Example application: shift beeper

    virtual bool FMOD_SetGlobalReverb( FMOD_ReverbParams &reverb_params ) { return false ; }
                // Called at UpdateTelemetry rate immediately after sub-plugin call to UpdateTelemetry.
                // Set the structure values to control the reverb setting of the FMOD sound system.
                // Return true to use the parameter values. Return false to use original values.

};

class InternalsPluginV07Extras : public InternalsPluginV06Extras    // Return a 7 when creating the plugin in your .cpp file
{
public:
    /* Hi-Definition road surface pre-processing
       The difference between this API and the V05Extras tyre API is that this does not use
       discrete sample points.  Instead an entire Hi-Def surface within a nominal tyre
       volume is processed.
       Note that Hi-Def surface data might not be available for all areas of the circuit
       (e.g. verges, run-offs, pit lane). The V05Extras tyre API must be used in parallel with
       the V07Extras tyre API and is used as a fall-back for areas where Hi-Def surface
       data is unavailable.  Therefore the V05Extras WantsTyreContactUpdates function is still
       important - although note that if it is not implemented at all, a single-sample default
       will be used (i.e. assuming a return value of 4).
       If WantsHDTyreContactUpdates returns true, then all contact patch data will be returned
       through UpdateHDTyreContact.  The V05Extras UpdateTyreContact will not be called. Use
       the mHDvalid member to check whether HD surface data was available for each wheel.  
    */
    virtual bool WantsHDTyreContactUpdates( TyreDefinitionInputV01 &tyreDef ) { return false; }
                // tyreDef can be used to set the effective tyre radius and width. This defines the region of space scanned for ground surface.
                // A good starting point for this is the unloaded tyre radius.
                // Regardless of what this call returns, WantsTyreContactUpdates (in V05Extras) will still be called. It will be used to
                // configure the behaviour when HD/LiDAR data is not available (typically when a wheel is on the grass).
    virtual void UpdateHDTyreContact( HDContactPatchV01 contactPatch[4] )  {}
                // Provides the plugin with the information requested in WantsHDTyreContactUpdates.
                // Called immediately before RunPhysics().
};

class InternalsPluginV08Extras : public InternalsPluginV07Extras    // Return a 8 when creating the plugin in your .cpp file
{
public:
    virtual bool WantsReinitialising() { return false; }
                // Normally a sub-plugin's 'WantsNNN' functions are called only on startup.
                // However, some models want to change the operating parameters without having to be restarted.
                // This function allows all the 'WantsNNN' functions to be called again.
                // This function is called at a low rate (typically between 1 and 10 Hz).
                // If the function returns true then the following sub-plugin functions are called immediately:
                //      WantsTelemetryUpdates, WantsGraphicsUpdates, WantsMotionStates, WantsChassisContactUpdates,
                //      WantsTyreContactUpdates, WantsHDTyreContactUpdates, WantsObjectProximityUpdates
                // If the function returns false then there is no effect.

    virtual bool WantsToOverrideAxisConversion( bool &useISO ) { return false; }
                // Normally the data passed to a sub-plugin's uses the axis convention specified in SimControlPlugin.ini (UseISO).
                // This function allows the sub-plugin to override the ini file setting.
                // The value of useISO passed into this function will be the ini file setting.
                // Return false from this function to continue to use the ini file setting.
                // Set the value of useISO and return true from this function to override the setting.
                // A useISO value of true will use the ISO axis convention (X=forwards,Y=left,Z=up).
                // A useISO value of false will use the rFactor axis convention (X=left,Y=up,Z=backwards).
};

class InternalsPluginV09Extras : public InternalsPluginV08Extras    // Return a 9 when creating the plugin in your .cpp file
{
public:
    virtual void FMOD_ValidParameters( long numParams, const char **paramNames ) {}
    virtual void FMOD_UpdateNew( long numParams, float *paramValues ) {}
};

class InternalsPluginV10Extras : public InternalsPluginV09Extras    // Return a 10 when creating the plugin in your .cpp file
{
    // REMINDER: Exported function GetPluginVersion() should return 10 if you are deriving from this.

public:

    // IMPORTANT NOTE: These callback functions are supported ONLY in rFactor Pro 1020 and later.

    // EXTENDED NOTIFICATIONS
    virtual void SetEnvironment( const EnvironmentInfoV01 &info )       {} // may be called whenever the environment changes

    // SCREEN INFO NOTIFICATIONS
    virtual void InitScreen( const ScreenInfoV01 &info )                {} // immediately after graphics device initialization
    virtual void UninitScreen( const ScreenInfoV01 &info )              {} // immediately before graphics device uninitialization
    virtual void DeactivateScreen( const ScreenInfoV01 &info )          {} // Window deactivation (e.g. focus lost)
    virtual void ReactivateScreen( const ScreenInfoV01 &info )          {} // Window reactivation (e.g. focus regain)
    virtual void RenderScreenBeforeOverlays( const ScreenInfoV01 &info ){} // before rFactor overlays (e.g. once per graphics frame)
    virtual void RenderScreenAfterOverlays( const ScreenInfoV01 &info ) {} // after rFactor overlays (e.g. once per graphics frame)
    virtual void PreReset( const ScreenInfoV01 &info )                  {} // after detecting device lost but before resetting (e.g. on fullscreen mode restore)
    virtual void PostReset( const ScreenInfoV01 &info )                 {} // after resetting

    // CONDITIONS CONTROL [-# WARNING: EXPERIMENTAL! #-]
    virtual bool WantsToChangeTimeOfDay( double &secondsSinceMidnight ) { return false; } // current seconds since midnight passed in; return true if intentionally changed.
    virtual bool WantsWeatherAccess()                                   { return false; } // change to true in order to read or write weather with AccessWeather() call:
    virtual bool AccessWeather( double trackNodeSize, WeatherControlInfoV01 &info ) { return false; } // current weather is passed in; return true if you want to change it

    // LIGHT CONTROL
    virtual bool WantsLightControl()                                    { return false; } // whether plugin wants light info and control
    virtual void SetLights( long slotID, long numLights, LightInfoV01 *lightInfo )    {}  // provides an array of lights, specific to a vehicle if slotID >= 0, or for the scene if slotID < 0
    virtual bool GetLights( LightControlV01 &lightControl )             { return false; } // return true in order to control a single light; this will be called each frame continuously until it returns false
    virtual bool UpdateSceneLights( SceneLightStatesV01 &state )        { return false; } // return true for rFP to read new states

    // HEAD TRACKING
    // To provide head tracking data return true to WantsToProvideHeadTracking and then complete the struct
    // in UpdateHeadTrackingState(). Head tracking states are relative to the motion platform state (if any).
    // For example, if the motion platform state has a position of (1,-2,0) and the head tracking state has a
    // position of (0.1, 0.2, 0.05) then the head will be considered to be at (1.1,-1.8,0.05).
    // For 3D IGs, the head tracking state origin is used to apply the correct IPD offset vector.
    virtual bool WantsToProvideHeadTracking()                           { return false; } 
    virtual void UpdateHeadTrackingState(MotionPlatformV01 &motionInfo) {}
};

class InternalsPluginV11Extras : public InternalsPluginV10Extras    // Return an 11 when creating the plugin in your .cpp file
{
    // REMINDER: Exported function GetPluginVersion() should return 11 if you are deriving from this.

public:

    // SENSOR_IG
    virtual bool WantsSensorImageAccess()                               { return false; } // return true to enable SensorImage callbacks (requires additional licence)
    virtual bool GetSensorImageSettings( SensorImageSettingsV01 &settings ) { return false; } // return true to set settings values
    virtual void SetSensorImage( const SensorImageInfoV01 &info )       {} // provides read-access for the CPU to the image memory

    // BEACON DETECTION
    virtual void TouchingBeacon( const BeaconContact &info )            {} // vehicle is touching instance XPITIN, XPITOUT, XFINISH, XSECTOR1, XSECTOR2, or XBEACON* (will be triggered every frame while it is touching)

    // ANIMATION CONTROL
    virtual bool WantsAnimationInfo()                                   { return false; } // whether plugin wants information about available animations
    virtual void SetAnimation( const QueryInstance &instance )          {} // provide info about instances in scene
    virtual bool GetAnimation( AnimationActionV01 &info )               { return false; } // generic/extendable interface to control the scene (loading objects, moving instances, starting animations, etc.)

    // OBJECT PROXIMITY
    virtual long WantsObjectProximityUpdates( long maxNumPoints, ProximityQueryPointDefnV01 *points ) { return 0; }
                // maxNumPoints is the maximum possible number of points available (and indicates the size of the points array).
                // Return the number of points populated in the array and to be passed back in the UpdateObjectProximity function.
                // Return 0 to disable this functionality
    virtual void UpdateObjectProximity( const ProximityQueryInputV01 &input )  {}
                // Provides the plugin with the information requested in WantsObjectProximityUpdates.
                // Called immediately before RunPhysics().
};

class InternalsPluginV12Extras : public InternalsPluginV11Extras
{
    // REMINDER: Exported function GetPluginVersion() should return 12 if you are deriving from this.

public:
    virtual void WantsDataModel(rFpro::DataModelProvider * provider) {}
};

class InternalsPluginV13Extras : public InternalsPluginV12Extras
{
public:
    enum { API_VERSION = 13 }; // Return this from exported function GetPluginVersion() (E.g. return YourDerivedPluginClass::API_VERSION;)

    virtual void SetSimulationTiming(const ThreadTimingV01 &info) {} // inform plugin of the current simulation thread timing. Called once per model frame.
    virtual void SetMultimediaTiming(const ThreadTimingV01 &info) {} // inform plugin of the current multimedia thread timing. Called once per graphics frame.
};

class InternalsPluginV14Extras : public InternalsPluginV13Extras
{
public:
    enum { API_VERSION = 14 }; // Return this from exported function GetPluginVersion() (E.g. return YourDerivedPluginClass::API_VERSION;)

    virtual size_t WantsToSetGuideLine() { return 0; }      // returns number of points, requires at least 2 to generate a line.
    virtual void SetGuideLine(GuideLinePointsV01 *points, bool &loop, bool &calculateHeight) {} // set the guide points, if we should loop the last point to the first, and if we should be calculating the height values. (Important: first point must not be underneath another valid height location)

    virtual void ReportLights(LightControlV01 &lightControl) {} // report light changes locally that were initiated by GetLights() on any machine

    virtual bool WantsToChangeDate(long &year, long &month, long &day) { return false; } // current date passed in; return true if intentionally changed (year 1-8000, month 1-12, day 1-31)
};

class InternalsPluginV15Extras : public InternalsPluginV14Extras
{
public:
    enum { API_VERSION = 15 }; // Return this from exported function GetPluginVersion() (E.g. return YourDerivedPluginClass::API_VERSION;)

    virtual bool WantsBoundingBoxes() { return false; }
    virtual bool WantsBoundingBox(const QueryInstance& instance) { return false; }
    virtual void SetBoundingBoxes(long num, const BoundingBoxV01 *boundingBoxArray) {}
};

class InternalsPluginV16Extras : public InternalsPluginV15Extras
{
public:
    enum { API_VERSION = 16 }; // Return this from exported function GetPluginVersion() (E.g. return YourDerivedPluginClass::API_VERSION;)

    virtual bool WantsRayCast() { return false; }                                   // Does the plugin want the RayCast API to be called?
    virtual bool WantsRayCastInstance(const InstanceInfoV01& info) { return true; } // Does the plugin want rays to be tested against this instance?
    virtual bool WantsRayCastMaterial(const MaterialInfoV01& info) { return true; } // Does the plugin want rays to be tested against this material?
    virtual void RayCast(const RayCastInfoV01& info, RayCastInterface* rays) {}
};

class InternalsPluginV17Extras: public InternalsPluginV16Extras
{
public:
    enum { API_VERSION = 17 }; // Return this from exported function GetPluginVersion() (E.g. return YourDerivedPluginClass::API_VERSION;)

    virtual long WarpImage() { return 1; } // Warp the image/aux buffers prior to sensor callbacks
};

class InternalsPluginV18Extras: public InternalsPluginV17Extras
{
public:
    enum { API_VERSION = 18 }; // Return this from exported function GetPluginVersion() (E.g. return YourDerivedPluginClass::API_VERSION;)

    virtual bool WantsFunctionalSplines() { return false; }                 // Does the plugin want functional spline output?
    virtual bool WantsFunctionalSpline(uint32_t splineID) { return false; } // Does the plugin want output for this functional spline?
    virtual void SetFunctionalSplineNodes(long count, const SplineNodeInfoV01* splineNodeArray) {}
};


// See #pragma at top of file
#pragma pack( pop )

#endif // _InternalsPluginEXTRAS_H
