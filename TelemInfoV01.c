// Copyright (c) 2011 Kangaloosh Ltd t/a rFpro, All Rights Reserved.

#define S_FUNCTION_NAME TelemInfoV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUM_INS 0
#define NUM_PARAMS 0
enum {
    // Time
    //long mID;                      // slot ID (note that it can be re-used in multiplayer after someone leaves)
    OUT_DELTA_TIME,             // time since last update (seconds)
    OUT_ELAPSED_TIME,           // game session time
    OUT_LAP_NUMBER,               // current lap number
    OUT_LAP_START_ET,            // time this lap was started
    OUT_VEHICLE_NAME,         // current vehicle name
    OUT_TRACK_NAME,           // current track name

    // Position and derivatives
    OUT_POS,               // world position in meters
    OUT_LOCAL_VEL,          // velocity (meters/sec) in local vehicle coordinates
    OUT_LOCAL_ACCEL,        // acceleration (meters/sec^2) in local vehicle coordinates
    
    // Orientation and derivatives
    OUT_ORI,            // rows of orientation matrix (use TelemQuat conversions if desired), also converts local
    OUT_LOCAL_ROT,          // rotation (radians/sec) in local vehicle coordinates
    OUT_LOCAL_ROT_ACCEL,     // rotational acceleration (radians/sec^2) in local vehicle coordinates
    
    // Vehicle status
    OUT_GEAR,                    // -1=reverse, 0=neutral, 1+=forward gears
    OUT_ENGINE_RPM,             // engine RPM
    OUT_ENGINE_WATER_TEMP,       // Celsius
    OUT_ENGINE_OIL_TEMP,         // Celsius
    OUT_CLUTCH_RPM,             // clutch RPM
    
    // Driver input
    OUT_UNFILTERED_THROTTLE,    // ranges  0.0-1.0
    OUT_UNFILTERED_BRAKE,       // ranges  0.0-1.0
    OUT_UNFILTERED_STEERING,    // ranges -1.0-1.0 (left to right)
    OUT_UNFILTERED_CLUTCH,      // ranges  0.0-1.0
    
    // Filtered input (various adjustments for rev or speed limiting, TC, ABS?, speed sensitive steering, clutch work for semi-automatic shifting, etc.)
    OUT_FILTERED_THROTTLE,      // ranges  0.0-1.0
    OUT_FILTERED_BRAKE,         // ranges  0.0-1.0
    OUT_FILTERED_STEERING,      // ranges -1.0-1.0 (left to right)
    OUT_FILTERED_CLUTCH,        // ranges  0.0-1.0
    
    // Misc
    OUT_STEERING_ARM_FORCE,      // force on steering arms
    OUT_FRONT_3RD_DEFLECTION,    // deflection at front 3rd spring
    OUT_REAR_3RD_DEFLECTION,     // deflection at rear 3rd spring
    
    // Aerodynamics
    OUT_FRONT_WING_HEIGHT,       // front wing height
    OUT_FRONT_RIDE_HEIGHT,       // front ride height
    OUT_REAR_RIDE_HEIGHT,        // rear ride height
    OUT_DRAG,                  // drag
    OUT_FRONT_DOWNFORCE,        // front downforce
    OUT_REAR_DOWNFORCE,         // rear downforce
    
    // State/damage info
    OUT_FUEL,                  // amount of fuel (liters)
    OUT_ENGINE_MAX_RPM,        // rev limit
    OUT_SCHEDULED_STOPS,       // number of scheduled pitstops
    OUT_OVERHEATING,           // whether overheating icon is shown
    OUT_DETACHED,              // whether any parts (besides wheels) have been detached
    OUT_DENT_SEVERITY,         // dent severity at 8 locations around the car (0=none, 1=some, 2=more)
    OUT_LAST_IMPACT_ET,        // time of last impact
    OUT_LAST_IMPACT_MAGNITUDE, // magnitude of last impact
    OUT_LAST_IMPACT_POS,       // location of last impact
    
    // Future use
    OUT_ENGINE_TORQUE,
    OUT_SECTOR,
    OUT_PIT,
    OUT_HEADLIGHT,             // headlight status (0=off, 1=lowbeam, 2=highbeam)
    
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    int p;
    
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;
    
    // Set default output size to scalar/1
    for (p=0; p<NUM_OUTS; ssSetOutputPortWidth(S, p++, 1));
    
    // Overrite special cases; port sizes > 1
    ssSetOutputPortWidth(S, OUT_VEHICLE_NAME, 64);
    ssSetOutputPortWidth(S, OUT_TRACK_NAME, 64);
    ssSetOutputPortWidth(S, OUT_POS, 3);
    ssSetOutputPortWidth(S, OUT_LOCAL_VEL, 3);
    ssSetOutputPortWidth(S, OUT_LOCAL_ACCEL, 3);
    ssSetOutputPortMatrixDimensions(S, OUT_ORI, 3, 3);
    ssSetOutputPortWidth(S, OUT_LOCAL_ROT, 3);
    ssSetOutputPortWidth(S, OUT_LOCAL_ROT_ACCEL, 3);
    ssSetOutputPortWidth(S, OUT_LAST_IMPACT_POS, 3);
    ssSetOutputPortWidth(S, OUT_DENT_SEVERITY, 8);

    ssSetOutputPortDataType(S, OUT_LAP_NUMBER, SS_INT32 );
    ssSetOutputPortDataType(S, OUT_VEHICLE_NAME, SS_UINT8 );
    ssSetOutputPortDataType(S, OUT_TRACK_NAME, SS_UINT8 );
    ssSetOutputPortDataType(S, OUT_GEAR, SS_INT32 );
    ssSetOutputPortDataType(S, OUT_SCHEDULED_STOPS, SS_UINT8 );
    ssSetOutputPortDataType(S, OUT_OVERHEATING, SS_BOOLEAN );
    ssSetOutputPortDataType(S, OUT_DETACHED, SS_BOOLEAN );
    ssSetOutputPortDataType(S, OUT_DENT_SEVERITY, SS_UINT8 );
    ssSetOutputPortDataType(S, OUT_HEADLIGHT, SS_UINT8 );
    ssSetOutputPortDataType(S, OUT_SECTOR, SS_INT32 );
    ssSetOutputPortDataType(S, OUT_PIT, SS_BOOLEAN );

    ssSetNumContStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, DYNAMICALLY_SIZED);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    ssSetOptions(S,  SS_OPTION_ALLOW_PARTIAL_DIMENSIONS_CALL);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
}

static void mdlTerminate(SimStruct *S)
{
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
