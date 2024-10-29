// Copyright (c) 2011 Kangaloosh Ltd t/a rFpro, All Rights Reserved.

#define S_FUNCTION_NAME TelemWheelV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUM_INS 0

enum {PAR_WHEEL, NUM_PARAMS};

enum {
    OUT_SUSPENSION_DEFLECTION,   // meters
    OUT_RIDE_HEIGHT,             // meters
    OUT_SUSP_FORCE,              // pushrod load in Newtons
    OUT_BRAKE_TEMP,              // Celsius
    OUT_BRAKE_PRESSURE,          // currently 0.0-1.0, depending on driver input and brake balance; will convert to true brake pressure (kPa) in future

    OUT_ROTATION,                // radians/sec
    OUT_LATERAL_PATCH_VEL,       // lateral velocity at contact patch
    OUT_LONGITUDINAL_PATCH_VEL,  // longitudinal velocity at contact patch
    OUT_LATERAL_GROUND_VEL,      // lateral velocity at contact patch
    OUT_LONGITUDINAL_GROUND_VEL, // longitudinal velocity at contact patch
    OUT_CAMBER,                  // radians (positive is left for left-side wheels, right for right-side wheels)
    OUT_LATERAL_FORCE,           // Newtons
    OUT_LONGITUDINAL_FORCE,      // Newtons
    OUT_TIRE_LOAD,               // Newtons

    OUT_GRIP_FRACT,              // an approximation of what fraction of the contact patch is sliding
    OUT_PRESSURE,                 // kPa (tire pressure)
    OUT_TEMPERATURE,              // Kelvin (subtract 273.16 to get Celsius), left/center/right (not to be confused with inside/center/outside!)
    OUT_WEAR,                     // wear (0.0-1.0, fraction of maximum) ... this is not necessarily proportional with grip loss
    OUT_TERRAIN_NAME,             // the material prefixes from the TDF file
    OUT_SURFACE_TYPE,             // 0=dry, 1=wet, 2=grass, 3=dirt, 4=gravel, 5=rumblestrip
    OUT_FLAT,                     // whether tire is flat
    OUT_DETACHED,                 // whether wheel is detached
                        
    OUT_VERTICAL_TIRE_DEFLECTION, // how much is tire deflected from its (speed-sensitive) radius
    OUT_WHEEL_Y_LOCATION,         // wheel's y location relative to vehicle y location

    OUT_TOE_ANGLE,                // toe angle w.r.t. the vehicle

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
    ssSetOutputPortWidth(S, OUT_TEMPERATURE, 3);
    ssSetOutputPortWidth(S, OUT_TERRAIN_NAME, 16);

    ssSetOutputPortDataType(S, OUT_TERRAIN_NAME, SS_UINT8 );
    ssSetOutputPortDataType(S, OUT_SURFACE_TYPE, SS_UINT8 );
    ssSetOutputPortDataType(S, OUT_FLAT, SS_BOOLEAN );
    ssSetOutputPortDataType(S, OUT_DETACHED, SS_BOOLEAN );

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
