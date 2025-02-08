// Copyright (c) 2011-2020 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME ISOTyreInputV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUM_INS 0
#define NUM_PARAMS 0
enum {
    OUT_ET,
    
    // Note: ISO definitions used here
    OUT_TYRE_LOAD,
    OUT_SLIP_ANGLE,
    OUT_SLIP_RATIO,
    OUT_PRESSURE_PSI,
    OUT_FORWARD_VELOCITY,
    OUT_TRACK_TEMP,
    OUT_AMBIENT_TEMP,
    OUT_ROLL_ANGLE,

    OUT_SURFACE_TEMP,
    OUT_BULK_TEMP,
    
    // surface properties (can use either look-up or simple gain)
    OUT_TERRAIN_NAME,
    OUT_SURFACE_TYPE,
    OUT_SURFACE_GAIN,
    
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
    
    ssSetOutputPortWidth(S, OUT_TERRAIN_NAME, 16);
    
    ssSetOutputPortDataType(S, OUT_TERRAIN_NAME, SS_UINT8 );
    ssSetOutputPortDataType(S, OUT_SURFACE_TYPE, SS_INT32 );

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
