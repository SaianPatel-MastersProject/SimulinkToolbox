// Copyright (c) 2011-2020 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME DifferentialInputV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUM_INS 0
#define NUM_PARAMS 0
enum {
    OUT_DT,
    OUT_ET,
    
    OUT_ENGINE_RPM,
    OUT_THROTTLE,
    OUT_GEAR_RATIO,

    OUT_FINAL_DRIVE_INPUT_RPM,
    OUT_LOCAL_ACCEL,
    OUT_FINAL_DRIVE_INPUT_TORQUE,
    OUT_DIFFERENTIAL_WHEEL_RPM,

    OUT_LAP_DISTANCE,
    OUT_STEERING,
    OUT_LOCAL_ROT,
    
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
    
    ssSetOutputPortWidth(S, OUT_LOCAL_ACCEL, 3);
    ssSetOutputPortWidth(S, OUT_LOCAL_ROT, 3);
    
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
