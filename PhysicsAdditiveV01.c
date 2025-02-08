// Copyright (c) 2011 Kangaloosh Ltd t/a rFpro, All Rights Reserved.

#define S_FUNCTION_NAME PhysicsAdditiveV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUM_OUTS 0
#define NUM_PARAMS 0
enum {
    IN_LOCAL_FORCE,
    IN_LOCAL_TORQUE,
    IN_ENGINE_TORQUE,
    IN_REAR_BRAKE_BIAS,
    NUM_INS
};

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;
    
    ssSetInputPortWidth(S, IN_LOCAL_FORCE, 3);
    ssSetInputPortWidth(S, IN_LOCAL_TORQUE, 3);
    ssSetInputPortWidth(S, IN_ENGINE_TORQUE, 1);
    ssSetInputPortWidth(S, IN_REAR_BRAKE_BIAS, 1);
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
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
