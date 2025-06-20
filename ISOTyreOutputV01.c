// Copyright (c) 2011-2020 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME ISOTyreOutputV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUM_PARAMS 0
#define NUM_OUTS 0
enum {
    IN_LONGITUDINAL_FORCE,
    IN_LATERAL_FORCE,
    IN_ALIGNING_MOMENT,
    IN_PRESSURE_PSI,
    IN_SURFACE_TEMP,
    IN_BULK_TEMP,
    IN_CARCASS_TEMP,
    IN_ROLLING_RESISTANCE,
    NUM_INS};

static void mdlInitializeSizes(SimStruct *S)
{
    int p;
    
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;

    // Set all input sizes to scalar/1
    for (p=0; p<NUM_INS; ssSetInputPortWidth(S, p++, 1));
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, SS_OPTION_ALLOW_PARTIAL_DIMENSIONS_CALL);
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
