// Copyright (c) 2022 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME ReducedTelemInfoV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUM_INS 0
#define NUM_PARAMS 0
enum {
    OUT_VALID,
    OUT_TIME,
    OUT_POS,
    OUT_DRIVER,
    OUT_LAP_POS,
    OUT_ORI,
    OUT_VEL,
    OUT_ANG_VEL,
    OUT_LAP_REF,
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    int p;
    
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;
    
    ssSetOutputPortDataType(S, OUT_VALID, SS_BOOLEAN);
    
    ssSetOutputPortWidth(S, OUT_VALID, 8);
    ssSetOutputPortWidth(S, OUT_TIME, 1);
    ssSetOutputPortWidth(S, OUT_POS, 3);
    ssSetOutputPortWidth(S, OUT_DRIVER, 4);
    ssSetOutputPortWidth(S, OUT_LAP_POS, 1);
    ssSetOutputPortWidth(S, OUT_ORI, 9);
    ssSetOutputPortWidth(S, OUT_VEL, 3);
    ssSetOutputPortWidth(S, OUT_ANG_VEL, 3);
    ssSetOutputPortWidth(S, OUT_LAP_REF, 1);

    ssSetNumContStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
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
