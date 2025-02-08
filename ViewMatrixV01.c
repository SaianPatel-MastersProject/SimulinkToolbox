// Copyright (c) 2018 Kangaloosh Ltd t/a rFpro, All Rights Reserved.

#define S_FUNCTION_NAME ViewMatrixV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

enum
{
    IN_MATRIX,
    NUM_INS
};

enum
{
    NUM_PARAMS
};

enum
{
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    int p;
    
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;

    ssSetInputPortMatrixDimensions(S, IN_MATRIX, 4, 4);

    for (p=0; p<NUM_INS; p++)
    {
        ssSetInputPortRequiredContiguous(S, p, 1);
        ssSetInputPortDirectFeedThrough(S, p, 1);
    }

    ssSetNumContStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    ssSetOptions(S, SS_OPTION_ALLOW_PARTIAL_DIMENSIONS_CALL);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
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
