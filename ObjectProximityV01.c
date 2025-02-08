// Copyright (c) 2013-2021 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME ObjectProximityV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

enum
{
    NUM_INS
};

enum
{
    PAR_OFFSETS,
    PAR_RAYS,
    NUM_PARAMS
};

enum
{
    OUT_VALID,
    OUT_RANGE,
    OUT_NORMAL,
    OUT_INSTANCE_NAME,
    OUT_MATERIAL_NAME,
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    int NUM_PTS;

    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;

    NUM_PTS = mxGetN(ssGetSFcnParam(S, PAR_OFFSETS));

    ssSetOutputPortWidth(S, OUT_VALID, NUM_PTS);
    ssSetOutputPortDataType(S, OUT_VALID, SS_BOOLEAN );

    ssSetOutputPortWidth(S, OUT_RANGE, NUM_PTS);

    ssSetOutputPortMatrixDimensions(S, OUT_NORMAL, 3, NUM_PTS);

    ssSetOutputPortMatrixDimensions(S, OUT_INSTANCE_NAME, 20, NUM_PTS);
    ssSetOutputPortDataType(S, OUT_INSTANCE_NAME, SS_INT8 );

    ssSetOutputPortMatrixDimensions(S, OUT_MATERIAL_NAME, 20, NUM_PTS);
    ssSetOutputPortDataType(S, OUT_MATERIAL_NAME, SS_INT8 );

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    //ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    //ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
    ssSetSampleTime(S, 0, 0);
    ssSetOffsetTime(S, 0, 0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
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
