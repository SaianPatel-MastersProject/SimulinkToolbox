// Copyright (c) 2021 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME BoundingBoxV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

enum
{
    NUM_INS
};

enum
{
    PARAM_BOX_NAMES,
    PARAM_GROUP_NUM,
    PARAM_NUM_BOXES,
    NUM_PARAMS
};

enum
{
    OUT_MESH_ID,
    OUT_CORNERS,
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    int iter;
    
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;

    ssSetOutputPortWidth(S, OUT_MESH_ID, (int_T)mxGetScalar(ssGetSFcnParam(S, PARAM_NUM_BOXES)));
    ssSetOutputPortDataType(S, OUT_MESH_ID, SS_INT32);
    
    ssSetOutputPortWidth(S, OUT_CORNERS, 24*(int_T)mxGetScalar(ssGetSFcnParam(S, PARAM_NUM_BOXES)));
    ssSetOutputPortDataType(S, OUT_CORNERS, SS_DOUBLE );

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
}

static void mdlSetWorkWidths(SimStruct *S)
{
    /* Define name of run-time parameter */
    const char_T *rtParamName = "GroupID";

    ssSetNumRunTimeParams(S, 1); /* One run-time parameter */
    if (ssGetErrorStatus(S) != NULL) return;
    ssRegDlgParamAsRunTimeParam(S, PARAM_GROUP_NUM, 0, rtParamName, SS_UINT16);
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
