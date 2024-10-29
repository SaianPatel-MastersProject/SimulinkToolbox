// Copyright (c) 2024 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME TelemOpponentsV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUM_PARAMS 0
enum
{
    IN_ID,
    IN_VEH_SELECT,
    NUM_INS
};

enum
{
    OUT_INT,
    OUT_SINGLE,
    OUT_DOUBLE,
    OUT_BYTE,
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;

    ssSetInputPortWidth(S, IN_ID, 1);
    ssSetInputPortDataType(S, IN_ID, SS_INT32 );
    ssSetInputPortDirectFeedThrough(S, IN_ID, 1);
    
    ssSetInputPortWidth(S, IN_VEH_SELECT, 1);
    ssSetInputPortDataType(S, IN_VEH_SELECT, SS_BOOLEAN );
    ssSetInputPortDirectFeedThrough(S, IN_VEH_SELECT, 1);

    // Overrite special cases; port sizes > 1
    ssSetOutputPortWidth(S, OUT_INT, 4);
    ssSetOutputPortWidth(S, OUT_SINGLE, 13);
    ssSetOutputPortWidth(S, OUT_DOUBLE, 61);
    ssSetOutputPortWidth(S, OUT_BYTE, 28);

    ssSetOutputPortDataType(S, OUT_INT, SS_INT32 );
    ssSetOutputPortDataType(S, OUT_SINGLE, SS_SINGLE );
    ssSetOutputPortDataType(S, OUT_DOUBLE, SS_DOUBLE );
    ssSetOutputPortDataType(S, OUT_BYTE, SS_UINT8 );

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
    ssSetSampleTime(S, 0, 0);
    ssSetOffsetTime(S, 0, 0);
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
