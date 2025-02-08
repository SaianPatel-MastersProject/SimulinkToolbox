// Copyright (c) 2011-2020 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME TyreContactInputV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUM_INS 0

enum
{
    PAR_WHEEL,
    PAR_CONTACT_DATA,
    PAR_NOMINAL_RADIUS,
    PAR_NOMINAL_WIDTH,
    PAR_SAMPLE_TIME,
    NUM_PARAMS
};

enum
{
    OUT_SURFACE_GAIN,
    OUT_HEIGHT,
    OUT_NORMAL,
    OUT_SURFACE_TYPE,
    OUT_TERRAIN_NAME,
    OUT_HD_VALID,
    OUT_PATCH_CENTRE,
    NUM_OUTS
};

const int NUM_TERRAIN_NAME_CHARS = 15; // We don't need to output terminating NULL

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;

    ssSetOutputPortWidth(S, OUT_SURFACE_GAIN, 1);
    ssSetOutputPortWidth(S, OUT_HEIGHT, 1);
    ssSetOutputPortWidth(S, OUT_NORMAL, 3);
    ssSetOutputPortWidth(S, OUT_SURFACE_TYPE, 1);
    ssSetOutputPortWidth(S, OUT_TERRAIN_NAME, NUM_TERRAIN_NAME_CHARS);
    ssSetOutputPortWidth(S, OUT_HD_VALID, 1);
    ssSetOutputPortWidth(S, OUT_PATCH_CENTRE, 3);

    ssSetOutputPortDataType(S, OUT_HD_VALID, SS_BOOLEAN );
    ssSetOutputPortDataType(S, OUT_SURFACE_TYPE, SS_UINT8 );
    ssSetOutputPortDataType(S, OUT_TERRAIN_NAME, SS_INT8 );

    ssSetNumContStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, DYNAMICALLY_SIZED);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S,  SS_OPTION_ALLOW_PARTIAL_DIMENSIONS_CALL);
    
    if (!mxIsEmpty(ssGetSFcnParam(S, PAR_CONTACT_DATA))) // Parameter is optional
    {
        if (mxGetN(ssGetSFcnParam(S, PAR_CONTACT_DATA)) != 4)
        {
            ssSetErrorStatus(S, "Expected Contact Patch Definition parameter to be 4 columns");
            return;
        }
    
        if (mxGetM(ssGetSFcnParam(S, PAR_CONTACT_DATA)) > 10)
        {
            ssSetErrorStatus(S, "Expected Contact Patch Definition parameter to be less than 10 rows");
            return;
        }
    }
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    const double sampleTime = mxGetScalar(ssGetSFcnParam(S, PAR_SAMPLE_TIME));
    ssSetSampleTime(S, 0, sampleTime);
    if (sampleTime == INHERITED_SAMPLE_TIME)
    {
        ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
    }

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
