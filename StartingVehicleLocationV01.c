// Copyright (c) 2011-2020 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME StartingVehicleLocationV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

enum
{
   OUT_SET,
   OUT_POS,
   OUT_ORI,
   NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, 0)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;

    ssSetOutputPortWidth(S, OUT_SET, 1);
    ssSetOutputPortWidth(S, OUT_POS, 3);
    ssSetOutputPortMatrixDimensions(S, OUT_ORI, 3, 3);
    
    ssSetOutputPortDataType(S, OUT_SET, SS_BOOLEAN );

    ssSetNumContStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, DYNAMICALLY_SIZED);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S,  SS_OPTION_ALLOW_PARTIAL_DIMENSIONS_CALL);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    const real_T identity[] = {1,0,0, 0,1,0, 0,0,1};
    real_T *ori = ssGetOutputPortRealSignal(S, OUT_ORI);
    {
        unsigned i;
        for (i=0; i<9; i++) ori[i] = identity[i];
    }
}    

#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0);
    ssSetOffsetTime(S, 0, 0);

    //ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    //ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
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
