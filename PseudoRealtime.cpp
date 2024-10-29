#define S_FUNCTION_NAME PseudoRealtime
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "../Include/timer.hpp"

static PerformanceTimer timer;

enum
{
    NUM_INS
};

enum
{
    PAR_TIMESTEP,
    NUM_PARAMS
};

enum
{
    OUT_LOAD,
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;

    ssSetOutputPortWidth(S, OUT_LOAD, 1);

    ssSetNumContStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    ssSetOptions(S, SS_OPTION_ALLOW_PARTIAL_DIMENSIONS_CALL);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    if ((ssGetT(S) == 0.0)) timer.Reset();
    
    double load = 1.0 - (ssGetT(S) - timer.getTimeMS()/1000.0)/mxGetScalar(ssGetSFcnParam(S, PAR_TIMESTEP));
    *(double *)ssGetOutputPortSignal(S, OUT_LOAD) = load;
    
    while (timer.getTimeMS() < ssGetT(S)*1000.0)
    {
        // Delay progress
    }
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, PAR_TIMESTEP)));
    ssSetOffsetTime(S, 0, 0.0);
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
