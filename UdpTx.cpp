// UdpTx.cpp
// Copyright (c) rFpro 2014. All rights reserved.

#define S_FUNCTION_NAME UdpTx
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "UDPLink.hpp"

enum
{
    IN_DATA,
    IN_MESSAGE_SIZE,
    IN_ENABLE,
    NUM_INS
};

enum {
    PAR_SAMPLE_TIME,
    PAR_PORT,
    PAR_ADDRESS,
    NUM_PARAMS
};

enum
{
    OUT_STATUS,
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;
    
    ssSetInputPortWidth(S, IN_DATA, DYNAMICALLY_SIZED);
    ssSetInputPortWidth(S, IN_MESSAGE_SIZE, 1);
    ssSetInputPortWidth(S, IN_ENABLE, 1);
    
    ssSetOutputPortWidth(S, OUT_STATUS, 1);
    
    ssSetInputPortDataType(S, IN_DATA, SS_UINT8 );
    ssSetInputPortDataType(S, IN_ENABLE, SS_BOOLEAN );
    
    for (int p=0; p<NUM_INS; p++)
    {
        ssSetInputPortRequiredContiguous(S, p, 1);
        ssSetInputPortDirectFeedThrough(S, p, 1);
    }
    
    ssSetNumContStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
}

# define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
{
    ssSetInputPortVectorDimension(S, IN_DATA, 200);
}

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if (port == IN_DATA)
    {
        ssSetInputPortDimensionInfo(S, port, dimsInfo);
    }
}

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    // Don't open IP sockets etc. when just compiling the model
    // Accelerator mode appears to use this mdlStart rather than the TLC one
    if (ssGetSimMode(S) == SS_SIMMODE_RTWGEN && !ssRTWGenIsAccelerator(S)) return;

    try
    {
#if defined(MATLAB_MEX_FILE)
        char portString[10];
        if (mxIsChar(ssGetSFcnParam(S, PAR_PORT)))
        {
            strcpy(portString, mxArrayToString(ssGetSFcnParam(S, PAR_PORT)));
        }
        else
        {
            sprintf(portString, "%9.0f", mxGetScalar(ssGetSFcnParam(S, PAR_PORT)));
        }
        ssGetPWorkValue(S, 0) = new UDPLink::Sender( portString, mxArrayToString(ssGetSFcnParam(S, PAR_ADDRESS)));
#else
        ssGetPWorkValue(S, 0) = new UDPLink::Sender( "1000", "127.0.0.1");
#endif
    }
    catch (UDPLink::Exception &e)
    {
        static char msg[200];
        // TODO Couldn't get snprintf to compile. Function not available?
#if defined(MATLAB_MEX_FILE)
        sprintf(msg, "Couldn't open UDP sender %s:%s\n", mxArrayToString(ssGetSFcnParam(S, PAR_ADDRESS)), mxArrayToString(ssGetSFcnParam(S, PAR_PORT)));
#endif
        ssSetErrorStatus(S, msg);
        return;
    }
    ssPrintf("Opened UDP sender on %s:%d\n", ((UDPLink::Sender *)ssGetPWorkValue(S, 0))->GetDestAddress().c_str(), ((UDPLink::Sender *)ssGetPWorkValue(S, 0))->GetDestPort());
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    if (*(bool *)ssGetInputPortSignal(S, IN_ENABLE))
    {
        const size_t messageSize = (size_t)(*ssGetInputPortRealSignal(S, IN_MESSAGE_SIZE));
        ((UDPLink::Sender *)ssGetPWorkValue(S,0))->Send((uint8_t *)ssGetInputPortSignal(S, IN_DATA), messageSize);
        //ssPrintf("Sent %d bytes\n", ((UDPLink::Sender *)ssGetPWorkValue(S,0))->BytesSent());
        *ssGetOutputPortRealSignal(S, OUT_STATUS) = ((UDPLink::Sender *)ssGetPWorkValue(S, 0))->BytesSent();
    }
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
    delete (UDPLink::Sender *)ssGetPWorkValue(S,0);
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
