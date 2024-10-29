// UdpRx.cpp
// Copyright (c) rFpro 2014. All rights reserved.

#define S_FUNCTION_NAME UdpRx
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "UDPLink.hpp"

enum
{
    NUM_INS
};

enum
{
    PAR_SAMPLE_TIME,
    PAR_PORT,
    PAR_ADDRESS,
    PAR_MAX_MESSAGE_SIZE,
    NUM_PARAMS
};

enum
{
    OUT_DATA,
    OUT_MESSAGE_SIZE,
    OUT_BYTES,
    OUT_DATAGRAMS,
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;

    ssSetOutputPortWidth(S, OUT_DATA, (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_MAX_MESSAGE_SIZE)));
    ssSetOutputPortWidth(S, OUT_MESSAGE_SIZE, 1);
    ssSetOutputPortWidth(S, OUT_BYTES, 1);
    ssSetOutputPortWidth(S, OUT_DATAGRAMS, 1);
    
    ssSetOutputPortDataType(S, OUT_DATA, SS_UINT8 );
    
    ssSetNumContStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    // Don't open IP sockets etc. when just compiling the model
    // Accelerator mode appears to use this mdlStart rather than the TLC one
    if (ssGetSimMode(S) == SS_SIMMODE_RTWGEN && !ssRTWGenIsAccelerator(S)) return;

    try
    {
        char portString[10];
//         if (mxIsChar(ssGetSFcnParam(S, PAR_PORT)))
//         {
//             strcpy(portString, mxArrayToString(ssGetSFcnParam(S, PAR_PORT)));
//         }
//         else
//         {
            sprintf(portString, "%9.0f", mxGetScalar(ssGetSFcnParam(S, PAR_PORT)));
//         }
#if defined(MATLAB_MEX_FILE)
        ssGetPWorkValue(S, 0) = new UDPLink::Receiver( portString, mxArrayToString(ssGetSFcnParam(S, PAR_ADDRESS)));
#else
        ssGetPWorkValue(S, 0) = new UDPLink::Receiver( portString, "127.0.0.1");
#endif
    }
    catch (UDPLink::Exception &e)
    {
        static char msg[200];
        // TODO Couldn't get snprintf to compile. Function not available?
#if defined(MATLAB_MEX_FILE)
        sprintf(msg, "Couldn't open UDP receiver %s:%s\n", mxArrayToString(ssGetSFcnParam(S, PAR_ADDRESS)), mxArrayToString(ssGetSFcnParam(S, PAR_PORT)));
#endif
        ssSetErrorStatus(S, msg);
        return;
    }
    ssPrintf("Opened UDP receiver on %s:%d\n", ((UDPLink::Receiver *)ssGetPWorkValue(S, 0))->GetAddress().c_str(), ((UDPLink::Receiver *)ssGetPWorkValue(S, 0))->GetPort());
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    size_t bytesReceived = ((UDPLink::Receiver *)ssGetPWorkValue(S, 0))->GetNextMessage();
    if (bytesReceived)
    {
        *ssGetOutputPortRealSignal(S, OUT_MESSAGE_SIZE) = bytesReceived;
        
        size_t bytesToCopy = bytesReceived < (unsigned)ssGetOutputPortWidth(S, OUT_DATA)? bytesReceived : ssGetOutputPortWidth(S, OUT_DATA); // TODO Couldn't find std::min
        memcpy((uint8_t *)ssGetOutputPortSignal(S, OUT_DATA), ((UDPLink::Receiver *)ssGetPWorkValue(S, 0))->GetMessage(), bytesToCopy);
        *ssGetOutputPortRealSignal(S, OUT_BYTES) = ((UDPLink::Receiver *)ssGetPWorkValue(S, 0))->BytesReceived();
        *ssGetOutputPortRealSignal(S, OUT_DATAGRAMS) = ((UDPLink::Receiver *)ssGetPWorkValue(S, 0))->DatagramsReceived();
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
    delete ((UDPLink::Receiver *)ssGetPWorkValue(S, 0));
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
