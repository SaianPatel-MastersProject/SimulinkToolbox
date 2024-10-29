// Copyright (c) 2024 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME ByteTransform
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

enum
{
    DATA_IN,
    NUM_INS
};

enum
{
    DATA_TYPE,
    NUM_VALUES,
    TO_UINT8,    
    NUM_PARAMS
};

enum
{
    DATA_OUT,
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    int iter;
    int to_uint8;
    int num_values;
    DTypeId wider_data_type;

    // 0  SS_DOUBLE   1  SS_SINGLE    2  SS_INT8    3  SS_UINT8
    // 4  SS_INT16    5  SS_UINT16    6  SS_INT32   7  SS_UINT32
    // 8  SS_BOOLEAN
    int dataSizes[] = {8, 4, 1, 1, 2, 2, 4, 4, 1};
    
    to_uint8 = (int)mxGetScalar(ssGetSFcnParam(S, TO_UINT8));
    num_values = (int)mxGetScalar(ssGetSFcnParam(S, NUM_VALUES));
    wider_data_type = (DTypeId)mxGetScalar(ssGetSFcnParam(S, DATA_TYPE));
    
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    for ( iter = 0 ; iter < NUM_PARAMS ; ++iter )
        ssSetSFcnParamTunable(S, iter, SS_PRM_NOT_TUNABLE);

    if (to_uint8)
    {
        ssSetOutputPortWidth(S, DATA_OUT, (int_T)mxGetScalar(ssGetSFcnParam(S, NUM_VALUES)) * dataSizes[wider_data_type]);
        ssSetOutputPortDataType(S, DATA_OUT, SS_UINT8);
        
        ssSetInputPortWidth(S, DATA_OUT, num_values);
        ssSetInputPortDataType(S, 0, wider_data_type);
    }
    else
    {
        ssSetOutputPortWidth(S, DATA_OUT, (int_T)mxGetScalar(ssGetSFcnParam(S, NUM_VALUES)));
        ssSetOutputPortDataType(S, DATA_OUT, wider_data_type);
        
        ssSetInputPortWidth(S, DATA_OUT, (int_T)mxGetScalar(ssGetSFcnParam(S, NUM_VALUES)) * dataSizes[wider_data_type]);
        ssSetInputPortDataType(S, 0, SS_UINT8);
    }
            

}

static void mdlSetWorkWidths(SimStruct *S)
{
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
    //ssSetSampleTime(S, 0, 0);
    //ssSetOffsetTime(S, 0, 0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    int to_uint8;
    DTypeId wider_data_type;

    // 0  SS_DOUBLE   1  SS_SINGLE    2  SS_INT8    3  SS_UINT8
    // 4  SS_INT16    5  SS_UINT16    6  SS_INT32   7  SS_UINT32
    // 8  SS_BOOLEAN
    int dataSizes[] = {8, 4, 1, 1, 2, 2, 4, 4, 1};
    
    to_uint8 = (int)mxGetScalar(ssGetSFcnParam(S, TO_UINT8));
    wider_data_type = (DTypeId)mxGetScalar(ssGetSFcnParam(S, DATA_TYPE));

    if (to_uint8)
    {
        memcpy( (void *)((uint8_T *)ssGetOutputPortSignal(S, 0)),
                (void *)ssGetInputPortSignal(S, 0),
                (int_T)mxGetScalar(ssGetSFcnParam(S, NUM_VALUES)) * dataSizes[wider_data_type] );
    }
    else
    {
        memcpy( (void *)ssGetOutputPortSignal(S, 0),
                (void *)ssGetInputPortSignal(S, 0),
                (int_T)mxGetScalar(ssGetSFcnParam(S, NUM_VALUES)) * dataSizes[wider_data_type]);
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
