// Copyright (c) 2024 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME VectorCoalesce
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

enum
{
    INITIAL_OFFSET,
    COPY_SIZE,
    OFFSET,
    REPEATS,
    DATA_IN,
    NUM_INS
};

enum
{
    INPUT_SIZE,
    OUTPUT_SIZE,
    DATA_TYPE,
    NUM_PARAMS
};

enum
{
    DATA_OUT,
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    int idx;
           
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

    for (idx = 0; idx < NUM_INS; ++idx)
    {
        ssSetInputPortDirectFeedThrough(S, idx, 1);
    }
    
    ssSetInputPortWidth(S, INITIAL_OFFSET, 1);
    ssSetInputPortDataType(S, INITIAL_OFFSET, SS_UINT16);

    ssSetInputPortWidth(S, COPY_SIZE, 1);
    ssSetInputPortDataType(S, COPY_SIZE, SS_UINT16);

    ssSetInputPortWidth(S, OFFSET, 1);
    ssSetInputPortDataType(S, OFFSET, SS_UINT16);

    ssSetInputPortWidth(S, REPEATS, 1);
    ssSetInputPortDataType(S, REPEATS, SS_UINT16);

    ssSetInputPortRequiredContiguous(S, DATA_IN, 1);
    ssSetInputPortWidth(S, DATA_IN, (int_T)mxGetScalar(ssGetSFcnParam(S, INPUT_SIZE)));
    ssSetInputPortDataType(S, DATA_IN, (int_T)mxGetScalar(ssGetSFcnParam(S, DATA_TYPE)));

    for ( idx = 0 ; idx < NUM_PARAMS ; ++idx )
        ssSetSFcnParamTunable(S, idx, SS_PRM_NOT_TUNABLE);

    ssSetOutputPortWidth(S, DATA_OUT, (int_T)mxGetScalar(ssGetSFcnParam(S, OUTPUT_SIZE)));
    ssSetOutputPortDataType(S, DATA_OUT, (int_T)mxGetScalar(ssGetSFcnParam(S, DATA_TYPE)));
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
    // 0  SS_DOUBLE   1  SS_SINGLE    2  SS_INT8    3  SS_UINT8
    // 4  SS_INT16    5  SS_UINT16    6  SS_INT32   7  SS_UINT32
    // 8  SS_BOOLEAN
    size_t dataSizes[] = {8, 4, 1, 1, 2, 2, 4, 4, 1};
    DTypeId data_type = (DTypeId)mxGetScalar(ssGetSFcnParam(S, DATA_TYPE));
    size_t dataSize = dataSizes[data_type];
    
    const InputPtrsType copy_size_ptrs = ssGetInputPortSignalPtrs(S, COPY_SIZE);
    const InputPtrsType repeats_ptrs = ssGetInputPortSignalPtrs(S, REPEATS);
    const InputPtrsType offset_ptrs = ssGetInputPortSignalPtrs(S, OFFSET);
    const InputPtrsType init_offset_ptrs = ssGetInputPortSignalPtrs(S, INITIAL_OFFSET);
    
    uint16_T* copy_size_ = *copy_size_ptrs;
    uint8_T* repeats_ = *repeats_ptrs;
    uint16_T* offset_ = *offset_ptrs;
    uint16_T* init_offset_ = *init_offset_ptrs;
    
    int idx;
    int input_offset = *init_offset_;
    int output_offset = 0;
    
    memset(ssGetOutputPortSignal(S, DATA_OUT), 0, (size_t)mxGetScalar(ssGetSFcnParam(S, OUTPUT_SIZE)) * dataSize);

    for (idx = 0; idx < repeats_[0]; ++idx)
    {
        memcpy( (void*)((uint8_T*)ssGetOutputPortSignal(S, DATA_OUT) + output_offset * dataSize),
                (void*)((uint8_T*)ssGetInputPortSignal(S, DATA_IN) + input_offset * dataSize),
                copy_size_[0] * dataSizes[data_type] );
         
        input_offset += offset_[0];
        output_offset += copy_size_[0];
        
        // Check for signal overrun
        if (input_offset + copy_size_[0] > ssGetInputPortWidth(S, DATA_IN) ||
            output_offset + copy_size_[0] > ssGetOutputPortWidth(S, DATA_OUT))
            break;
        
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
