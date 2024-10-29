    // Copyright (c) 2024 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME VectorExpand
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

enum
{
    VALID,
    NUM_OFFSETS,
    BIT_MASK,
    DATA_IN,
    NUM_INS
};

enum
{
    DATA_TYPE,
    INPUT_SIZE,
    OUTPUT_SIZE,
    MASK_SIZE,
    SIGNAL_SIZES,
    MASK_GO_RIGHT, // if 1 work from most to least significant bits
    NUM_PARAMS
};

enum
{
    DATA_OUT,
    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
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

    DTypeId data_type = (DTypeId)mxGetScalar(ssGetSFcnParam(S, DATA_TYPE));

    ssSetInputPortDirectFeedThrough(S, VALID, 1);
    ssSetInputPortWidth(S, VALID, 1);
    ssSetInputPortDataType(S, VALID, SS_UINT8);
    
    ssSetInputPortDirectFeedThrough(S, NUM_OFFSETS, 1);
    ssSetInputPortWidth(S, NUM_OFFSETS, 1);
    ssSetInputPortDataType(S, NUM_OFFSETS, SS_UINT8);

    ssSetInputPortDirectFeedThrough(S, BIT_MASK, 1);
    ssSetInputPortWidth(S, BIT_MASK, (int_T)mxGetScalar(ssGetSFcnParam(S, MASK_SIZE)));
    ssSetInputPortDataType(S, BIT_MASK, SS_UINT8);

    ssSetInputPortDirectFeedThrough(S, DATA_IN, 1);
    ssSetInputPortRequiredContiguous(S, DATA_IN, 1);
    ssSetInputPortWidth(S, DATA_IN, (int_T)mxGetScalar(ssGetSFcnParam(S, INPUT_SIZE)));
    ssSetInputPortDataType(S, DATA_IN, data_type);
    
    for (int idx = 0 ; idx < NUM_PARAMS ; ++idx )
        ssSetSFcnParamTunable(S, idx, SS_PRM_NOT_TUNABLE);

    ssSetOutputPortWidth(S, DATA_OUT, (int_T)mxGetScalar(ssGetSFcnParam(S, OUTPUT_SIZE)));
    ssSetOutputPortDataType(S, DATA_OUT, data_type);
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
    int dataSizes[] = {8, 4, 1, 1, 2, 2, 4, 4, 1};
    int data_size = dataSizes[ssGetOutputPortDataType(S, DATA_OUT)];

    int bm_go_right = (int_T)mxGetScalar(ssGetSFcnParam(S, MASK_GO_RIGHT));
    
    InputUInt8PtrsType valid_ptr = (InputInt8PtrsType)ssGetInputPortSignalPtrs(S, VALID);
    uint8_T valid_ = (uint8_T)*valid_ptr[0];
   
    InputUInt8PtrsType num_offset_ptr = (InputInt8PtrsType)ssGetInputPortSignalPtrs(S, NUM_OFFSETS);
    uint8_T num_offset_ = (uint8_T)*num_offset_ptr[0];
   
    InputUInt8PtrsType bit_mask_ptrs = (InputInt8PtrsType)ssGetInputPortSignalPtrs(S, BIT_MASK);
    
    int_T bm_size = ssGetInputPortWidth(S, BIT_MASK);

    size_t num_param_sizes = mxGetN(ssGetSFcnParam(S, SIGNAL_SIZES));
    mxDouble* param_sizes = mxGetDoubles(ssGetSFcnParam(S, SIGNAL_SIZES));

    uint8_T* u = (uint8_T*)ssGetInputPortSignal(S, DATA_IN);
    uint8_T* y = (uint8_T*)ssGetOutputPortSignal(S, DATA_OUT);

    memset((uint8_T*)ssGetOutputPortSignal(S, DATA_OUT), 0, (size_t)mxGetScalar(ssGetSFcnParam(S, OUTPUT_SIZE)) * data_size);

    /*
    printf("Valid: %d\n", valid_);
    printf("#Offset: %d\n", num_offset_);
    printf("BM_ptr: %X\n", bit_mask_ptrs[0]);
    printf("BM size: %d\n", bm_size);
    for (int ii = 0; ii < bm_size; ++ii)
        printf("      %d: %u \n", ii, *bit_mask_ptrs[ii]);
    printf("#Param: %d\n",num_param_sizes);
    for (int ii = 0; ii < num_param_sizes; ++ii)
        printf("     %d: %d \n", ii, (int)param_sizes[ii]);
    printf("------------------------\n");
    */
    
    if (!valid_)
    return;

    // Calculate how many parameters are defined by the bit mask
    int input_count = 0;
    size_t input_marker = 0;
    size_t param_marker = 0;
    uint8_T curr_byte_mask = 0;

    if (num_offset_ > 0)
    {
        for (int byte_idx = 0; byte_idx < bm_size; ++byte_idx)
        {
            curr_byte_mask = bm_go_right ? 128 : 1;
            for (int bit_idx = 0; bit_idx < 8; ++bit_idx)
            {
                if ((*bit_mask_ptrs[byte_idx] & curr_byte_mask))
                {
                    input_count += (int)param_sizes[param_marker];
                }
                
                curr_byte_mask = bm_go_right ? curr_byte_mask >> 1 : curr_byte_mask << 1;

                // Bug out if all the parameters are covered
                if ( !(++param_marker < num_param_sizes) )
                    break;
            }
        }
    }
    
    //printf("#Params: %d, Pointer: %x\n", num_param_sizes, param_sizes);
    //for (int ii = 0; ii < num_param_sizes; ++ii)
    //printf("%d ,", param_sizes[ii]);
    //printf("\n");
    //printf("Offset in values: %d, offset in bytes: %d\n", input_count, num_offset_[0] * input_count * data_size);
    //printf("--------------------------------------------------\n");

    // Catch a buffer overrun by just choosing the first data set
    if (input_count < (size_t)mxGetScalar(ssGetSFcnParam(S, INPUT_SIZE)))
        input_marker = num_offset_ * input_count * data_size; 
    
    // Copy specified set of parameters to output
    size_t output_marker = 0;
    param_marker = 0;
    for (int byte_idx = 0; byte_idx < bm_size; ++byte_idx)
    {
        curr_byte_mask = bm_go_right ? 128 : 1;
        for (int bit_idx = 0; bit_idx < 8; ++bit_idx)
        {
            if ( curr_byte_mask & *(bit_mask_ptrs[byte_idx]) )
            {
                // Copy the data to the output
                memcpy(y + output_marker, u + input_marker, (int)param_sizes[param_marker] * data_size);
                input_marker += (int)param_sizes[param_marker] * data_size;
            }
            // Advance markers
            output_marker += data_size * (int)param_sizes[param_marker];
            curr_byte_mask = bm_go_right ? curr_byte_mask >> 1 : curr_byte_mask << 1;
            // Bug out if all the parameters are covered
            if ( !(++param_marker < num_param_sizes) )
                break;
        }
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
