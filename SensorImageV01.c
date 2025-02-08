// Copyright (c) 2016-2020 rFpro Limited, All Rights Reserved.
//
// NOTICE:  All information contained herein is, and remains the property of rFpro. The intellectual and technical concepts contained
// herein are proprietary to rFpro and may be covered by U.S. and foreign patents, patents in process, and are protected by trade secret or copyright law.
// Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained from rFpro.
//
// The copyright notice above does not evidence any actual or intended publication or disclosure of this source code, which includes information that is confidential
// and/or proprietary, and is a trade secret, of rFpro.  ANY REPRODUCTION, DISTRIBUTION, PUBLIC PERFORMANCE, OR PUBLIC DISPLAY OF THIS SOURCE CODE
// WITHOUT THE EXPRESS WRITTEN CONSENT OF RFPRO IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL TREATIES.
// THE RECEIPT OR POSSESSION OF THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS TO REPRODUCE,
// DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT MAY DESCRIBE, IN WHOLE OR IN PART.

#define S_FUNCTION_NAME SensorImageV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUM_INS 0

enum {
    PAR_SAMPLE_TIME,
    PAR_WIDTH,
    PAR_HEIGHT,
    PAR_FORMAT,
    NUM_PARAMS
};

enum {
    OUT_TIME,
    OUT_IMAGE_I,    // image              (4 values per pixel)
    OUT_IMAGE_D,    // distance/range     (1 value per pixel)
    OUT_IMAGE_M,    // mesh ID            (1 value per pixel)
    OUT_IMAGE_S,    // segmented          (4 values per pixel)
    OUT_IMAGE_N,    // normal             (4 values per pixel)
    OUT_IMAGE_V,    // velocity           (4 values per pixel)
    OUT_IMAGE_L,    // light              (4 values per pixel)
    OUT_IMAGE_MAT,  // material ID        (1 value per pixel)
    OUT_IMAGE_FGT,  // functional mesh ID (1 value per pixel)
    NUM_OUTS
};

#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)
static void mdlSetWorkWidths(SimStruct *S)
{
    ssSetNumRunTimeParams(S, NUM_PARAMS);
    if (ssGetErrorStatus(S) != NULL) return;
    ssRegDlgParamAsRunTimeParam(S, PAR_SAMPLE_TIME, PAR_SAMPLE_TIME, "SampleTime", SS_DOUBLE);
    ssRegDlgParamAsRunTimeParam(S, PAR_WIDTH,       PAR_WIDTH,       "Width",      SS_INT32);
    ssRegDlgParamAsRunTimeParam(S, PAR_HEIGHT,      PAR_HEIGHT,      "Height",     SS_INT32);
    ssRegDlgParamAsRunTimeParam(S, PAR_FORMAT,      PAR_FORMAT,      "Format",     SS_UINT32);
}
#endif /* MDL_SET_WORK_WIDTHS */

static void mdlInitializeSizes(SimStruct *S)
{
    ssAllowSignalsWithMoreThan2D(S);
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, NUM_INS)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;
    
    ssSetOutputPortWidth(S, OUT_TIME, 1);
    ssSetOutputPortMatrixDimensions(S, OUT_IMAGE_D, (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_HEIGHT)), (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_WIDTH)));
    ssSetOutputPortMatrixDimensions(S, OUT_IMAGE_M, (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_HEIGHT)), (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_WIDTH)));
    ssSetOutputPortMatrixDimensions(S, OUT_IMAGE_S, (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_HEIGHT)), (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_WIDTH)));
    ssSetOutputPortMatrixDimensions(S, OUT_IMAGE_MAT, (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_HEIGHT)), (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_WIDTH)));
    ssSetOutputPortMatrixDimensions(S, OUT_IMAGE_FGT, (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_HEIGHT)), (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_WIDTH)));
    
    {
        DECL_AND_INIT_DIMSINFO(di);
        int_T dims[3];

        di.numDims = 3;
        dims[0] = (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_HEIGHT));
        dims[1] = (int_T)mxGetScalar(ssGetSFcnParam(S, PAR_WIDTH));
        dims[2] = 3;
        di.dims = dims;
        di.width = dims[0] * dims[1] * dims[2];
        ssSetOutputPortDimensionInfo(S,  OUT_IMAGE_I, &di);
        ssSetOutputPortDimensionInfo(S,  OUT_IMAGE_S, &di);
        ssSetOutputPortDimensionInfo(S,  OUT_IMAGE_N, &di);
        ssSetOutputPortDimensionInfo(S,  OUT_IMAGE_V, &di);
        ssSetOutputPortDimensionInfo(S,  OUT_IMAGE_L, &di);
    }
    
    {
        const DTypeId CHANNEL_WIDTH =
            ((unsigned)mxGetScalar(ssGetSFcnParam(S, PAR_FORMAT)) & 2) ?
                SS_SINGLE : SS_UINT8;
    
        ssSetOutputPortDataType(S, OUT_IMAGE_I, CHANNEL_WIDTH );
    }
    
    ssSetOutputPortDataType(S, OUT_IMAGE_D, SS_SINGLE );
    ssSetOutputPortDataType(S, OUT_IMAGE_M, SS_UINT32 );
    ssSetOutputPortDataType(S, OUT_IMAGE_S, SS_UINT8 );
    ssSetOutputPortDataType(S, OUT_IMAGE_N, SS_SINGLE );
    ssSetOutputPortDataType(S, OUT_IMAGE_V, SS_SINGLE );
    ssSetOutputPortDataType(S, OUT_IMAGE_L, SS_SINGLE );
    ssSetOutputPortDataType(S, OUT_IMAGE_MAT, SS_UINT32 );
    ssSetOutputPortDataType(S, OUT_IMAGE_FGT, SS_UINT32 );
    
    ssSetSFcnParamTunable(S,PAR_SAMPLE_TIME,false);
    ssSetSFcnParamTunable(S,PAR_WIDTH,false);
    ssSetSFcnParamTunable(S,PAR_HEIGHT,false);  
    ssSetSFcnParamTunable(S,PAR_FORMAT,true);  
    
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
