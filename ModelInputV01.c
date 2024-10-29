// Copyright (c) 2020 rFpro Limited, All Rights Reserved.

#define S_FUNCTION_NAME ModelInputV01
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define MAX_OPPONENTS 40

enum 
{
    NUM_PAR
};

enum 
{
    // weather
    OUT_AIR_DENSITY,
    OUT_HUMIDITY,
    OUT_PRESSURE,
    OUT_AMBIENT_TEMP,
    OUT_TRACK_TEMP,
    OUT_WIND,
    
    OUT_COLLISION,
    OUT_COLLISION_EXIT_VECTOR,
    OUT_COLLISION_FORCE,
    OUT_COLLISION_TORQUE,

    // miscellaneous
    OUT_PITTING,
    OUT_BEACON,
    
    // Internal state
    OUT_EGO_POS,
    OUT_EGO_ORI,
    OUT_EGO_VEL,
    OUT_EGO_ROT,
    
    // Driver inputs
    OUT_STEERING,
    OUT_THROTTLE,
    OUT_HANDBRAKE,
    OUT_BRAKES,
    OUT_CLUTCH,
    OUT_POWER_DEMAND,  // KERS or other

    OUT_DIRECT_MANUAL_SHIFT, // -2="no selection"/invalid, -1=reverse, 0=neutral, 1+=forward gear
    OUT_SHIFT_UP,
    OUT_SHIFT_DOWN,
    OUT_SHIFT_TO_NEUTRAL,
    OUT_TC_OVERRIDE,
    OUT_LAUNCH_CONTROL,

    OUT_NUM_OPPONENTS,
    OUT_OPPONENT_ID,
    OUT_POS,
    OUT_ORI,
    OUT_VEL,
    OUT_ROT,

    NUM_OUTS
};

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    if (!ssSetNumInputPorts(S, 0)) return;
    if (!ssSetNumOutputPorts(S, NUM_OUTS)) return;
    
    ssSetOutputPortWidth(S, OUT_AIR_DENSITY, 1);
    ssSetOutputPortWidth(S, OUT_HUMIDITY, 1);
    ssSetOutputPortWidth(S, OUT_PRESSURE, 1);
    ssSetOutputPortWidth(S, OUT_AMBIENT_TEMP, 1);
    ssSetOutputPortWidth(S, OUT_TRACK_TEMP, 1);
    ssSetOutputPortWidth(S, OUT_WIND, 3);
    
    ssSetOutputPortWidth(S, OUT_COLLISION, 1);
    ssSetOutputPortWidth(S, OUT_COLLISION_EXIT_VECTOR, 3);
    ssSetOutputPortWidth(S, OUT_COLLISION_FORCE, 3);
    ssSetOutputPortWidth(S, OUT_COLLISION_TORQUE, 3);

    ssSetOutputPortWidth(S, OUT_PITTING, 1);
    ssSetOutputPortWidth(S, OUT_BEACON, 6);

    ssSetOutputPortWidth(S, OUT_EGO_POS, 3);
    ssSetOutputPortWidth(S, OUT_EGO_ORI, 9);
    ssSetOutputPortWidth(S, OUT_EGO_VEL, 3);
    ssSetOutputPortWidth(S, OUT_EGO_ROT, 3);
    
    ssSetOutputPortWidth(S, OUT_STEERING, 1);
    ssSetOutputPortWidth(S, OUT_THROTTLE, 1);
    ssSetOutputPortWidth(S, OUT_HANDBRAKE, 1);
    ssSetOutputPortWidth(S, OUT_BRAKES, 1);
    ssSetOutputPortWidth(S, OUT_CLUTCH, 1);
    ssSetOutputPortWidth(S, OUT_POWER_DEMAND, 1);

    ssSetOutputPortWidth(S, OUT_DIRECT_MANUAL_SHIFT, 1);
    ssSetOutputPortDataType(S, OUT_DIRECT_MANUAL_SHIFT, 6); // int2
    
    ssSetOutputPortWidth(S, OUT_SHIFT_UP, 1);
    ssSetOutputPortDataType(S, OUT_SHIFT_UP, 8); // Boolean
    ssSetOutputPortWidth(S, OUT_SHIFT_DOWN, 1);
    ssSetOutputPortDataType(S, OUT_SHIFT_DOWN, 8); // Boolean
    ssSetOutputPortWidth(S, OUT_SHIFT_TO_NEUTRAL, 1);
    ssSetOutputPortDataType(S, OUT_SHIFT_TO_NEUTRAL, 8); // Boolean
    ssSetOutputPortWidth(S, OUT_TC_OVERRIDE, 1);
    ssSetOutputPortDataType(S, OUT_TC_OVERRIDE, 8); // Boolean
    ssSetOutputPortWidth(S, OUT_LAUNCH_CONTROL, 1);
    ssSetOutputPortDataType(S, OUT_LAUNCH_CONTROL, 8); // Boolean
    
    ssSetOutputPortWidth(S, OUT_NUM_OPPONENTS, 1);
    ssSetOutputPortDataType(S, OUT_NUM_OPPONENTS, 6); // int32
    ssSetOutputPortWidth(S, OUT_OPPONENT_ID,  MAX_OPPONENTS);
    ssSetOutputPortMatrixDimensions(S, OUT_POS, 3, MAX_OPPONENTS);
    ssSetOutputPortMatrixDimensions(S, OUT_ORI, 9, MAX_OPPONENTS);
    ssSetOutputPortMatrixDimensions(S, OUT_VEL, 3, MAX_OPPONENTS);
    ssSetOutputPortMatrixDimensions(S, OUT_ROT, 3, MAX_OPPONENTS);
    
    ssSetNumContStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
}

#define MDL_OUTPUTS
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
