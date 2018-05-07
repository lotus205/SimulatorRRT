/*
 * call_stats_bolck1.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "call_stats_bolck1".
 *
 * Model version              : 1.3
 * Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
 * C source code generated on : Tue May  8 01:36:17 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "call_stats_bolck1.h"
#include "call_stats_bolck1_private.h"

/* Real-time model */
RT_MODEL_call_stats_bolck1_T call_stats_bolck1_M_;
RT_MODEL_call_stats_bolck1_T *const call_stats_bolck1_M = &call_stats_bolck1_M_;

/* Model step function */
void call_stats_bolck1_step(void)
{
  /* Matfile logging */
  rt_UpdateTXYLogVars(call_stats_bolck1_M->rtwLogInfo,
                      (&call_stats_bolck1_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.2s, 0.0s] */
    if ((rtmGetTFinal(call_stats_bolck1_M)!=-1) &&
        !((rtmGetTFinal(call_stats_bolck1_M)-
           call_stats_bolck1_M->Timing.taskTime0) >
          call_stats_bolck1_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(call_stats_bolck1_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++call_stats_bolck1_M->Timing.clockTick0)) {
    ++call_stats_bolck1_M->Timing.clockTickH0;
  }

  call_stats_bolck1_M->Timing.taskTime0 = call_stats_bolck1_M->Timing.clockTick0
    * call_stats_bolck1_M->Timing.stepSize0 +
    call_stats_bolck1_M->Timing.clockTickH0 *
    call_stats_bolck1_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void call_stats_bolck1_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)call_stats_bolck1_M, 0,
                sizeof(RT_MODEL_call_stats_bolck1_T));
  rtmSetTFinal(call_stats_bolck1_M, 10.0);
  call_stats_bolck1_M->Timing.stepSize0 = 0.2;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    call_stats_bolck1_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(call_stats_bolck1_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(call_stats_bolck1_M->rtwLogInfo, (NULL));
    rtliSetLogT(call_stats_bolck1_M->rtwLogInfo, "tout");
    rtliSetLogX(call_stats_bolck1_M->rtwLogInfo, "");
    rtliSetLogXFinal(call_stats_bolck1_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(call_stats_bolck1_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(call_stats_bolck1_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(call_stats_bolck1_M->rtwLogInfo, 0);
    rtliSetLogDecimation(call_stats_bolck1_M->rtwLogInfo, 1);
    rtliSetLogY(call_stats_bolck1_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(call_stats_bolck1_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(call_stats_bolck1_M->rtwLogInfo, (NULL));
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(call_stats_bolck1_M->rtwLogInfo, 0.0,
    rtmGetTFinal(call_stats_bolck1_M), call_stats_bolck1_M->Timing.stepSize0,
    (&rtmGetErrorStatus(call_stats_bolck1_M)));
}

/* Model terminate function */
void call_stats_bolck1_terminate(void)
{
  /* (no terminate code required) */
}
