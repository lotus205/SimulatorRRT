/*
 * Vehicle.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Vehicle".
 *
 * Model version              : 1.7
 * Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
 * C source code generated on : Fri May 11 03:34:41 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Vehicle_h_
#define RTW_HEADER_Vehicle_h_
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>
#ifndef Vehicle_COMMON_INCLUDES_
# define Vehicle_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* Vehicle_COMMON_INCLUDES_ */

#include "Vehicle_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T VectorConcatenate3[2];        /* '<S28>/Vector Concatenate3' */
  real_T VectorConcatenate[4];         /* '<S85>/Vector Concatenate' */
  real_T VectorConcatenate_a[3];       /* '<S29>/Vector Concatenate' */
  real_T VectorConcatenate1[2];        /* '<S56>/Vector Concatenate1' */
  real_T VectorConcatenate1_o[2];      /* '<S62>/Vector Concatenate1' */
  real_T VectorConcatenate1_f[2];      /* '<S72>/Vector Concatenate1' */
  real_T VectorConcatenate1_n[3];      /* '<S88>/Vector Concatenate1' */
  real_T UnaryMinus[3];                /* '<S20>/Unary Minus' */
  real_T VectorConcatenate_b[6];       /* '<S20>/Vector Concatenate' */
  real_T jxk;                          /* '<S36>/j x k' */
  real_T ixj;                          /* '<S36>/i x j' */
  real_T ixk;                          /* '<S37>/i x k' */
  real_T jxi;                          /* '<S37>/j x i' */
  real_T Product1;                     /* '<S82>/Product1' */
  real_T Product1_j;                   /* '<S83>/Product1' */
  real_T yOut[15];                     /* '<S2>/vehicle model' */
  real_T stateDer[4];                  /* '<S2>/vehicle model' */
} B_Vehicle_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  int_T Integrator_IWORK;              /* '<S28>/Integrator' */
  int_T Integrator_IWORK_c;            /* '<S85>/Integrator' */
} DW_Vehicle_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE[2];         /* '<S28>/Integrator' */
  real_T Integrator_CSTATE_e[4];       /* '<S85>/Integrator' */
  real_T lateral_CSTATE;               /* '<S82>/lateral' */
  real_T lateral_CSTATE_e;             /* '<S83>/lateral' */
} X_Vehicle_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE[2];         /* '<S28>/Integrator' */
  real_T Integrator_CSTATE_e[4];       /* '<S85>/Integrator' */
  real_T lateral_CSTATE;               /* '<S82>/lateral' */
  real_T lateral_CSTATE_e;             /* '<S83>/lateral' */
} XDot_Vehicle_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE[2];      /* '<S28>/Integrator' */
  boolean_T Integrator_CSTATE_e[4];    /* '<S85>/Integrator' */
  boolean_T lateral_CSTATE;            /* '<S82>/lateral' */
  boolean_T lateral_CSTATE_e;          /* '<S83>/lateral' */
} XDis_Vehicle_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T EgoVelocity;                  /* '<Root>/Ego Velocity' */
  real_T SteeringAngle;                /* '<Root>/Steering Angle' */
} ExtU_Vehicle_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T XYPositions[2];               /* '<Root>/XY Positions' */
  real_T XYVelocities[2];              /* '<Root>/XY Velocities' */
  real_T YawAngle;                     /* '<Root>/Yaw Angle' */
  real_T YawRate;                      /* '<Root>/Yaw Rate' */
} ExtY_Vehicle_T;

/* Parameters (default storage) */
struct P_Vehicle_T_ {
  real_T Cf;                           /* Variable: Cf
                                        * Referenced by: '<S15>/Cyf'
                                        */
  real_T Cr;                           /* Variable: Cr
                                        * Referenced by: '<S15>/Cyr'
                                        */
  real_T Iz;                           /* Variable: Iz
                                        * Referenced by: '<S2>/vehicle model'
                                        */
  real_T lf;                           /* Variable: lf
                                        * Referenced by:
                                        *   '<S2>/vehicle model'
                                        *   '<S20>/Constant3'
                                        *   '<S29>/R_T1'
                                        */
  real_T lr;                           /* Variable: lr
                                        * Referenced by:
                                        *   '<S2>/vehicle model'
                                        *   '<S20>/Constant3'
                                        *   '<S30>/R_T1'
                                        */
  real_T m;                            /* Variable: m
                                        * Referenced by: '<S2>/vehicle model'
                                        */
  real_T x0_ego;                       /* Variable: x0_ego
                                        * Referenced by: '<S28>/X_o'
                                        */
  real_T y0_ego;                       /* Variable: y0_ego
                                        * Referenced by: '<S28>/Y_o'
                                        */
  real_T yaw0_ego;                     /* Variable: yaw0_ego
                                        * Referenced by: '<S85>/psi_o'
                                        */
  real_T BicycleModelVelocityInput_Af; /* Mask Parameter: BicycleModelVelocityInput_Af
                                        * Referenced by: '<S20>/.5.*A.*Pabs.//R.//T'
                                        */
  real_T BicycleModelVelocityInput_Cd; /* Mask Parameter: BicycleModelVelocityInput_Cd
                                        * Referenced by: '<S20>/Constant'
                                        */
  real_T BicycleModelVelocityInput_Cl; /* Mask Parameter: BicycleModelVelocityInput_Cl
                                        * Referenced by: '<S20>/Constant1'
                                        */
  real_T BicycleModelVelocityInput_Cpm;/* Mask Parameter: BicycleModelVelocityInput_Cpm
                                        * Referenced by: '<S20>/Constant2'
                                        */
  real_T BicycleModelVelocityInput_Cs[31];/* Mask Parameter: BicycleModelVelocityInput_Cs
                                           * Referenced by: '<S20>/Cs'
                                           */
  real_T BicycleModelVelocityInput_Cym[31];/* Mask Parameter: BicycleModelVelocityInput_Cym
                                            * Referenced by: '<S20>/Cym'
                                            */
  real_T BicycleModelVelocityInput_Fznom;/* Mask Parameter: BicycleModelVelocityInput_Fznom
                                          * Referenced by: '<S2>/vehicle model'
                                          */
  real_T BicycleModelVelocityInput_NF; /* Mask Parameter: BicycleModelVelocityInput_NF
                                        * Referenced by: '<S2>/vehicle model'
                                        */
  real_T BicycleModelVelocityInput_NR; /* Mask Parameter: BicycleModelVelocityInput_NR
                                        * Referenced by: '<S2>/vehicle model'
                                        */
  real_T BicycleModelVelocityInput_Pabs;/* Mask Parameter: BicycleModelVelocityInput_Pabs
                                         * Referenced by: '<S20>/.5.*A.*Pabs.//R.//T'
                                         */
  real_T DragForce_R;                  /* Mask Parameter: DragForce_R
                                        * Referenced by: '<S20>/.5.*A.*Pabs.//R.//T'
                                        */
  real_T HardPointCoordinateTransformFro;/* Mask Parameter: HardPointCoordinateTransformFro
                                          * Referenced by: '<S29>/R_T2'
                                          */
  real_T HardPointCoordinateTransformRea;/* Mask Parameter: HardPointCoordinateTransformRea
                                          * Referenced by: '<S30>/R_T2'
                                          */
  real_T BicycleModelVelocityInput_Tair;/* Mask Parameter: BicycleModelVelocityInput_Tair
                                         * Referenced by: '<S20>/.5.*A.*Pabs.//R.//T'
                                         */
  real_T BicycleModelVelocityInput_beta_[31];/* Mask Parameter: BicycleModelVelocityInput_beta_
                                              * Referenced by:
                                              *   '<S20>/Cs'
                                              *   '<S20>/Cym'
                                              */
  real_T BicycleModelVelocityInput_g;  /* Mask Parameter: BicycleModelVelocityInput_g
                                        * Referenced by: '<S2>/vehicle model'
                                        */
  real_T BicycleModelVelocityInput_h;  /* Mask Parameter: BicycleModelVelocityInput_h
                                        * Referenced by:
                                        *   '<S2>/vehicle model'
                                        *   '<S29>/R_T3'
                                        *   '<S30>/R_T3'
                                        */
  real_T BicycleModelVelocityInput_r_o;/* Mask Parameter: BicycleModelVelocityInput_r_o
                                        * Referenced by: '<S85>/r_o'
                                        */
  real_T VehicleBody3DOFSingleTrack_sigm;/* Mask Parameter: VehicleBody3DOFSingleTrack_sigm
                                          * Referenced by: '<S80>/Constant1'
                                          */
  real_T VehicleBody3DOFSingleTrack_si_m;/* Mask Parameter: VehicleBody3DOFSingleTrack_si_m
                                          * Referenced by: '<S80>/Constant2'
                                          */
  real_T BicycleModelVelocityInput_xdot_;/* Mask Parameter: BicycleModelVelocityInput_xdot_
                                          * Referenced by: '<S2>/vehicle model'
                                          */
  real_T BicycleModelVelocityInput_ydot_;/* Mask Parameter: BicycleModelVelocityInput_ydot_
                                          * Referenced by: '<S85>/ydot_o'
                                          */
  real_T vehiclemodel_Fxtire_sat;      /* Expression: Fxtire_sat
                                        * Referenced by: '<S2>/vehicle model'
                                        */
  real_T vehiclemodel_Fytire_sat;      /* Expression: Fytire_sat
                                        * Referenced by: '<S2>/vehicle model'
                                        */
  real_T vehiclemodel_w[2];            /* Expression: w
                                        * Referenced by: '<S2>/vehicle model'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S28>/Constant'
                                        */
  real_T Constant7_Value;              /* Expression: 0
                                        * Referenced by: '<S28>/Constant7'
                                        */
  real_T Constant2_Value;              /* Expression: 0
                                        * Referenced by: '<S28>/Constant2'
                                        */
  real_T Constant_Value_c;             /* Expression: mu
                                        * Referenced by: '<S56>/Constant'
                                        */
  real_T Constant4_Value[3];           /* Expression: ones(1,3)
                                        * Referenced by: '<S20>/Constant4'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<S20>/Switch'
                                        */
  real_T Crm_tableData[2];             /* Expression: [0 0]
                                        * Referenced by: '<S20>/Crm'
                                        */
  real_T Crm_bp01Data[2];              /* Expression: [-1 1]
                                        * Referenced by: '<S20>/Crm'
                                        */
  real_T Constant8_Value;              /* Expression: 0
                                        * Referenced by: '<S28>/Constant8'
                                        */
  real_T Constant5_Value;              /* Expression: 0
                                        * Referenced by: '<S28>/Constant5'
                                        */
  real_T Constant4_Value_c;            /* Expression: 0
                                        * Referenced by: '<S28>/Constant4'
                                        */
  real_T Constant6_Value;              /* Expression: 0
                                        * Referenced by: '<S28>/Constant6'
                                        */
  real_T lateral_IC;                   /* Expression: 0
                                        * Referenced by: '<S82>/lateral'
                                        */
  real_T lateral_IC_e;                 /* Expression: 0
                                        * Referenced by: '<S83>/lateral'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Vehicle_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  X_Vehicle_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[8];
  real_T odeF[3][8];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    boolean_T firstInitCondFlag;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_Vehicle_T Vehicle_P;

/* Block signals (default storage) */
extern B_Vehicle_T Vehicle_B;

/* Continuous states (default storage) */
extern X_Vehicle_T Vehicle_X;

/* Block states (default storage) */
extern DW_Vehicle_T Vehicle_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Vehicle_T Vehicle_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Vehicle_T Vehicle_Y;

/* Model entry point functions */
extern void Vehicle_initialize(void);
extern void Vehicle_step(void);
extern void Vehicle_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Vehicle_T *const Vehicle_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Vehicle'
 * '<S1>'   : 'Vehicle/Bicycle Model - Velocity Input'
 * '<S2>'   : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track'
 * '<S3>'   : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Cy'
 * '<S4>'   : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Drag'
 * '<S5>'   : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing'
 * '<S6>'   : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/friction'
 * '<S7>'   : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front forces'
 * '<S8>'   : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front steer'
 * '<S9>'   : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear forces'
 * '<S10>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear steer'
 * '<S11>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/sigma'
 * '<S12>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/state'
 * '<S13>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/vehicle model'
 * '<S14>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/wind'
 * '<S15>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Cy/Cy const'
 * '<S16>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Cy/Cy const dual'
 * '<S17>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Cy/Cy table'
 * '<S18>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Cy/Cy table dual'
 * '<S19>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Cy/Cy table dual/For Each Subsystem'
 * '<S20>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Drag/Drag Force'
 * '<S21>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Drag/inertial2body'
 * '<S22>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing'
 * '<S23>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing Dual'
 * '<S24>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Forces 3DOF'
 * '<S25>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF'
 * '<S26>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Moments'
 * '<S27>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Power'
 * '<S28>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/state2bus'
 * '<S29>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Front'
 * '<S30>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Rear'
 * '<S31>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Front/Rotation Angles to Direction Cosine Matrix'
 * '<S32>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Front/transform to Inertial axes'
 * '<S33>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Front/transform to Inertial axes1'
 * '<S34>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Front/wxR'
 * '<S35>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Front/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S36>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Front/wxR/Subsystem'
 * '<S37>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Front/wxR/Subsystem1'
 * '<S38>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Rear/Rotation Angles to Direction Cosine Matrix'
 * '<S39>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Rear/transform to Inertial axes'
 * '<S40>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Rear/transform to Inertial axes1'
 * '<S41>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Rear/wxR'
 * '<S42>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Rear/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S43>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Rear/wxR/Subsystem'
 * '<S44>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/Lateral 3DOF/Hard Point Coordinate Transform Rear/wxR/Subsystem1'
 * '<S45>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/state2bus/xddot2ax'
 * '<S46>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing/state2bus/xddot2ax/m^22gn'
 * '<S47>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing Dual/Forces 3DOF'
 * '<S48>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing Dual/Lateral 3DOF'
 * '<S49>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing Dual/state2bus'
 * '<S50>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Left'
 * '<S51>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Right'
 * '<S52>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left'
 * '<S53>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left1'
 * '<S54>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/friction/mu ext'
 * '<S55>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/friction/mu ext dual'
 * '<S56>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/friction/mu int'
 * '<S57>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/friction/mu int dual'
 * '<S58>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front forces/ext'
 * '<S59>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front forces/ext dual'
 * '<S60>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front forces/ext long'
 * '<S61>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front forces/ext long dual'
 * '<S62>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front forces/int'
 * '<S63>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front forces/int dual'
 * '<S64>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front steer/delta ext'
 * '<S65>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front steer/delta ext dual'
 * '<S66>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front steer/delta int'
 * '<S67>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/front steer/delta int dual'
 * '<S68>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear forces/ext'
 * '<S69>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear forces/ext dual'
 * '<S70>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear forces/ext long'
 * '<S71>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear forces/ext long dual'
 * '<S72>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear forces/int'
 * '<S73>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear forces/int dual'
 * '<S74>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear steer/delta ext'
 * '<S75>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear steer/delta ext dual'
 * '<S76>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear steer/delta int'
 * '<S77>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/rear steer/delta int dual'
 * '<S78>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/sigma/no sigma'
 * '<S79>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/sigma/no sigma dual'
 * '<S80>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/sigma/sigma'
 * '<S81>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/sigma/sigma dual'
 * '<S82>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/sigma/sigma/relaxation front'
 * '<S83>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/sigma/sigma/relaxation rear'
 * '<S84>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/sigma/sigma dual/relaxation'
 * '<S85>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/state/xdot ext'
 * '<S86>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/state/xdot int'
 * '<S87>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/wind/wind ext'
 * '<S88>'  : 'Vehicle/Bicycle Model - Velocity Input/Vehicle Body 3DOF Single Track/wind/wind int'
 */
#endif                                 /* RTW_HEADER_Vehicle_h_ */
