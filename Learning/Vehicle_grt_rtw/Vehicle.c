/*
 * Vehicle.c
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

#include "Vehicle.h"
#include "Vehicle_private.h"

/* Block signals (default storage) */
B_Vehicle_T Vehicle_B;

/* Continuous states */
X_Vehicle_T Vehicle_X;

/* Block states (default storage) */
DW_Vehicle_T Vehicle_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Vehicle_T Vehicle_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Vehicle_T Vehicle_Y;

/* Real-time model */
RT_MODEL_Vehicle_T Vehicle_M_;
RT_MODEL_Vehicle_T *const Vehicle_M = &Vehicle_M_;

/* Forward declaration for local functions */
static void Vehicle_automlvehdynftiresat(real_T Ftire_y, real_T b_Fxtire_sat,
  real_T b_Fytire_sat, real_T *Ftire_xs, real_T *Ftire_ys);
real_T look1_binlcpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = 1.0;
  }

  /* Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 8;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  Vehicle_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  Vehicle_step();
  Vehicle_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  Vehicle_step();
  Vehicle_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<S2>/vehicle model' */
static void Vehicle_automlvehdynftiresat(real_T Ftire_y, real_T b_Fxtire_sat,
  real_T b_Fytire_sat, real_T *Ftire_xs, real_T *Ftire_ys)
{
  real_T theta_Ftire;
  real_T b_a;
  real_T c_a;
  theta_Ftire = rt_atan2d_snf(0.0, Ftire_y);
  b_a = b_Fxtire_sat * cos(theta_Ftire);
  c_a = b_Fytire_sat * sin(theta_Ftire);
  theta_Ftire = b_Fxtire_sat * b_Fytire_sat / sqrt(b_a * b_a + c_a * c_a) * cos
    (theta_Ftire);
  *Ftire_xs = 0.0;
  *Ftire_ys = Ftire_y;
  if (fabs(Ftire_y) > fabs(theta_Ftire)) {
    *Ftire_ys = theta_Ftire;
  }
}

/* Model step function */
void Vehicle_step(void)
{
  real_T alfa_f;
  real_T alfa_r;
  real_T Fy_f;
  real_T Fy_r;
  real_T yddot;
  real_T Fx_ft;
  real_T rtb_Product_e;
  real_T rtb_VectorConcatenate_l[9];
  real_T rtb_Sum_n;
  real_T rtb_uAPabsRT[6];
  real_T rtb_Transpose1[9];
  int32_T i;
  int32_T loop_ub;
  real_T tmp[3];
  int32_T i_0;
  real_T rtb_Transpose1_0[3];
  real_T rtb_UnaryMinus;
  real_T rtb_VectorConcatenate1_p_idx_2;
  real_T rtb_UnaryMinus_idx_1;
  real_T rtb_VectorConcatenate1_p_idx_1;
  real_T rtb_UnaryMinus_idx_2;
  real_T rtb_UnaryMinus_idx_0;
  real_T rtb_VectorConcatenate1_idx_3;
  real_T Fz_idx_0;
  real_T Fz_idx_1;
  real_T rtb_UnaryMinus_idx_0_tmp;
  real_T rtb_VectorConcatenate1_p_idx_0_;
  real_T rtb_Sum_i_tmp;
  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* set solver stop time */
    if (!(Vehicle_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&Vehicle_M->solverInfo,
                            ((Vehicle_M->Timing.clockTickH0 + 1) *
        Vehicle_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&Vehicle_M->solverInfo,
                            ((Vehicle_M->Timing.clockTick0 + 1) *
        Vehicle_M->Timing.stepSize0 + Vehicle_M->Timing.clockTickH0 *
        Vehicle_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(Vehicle_M)) {
    Vehicle_M->Timing.t[0] = rtsiGetT(&Vehicle_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* Constant: '<S28>/X_o' */
    Vehicle_B.VectorConcatenate3[0] = Vehicle_P.x0_ego;

    /* Constant: '<S28>/Y_o' */
    Vehicle_B.VectorConcatenate3[1] = Vehicle_P.y0_ego;
  }

  /* Integrator: '<S28>/Integrator' */
  if (Vehicle_DW.Integrator_IWORK != 0) {
    Vehicle_X.Integrator_CSTATE[0] = Vehicle_B.VectorConcatenate3[0];
    Vehicle_X.Integrator_CSTATE[1] = Vehicle_B.VectorConcatenate3[1];
  }

  /* SignalConversion: '<S85>/ConcatBufferAtVector ConcatenateIn1' incorporates:
   *  Inport: '<Root>/Ego Velocity'
   */
  /* Unit Conversion - from: m/s to: m/s
     Expression: output = (1*input) + (0) */
  Vehicle_B.VectorConcatenate[0] = Vehicle_U.EgoVelocity;
  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* Constant: '<S85>/ydot_o' */
    Vehicle_B.VectorConcatenate[1] = Vehicle_P.BicycleModelVelocityInput_ydot_;

    /* Constant: '<S85>/psi_o' */
    Vehicle_B.VectorConcatenate[2] = Vehicle_P.yaw0_ego * 3.1415926535897931 /
      180.0;

    /* Constant: '<S85>/r_o' */
    Vehicle_B.VectorConcatenate[3] = Vehicle_P.BicycleModelVelocityInput_r_o;
  }

  /* Integrator: '<S85>/Integrator' */
  if (Vehicle_DW.Integrator_IWORK_c != 0) {
    Vehicle_X.Integrator_CSTATE_e[0] = Vehicle_B.VectorConcatenate[0];
    Vehicle_X.Integrator_CSTATE_e[1] = Vehicle_B.VectorConcatenate[1];
    Vehicle_X.Integrator_CSTATE_e[2] = Vehicle_B.VectorConcatenate[2];
    Vehicle_X.Integrator_CSTATE_e[3] = Vehicle_B.VectorConcatenate[3];
  }

  /* SignalConversion: '<S85>/ConcatBufferAtVector Concatenate1In2' incorporates:
   *  Integrator: '<S85>/Integrator'
   */
  rtb_VectorConcatenate1_idx_3 = Vehicle_X.Integrator_CSTATE_e[3];

  /* Trigonometry: '<S31>/sincos' incorporates:
   *  Constant: '<S28>/Constant2'
   *  Constant: '<S28>/Constant7'
   *  Integrator: '<S85>/Integrator'
   *  MATLAB Function: '<S2>/vehicle model'
   *  Trigonometry: '<S21>/Trigonometric Function'
   */
  rtb_UnaryMinus_idx_0_tmp = sin(Vehicle_X.Integrator_CSTATE_e[2]);
  rtb_VectorConcatenate1_p_idx_0_ = cos(Vehicle_X.Integrator_CSTATE_e[2]);
  rtb_UnaryMinus_idx_1 = sin(Vehicle_P.Constant7_Value);
  rtb_VectorConcatenate1_p_idx_1 = cos(Vehicle_P.Constant7_Value);
  rtb_UnaryMinus_idx_2 = sin(Vehicle_P.Constant2_Value);
  rtb_VectorConcatenate1_p_idx_2 = cos(Vehicle_P.Constant2_Value);

  /* Fcn: '<S31>/Fcn11' incorporates:
   *  Trigonometry: '<S31>/sincos'
   */
  rtb_VectorConcatenate_l[0] = rtb_VectorConcatenate1_p_idx_1 *
    rtb_VectorConcatenate1_p_idx_0_;

  /* Fcn: '<S31>/Fcn21' incorporates:
   *  Fcn: '<S31>/Fcn22'
   *  Trigonometry: '<S31>/sincos'
   */
  rtb_UnaryMinus_idx_0 = rtb_UnaryMinus_idx_2 * rtb_UnaryMinus_idx_1;
  rtb_VectorConcatenate_l[1] = rtb_UnaryMinus_idx_0 *
    rtb_VectorConcatenate1_p_idx_0_ - rtb_VectorConcatenate1_p_idx_2 *
    rtb_UnaryMinus_idx_0_tmp;

  /* Fcn: '<S31>/Fcn31' incorporates:
   *  Fcn: '<S31>/Fcn32'
   *  Trigonometry: '<S31>/sincos'
   */
  rtb_Sum_n = rtb_VectorConcatenate1_p_idx_2 * rtb_UnaryMinus_idx_1;
  rtb_VectorConcatenate_l[2] = rtb_Sum_n * rtb_VectorConcatenate1_p_idx_0_ +
    rtb_UnaryMinus_idx_2 * rtb_UnaryMinus_idx_0_tmp;

  /* Fcn: '<S31>/Fcn12' incorporates:
   *  Trigonometry: '<S31>/sincos'
   */
  rtb_VectorConcatenate_l[3] = rtb_VectorConcatenate1_p_idx_1 *
    rtb_UnaryMinus_idx_0_tmp;

  /* Fcn: '<S31>/Fcn22' incorporates:
   *  Trigonometry: '<S31>/sincos'
   */
  rtb_VectorConcatenate_l[4] = rtb_UnaryMinus_idx_0 * rtb_UnaryMinus_idx_0_tmp +
    rtb_VectorConcatenate1_p_idx_2 * rtb_VectorConcatenate1_p_idx_0_;

  /* Fcn: '<S31>/Fcn32' incorporates:
   *  Trigonometry: '<S31>/sincos'
   */
  rtb_VectorConcatenate_l[5] = rtb_Sum_n * rtb_UnaryMinus_idx_0_tmp -
    rtb_UnaryMinus_idx_2 * rtb_VectorConcatenate1_p_idx_0_;

  /* Fcn: '<S31>/Fcn13' */
  rtb_VectorConcatenate_l[6] = -rtb_UnaryMinus_idx_1;

  /* Fcn: '<S31>/Fcn23' */
  rtb_VectorConcatenate_l[7] = rtb_UnaryMinus_idx_2 *
    rtb_VectorConcatenate1_p_idx_1;

  /* Fcn: '<S31>/Fcn33' */
  rtb_VectorConcatenate_l[8] = rtb_VectorConcatenate1_p_idx_2 *
    rtb_VectorConcatenate1_p_idx_1;

  /* Math: '<S29>/Transpose1' */
  for (i = 0; i < 3; i++) {
    rtb_Transpose1[3 * i] = rtb_VectorConcatenate_l[i];
    rtb_Transpose1[1 + 3 * i] = rtb_VectorConcatenate_l[i + 3];
    rtb_Transpose1[2 + 3 * i] = rtb_VectorConcatenate_l[i + 6];
  }

  /* End of Math: '<S29>/Transpose1' */
  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* Constant: '<S29>/R_T1' */
    Vehicle_B.VectorConcatenate_a[0] = Vehicle_P.lf;

    /* Constant: '<S29>/R_T2' */
    Vehicle_B.VectorConcatenate_a[1] = Vehicle_P.HardPointCoordinateTransformFro;

    /* Constant: '<S29>/R_T3' */
    Vehicle_B.VectorConcatenate_a[2] = Vehicle_P.BicycleModelVelocityInput_h;
  }

  /* Sum: '<S29>/Add' incorporates:
   *  Constant: '<S28>/Constant'
   *  Integrator: '<S28>/Integrator'
   */
  rtb_Transpose1_0[0] = Vehicle_X.Integrator_CSTATE[0];
  rtb_Transpose1_0[1] = Vehicle_X.Integrator_CSTATE[1];
  rtb_Transpose1_0[2] = Vehicle_P.Constant_Value;
  for (i = 0; i < 3; i++) {
    /* Sum: '<S29>/Add' incorporates:
     *  Outport: '<Root>/XY Positions'
     *  Product: '<S33>/Product'
     */
    tmp[i] = rtb_Transpose1_0[i] + (rtb_Transpose1[i + 6] *
      Vehicle_B.VectorConcatenate_a[2] + (rtb_Transpose1[i + 3] *
      Vehicle_B.VectorConcatenate_a[1] + rtb_Transpose1[i] *
      Vehicle_B.VectorConcatenate_a[0]));
  }

  /* Outport: '<Root>/XY Positions' */
  Vehicle_Y.XYPositions[0] = tmp[0];
  Vehicle_Y.XYPositions[1] = tmp[1];

  /* Fcn: '<Root>/Fcn' incorporates:
   *  Inport: '<Root>/Steering Angle'
   */
  rtb_UnaryMinus_idx_2 = 3.1415926535897931 * Vehicle_U.SteeringAngle / 180.0;
  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* SignalConversion: '<S56>/ConcatBufferAtVector Concatenate1In1' incorporates:
     *  Constant: '<S56>/Constant'
     */
    Vehicle_B.VectorConcatenate1[0] = Vehicle_P.Constant_Value_c;

    /* SignalConversion: '<S56>/ConcatBufferAtVector Concatenate1In2' incorporates:
     *  Constant: '<S56>/Constant'
     */
    Vehicle_B.VectorConcatenate1[1] = Vehicle_P.Constant_Value_c;

    /* SignalConversion: '<S62>/ConcatBufferAtVector Concatenate1In1' */
    Vehicle_B.VectorConcatenate1_o[0] = 0.0;

    /* SignalConversion: '<S62>/ConcatBufferAtVector Concatenate1In2' */
    Vehicle_B.VectorConcatenate1_o[1] = 0.0;

    /* SignalConversion: '<S72>/ConcatBufferAtVector Concatenate1In1' */
    Vehicle_B.VectorConcatenate1_f[0] = 0.0;

    /* SignalConversion: '<S72>/ConcatBufferAtVector Concatenate1In2' */
    Vehicle_B.VectorConcatenate1_f[1] = 0.0;

    /* SignalConversion: '<S88>/ConcatBufferAtVector Concatenate1In1' */
    Vehicle_B.VectorConcatenate1_n[0] = 0.0;

    /* SignalConversion: '<S88>/ConcatBufferAtVector Concatenate1In2' */
    Vehicle_B.VectorConcatenate1_n[1] = 0.0;

    /* SignalConversion: '<S88>/ConcatBufferAtVector Concatenate1In3' */
    Vehicle_B.VectorConcatenate1_n[2] = 0.0;

    /* UnaryMinus: '<S20>/Unary Minus' incorporates:
     *  Constant: '<S20>/Constant4'
     */
    Vehicle_B.UnaryMinus[0] = -Vehicle_P.Constant4_Value[0];
    Vehicle_B.UnaryMinus[1] = -Vehicle_P.Constant4_Value[1];
    Vehicle_B.UnaryMinus[2] = -Vehicle_P.Constant4_Value[2];
  }

  /* Sum: '<S20>/Add1' incorporates:
   *  Inport: '<Root>/Ego Velocity'
   *  Integrator: '<S85>/Integrator'
   *  Product: '<S21>/Product'
   *  Product: '<S21>/Product1'
   *  Product: '<S21>/Product2'
   *  Product: '<S21>/Product3'
   *  Sum: '<S21>/Add'
   *  Sum: '<S21>/Add1'
   */
  rtb_VectorConcatenate1_p_idx_2 = Vehicle_U.EgoVelocity -
    (rtb_VectorConcatenate1_p_idx_0_ * Vehicle_B.VectorConcatenate1_n[0] +
     rtb_UnaryMinus_idx_0_tmp * Vehicle_B.VectorConcatenate1_n[1]);
  rtb_VectorConcatenate1_p_idx_1 = Vehicle_X.Integrator_CSTATE_e[1] -
    (rtb_VectorConcatenate1_p_idx_0_ * Vehicle_B.VectorConcatenate1_n[1] -
     rtb_UnaryMinus_idx_0_tmp * Vehicle_B.VectorConcatenate1_n[0]);

  /* Switch: '<S20>/Switch' incorporates:
   *  Constant: '<S20>/Constant4'
   */
  if (rtb_VectorConcatenate1_p_idx_2 >= Vehicle_P.Switch_Threshold) {
    rtb_UnaryMinus_idx_0 = Vehicle_P.Constant4_Value[0];
  } else {
    rtb_UnaryMinus_idx_0 = Vehicle_B.UnaryMinus[0];
  }

  if (rtb_VectorConcatenate1_p_idx_1 >= Vehicle_P.Switch_Threshold) {
    rtb_UnaryMinus_idx_1 = Vehicle_P.Constant4_Value[1];
  } else {
    rtb_UnaryMinus_idx_1 = Vehicle_B.UnaryMinus[1];
  }

  /* Sqrt: '<S20>/Sqrt' incorporates:
   *  Product: '<S20>/Product'
   *  SignalConversion: '<S21>/ConcatBufferAtVector Concatenate2In3'
   *  Sum: '<S20>/Add1'
   *  Sum: '<S20>/Sum of Elements'
   */
  rtb_Sum_n = sqrt((rtb_VectorConcatenate1_p_idx_2 *
                    rtb_VectorConcatenate1_p_idx_2 +
                    rtb_VectorConcatenate1_p_idx_1 *
                    rtb_VectorConcatenate1_p_idx_1) + (0.0 -
    Vehicle_B.VectorConcatenate1_n[2]) * (0.0 - Vehicle_B.VectorConcatenate1_n[2]));

  /* Product: '<S20>/Product2' */
  rtb_Product_e = rtb_Sum_n * rtb_Sum_n;
  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* Constant: '<S20>/Constant' */
    Vehicle_B.VectorConcatenate_b[0] = Vehicle_P.BicycleModelVelocityInput_Cd;
  }

  /* Trigonometry: '<S20>/Trigonometric Function' */
  rtb_Sum_n = rt_atan2d_snf(rtb_VectorConcatenate1_p_idx_1,
    rtb_VectorConcatenate1_p_idx_2);

  /* Lookup_n-D: '<S20>/Cs' */
  Vehicle_B.VectorConcatenate_b[1] = look1_binlcpw(rtb_Sum_n,
    Vehicle_P.BicycleModelVelocityInput_beta_,
    Vehicle_P.BicycleModelVelocityInput_Cs, 30U);
  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* Constant: '<S20>/Constant1' */
    Vehicle_B.VectorConcatenate_b[2] = Vehicle_P.BicycleModelVelocityInput_Cl;
  }

  /* Lookup_n-D: '<S20>/Crm' */
  Vehicle_B.VectorConcatenate_b[3] = look1_binlxpw(rtb_Sum_n,
    Vehicle_P.Crm_bp01Data, Vehicle_P.Crm_tableData, 1U);

  /* Product: '<S20>/Product5' incorporates:
   *  Constant: '<S20>/Constant2'
   */
  Vehicle_B.VectorConcatenate_b[4] = rtb_UnaryMinus_idx_0 *
    Vehicle_P.BicycleModelVelocityInput_Cpm;

  /* Lookup_n-D: '<S20>/Cym' */
  Vehicle_B.VectorConcatenate_b[5] = look1_binlxpw(rtb_Sum_n,
    Vehicle_P.BicycleModelVelocityInput_beta_,
    Vehicle_P.BicycleModelVelocityInput_Cym, 30U);

  /* Gain: '<S20>/.5.*A.*Pabs.//R.//T' incorporates:
   *  Product: '<S20>/Product1'
   */
  rtb_Sum_n = 0.5 * Vehicle_P.BicycleModelVelocityInput_Af *
    Vehicle_P.BicycleModelVelocityInput_Pabs / Vehicle_P.DragForce_R /
    Vehicle_P.BicycleModelVelocityInput_Tair;
  for (i = 0; i < 6; i++) {
    rtb_uAPabsRT[i] = rtb_Product_e * Vehicle_B.VectorConcatenate_b[i] *
      rtb_Sum_n;
  }

  /* End of Gain: '<S20>/.5.*A.*Pabs.//R.//T' */

  /* Product: '<S20>/Product4' incorporates:
   *  Constant: '<S20>/Constant3'
   *  MATLAB Function: '<S2>/vehicle model'
   */
  rtb_Sum_i_tmp = Vehicle_P.lf + Vehicle_P.lr;

  /* Product: '<S20>/Product3' incorporates:
   *  UnaryMinus: '<S4>/Unary Minus'
   */
  rtb_UnaryMinus_idx_0 = -(rtb_UnaryMinus_idx_0 * rtb_uAPabsRT[0]);

  /* Sum: '<S2>/Add' incorporates:
   *  Constant: '<S20>/Constant3'
   *  Product: '<S20>/Product4'
   *  UnaryMinus: '<S4>/Unary Minus1'
   */
  rtb_VectorConcatenate1_p_idx_1 = -(rtb_uAPabsRT[4] * rtb_Sum_i_tmp);

  /* Product: '<S20>/Product3' incorporates:
   *  UnaryMinus: '<S4>/Unary Minus'
   */
  rtb_UnaryMinus_idx_1 = -(rtb_UnaryMinus_idx_1 * rtb_uAPabsRT[1]);

  /* Switch: '<S20>/Switch' incorporates:
   *  Constant: '<S20>/Constant4'
   *  SignalConversion: '<S21>/ConcatBufferAtVector Concatenate2In3'
   *  Sum: '<S20>/Add1'
   */
  if (0.0 - Vehicle_B.VectorConcatenate1_n[2] >= Vehicle_P.Switch_Threshold) {
    rtb_VectorConcatenate1_p_idx_2 = Vehicle_P.Constant4_Value[2];
  } else {
    rtb_VectorConcatenate1_p_idx_2 = Vehicle_B.UnaryMinus[2];
  }

  /* UnaryMinus: '<S4>/Unary Minus' incorporates:
   *  Product: '<S20>/Product3'
   */
  rtb_UnaryMinus = -(rtb_VectorConcatenate1_p_idx_2 * rtb_uAPabsRT[2]);

  /* Sum: '<S2>/Add' incorporates:
   *  Constant: '<S20>/Constant3'
   *  Product: '<S20>/Product4'
   *  UnaryMinus: '<S4>/Unary Minus1'
   */
  rtb_VectorConcatenate1_p_idx_2 = -(rtb_uAPabsRT[5] * rtb_Sum_i_tmp);

  /* MATLAB Function: '<S2>/vehicle model' incorporates:
   *  Constant: '<S15>/Cyf'
   *  Constant: '<S15>/Cyr'
   *  Inport: '<Root>/Ego Velocity'
   *  Integrator: '<S85>/Integrator'
   *  Sum: '<S20>/Sum of Elements'
   */
  rtb_Sum_n = Vehicle_P.lf * Vehicle_X.Integrator_CSTATE_e[3] +
    Vehicle_X.Integrator_CSTATE_e[1];
  alfa_f = Vehicle_U.EgoVelocity * Vehicle_U.EgoVelocity;
  rtb_Sum_n = sqrt(rtb_Sum_n * rtb_Sum_n + alfa_f);
  rtb_Product_e = Vehicle_X.Integrator_CSTATE_e[1] - Vehicle_P.lr *
    Vehicle_X.Integrator_CSTATE_e[3];
  rtb_Product_e = sqrt(rtb_Product_e * rtb_Product_e + alfa_f);
  alfa_r = fabs(Vehicle_U.EgoVelocity);
  i = 0;
  if (alfa_r < Vehicle_P.BicycleModelVelocityInput_xdot_) {
    i = 1;
  }

  loop_ub = i - 1;
  for (i_0 = 0; i_0 <= loop_ub; i_0++) {
    Fz_idx_0 = alfa_r / Vehicle_P.BicycleModelVelocityInput_xdot_;
  }

  loop_ub = i - 1;
  if (0 <= loop_ub) {
    memcpy(&Fz_idx_1, &Fy_r, (loop_ub + 1) * sizeof(real_T));
  }

  if (1 <= i) {
    Fz_idx_1 = Fz_idx_0 * Fz_idx_0;
  }

  loop_ub = i - 1;
  for (i = 0; i <= loop_ub; i++) {
    Fz_idx_1 = 2.0 * Vehicle_P.BicycleModelVelocityInput_xdot_ / (3.0 - Fz_idx_1);
  }

  Fy_r = alfa_r;
  if (alfa_r < Vehicle_P.BicycleModelVelocityInput_xdot_) {
    Fy_r = Fz_idx_1;
  }

  i = 0;
  if (Vehicle_U.EgoVelocity < 0.0) {
    i = 1;
  }

  loop_ub = i - 1;
  for (i = 0; i <= loop_ub; i++) {
    Fz_idx_0 = -Fy_r;
  }

  if (Vehicle_U.EgoVelocity < 0.0) {
    Fy_r = Fz_idx_0;
  }

  if (alfa_r <= Vehicle_P.BicycleModelVelocityInput_xdot_) {
    alfa_r = tanh(4.0 * fabs(Vehicle_X.Integrator_CSTATE_e[1]));
    alfa_f = (rt_atan2d_snf(Vehicle_P.lf * Vehicle_X.Integrator_CSTATE_e[3] +
               Vehicle_X.Integrator_CSTATE_e[1], Fy_r) - rtb_UnaryMinus_idx_2) *
      alfa_r;
    alfa_r *= rt_atan2d_snf(Vehicle_X.Integrator_CSTATE_e[1] - Vehicle_P.lr *
      Vehicle_X.Integrator_CSTATE_e[3], Fy_r) - tanh(4.0 *
      Vehicle_X.Integrator_CSTATE_e[1]) * 0.0;
  } else {
    alfa_f = rt_atan2d_snf(Vehicle_P.lf * Vehicle_X.Integrator_CSTATE_e[3] +
      Vehicle_X.Integrator_CSTATE_e[1], Fy_r) - rtb_UnaryMinus_idx_2;
    alfa_r = rt_atan2d_snf(Vehicle_X.Integrator_CSTATE_e[1] - Vehicle_P.lr *
      Vehicle_X.Integrator_CSTATE_e[3], Fy_r);
  }

  Fz_idx_0 = 0.0;
  Fz_idx_1 = 0.0;
  for (i = 0; i < 6; i++) {
    if (i == 0) {
      Fy_r = rtb_UnaryMinus_idx_0 * Vehicle_P.BicycleModelVelocityInput_h;
      Fz_idx_0 = (((Vehicle_P.BicycleModelVelocityInput_g * Vehicle_P.lr *
                    Vehicle_P.m + rtb_UnaryMinus * Vehicle_P.lr) + Fy_r) -
                  rtb_VectorConcatenate1_p_idx_1) / rtb_Sum_i_tmp;
      Fz_idx_1 = (((Vehicle_P.BicycleModelVelocityInput_g * Vehicle_P.lf *
                    Vehicle_P.m + rtb_UnaryMinus * Vehicle_P.lf) - Fy_r) +
                  rtb_VectorConcatenate1_p_idx_1) / rtb_Sum_i_tmp;
      if (Fz_idx_0 < 0.0) {
        Fz_idx_0 = 0.0;
      }

      if (Fz_idx_1 < 0.0) {
        Fz_idx_1 = 0.0;
      }
    }

    Fy_f = -Vehicle_P.Cf * alfa_f * Vehicle_B.VectorConcatenate1[0] * Fz_idx_0 /
      Vehicle_P.BicycleModelVelocityInput_Fznom;
    Fy_r = -Vehicle_P.Cr * alfa_r * Vehicle_B.VectorConcatenate1[1] * Fz_idx_1 /
      Vehicle_P.BicycleModelVelocityInput_Fznom;
    Vehicle_automlvehdynftiresat(Fy_f, Vehicle_P.vehiclemodel_Fxtire_sat *
      Fz_idx_0 / Vehicle_P.BicycleModelVelocityInput_Fznom,
      Vehicle_P.vehiclemodel_Fytire_sat * Fz_idx_0 /
      Vehicle_P.BicycleModelVelocityInput_Fznom, &Fx_ft, &yddot);
    Vehicle_automlvehdynftiresat(Fy_r, Vehicle_P.vehiclemodel_Fxtire_sat *
      Fz_idx_1 / Vehicle_P.BicycleModelVelocityInput_Fznom,
      Vehicle_P.vehiclemodel_Fytire_sat * Fz_idx_1 /
      Vehicle_P.BicycleModelVelocityInput_Fznom, &Fx_ft, &Fz_idx_0);
    Fz_idx_1 = sin(rtb_UnaryMinus_idx_2);
    Fx_ft = cos(rtb_UnaryMinus_idx_2);
    Fy_f = -(0.0 * Fx_ft - yddot * Fz_idx_1) * Fz_idx_1 + Fy_f * Fx_ft;
    Fy_r += -(0.0 - Fz_idx_0 * 0.0) * 0.0;
    yddot = ((Fy_f + Fy_r) + rtb_UnaryMinus_idx_1) / Vehicle_P.m +
      -Vehicle_U.EgoVelocity * rtb_VectorConcatenate1_idx_3;
    Fy_f = ((Vehicle_P.lf * Fy_f - Vehicle_P.lr * Fy_r) +
            rtb_VectorConcatenate1_p_idx_2) / Vehicle_P.Iz;
    Fy_r = (0.0 - Vehicle_X.Integrator_CSTATE_e[1] *
            rtb_VectorConcatenate1_idx_3) *
      Vehicle_P.BicycleModelVelocityInput_h;
    Fz_idx_0 = ((((Vehicle_P.BicycleModelVelocityInput_g * Vehicle_P.lr - Fy_r) *
                  Vehicle_P.m + rtb_UnaryMinus * Vehicle_P.lr) +
                 rtb_UnaryMinus_idx_0 * Vehicle_P.BicycleModelVelocityInput_h) -
                rtb_VectorConcatenate1_p_idx_1) / rtb_Sum_i_tmp;
    Fz_idx_1 = ((((Fy_r + Vehicle_P.BicycleModelVelocityInput_g * Vehicle_P.lf) *
                  Vehicle_P.m + rtb_UnaryMinus * Vehicle_P.lf) -
                 rtb_UnaryMinus_idx_0 * Vehicle_P.BicycleModelVelocityInput_h) +
                rtb_VectorConcatenate1_p_idx_1) / rtb_Sum_i_tmp;
    if (Fz_idx_0 < 0.0) {
      Fz_idx_0 = 0.0;
    }

    if (Fz_idx_1 < 0.0) {
      Fz_idx_1 = 0.0;
    }
  }

  Vehicle_B.yOut[0] = Vehicle_U.EgoVelocity;
  Vehicle_B.yOut[1] = Vehicle_X.Integrator_CSTATE_e[1];
  Vehicle_B.yOut[2] = 0.0;
  Vehicle_B.yOut[3] = yddot;
  Vehicle_B.yOut[4] = Vehicle_U.EgoVelocity * rtb_VectorConcatenate1_p_idx_0_ -
    Vehicle_X.Integrator_CSTATE_e[1] * rtb_UnaryMinus_idx_0_tmp;
  Vehicle_B.yOut[5] = Vehicle_U.EgoVelocity * rtb_UnaryMinus_idx_0_tmp +
    Vehicle_X.Integrator_CSTATE_e[1] * rtb_VectorConcatenate1_p_idx_0_;
  Vehicle_B.yOut[6] = Vehicle_X.Integrator_CSTATE_e[2];
  Vehicle_B.yOut[7] = Vehicle_X.Integrator_CSTATE_e[3];
  Vehicle_B.yOut[8] = rtb_Sum_n * cos(alfa_f);
  Vehicle_B.yOut[9] = rtb_Sum_n * sin(alfa_f);
  Vehicle_B.yOut[10] = rtb_Product_e * cos(alfa_r);
  Vehicle_B.yOut[11] = atan(Vehicle_X.Integrator_CSTATE_e[1] /
    Vehicle_U.EgoVelocity);
  Vehicle_B.yOut[12] = rtb_Product_e * sin(alfa_r);
  Vehicle_B.yOut[13] = alfa_f;
  Vehicle_B.yOut[14] = alfa_r;
  Vehicle_B.stateDer[0] = 0.0;
  Vehicle_B.stateDer[1] = yddot;
  Vehicle_B.stateDer[2] = Vehicle_X.Integrator_CSTATE_e[3];
  Vehicle_B.stateDer[3] = Fy_f;
  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* Product: '<S36>/j x k' incorporates:
     *  Constant: '<S28>/Constant5'
     */
    Vehicle_B.jxk = Vehicle_P.Constant5_Value * Vehicle_B.VectorConcatenate_a[2];

    /* Product: '<S36>/i x j' incorporates:
     *  Constant: '<S28>/Constant4'
     */
    Vehicle_B.ixj = Vehicle_P.Constant4_Value_c * Vehicle_B.VectorConcatenate_a
      [1];

    /* Product: '<S37>/i x k' incorporates:
     *  Constant: '<S28>/Constant4'
     */
    Vehicle_B.ixk = Vehicle_P.Constant4_Value_c * Vehicle_B.VectorConcatenate_a
      [2];

    /* Product: '<S37>/j x i' incorporates:
     *  Constant: '<S28>/Constant5'
     */
    Vehicle_B.jxi = Vehicle_P.Constant5_Value * Vehicle_B.VectorConcatenate_a[0];
  }

  /* Product: '<S32>/Product' incorporates:
   *  Integrator: '<S85>/Integrator'
   *  Product: '<S36>/k x i'
   *  Product: '<S37>/k x j'
   *  Sum: '<S34>/Sum'
   */
  rtb_VectorConcatenate1_idx_3 = Vehicle_B.jxk - Vehicle_X.Integrator_CSTATE_e[3]
    * Vehicle_B.VectorConcatenate_a[1];
  rtb_UnaryMinus_idx_0_tmp = Vehicle_X.Integrator_CSTATE_e[3] *
    Vehicle_B.VectorConcatenate_a[0] - Vehicle_B.ixk;
  rtb_VectorConcatenate1_p_idx_0_ = Vehicle_B.ixj - Vehicle_B.jxi;
  for (i = 0; i < 3; i++) {
    rtb_Transpose1_0[i] = rtb_Transpose1[i + 6] *
      rtb_VectorConcatenate1_p_idx_0_ + (rtb_Transpose1[i + 3] *
      rtb_UnaryMinus_idx_0_tmp + rtb_Transpose1[i] *
      rtb_VectorConcatenate1_idx_3);
  }

  /* Outport: '<Root>/XY Velocities' incorporates:
   *  Product: '<S32>/Product'
   *  Sum: '<S29>/Add4'
   */
  Vehicle_Y.XYVelocities[0] = Vehicle_B.yOut[4] + rtb_Transpose1_0[0];
  Vehicle_Y.XYVelocities[1] = Vehicle_B.yOut[5] + rtb_Transpose1_0[1];

  /* Outport: '<Root>/Yaw Angle' incorporates:
   *  Integrator: '<S85>/Integrator'
   *  UnitConversion: '<Root>/Unit Conversion'
   */
  /* Unit Conversion - from: rad to: deg
     Expression: output = (57.2958*input) + (0) */
  Vehicle_Y.YawAngle = 57.295779513082323 * Vehicle_X.Integrator_CSTATE_e[2];

  /* Outport: '<Root>/Yaw Rate' incorporates:
   *  Integrator: '<S85>/Integrator'
   *  UnitConversion: '<Root>/Unit Conversion1'
   */
  /* Unit Conversion - from: rad/s to: deg/s
     Expression: output = (57.2958*input) + (0) */
  Vehicle_Y.YawRate = 57.295779513082323 * Vehicle_X.Integrator_CSTATE_e[3];

  /* Product: '<S82>/Product1' incorporates:
   *  Constant: '<S80>/Constant1'
   *  Integrator: '<S82>/lateral'
   *  MATLAB Function: '<S2>/vehicle model'
   *  Sum: '<S82>/Sum'
   */
  Vehicle_B.Product1 = (alfa_f - Vehicle_X.lateral_CSTATE) * rtb_Sum_n /
    Vehicle_P.VehicleBody3DOFSingleTrack_sigm;

  /* Product: '<S83>/Product1' incorporates:
   *  Constant: '<S80>/Constant2'
   *  Integrator: '<S83>/lateral'
   *  MATLAB Function: '<S2>/vehicle model'
   *  Sum: '<S83>/Sum'
   */
  Vehicle_B.Product1_j = (alfa_r - Vehicle_X.lateral_CSTATE_e) * rtb_Product_e /
    Vehicle_P.VehicleBody3DOFSingleTrack_si_m;
  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(Vehicle_M->rtwLogInfo, (Vehicle_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* Update for Integrator: '<S28>/Integrator' */
    Vehicle_DW.Integrator_IWORK = 0;

    /* Update for Integrator: '<S85>/Integrator' */
    Vehicle_DW.Integrator_IWORK_c = 0;
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Vehicle_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(Vehicle_M)!=-1) &&
          !((rtmGetTFinal(Vehicle_M)-(((Vehicle_M->Timing.clockTick1+
               Vehicle_M->Timing.clockTickH1* 4294967296.0)) * 0.2)) >
            (((Vehicle_M->Timing.clockTick1+Vehicle_M->Timing.clockTickH1*
               4294967296.0)) * 0.2) * (DBL_EPSILON))) {
        rtmSetErrorStatus(Vehicle_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&Vehicle_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++Vehicle_M->Timing.clockTick0)) {
      ++Vehicle_M->Timing.clockTickH0;
    }

    Vehicle_M->Timing.t[0] = rtsiGetSolverStopTime(&Vehicle_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.2s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.2, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      Vehicle_M->Timing.clockTick1++;
      if (!Vehicle_M->Timing.clockTick1) {
        Vehicle_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void Vehicle_derivatives(void)
{
  XDot_Vehicle_T *_rtXdot;
  _rtXdot = ((XDot_Vehicle_T *) Vehicle_M->derivs);

  /* Derivatives for Integrator: '<S28>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] = Vehicle_B.yOut[4];
  _rtXdot->Integrator_CSTATE[1] = Vehicle_B.yOut[5];

  /* Derivatives for Integrator: '<S85>/Integrator' */
  _rtXdot->Integrator_CSTATE_e[0] = Vehicle_B.stateDer[0];
  _rtXdot->Integrator_CSTATE_e[1] = Vehicle_B.stateDer[1];
  _rtXdot->Integrator_CSTATE_e[2] = Vehicle_B.stateDer[2];
  _rtXdot->Integrator_CSTATE_e[3] = Vehicle_B.stateDer[3];

  /* Derivatives for Integrator: '<S82>/lateral' */
  _rtXdot->lateral_CSTATE = Vehicle_B.Product1;

  /* Derivatives for Integrator: '<S83>/lateral' */
  _rtXdot->lateral_CSTATE_e = Vehicle_B.Product1_j;
}

/* Model initialize function */
void Vehicle_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Vehicle_M, 0,
                sizeof(RT_MODEL_Vehicle_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&Vehicle_M->solverInfo, &Vehicle_M->Timing.simTimeStep);
    rtsiSetTPtr(&Vehicle_M->solverInfo, &rtmGetTPtr(Vehicle_M));
    rtsiSetStepSizePtr(&Vehicle_M->solverInfo, &Vehicle_M->Timing.stepSize0);
    rtsiSetdXPtr(&Vehicle_M->solverInfo, &Vehicle_M->derivs);
    rtsiSetContStatesPtr(&Vehicle_M->solverInfo, (real_T **)
                         &Vehicle_M->contStates);
    rtsiSetNumContStatesPtr(&Vehicle_M->solverInfo,
      &Vehicle_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&Vehicle_M->solverInfo,
      &Vehicle_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&Vehicle_M->solverInfo,
      &Vehicle_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&Vehicle_M->solverInfo,
      &Vehicle_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&Vehicle_M->solverInfo, (&rtmGetErrorStatus(Vehicle_M)));
    rtsiSetRTModelPtr(&Vehicle_M->solverInfo, Vehicle_M);
  }

  rtsiSetSimTimeStep(&Vehicle_M->solverInfo, MAJOR_TIME_STEP);
  Vehicle_M->intgData.y = Vehicle_M->odeY;
  Vehicle_M->intgData.f[0] = Vehicle_M->odeF[0];
  Vehicle_M->intgData.f[1] = Vehicle_M->odeF[1];
  Vehicle_M->intgData.f[2] = Vehicle_M->odeF[2];
  Vehicle_M->contStates = ((X_Vehicle_T *) &Vehicle_X);
  rtsiSetSolverData(&Vehicle_M->solverInfo, (void *)&Vehicle_M->intgData);
  rtsiSetSolverName(&Vehicle_M->solverInfo,"ode3");
  rtmSetTPtr(Vehicle_M, &Vehicle_M->Timing.tArray[0]);
  rtmSetTFinal(Vehicle_M, 10.0);
  Vehicle_M->Timing.stepSize0 = 0.2;
  rtmSetFirstInitCond(Vehicle_M, 1);

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    Vehicle_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Vehicle_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Vehicle_M->rtwLogInfo, (NULL));
    rtliSetLogT(Vehicle_M->rtwLogInfo, "");
    rtliSetLogX(Vehicle_M->rtwLogInfo, "");
    rtliSetLogXFinal(Vehicle_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Vehicle_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Vehicle_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(Vehicle_M->rtwLogInfo, 0);
    rtliSetLogDecimation(Vehicle_M->rtwLogInfo, 1);
    rtliSetLogY(Vehicle_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(Vehicle_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(Vehicle_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &Vehicle_B), 0,
                sizeof(B_Vehicle_T));

  /* states (continuous) */
  {
    (void) memset((void *)&Vehicle_X, 0,
                  sizeof(X_Vehicle_T));
  }

  /* states (dwork) */
  (void) memset((void *)&Vehicle_DW, 0,
                sizeof(DW_Vehicle_T));

  /* external inputs */
  (void)memset((void *)&Vehicle_U, 0, sizeof(ExtU_Vehicle_T));

  /* external outputs */
  (void) memset((void *)&Vehicle_Y, 0,
                sizeof(ExtY_Vehicle_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Vehicle_M->rtwLogInfo, 0.0, rtmGetTFinal
    (Vehicle_M), Vehicle_M->Timing.stepSize0, (&rtmGetErrorStatus(Vehicle_M)));

  /* InitializeConditions for Integrator: '<S28>/Integrator' incorporates:
   *  Integrator: '<S85>/Integrator'
   */
  if (rtmIsFirstInitCond(Vehicle_M)) {
    Vehicle_X.Integrator_CSTATE[0] = 0.0;
    Vehicle_X.Integrator_CSTATE[1] = 0.0;
    Vehicle_X.Integrator_CSTATE_e[0] = 0.0;
    Vehicle_X.Integrator_CSTATE_e[1] = 0.0;
    Vehicle_X.Integrator_CSTATE_e[2] = 0.0;
    Vehicle_X.Integrator_CSTATE_e[3] = 0.0;
  }

  Vehicle_DW.Integrator_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<S28>/Integrator' */

  /* InitializeConditions for Integrator: '<S85>/Integrator' */
  Vehicle_DW.Integrator_IWORK_c = 1;

  /* InitializeConditions for Integrator: '<S82>/lateral' */
  Vehicle_X.lateral_CSTATE = Vehicle_P.lateral_IC;

  /* InitializeConditions for Integrator: '<S83>/lateral' */
  Vehicle_X.lateral_CSTATE_e = Vehicle_P.lateral_IC_e;

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(Vehicle_M)) {
    rtmSetFirstInitCond(Vehicle_M, 0);
  }
}

/* Model terminate function */
void Vehicle_terminate(void)
{
  /* (no terminate code required) */
}
