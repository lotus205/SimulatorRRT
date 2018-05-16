/*
 * Vehicle_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Vehicle".
 *
 * Model version              : 1.12
 * Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
 * C source code generated on : Thu May 17 00:55:31 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Vehicle.h"
#include "Vehicle_private.h"

/* Block parameters (default storage) */
P_Vehicle_T Vehicle_P = {
  /* Variable: Cf
   * Referenced by: '<S15>/Cyf'
   */
  19000.0,

  /* Variable: Cr
   * Referenced by: '<S15>/Cyr'
   */
  33000.0,

  /* Variable: Iz
   * Referenced by: '<S2>/vehicle model'
   */
  2875.0,

  /* Variable: lf
   * Referenced by:
   *   '<S2>/vehicle model'
   *   '<S20>/Constant3'
   *   '<S29>/R_T1'
   */
  1.2,

  /* Variable: lr
   * Referenced by:
   *   '<S2>/vehicle model'
   *   '<S20>/Constant3'
   *   '<S30>/R_T1'
   */
  1.6,

  /* Variable: m
   * Referenced by: '<S2>/vehicle model'
   */
  1575.0,

  /* Variable: x0_ego
   * Referenced by: '<S28>/X_o'
   */
  0.0,

  /* Variable: y0_ego
   * Referenced by: '<S28>/Y_o'
   */
  0.0,

  /* Variable: yaw0_ego
   * Referenced by: '<S86>/psi_o'
   */
  0.0,

  /* Mask Parameter: BicycleModelVelocityInput_Af
   * Referenced by: '<S20>/.5.*A.*Pabs.//R.//T'
   */
  2.0,

  /* Mask Parameter: BicycleModelVelocityInput_Cd
   * Referenced by: '<S20>/Constant'
   */
  0.3,

  /* Mask Parameter: BicycleModelVelocityInput_Cl
   * Referenced by: '<S20>/Constant1'
   */
  0.1,

  /* Mask Parameter: BicycleModelVelocityInput_Cpm
   * Referenced by: '<S20>/Constant2'
   */
  0.1,

  /* Mask Parameter: BicycleModelVelocityInput_Cs
   * Referenced by: '<S20>/Cs'
   */
  { 0.0, 0.03, 0.06, 0.09, 0.12, 0.15, 0.18, 0.21, 0.24, 0.27, 0.3,
    0.32999999999999996, 0.36, 0.39, 0.42, 0.45, 0.48000000000000004, 0.51, 0.54,
    0.57000000000000006, 0.60000000000000009, 0.63, 0.66, 0.69000000000000006,
    0.72, 0.75, 0.78, 0.81, 0.84000000000000008, 0.87, 0.9 },

  /* Mask Parameter: BicycleModelVelocityInput_Cym
   * Referenced by: '<S20>/Cym'
   */
  { 0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12,
    0.13, 0.14, 0.15, 0.15999999999999998, 0.16999999999999998, 0.18, 0.19,
    0.19999999999999998, 0.21, 0.21999999999999997, 0.22999999999999998, 0.24,
    0.25, 0.26, 0.27, 0.27999999999999997, 0.29, 0.3 },

  /* Mask Parameter: BicycleModelVelocityInput_Fznom
   * Referenced by: '<S2>/vehicle model'
   */
  5000.0,

  /* Mask Parameter: BicycleModelVelocityInput_NF
   * Referenced by: '<S2>/vehicle model'
   */
  2.0,

  /* Mask Parameter: BicycleModelVelocityInput_NR
   * Referenced by: '<S2>/vehicle model'
   */
  2.0,

  /* Mask Parameter: BicycleModelVelocityInput_Pabs
   * Referenced by: '<S20>/.5.*A.*Pabs.//R.//T'
   */
  101325.0,

  /* Mask Parameter: DragForce_R
   * Referenced by: '<S20>/.5.*A.*Pabs.//R.//T'
   */
  287.058,

  /* Mask Parameter: HardPointCoordinateTransformFro
   * Referenced by: '<S29>/R_T2'
   */
  0.0,

  /* Mask Parameter: HardPointCoordinateTransformRea
   * Referenced by: '<S30>/R_T2'
   */
  0.0,

  /* Mask Parameter: BicycleModelVelocityInput_Tair
   * Referenced by: '<S20>/.5.*A.*Pabs.//R.//T'
   */
  273.0,

  /* Mask Parameter: BicycleModelVelocityInput_beta_
   * Referenced by:
   *   '<S20>/Cs'
   *   '<S20>/Cym'
   */
  { 0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12,
    0.13, 0.14, 0.15, 0.15999999999999998, 0.16999999999999998, 0.18, 0.19,
    0.19999999999999998, 0.21, 0.21999999999999997, 0.22999999999999998, 0.24,
    0.25, 0.26, 0.27, 0.27999999999999997, 0.29, 0.3 },

  /* Mask Parameter: BicycleModelVelocityInput_g
   * Referenced by: '<S2>/vehicle model'
   */
  9.81,

  /* Mask Parameter: BicycleModelVelocityInput_h
   * Referenced by:
   *   '<S2>/vehicle model'
   *   '<S29>/R_T3'
   *   '<S30>/R_T3'
   */
  0.35,

  /* Mask Parameter: BicycleModelVelocityInput_r_o
   * Referenced by: '<S86>/r_o'
   */
  0.0,

  /* Mask Parameter: VehicleBody3DOFSingleTrack_sigm
   * Referenced by: '<S80>/Constant1'
   */
  0.1,

  /* Mask Parameter: VehicleBody3DOFSingleTrack_si_m
   * Referenced by: '<S80>/Constant2'
   */
  0.1,

  /* Mask Parameter: BicycleModelVelocityInput_xdot_
   * Referenced by: '<S86>/xdot_o'
   */
  0.0,

  /* Mask Parameter: BicycleModelVelocityInput_xdo_c
   * Referenced by: '<S2>/vehicle model'
   */
  0.01,

  /* Mask Parameter: BicycleModelVelocityInput_ydot_
   * Referenced by: '<S86>/ydot_o'
   */
  0.0,

  /* Expression: Fxtire_sat
   * Referenced by: '<S2>/vehicle model'
   */
  1.0E+6,

  /* Expression: Fytire_sat
   * Referenced by: '<S2>/vehicle model'
   */
  1.0E+6,

  /* Expression: w
   * Referenced by: '<S2>/vehicle model'
   */
  { 1.4, 1.4 },

  /* Expression: mu
   * Referenced by: '<S56>/Constant'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S1>/Constant'
   */
  0.0,

  /* Expression: ones(1,3)
   * Referenced by: '<S20>/Constant4'
   */
  { 1.0, 1.0, 1.0 },

  /* Expression: 0
   * Referenced by: '<S20>/Switch'
   */
  0.0,

  /* Expression: [0 0]
   * Referenced by: '<S20>/Crm'
   */
  { 0.0, 0.0 },

  /* Expression: [-1 1]
   * Referenced by: '<S20>/Crm'
   */
  { -1.0, 1.0 },

  /* Expression: 0
   * Referenced by: '<S28>/Constant8'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S28>/Constant7'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S28>/Constant2'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S28>/Constant5'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S28>/Constant4'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S28>/Constant6'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S82>/lateral'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S83>/lateral'
   */
  0.0
};
