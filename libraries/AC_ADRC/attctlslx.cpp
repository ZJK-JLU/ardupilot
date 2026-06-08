//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: attctlslx.cpp
//
// Code generated for Simulink model 'attctlslx'.
//
// Model version                  : 1.2
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Mon Jun  8 11:46:04 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "attctlslx.h"
#include "rtwtypes.h"

// Model step function
void attctlslx::step(real32_T *arg_atterr, real32_T *arg_rate, real32_T
                     *arg_Out1)
{
  real32_T rtb_FilterCoefficient;
  real32_T rtb_Sum_p;

  // Sum: '<Root>/Sum' incorporates:
  //   Gain: '<Root>/Gain'
  //   Inport: '<Root>/atterr'
  //   Inport: '<Root>/rate'

  rtb_Sum_p = attctlslx_P.Gain_Gain * *arg_atterr - *arg_rate;

  // Gain: '<S36>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S28>/Filter'
  //   Gain: '<S27>/Derivative Gain'
  //   Sum: '<S28>/SumD'

  rtb_FilterCoefficient = (attctlslx_P.DiscretePIDController_D * rtb_Sum_p -
    attctlslx_DW.Filter_DSTATE) * attctlslx_P.DiscretePIDController_N;

  // Outport: '<Root>/Out1' incorporates:
  //   DiscreteIntegrator: '<S33>/Integrator'
  //   Gain: '<S38>/Proportional Gain'
  //   Sum: '<S42>/Sum'

  *arg_Out1 = (attctlslx_P.DiscretePIDController_P * rtb_Sum_p +
               attctlslx_DW.Integrator_DSTATE) + rtb_FilterCoefficient;

  // Update for DiscreteIntegrator: '<S33>/Integrator' incorporates:
  //   Gain: '<S30>/Integral Gain'

  attctlslx_DW.Integrator_DSTATE += attctlslx_P.DiscretePIDController_I *
    rtb_Sum_p * attctlslx_P.Integrator_gainval;

  // Update for DiscreteIntegrator: '<S28>/Filter'
  attctlslx_DW.Filter_DSTATE += attctlslx_P.Filter_gainval *
    rtb_FilterCoefficient;
}

// Model initialize function
void attctlslx::initialize()
{
  // InitializeConditions for DiscreteIntegrator: '<S33>/Integrator'
  attctlslx_DW.Integrator_DSTATE = attctlslx_P.DiscretePIDController_Initial_p;

  // InitializeConditions for DiscreteIntegrator: '<S28>/Filter'
  attctlslx_DW.Filter_DSTATE = attctlslx_P.DiscretePIDController_InitialCo;
}

// Model terminate function
void attctlslx::terminate()
{
  // (no terminate code required)
}

// Constructor
attctlslx::attctlslx() :
  attctlslx_DW(),
  attctlslx_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
attctlslx::~attctlslx() = default;

// Real-Time Model get method
attctlslx::RT_MODEL_attctlslx_T * attctlslx::getRTM()
{
  return (&attctlslx_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
