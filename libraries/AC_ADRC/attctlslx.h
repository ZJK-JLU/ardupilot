//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: attctlslx.h
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
#ifndef RTW_HEADER_attctlslx_h_
#define RTW_HEADER_attctlslx_h_
#include "rtwtypes.h"
#include "attctlslx_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model attctlslx
class attctlslx final
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_attctlslx_T {
    real32_T Integrator_DSTATE;        // '<S33>/Integrator'
    real32_T Filter_DSTATE;            // '<S28>/Filter'
  };

  // Parameters (default storage)
  struct P_attctlslx_T {
    real32_T DiscretePIDController_D; // Mask Parameter: DiscretePIDController_D
                                         //  Referenced by: '<S27>/Derivative Gain'

    real32_T DiscretePIDController_I; // Mask Parameter: DiscretePIDController_I
                                         //  Referenced by: '<S30>/Integral Gain'

    real32_T DiscretePIDController_InitialCo;
                              // Mask Parameter: DiscretePIDController_InitialCo
                                 //  Referenced by: '<S28>/Filter'

    real32_T DiscretePIDController_Initial_p;
                              // Mask Parameter: DiscretePIDController_Initial_p
                                 //  Referenced by: '<S33>/Integrator'

    real32_T DiscretePIDController_N; // Mask Parameter: DiscretePIDController_N
                                         //  Referenced by: '<S36>/Filter Coefficient'

    real32_T DiscretePIDController_P; // Mask Parameter: DiscretePIDController_P
                                         //  Referenced by: '<S38>/Proportional Gain'

    real32_T Gain_Gain;                // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<Root>/Gain'

    real32_T Integrator_gainval;       // Computed Parameter: Integrator_gainval
                                          //  Referenced by: '<S33>/Integrator'

    real32_T Filter_gainval;           // Computed Parameter: Filter_gainval
                                          //  Referenced by: '<S28>/Filter'

  };

  // Real-time Model Data Structure
  struct RT_MODEL_attctlslx_T {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  attctlslx(attctlslx const&) = delete;

  // Assignment Operator
  attctlslx& operator= (attctlslx const&) & = delete;

  // Move Constructor
  attctlslx(attctlslx &&) = delete;

  // Move Assignment Operator
  attctlslx& operator= (attctlslx &&) = delete;

  // Real-Time Model get method
  attctlslx::RT_MODEL_attctlslx_T * getRTM();

  // Tunable parameters
  static P_attctlslx_T attctlslx_P;

  // model initialize function
  void initialize();

  // model step function
  void step(real32_T *arg_atterr, real32_T *arg_rate, real32_T *arg_Out1);

  // model terminate function
  static void terminate();

  // Constructor
  attctlslx();

  // Destructor
  ~attctlslx();

  // private data and function members
 private:
  // Block states
  DW_attctlslx_T attctlslx_DW;

  // Real-Time Model
  RT_MODEL_attctlslx_T attctlslx_M;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'attctlslx'
//  '<S1>'   : 'attctlslx/Discrete PID Controller'
//  '<S2>'   : 'attctlslx/Discrete PID Controller/Anti-windup'
//  '<S3>'   : 'attctlslx/Discrete PID Controller/D Gain'
//  '<S4>'   : 'attctlslx/Discrete PID Controller/Filter'
//  '<S5>'   : 'attctlslx/Discrete PID Controller/Filter ICs'
//  '<S6>'   : 'attctlslx/Discrete PID Controller/I Gain'
//  '<S7>'   : 'attctlslx/Discrete PID Controller/Ideal P Gain'
//  '<S8>'   : 'attctlslx/Discrete PID Controller/Ideal P Gain Fdbk'
//  '<S9>'   : 'attctlslx/Discrete PID Controller/Integrator'
//  '<S10>'  : 'attctlslx/Discrete PID Controller/Integrator ICs'
//  '<S11>'  : 'attctlslx/Discrete PID Controller/N Copy'
//  '<S12>'  : 'attctlslx/Discrete PID Controller/N Gain'
//  '<S13>'  : 'attctlslx/Discrete PID Controller/P Copy'
//  '<S14>'  : 'attctlslx/Discrete PID Controller/Parallel P Gain'
//  '<S15>'  : 'attctlslx/Discrete PID Controller/Reset Signal'
//  '<S16>'  : 'attctlslx/Discrete PID Controller/Saturation'
//  '<S17>'  : 'attctlslx/Discrete PID Controller/Saturation Fdbk'
//  '<S18>'  : 'attctlslx/Discrete PID Controller/Sum'
//  '<S19>'  : 'attctlslx/Discrete PID Controller/Sum Fdbk'
//  '<S20>'  : 'attctlslx/Discrete PID Controller/Tracking Mode'
//  '<S21>'  : 'attctlslx/Discrete PID Controller/Tracking Mode Sum'
//  '<S22>'  : 'attctlslx/Discrete PID Controller/Tsamp - Integral'
//  '<S23>'  : 'attctlslx/Discrete PID Controller/Tsamp - Ngain'
//  '<S24>'  : 'attctlslx/Discrete PID Controller/postSat Signal'
//  '<S25>'  : 'attctlslx/Discrete PID Controller/preSat Signal'
//  '<S26>'  : 'attctlslx/Discrete PID Controller/Anti-windup/Passthrough'
//  '<S27>'  : 'attctlslx/Discrete PID Controller/D Gain/Internal Parameters'
//  '<S28>'  : 'attctlslx/Discrete PID Controller/Filter/Disc. Forward Euler Filter'
//  '<S29>'  : 'attctlslx/Discrete PID Controller/Filter ICs/Internal IC - Filter'
//  '<S30>'  : 'attctlslx/Discrete PID Controller/I Gain/Internal Parameters'
//  '<S31>'  : 'attctlslx/Discrete PID Controller/Ideal P Gain/Passthrough'
//  '<S32>'  : 'attctlslx/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
//  '<S33>'  : 'attctlslx/Discrete PID Controller/Integrator/Discrete'
//  '<S34>'  : 'attctlslx/Discrete PID Controller/Integrator ICs/Internal IC'
//  '<S35>'  : 'attctlslx/Discrete PID Controller/N Copy/Disabled'
//  '<S36>'  : 'attctlslx/Discrete PID Controller/N Gain/Internal Parameters'
//  '<S37>'  : 'attctlslx/Discrete PID Controller/P Copy/Disabled'
//  '<S38>'  : 'attctlslx/Discrete PID Controller/Parallel P Gain/Internal Parameters'
//  '<S39>'  : 'attctlslx/Discrete PID Controller/Reset Signal/Disabled'
//  '<S40>'  : 'attctlslx/Discrete PID Controller/Saturation/Passthrough'
//  '<S41>'  : 'attctlslx/Discrete PID Controller/Saturation Fdbk/Disabled'
//  '<S42>'  : 'attctlslx/Discrete PID Controller/Sum/Sum_PID'
//  '<S43>'  : 'attctlslx/Discrete PID Controller/Sum Fdbk/Disabled'
//  '<S44>'  : 'attctlslx/Discrete PID Controller/Tracking Mode/Disabled'
//  '<S45>'  : 'attctlslx/Discrete PID Controller/Tracking Mode Sum/Passthrough'
//  '<S46>'  : 'attctlslx/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
//  '<S47>'  : 'attctlslx/Discrete PID Controller/Tsamp - Ngain/Passthrough'
//  '<S48>'  : 'attctlslx/Discrete PID Controller/postSat Signal/Forward_Path'
//  '<S49>'  : 'attctlslx/Discrete PID Controller/preSat Signal/Forward_Path'

#endif                                 // RTW_HEADER_attctlslx_h_

//
// File trailer for generated code.
//
// [EOF]
//
