//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: heave_pid_rt.h
//
// Code generated for Simulink model 'heave_pid_rt'.
//
// Model version                  : 1.7
// Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
// C/C++ source code generated on : Tue Dec 23 16:25:07 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef heave_pid_rt_h_
#define heave_pid_rt_h_
#include <cmath>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include <cstring>
#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
struct ODE3_IntgData {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
};

#endif

extern "C"
{
  static real_T rtGetNaN(void);
  static real32_T rtGetNaNF(void);
}                                      // extern "C"

extern "C"
{
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  static boolean_T rtIsInf(real_T value);
  static boolean_T rtIsInfF(real32_T value);
  static boolean_T rtIsNaN(real_T value);
  static boolean_T rtIsNaNF(real32_T value);
}                                      // extern "C"

// Class declaration for model heave_pid_rt
class heave_pid_rt final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW {
    real_T FilterCoefficient;          // '<S42>/Filter Coefficient'
    real_T Switch;                     // '<S29>/Switch'
    boolean_T AND3;                    // '<S29>/AND3'
    boolean_T Memory;                  // '<S29>/Memory'
    boolean_T Memory_PreviousInput;    // '<S29>/Memory'
  };

  // Continuous states (default storage)
  struct X {
    real_T Integrator_CSTATE;          // '<S39>/Integrator'
    real_T Filter_CSTATE;              // '<S34>/Filter'
  };

  // State derivatives (default storage)
  struct XDot {
    real_T Integrator_CSTATE;          // '<S39>/Integrator'
    real_T Filter_CSTATE;              // '<S34>/Filter'
  };

  // State disabled
  struct XDis {
    boolean_T Integrator_CSTATE;       // '<S39>/Integrator'
    boolean_T Filter_CSTATE;           // '<S34>/Filter'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU {
    real_T depth_measured;             // '<Root>/depth_measured'
    real_T depth_goal;                 // '<Root>/depth_goal'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY {
    real_T FL;                         // '<Root>/FL'
    real_T FR;                         // '<Root>/FR'
    real_T RL;                         // '<Root>/RL'
    real_T RR;                         // '<Root>/RR'
  };

  // Real-time Model Data Structure
  using odeFSubArray = real_T[2];
  struct RT_MODEL {
    const char_T *errorStatus;
    RTWSolverInfo solverInfo;
    X *contStates;
    int_T *periodicContStateIndices;
    real_T *periodicContStateRanges;
    real_T *derivs;
    XDis *contStateDisabled;
    boolean_T zCCacheNeedsReset;
    boolean_T derivCacheNeedsReset;
    boolean_T CTOutputIncnstWithState;
    real_T odeY[2];
    real_T odeF[3][2];
    ODE3_IntgData intgData;

    //
    //  Sizes:
    //  The following substructure contains sizes information
    //  for many of the model attributes such as inputs, outputs,
    //  dwork, sample times, etc.

    struct {
      int_T numContStates;
      int_T numPeriodicContStates;
      int_T numSampTimes;
    } Sizes;

    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      uint32_T clockTick0;
      time_T stepSize0;
      uint32_T clockTick1;
      time_T tStart;
      SimTimeStep simTimeStep;
      boolean_T stopRequestedFlag;
      time_T *t;
      time_T tArray[2];
    } Timing;

    time_T** getTPtrPtr();
    boolean_T getStopRequested() const;
    void setStopRequested(boolean_T aStopRequested);
    const char_T* getErrorStatus() const;
    void setErrorStatus(const char_T* const aErrorStatus);
    time_T* getTPtr() const;
    void setTPtr(time_T* aTPtr);
    boolean_T* getStopRequestedPtr();
    const char_T** getErrorStatusPtr();
    boolean_T isMajorTimeStep() const;
    boolean_T isMinorTimeStep() const;
    time_T getTStart() const;
  };

  // Copy Constructor
  heave_pid_rt(heave_pid_rt const&) = delete;

  // Assignment Operator
  heave_pid_rt& operator= (heave_pid_rt const&) & = delete;

  // Move Constructor
  heave_pid_rt(heave_pid_rt &&) = delete;

  // Move Assignment Operator
  heave_pid_rt& operator= (heave_pid_rt &&) = delete;

  // Real-Time Model get method
  heave_pid_rt::RT_MODEL * getRTM();

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  heave_pid_rt();

  // Destructor
  ~heave_pid_rt();

  // private data and function members
 private:
  // Block states
  DW rtDW;

  // Block continuous states
  X rtX;

  // Block Continuous state disabled vector
  XDis rtXDis;

  // Global mass matrix

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void heave_pid_rt_derivatives();

  // Real-Time Model
  RT_MODEL rtM;
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
//  '<Root>' : 'heave_pid_rt'
//  '<S1>'   : 'heave_pid_rt/PID Controller'
//  '<S2>'   : 'heave_pid_rt/Thruster Allocation'
//  '<S3>'   : 'heave_pid_rt/PID Controller/Anti-windup'
//  '<S4>'   : 'heave_pid_rt/PID Controller/D Gain'
//  '<S5>'   : 'heave_pid_rt/PID Controller/External Derivative'
//  '<S6>'   : 'heave_pid_rt/PID Controller/Filter'
//  '<S7>'   : 'heave_pid_rt/PID Controller/Filter ICs'
//  '<S8>'   : 'heave_pid_rt/PID Controller/I Gain'
//  '<S9>'   : 'heave_pid_rt/PID Controller/Ideal P Gain'
//  '<S10>'  : 'heave_pid_rt/PID Controller/Ideal P Gain Fdbk'
//  '<S11>'  : 'heave_pid_rt/PID Controller/Integrator'
//  '<S12>'  : 'heave_pid_rt/PID Controller/Integrator ICs'
//  '<S13>'  : 'heave_pid_rt/PID Controller/N Copy'
//  '<S14>'  : 'heave_pid_rt/PID Controller/N Gain'
//  '<S15>'  : 'heave_pid_rt/PID Controller/P Copy'
//  '<S16>'  : 'heave_pid_rt/PID Controller/Parallel P Gain'
//  '<S17>'  : 'heave_pid_rt/PID Controller/Reset Signal'
//  '<S18>'  : 'heave_pid_rt/PID Controller/Saturation'
//  '<S19>'  : 'heave_pid_rt/PID Controller/Saturation Fdbk'
//  '<S20>'  : 'heave_pid_rt/PID Controller/Sum'
//  '<S21>'  : 'heave_pid_rt/PID Controller/Sum Fdbk'
//  '<S22>'  : 'heave_pid_rt/PID Controller/Tracking Mode'
//  '<S23>'  : 'heave_pid_rt/PID Controller/Tracking Mode Sum'
//  '<S24>'  : 'heave_pid_rt/PID Controller/Tsamp - Integral'
//  '<S25>'  : 'heave_pid_rt/PID Controller/Tsamp - Ngain'
//  '<S26>'  : 'heave_pid_rt/PID Controller/postSat Signal'
//  '<S27>'  : 'heave_pid_rt/PID Controller/preInt Signal'
//  '<S28>'  : 'heave_pid_rt/PID Controller/preSat Signal'
//  '<S29>'  : 'heave_pid_rt/PID Controller/Anti-windup/Cont. Clamping Parallel'
//  '<S30>'  : 'heave_pid_rt/PID Controller/Anti-windup/Cont. Clamping Parallel/Dead Zone'
//  '<S31>'  : 'heave_pid_rt/PID Controller/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
//  '<S32>'  : 'heave_pid_rt/PID Controller/D Gain/Internal Parameters'
//  '<S33>'  : 'heave_pid_rt/PID Controller/External Derivative/Error'
//  '<S34>'  : 'heave_pid_rt/PID Controller/Filter/Cont. Filter'
//  '<S35>'  : 'heave_pid_rt/PID Controller/Filter ICs/Internal IC - Filter'
//  '<S36>'  : 'heave_pid_rt/PID Controller/I Gain/Internal Parameters'
//  '<S37>'  : 'heave_pid_rt/PID Controller/Ideal P Gain/Passthrough'
//  '<S38>'  : 'heave_pid_rt/PID Controller/Ideal P Gain Fdbk/Disabled'
//  '<S39>'  : 'heave_pid_rt/PID Controller/Integrator/Continuous'
//  '<S40>'  : 'heave_pid_rt/PID Controller/Integrator ICs/Internal IC'
//  '<S41>'  : 'heave_pid_rt/PID Controller/N Copy/Disabled'
//  '<S42>'  : 'heave_pid_rt/PID Controller/N Gain/Internal Parameters'
//  '<S43>'  : 'heave_pid_rt/PID Controller/P Copy/Disabled'
//  '<S44>'  : 'heave_pid_rt/PID Controller/Parallel P Gain/Internal Parameters'
//  '<S45>'  : 'heave_pid_rt/PID Controller/Reset Signal/Disabled'
//  '<S46>'  : 'heave_pid_rt/PID Controller/Saturation/Enabled'
//  '<S47>'  : 'heave_pid_rt/PID Controller/Saturation Fdbk/Disabled'
//  '<S48>'  : 'heave_pid_rt/PID Controller/Sum/Sum_PID'
//  '<S49>'  : 'heave_pid_rt/PID Controller/Sum Fdbk/Disabled'
//  '<S50>'  : 'heave_pid_rt/PID Controller/Tracking Mode/Disabled'
//  '<S51>'  : 'heave_pid_rt/PID Controller/Tracking Mode Sum/Passthrough'
//  '<S52>'  : 'heave_pid_rt/PID Controller/Tsamp - Integral/TsSignalSpecification'
//  '<S53>'  : 'heave_pid_rt/PID Controller/Tsamp - Ngain/Passthrough'
//  '<S54>'  : 'heave_pid_rt/PID Controller/postSat Signal/Forward_Path'
//  '<S55>'  : 'heave_pid_rt/PID Controller/preInt Signal/Internal PreInt'
//  '<S56>'  : 'heave_pid_rt/PID Controller/preSat Signal/Forward_Path'

#endif                                 // heave_pid_rt_h_

//
// File trailer for generated code.
//
// [EOF]
//