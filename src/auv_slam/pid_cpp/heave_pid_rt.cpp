//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: heave_pid_rt.cpp
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
#include "heave_pid_rt.h"
#include <cmath>
#include "rtwtypes.h"
#include "cmath"
#include "limits"

// private model entry point functions
extern void heave_pid_rt_derivatives();
extern "C"
{
  real_T rtNaN { -std::numeric_limits<real_T>::quiet_NaN() };

  real_T rtInf { std::numeric_limits<real_T>::infinity() };

  real_T rtMinusInf { -std::numeric_limits<real_T>::infinity() };

  real32_T rtNaNF { -std::numeric_limits<real32_T>::quiet_NaN() };

  real32_T rtInfF { std::numeric_limits<real32_T>::infinity() };

  real32_T rtMinusInfF { -std::numeric_limits<real32_T>::infinity() };
}

extern "C"
{
  // Return rtNaN needed by the generated code.
  static real_T rtGetNaN(void)
  {
    return rtNaN;
  }

  // Return rtNaNF needed by the generated code.
  static real32_T rtGetNaNF(void)
  {
    return rtNaNF;
  }
}

extern "C"
{
  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return std::isinf(value);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return std::isinf(value);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    return std::isnan(value);
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    return std::isnan(value);
  }
}

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
void heave_pid_rt::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE3_A[3]{
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3]{
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t { rtsiGetT(si) };

  time_T tnew { rtsiGetSolverStopTime(si) };

  time_T h { rtsiGetStepSize(si) };

  real_T *x { rtsiGetContStates(si) };

  ODE3_IntgData *id { static_cast<ODE3_IntgData *>(rtsiGetSolverData(si)) };

  real_T *y { id->y };

  real_T *f0 { id->f[0] };

  real_T *f1 { id->f[1] };

  real_T *f2 { id->f[2] };

  real_T hB[3];
  int_T i;
  int_T nXc { 2 };

  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) std::memcpy(y, x,
                     static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  heave_pid_rt_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  heave_pid_rt_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  heave_pid_rt_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

// Model step function
void heave_pid_rt::step()
{
  real_T rtb_IntegralGain;
  real_T rtb_SignPreIntegrator;
  real_T rtb_SignPreSat;
  real_T tmp;
  real_T tmp_0;
  if ((&rtM)->isMajorTimeStep()) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&rtM)->solverInfo,(((&rtM)->Timing.clockTick0+1)*
      (&rtM)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if ((&rtM)->isMinorTimeStep()) {
    (&rtM)->Timing.t[0] = rtsiGetT(&(&rtM)->solverInfo);
  }

  // Sum: '<Root>/Add' incorporates:
  //   Inport: '<Root>/depth_goal'
  //   Inport: '<Root>/depth_measured'

  rtb_IntegralGain = rtU.depth_goal - rtU.depth_measured;

  // Gain: '<S42>/Filter Coefficient' incorporates:
  //   Gain: '<S32>/Derivative Gain'
  //   Integrator: '<S34>/Filter'
  //   Sum: '<S34>/SumD'

  rtDW.FilterCoefficient = (2522.24845812222 * rtb_IntegralGain -
    rtX.Filter_CSTATE) * 153.359527842239;

  // Sum: '<S48>/Sum' incorporates:
  //   Gain: '<S44>/Proportional Gain'
  //   Integrator: '<S39>/Integrator'

  rtb_SignPreSat = (9793.05017402754 * rtb_IntegralGain + rtX.Integrator_CSTATE)
    + rtDW.FilterCoefficient;

  // Saturate: '<S46>/Saturation'
  if (rtb_SignPreSat > 100.0) {
    tmp = 100.0;
  } else if (rtb_SignPreSat < -100.0) {
    tmp = -100.0;
  } else {
    tmp = rtb_SignPreSat;
  }

  // Gain: '<S2>/Gain' incorporates:
  //   Saturate: '<S46>/Saturation'

  rtb_SignPreIntegrator = 0.25 * tmp;

  // Outport: '<Root>/FL'
  rtY.FL = rtb_SignPreIntegrator;

  // Outport: '<Root>/FR'
  rtY.FR = rtb_SignPreIntegrator;

  // Outport: '<Root>/RL'
  rtY.RL = rtb_SignPreIntegrator;

  // Outport: '<Root>/RR'
  rtY.RR = rtb_SignPreIntegrator;

  // Gain: '<S29>/ZeroGain'
  rtb_SignPreIntegrator = 0.0 * rtb_SignPreSat;

  // DeadZone: '<S31>/DeadZone'
  if (rtb_SignPreSat > 100.0) {
    rtb_SignPreSat -= 100.0;
  } else if (rtb_SignPreSat >= -100.0) {
    rtb_SignPreSat = 0.0;
  } else {
    rtb_SignPreSat -= -100.0;
  }

  // End of DeadZone: '<S31>/DeadZone'

  // Gain: '<S36>/Integral Gain'
  rtb_IntegralGain *= 7804.85614592368;

  // Signum: '<S29>/SignPreSat'
  if (std::isnan(rtb_SignPreSat)) {
    tmp = (rtNaN);
  } else if (rtb_SignPreSat < 0.0) {
    tmp = -1.0;
  } else {
    tmp = (rtb_SignPreSat > 0.0);
  }

  // Signum: '<S29>/SignPreIntegrator'
  if (std::isnan(rtb_IntegralGain)) {
    tmp_0 = (rtNaN);
  } else if (rtb_IntegralGain < 0.0) {
    tmp_0 = -1.0;
  } else {
    tmp_0 = (rtb_IntegralGain > 0.0);
  }

  // Logic: '<S29>/AND3' incorporates:
  //   DataTypeConversion: '<S29>/DataTypeConv1'
  //   DataTypeConversion: '<S29>/DataTypeConv2'
  //   RelationalOperator: '<S29>/Equal1'
  //   RelationalOperator: '<S29>/NotEqual'
  //   Signum: '<S29>/SignPreIntegrator'
  //   Signum: '<S29>/SignPreSat'

  rtDW.AND3 = ((rtb_SignPreIntegrator != rtb_SignPreSat) && (static_cast<int8_T>
    (tmp) == static_cast<int8_T>(tmp_0)));
  if ((&rtM)->isMajorTimeStep()) {
    // Memory: '<S29>/Memory'
    rtDW.Memory = rtDW.Memory_PreviousInput;
  }

  // Switch: '<S29>/Switch'
  if (rtDW.Memory) {
    // Switch: '<S29>/Switch' incorporates:
    //   Constant: '<S29>/Constant1'

    rtDW.Switch = 0.0;
  } else {
    // Switch: '<S29>/Switch'
    rtDW.Switch = rtb_IntegralGain;
  }

  // End of Switch: '<S29>/Switch'
  if ((&rtM)->isMajorTimeStep()) {
    if ((&rtM)->isMajorTimeStep()) {
      // Update for Memory: '<S29>/Memory'
      rtDW.Memory_PreviousInput = rtDW.AND3;
    }
  }                                    // end MajorTimeStep

  if ((&rtM)->isMajorTimeStep()) {
    rt_ertODEUpdateContinuousStates(&(&rtM)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&rtM)->Timing.clockTick0;
    (&rtM)->Timing.t[0] = rtsiGetSolverStopTime(&(&rtM)->solverInfo);

    {
      // Update absolute timer for sample time: [0.2s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.2, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&rtM)->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void heave_pid_rt::heave_pid_rt_derivatives()
{
  heave_pid_rt::XDot *_rtXdot;
  _rtXdot = ((XDot *) (&rtM)->derivs);

  // Derivatives for Integrator: '<S39>/Integrator'
  _rtXdot->Integrator_CSTATE = rtDW.Switch;

  // Derivatives for Integrator: '<S34>/Filter'
  _rtXdot->Filter_CSTATE = rtDW.FilterCoefficient;
}

// Model initialize function
void heave_pid_rt::initialize()
{
  // Registration code
  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&rtM)->solverInfo, &(&rtM)->Timing.simTimeStep);
    rtsiSetTPtr(&(&rtM)->solverInfo, (&rtM)->getTPtrPtr());
    rtsiSetStepSizePtr(&(&rtM)->solverInfo, &(&rtM)->Timing.stepSize0);
    rtsiSetdXPtr(&(&rtM)->solverInfo, &(&rtM)->derivs);
    rtsiSetContStatesPtr(&(&rtM)->solverInfo, (real_T **) &(&rtM)->contStates);
    rtsiSetNumContStatesPtr(&(&rtM)->solverInfo, &(&rtM)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&(&rtM)->solverInfo, (boolean_T**) &(&rtM)
      ->contStateDisabled);
    rtsiSetErrorStatusPtr(&(&rtM)->solverInfo, (&rtM)->getErrorStatusPtr());
    rtsiSetRTModelPtr(&(&rtM)->solverInfo, (&rtM));
  }

  rtsiSetSimTimeStep(&(&rtM)->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&(&rtM)->solverInfo, false);
  rtsiSetIsContModeFrozen(&(&rtM)->solverInfo, false);
  (&rtM)->intgData.y = (&rtM)->odeY;
  (&rtM)->intgData.f[0] = (&rtM)->odeF[0];
  (&rtM)->intgData.f[1] = (&rtM)->odeF[1];
  (&rtM)->intgData.f[2] = (&rtM)->odeF[2];
  (&rtM)->contStates = ((X *) &rtX);
  (&rtM)->contStateDisabled = ((XDis *) &rtXDis);
  (&rtM)->Timing.tStart = (0.0);
  rtsiSetSolverData(&(&rtM)->solverInfo, static_cast<void *>(&(&rtM)->intgData));
  rtsiSetSolverName(&(&rtM)->solverInfo,"ode3");
  (&rtM)->setTPtr(&(&rtM)->Timing.tArray[0]);
  (&rtM)->Timing.stepSize0 = 0.2;

  // InitializeConditions for Integrator: '<S39>/Integrator'
  rtX.Integrator_CSTATE = 0.0;

  // InitializeConditions for Integrator: '<S34>/Filter'
  rtX.Filter_CSTATE = 0.0;
}

time_T** heave_pid_rt::RT_MODEL::getTPtrPtr()
{
  return &(Timing.t);
}

boolean_T heave_pid_rt::RT_MODEL::getStopRequested() const
{
  return (Timing.stopRequestedFlag);
}

void heave_pid_rt::RT_MODEL::setStopRequested(boolean_T aStopRequested)
{
  (Timing.stopRequestedFlag = aStopRequested);
}

const char_T* heave_pid_rt::RT_MODEL::getErrorStatus() const
{
  return (errorStatus);
}

void heave_pid_rt::RT_MODEL::setErrorStatus(const char_T* const aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

time_T* heave_pid_rt::RT_MODEL::getTPtr() const
{
  return (Timing.t);
}

void heave_pid_rt::RT_MODEL::setTPtr(time_T* aTPtr)
{
  (Timing.t = aTPtr);
}

boolean_T* heave_pid_rt::RT_MODEL::getStopRequestedPtr()
{
  return (&(Timing.stopRequestedFlag));
}

const char_T** heave_pid_rt::RT_MODEL::getErrorStatusPtr()
{
  return &errorStatus;
}

boolean_T heave_pid_rt::RT_MODEL::isMajorTimeStep() const
{
  return ((Timing.simTimeStep) == MAJOR_TIME_STEP);
}

boolean_T heave_pid_rt::RT_MODEL::isMinorTimeStep() const
{
  return ((Timing.simTimeStep) == MINOR_TIME_STEP);
}

time_T heave_pid_rt::RT_MODEL::getTStart() const
{
  return (Timing.tStart);
}

// Constructor
heave_pid_rt::heave_pid_rt() :
  rtU(),
  rtY(),
  rtDW(),
  rtX(),
  rtXDis(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
heave_pid_rt::~heave_pid_rt() = default;

// Real-Time Model get method
heave_pid_rt::RT_MODEL * heave_pid_rt::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//