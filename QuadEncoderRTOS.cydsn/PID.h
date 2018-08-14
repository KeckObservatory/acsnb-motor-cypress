/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/


#ifndef PID_H_
#define PID_H_

#include <stdbool.h>    
    
/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef PID_C_
  #define EXTERN extern
#else
  #define EXTERN
#endif

#pragma pack(1)

/****************************************************************************
  INTERFACED ROUTINES
****************************************************************************/

/* This defines a structure to hold all of the PID control loop constants. */
typedef struct {

  /* These are the P, I and D gain factors that control how the loop operates. */
  float PGain;
  float IGain;
  float DGain;

  /* Limits for the integral term. */
  float IMin;
  float IMax;

} TPIDConstants;


/* This defines a structure to manage one PID control loop. */
typedef struct {

  /* This holds all of the control constants. */
  TPIDConstants ControlConstants;


  /* This is the current setpoint value. */
  float Setpoint;

  /* This is the current feedback position. */
  float Position;

  /* This is the previous feedback position. */
  float PreviousPosition;

  /* This is the current error of the position to the setpoint. Equal to Setpoint - Position,
     so this is a positive number when below the setpoint. */
  float Error;

  /* This holds the current control output value, as a percent, clipped from 0 -> 100 (the normal way). */
  float CurrentOutput;

  /* This holds the current extended control output value, as a percent, clipped from MinCurrentOutputExtended -> MaxCurrentOutputExtended.
     Note that we allow values less than zero here, and greater than 100. */
  float CurrentOutputExtended;

  /* This holds the current extended control output value, as a percent, clipped from -100 -> 100. */
  float CurrentOutputExtendedConfinedAsPercent;

  /* To add flexibility into the control, we will allow the Extended output to go outside the normal 0 - 100% bounds.
     These define the bounds of the CurrentOutputExtended value. */
  float MinCurrentOutputExtended;
  float MaxCurrentOutputExtended;

  /* This holds the last setpoint error. */
  float LastError;

  float pTerm;
  float iTerm;
  float dTerm;

  float ICurrentValue;
  float DCurrentValue;

  /* This tracks how long it has been since the last loop update. */
  //TStopWatch LastUpdate;

} TPIDLoop;


void StartPIDLoop( TPIDLoop* TheLoop );
void InitializePIDLoop( TPIDLoop* TheLoop, float PGain, float IGain, float DGain, float IMin, float IMax );
void InitializePIDLoopConstants( TPIDLoop* TheLoop, TPIDConstants ThePIDConstants, bool RetainIValuePercentage );
void UpdatePIDLoop( TPIDLoop* TheLoop, float TheSetpoint, float TheFeedbackPosition, float TheError );
void PresetPIDLoopIValue( TPIDLoop* TheLoop, float TheIValue );


#undef EXTERN

#endif


/* [] END OF FILE */
