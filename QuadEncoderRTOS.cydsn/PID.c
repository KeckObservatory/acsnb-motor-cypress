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


#define PID_C_

#include "PID.h"
#include <stdbool.h>
#include <math.h>

/******************************************************************************
  This is called to start using a PID loop.
******************************************************************************/

void StartPIDLoop( TPIDLoop* TheLoop ) {

  TheLoop->CurrentOutput = 0.0;
  TheLoop->LastError     = 0.0;
  TheLoop->ICurrentValue = 0.0;
  TheLoop->DCurrentValue = 0.0;
}

/******************************************************************************
  This is called to initialize a PID loop and get it ready to use. This should
  only need to be called once.
******************************************************************************/

void InitializePIDLoop( TPIDLoop* TheLoop, float PGain, float IGain, float DGain, float IMin, float IMax ) {

  /* Take what we are given. */
  TheLoop->ControlConstants.PGain = PGain;
  TheLoop->ControlConstants.IGain = IGain;
  TheLoop->ControlConstants.DGain = DGain;
  TheLoop->ControlConstants.IMin  = IMin;
  TheLoop->ControlConstants.IMax  = IMax;

  /* Start the loop, which initializes the live internal variables. */
  StartPIDLoop( TheLoop );
}

/******************************************************************************
  This is called to initialize a PID loop and get it ready to use. This should
  only need to be called once. This takes a structure as input.

  In some cases it is important to retain the percentage of the current I
  value in the PID, as in when we are reusing the PID from one type of
  control to another. In these cases, passing in RetainIValuePercentage as true.
******************************************************************************/

void InitializePIDLoopConstants( TPIDLoop* TheLoop, TPIDConstants ThePIDConstants, bool RetainIValuePercentage ) {

  float TheCurrentIValue, TheNewIValue;

  /* If asked to, retain the I value percentage of the current loop. Bypass this if the new constants have
     a 0 IGain, to eliminate division by 0. */
  if ( RetainIValuePercentage && ( ThePIDConstants.IGain != 0 ) ) {

    /* Save off the current PID I value. */
    TheCurrentIValue = TheLoop->ICurrentValue;

    /* Compute a new I value to start the PID loop out at. */
    TheNewIValue = ( TheCurrentIValue * TheLoop->ControlConstants.IGain ) / ThePIDConstants.IGain;

    /* Initialize the PID loop with the new PID constants. */
    InitializePIDLoop( TheLoop, ThePIDConstants.PGain, ThePIDConstants.IGain, ThePIDConstants.DGain, ThePIDConstants.IMin, ThePIDConstants.IMax );

    /* Update the I value into the PID loop. */
    PresetPIDLoopIValue( TheLoop, TheNewIValue );
  }
  else
    /* No I value retention, just update the loop constants. */
    InitializePIDLoop( TheLoop, ThePIDConstants.PGain, ThePIDConstants.IGain, ThePIDConstants.DGain, ThePIDConstants.IMin, ThePIDConstants.IMax );
}

/******************************************************************************
  This should be called periodically and regularly, to update the PID loop control.
  This should also be called each time the setpoint is changed.
******************************************************************************/

void UpdatePIDLoop( TPIDLoop* TheLoop, float TheSetpoint, float TheFeedbackPosition, float TheError ) {

  float TempOutput, TempOutputConfined;

  /* Retain these for completeness. */
  TheLoop->Setpoint         = TheSetpoint;
  TheLoop->PreviousPosition = TheLoop->Position;
  TheLoop->Position         = TheFeedbackPosition;
  TheLoop->Error            = TheError;

  /* =-=-= P Term =-=-= */
  TheLoop->pTerm = TheLoop->ControlConstants.PGain * TheLoop->Error;


  /* =-=-= I Term =-=-= */
  TheLoop->ICurrentValue += TheLoop->Error;

  /* Limit the I value. */
  if ( TheLoop->ICurrentValue < TheLoop->ControlConstants.IMin )
    TheLoop->ICurrentValue = TheLoop->ControlConstants.IMin;

  if ( TheLoop->ICurrentValue > TheLoop->ControlConstants.IMax )
    TheLoop->ICurrentValue = TheLoop->ControlConstants.IMax;

  TheLoop->iTerm = TheLoop->ICurrentValue * TheLoop->ControlConstants.IGain;



  /* =-=-= D Term =-=-= */
  TheLoop->dTerm = TheLoop->ControlConstants.DGain * ( TheLoop->Position - TheLoop->PreviousPosition );

  TheLoop->DCurrentValue = TheLoop->dTerm;


  /* All together now. */
  TempOutput = TheLoop->pTerm + TheLoop->iTerm - TheLoop->dTerm;

  /* Clip the output first based on the min/max extended output limits. */
  if ( TempOutput < TheLoop->MinCurrentOutputExtended )
    TempOutput = TheLoop->MinCurrentOutputExtended;

  if ( TempOutput > TheLoop->MaxCurrentOutputExtended )
    TempOutput = TheLoop->MaxCurrentOutputExtended;

  /* Save off the extended output value. */
  TheLoop->CurrentOutputExtended = TempOutput;

  /* Additionally, confine the extended output as a +/- 100 value so it fits into a PWM, by
     scaling it based on the min/max limits. */
  TempOutputConfined = TempOutput;

  if ( TempOutputConfined > 0 ) {
    /* Confine on the positive side. */
    if ( TheLoop->MaxCurrentOutputExtended != 100 )
      TempOutputConfined = ( TempOutputConfined * 100 ) / TheLoop->MaxCurrentOutputExtended;
  }
  else {
    /* Confine on the negative side. */
    if ( TheLoop->MinCurrentOutputExtended != 0 )
      TempOutputConfined = -( fabsf( ( TempOutputConfined * 100 ) / TheLoop->MinCurrentOutputExtended ) );
  }

  TheLoop->CurrentOutputExtendedConfinedAsPercent = TempOutputConfined;


  /* Further clip the output to be 0 - 100. */
  if ( TempOutput < 0 )
    TempOutput = 0;

  if ( TempOutput > 100 )
    TempOutput = 100;

  TheLoop->CurrentOutput = TempOutput;

  /* Restart the stop watch to reset it's starting time. */
  ///TODO FIX StopWatchStart( &TheLoop->LastUpdate );
}

/****************************************************************************
  This routine is used to preload the I term value for a PID loop.
****************************************************************************/

void PresetPIDLoopIValue( TPIDLoop* TheLoop, float TheIValue ) {

  TheLoop->ICurrentValue = TheIValue;
}




/* [] END OF FILE */
