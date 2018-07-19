/*
 * Flags.h
 *
 * Created: 12/17/2017 10:55:18 PM
 *  Author: abheesh
 */ 

#pragma once


//These are individual PID //
bool compassPID = true;
bool linetrackerPID = true;


bool FlagChangeSetpointCompass = false;
bool FlagInitialAngleSetpoint = true;
///////These are updating PID flags////////////
// volatile bool PidUpdateFlagLinetracker = true;
//volatile bool PidUpdateFlagCompass = true;
//volatile bool PidUpdateFlagDriveX = true;
//volatile bool PidUpdateFlagDriveY = true;