/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.Constants.DriveConstants;
import frc.robot.ConstantsOrig.DriveConstantsOrig;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command that will turn the robot to the specified angle using a motion profile.
 */
public class TurnToAngleProfiled extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */
  public TurnToAngleProfiled(double targetAngleDegrees, DriveSubsystem drive) {
    super(
        new ProfiledPIDController(DriveConstantsOrig.kTurnP, DriveConstantsOrig.kTurnI,
                                  DriveConstantsOrig.kTurnD, new TrapezoidProfile.Constraints(
                                  DriveConstantsOrig.kMaxTurnRateDegPerS,
                                  DriveConstantsOrig.kMaxTurnAccelerationDegPerSSquared)),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        // TODO: I'm concerned that the 0, 0 may override driver input. Keep an eye on this (SCH2020)
        (output, setpoint) -> drive.drive(0, 0, -output, false), // (output, setpoint) -> drive.arcadeDrive(0, output),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstantsOrig.kTurnToleranceDeg, DriveConstantsOrig.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
