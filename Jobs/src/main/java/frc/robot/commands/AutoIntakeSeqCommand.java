/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
//import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SequencerSubsystem;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class AutoIntakeSeqCommand extends ParallelCommandGroup {
  /**
   * Creates a new ComplexAutoCommand.
   *
   * @param intakeSubsystem The intake subsystem this command will run on
   * @param seqSubsystem The sequencer subsystem this command will run on
   */
  public AutoIntakeSeqCommand(IntakeSubsystem intakeSubsystem, SequencerSubsystem seqSubsystem) {
    addCommands(
        // Extend the intake
        new InstantCommand(intakeSubsystem::extend, intakeSubsystem),
        new ConditionalCommand(
          new InstantCommand(seqSubsystem::addBall, seqSubsystem),
          new InstantCommand(seqSubsystem::stop, seqSubsystem),
          seqSubsystem::getLowSensor
        )
    );   
  }
}