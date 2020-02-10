/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ShootSeqCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShootClimbSubsystem shootSubsystem;
  private final SequencerSubsystem seqSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param intakeSubsystem The intake subsystem this command will run on
   * @param seqSubsystem The sequencer subsystem this command will run on
   */
  public ShootSeqCommand(ShootClimbSubsystem shootClimbSubsystem, SequencerSubsystem sequenceSubsystem) {
    shootSubsystem = shootClimbSubsystem;
    seqSubsystem = sequenceSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootClimbSubsystem, seqSubsystem);
  }

  // Called when the command is initially scheduled.
  // If it is used as Default command then it gets call all the time
  @Override
  public void initialize() {
    seqSubsystem.forward();
    //m_SeqSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get necessary input
    //if (!m_SeqSubsystem.getMaxPowerCells()) {
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    seqSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
