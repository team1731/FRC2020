/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.ShootClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ClimbingCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShootClimbSubsystem m_ShootClimbSubsystem;
  
  private final DoubleSupplier climb;

  /**
   * Creates a new ExampleCommand.
   *
   * @param ShootClimbSubsystem The intake subsystem this command will run on
   * @param seqSubsystem The sequencer subsystem this command will run on
   */
  public ClimbingCommand(ShootClimbSubsystem shootClimbSubsystem, DoubleSupplier climb) {
    m_ShootClimbSubsystem = shootClimbSubsystem;  
    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootClimbSubsystem);
  }

  // Called when the command is initially scheduled.
  // If it is used as Default command then it gets call all the time
  @Override
  public void initialize() {
    //m_ShootClimbSubsystem.disable(); Can't call this all the time it will make it ineffective
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShootClimbSubsystem.setClimber(climb.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
