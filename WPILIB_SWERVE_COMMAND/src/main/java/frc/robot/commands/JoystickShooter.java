/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ShootClimbSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class JoystickShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShootClimbSubsystem shootSubsystem;
  private DoubleSupplier shootSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickShooter(ShootClimbSubsystem subsystem, DoubleSupplier shootSpeed) {
    shootSubsystem = subsystem;
    this.shootSpeed = shootSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shootSubsystem.enableShooting();
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootSubsystem.enableShooting((shootSpeed.getAsDouble()+1)/2);
    //shootSubsystem.enableShooting(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootSubsystem.stopShooting();
  }
  
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
