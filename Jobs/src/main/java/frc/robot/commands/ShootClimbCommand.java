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
public class ShootClimbCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShootClimbSubsystem m_ShootClimbSubsystem;
  private final SequencerSubsystem m_SeqSubsystem;
  
  private final BooleanSupplier shoot;
  private boolean last;
  private boolean activate;

  /**
   * Creates a new ExampleCommand.
   *
   * @param ShootClimbSubsystem The intake subsystem this command will run on
   * @param seqSubsystem The sequencer subsystem this command will run on
   */
  public ShootClimbCommand(ShootClimbSubsystem shootClimbSubsystem, SequencerSubsystem seqSubsystem, BooleanSupplier shoot) {
    m_ShootClimbSubsystem = shootClimbSubsystem;
    m_SeqSubsystem = seqSubsystem;

    this.shoot = shoot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootClimbSubsystem, seqSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShootClimbSubsystem.disable();
    activate = false;
    last = activate;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (shoot.getAsBoolean()) {
      activate = true;
    } else {
      activate = false;
    }

    if (last != activate) {
      if (activate) {
        m_SeqSubsystem.forward();
      } else {
        m_SeqSubsystem.stop();
      }
      last = activate;
    }
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
