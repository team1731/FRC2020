package frc.team1731.swerveexample.command;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team1731.swerveexample.Robot;
import frc.team1731.swerveexample.subsystem.DriveSubsystem;

public final class ZeroGyroCommand extends InstantCommand {

  private static final DriveSubsystem swerve = Robot.DRIVE;

  public ZeroGyroCommand() {
    requires(swerve);
  }

  @Override
  protected void initialize() {
    swerve.zeroGyro();
  }
}
