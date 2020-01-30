package frc.robot.command;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystem.DriveSubsystem;

//import org.slf4j.Logger;
//import org.slf4j.LoggerFactory;

public final class ZeroGyroCommand extends InstantCommand {

  //private static final Logger logger = LoggerFactory.getLogger(ZeroGyroCommand.class);
  private static final DriveSubsystem swerve = Robot.DRIVE;

  public ZeroGyroCommand() {
    requires(swerve);
  }

  @Override
  protected void initialize() {
    //logger.info("<b>ZeroGyroCommand</b>: initialize starting");
    swerve.zeroGyro();
    //logger.info("<b>ZeroGyroCommand</b>: initialize finished");
  }
}
