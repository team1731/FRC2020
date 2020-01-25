package frc.robot.command;

import static org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode.TELEOP;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.control.DriverControls;
import frc.robot.subsystem.DriveSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class TeleOpDriveCommand extends Command {

  /*
    PLEASE NOTE:
    PowerA controllers (the off brand wired controllers we get) have terrible left thumbsticks. They typically end up
    centering at about +0.87 or -0.87. However, the right thumbstick X is
    literally perfection, centering at 0.0.
    I have not tested official XBox controllers. That will need to happen later
    The fancy flight sim controller for whatever inane reason centers at -0.1 and -0.08. It could be that it's not trimmed,
    but the trim switches do nothing to change the values, and they also don't have a button in the driver station

    Please keep these notes in mind when adjusting the DEADBAND variable
  */

  private static final double DEADBAND = 0.09;

  //private static final DriveSubsystem swerve = Robot.DRIVE;
  private static final DriverControls controls = Robot.CONTROLS.getDriverControls();

  public TeleOpDriveCommand() {
    //requires(swerve);
  }

  @Override
  protected void initialize() {
    //swerve.setDriveMode(TELEOP);
  }

  @Override
  protected void execute() {
    double forward = deadband(controls.getForward());
    double strafe = deadband(controls.getStrafe());
    double azimuth = deadband(controls.getYaw());

    //swerve.drive(forward, strafe, azimuth);
    SmartDashboard.putNumber("FORWARD", forward);
    SmartDashboard.putNumber("FORWARD_RAW", controls.getForward());
    SmartDashboard.putNumber("STRAFE", strafe);
    SmartDashboard.putNumber("STRAFE_RAW", controls.getStrafe());
    SmartDashboard.putNumber("AZIMUTH", azimuth);
    SmartDashboard.putNumber("AZIMUTH_RAW", controls.getYaw());
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    //swerve.drive(0.0, 0.0, 0.0);
  }

  private double deadband(double value) {
    if (Math.abs(value) < DEADBAND) return 0.0;
    return value;
  }
}
