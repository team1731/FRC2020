/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.autonomous.AutoModes;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.InstrumentedSwerveControllerCommand;
import frc.robot.util.DebugOutput;
import frc.robot.util.ReflectingCSVWriter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  ReflectingCSVWriter<DebugOutput> mCSVWriter;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(ReflectingCSVWriter<DebugOutput> mCSVWriter) {
    this.mCSVWriter = mCSVWriter;

    m_robotDrive = new DriveSubsystem(mCSVWriter);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_robotDrive.drive(
            // Get the x speed. We are inverting this because Xbox controllers return
            // negative values when we push forward.
            -m_driverController.getY(GenericHID.Hand.kLeft),

            // Get the y speed or sideways/strafe speed. We are inverting this because
            // we want a positive value when we pull to the left. Xbox controllers
            // return positive values when you pull to the right by default.
            -m_driverController.getX(GenericHID.Hand.kLeft),

            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            -m_driverController.getX(GenericHID.Hand.kRight), true), m_robotDrive) // <------- RDB2020 I added m_robotDrive here to get rid of
                                                                                   //                  "Default Command must require subsytem"
        );
  }

/**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  public String getAutonomousName(int autoNum) {
    return AutoModes.getName(autoNum);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int autoNum) {
    Command command = AutoModes.getCommand(m_robotDrive, autoNum);

    Trajectory trajectory = AutoModes.getTrajectory(autoNum);
    if(trajectory == null){
      return null;
    }
    double duration = trajectory.getTotalTimeSeconds();
    System.out.println("Autonomous: " + getAutonomousName(autoNum));
    System.out.println("trajectory duration " +  duration);
    for(int i=0; i<=(int)duration; i++){
      Trajectory.State state = trajectory.sample(i);
      System.out.println("state " + i + "                 poseMetersX " + state.poseMeters.getTranslation().getX());
      System.out.println("state " + i + "                 poseMetersY " + state.poseMeters.getTranslation().getY());
      System.out.println("state " + i + "         poseMetersTheta Deg " + state.poseMeters.getRotation().getDegrees());
      System.out.println("state " + i + "     velocityMetersPerSecond " + state.velocityMetersPerSecond);
    }
    Trajectory.State state = trajectory.sample(duration);
    System.out.println("state (end)             poseMetersX " + state.poseMeters.getTranslation().getX());
    System.out.println("state (end)             poseMetersY " + state.poseMeters.getTranslation().getY());
    System.out.println("state (end)     poseMetersTheta Deg " + state.poseMeters.getRotation().getDegrees());
    System.out.println("state (end) velocityMetersPerSecond " + state.velocityMetersPerSecond);

    // Run path following command, then stop at the end.
    return command.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
