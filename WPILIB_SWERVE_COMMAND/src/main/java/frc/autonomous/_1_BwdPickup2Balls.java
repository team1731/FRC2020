package frc.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class _1_BwdPickup2Balls {
  public Command getCommand(DriveSubsystem m_robotDrive){
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(true);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
          //new Translation2d(-1, 1),
          //new Translation2d(-2, -1)

          // new Translation2d(-1.35,2.06),
          // new Translation2d(-2.56,3.29)

           new Translation2d(-1.03,2.637),
           new Translation2d(-3.33,2.82)
          ),
        // End 3 meters straight ahead of where we started, facing forward
        //new Pose2d(-3, 0, new Rotation2d(0)),
        //new Pose2d(-6.8,3.27, new Rotation2d(0)),

        new Pose2d(-5.93,2.28, new Rotation2d(0)),
        config
    );

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, //Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        //Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(-AutoConstants.kPYController, 0, 0),
        new ProfiledPIDController(-AutoConstants.kPThetaController, 0, 0,
                                  AutoConstants.kThetaControllerConstraints),

        m_robotDrive::setModuleStates,

        m_robotDrive

        //mCSVWriter

    );

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}