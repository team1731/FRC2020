package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class F1_Move_Forward extends _DelayableStrafingAutoMode {
  public F1_Move_Forward(DriveSubsystem m_robotDrive) {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(false);

    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        
        List.of(
          new Translation2d(0.5, 0)
    	),

        new Pose2d(1, 0, new Rotation2d(0)),
        config
    );

    double duration = trajectory.getTotalTimeSeconds();
    System.out.println("trajectory duration " +  duration);
    for(int i=0; i<=(int)duration * 2; i++){
      Trajectory.State state = trajectory.sample(i/2.0);
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

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        m_robotDrive::getPose, //Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        //Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                  AutoConstants.kThetaControllerConstraints),

        m_robotDrive::setModuleStates,

        m_robotDrive

    );

    SequentialCommandGroup commandGroup = new SequentialCommandGroup(
      new WaitCommand(getInitialDelaySeconds()),
      swerveControllerCommand,
      new WaitCommand(getSecondaryDelaySeconds()));

    command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
	}

}
