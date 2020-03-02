package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DriveSubsystem;


public class F1_Move_Forward extends _DelayableStrafingAutoMode {
  public F1_Move_Forward(DriveSubsystem m_robotDrive) {

    SequentialCommandGroup commandGroup = new SequentialCommandGroup(
      new WaitCommand(getInitialDelaySeconds()),

      createSwerveCommand(m_robotDrive, "MOVE FORWARD", TrajectoryDirection.FWD, 
                          TrajectoryHeading.DO_NOTHING, 0, new double[][]
        {{0.0, 0.0, 0.0},  //initial pose
          {0.5, 0.0},      // waypoint(s)
          {1.0, 0.0, 0.0}} // final pose
        )
    );

    command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
	}

  /* FOR REFERENCE ONLY
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(false);

    // An example trajectory to follow.  All units in meters.
    Trajectory moveForward = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        
        List.of(
          new Translation2d(0.5, 0)
    	),
      new Pose2d(1, 0, new Rotation2d(0)),
      config
    );

    //Utils.printTrajectory(this.getClass().getSimpleName() + ": moveForward", moveForward);
    
    SwerveControllerCommand moveForwardCommand = new SwerveControllerCommand(
        moveForward,
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

      moveForwardCommand
    );
  */
}
