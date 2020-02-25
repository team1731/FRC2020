package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Aim;
import frc.robot.commands.ShootSeqCommandAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.LedStringSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.util.Utils;

public class T5_ShootDriveBackward extends _DelayableStrafingAutoMode {
    public T5_ShootDriveBackward(LedStringSubsystem m_ledstring, DriveSubsystem m_robotDrive,
            SequencerSubsystem m_sequence, ShootClimbSubsystem m_shootclimb, JevoisVisionSubsystem m_vision,
            TargetingSubsystem m_targeting) {

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(true);

    // An example trajectory to follow.  All units in meters.
    Trajectory moveForward = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        
        List.of(
          new Translation2d(-0.5, 0)
    	),
      new Pose2d(-1, 0, new Rotation2d(0)),
      config
    );

    Utils.printTrajectory(this.getClass().getSimpleName() + ": moveForward", moveForward);
    
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


        new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),

        new WaitCommand(3),

        //new Aim(m_robotDrive, m_vision, m_targeting)),
        new ShootSeqCommandAuto(m_ledstring, m_shootclimb, m_sequence).withTimeout(2),

        new WaitCommand(getSecondaryDelaySeconds()),

        moveForwardCommand
        
        );

        //command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> m_shootclimb.stopShooting());
    }
}