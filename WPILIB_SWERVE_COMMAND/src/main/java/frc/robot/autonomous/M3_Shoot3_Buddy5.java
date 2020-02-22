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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.IntakeSeqCommand;
import frc.robot.commands.ShootSeqCommandAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.util.Utils;

public class M3_Shoot3_Buddy5 extends _DelayableStrafingAutoMode {
    public M3_Shoot3_Buddy5(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
            ShootClimbSubsystem m_shootclimb, JevoisVisionSubsystem m_vision, TargetingSubsystem m_targeting) {
                
        TrajectoryConfig config =
            new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(true);
    
        // BACKWARD TO BUDDY 5
        Trajectory backwardToBuddy5 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI/4)),
                //  BACKWARD TO BUDDY 5
                List.of(
                    new Translation2d(-2,-3.11),
                    new Translation2d(-3.93,-3.11)
            ),
            new Pose2d(-7.06,-3.01, new Rotation2d(0)),
            config
        );

        backwardToBuddy5 = new Trajectory(unrotateTrajectory(backwardToBuddy5.getStates(), 90)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": backwardToBuddy5", backwardToBuddy5);
    
        SwerveControllerCommand backwardToBuddy5Command = new SwerveControllerCommand(
            backwardToBuddy5,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                        AutoConstants.kThetaControllerConstraints),
            m_robotDrive::setModuleStates,
            m_robotDrive
        );
    
    
        // STRAFE TO SHOOT LOCATION
        Trajectory strafeToShootLocation = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI/4)),
                //  STRAFE TO SHOOT LOCATION
                List.of(
                    new Translation2d(-2,-3.11),
                    new Translation2d(-3.93,-3.11)
            ),
            new Pose2d(-7.06,-3.01, new Rotation2d(0)),
            config
        );
    
        strafeToShootLocation = new Trajectory(unrotateTrajectory(strafeToShootLocation.getStates(), 90)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": strafeToShootLocation", strafeToShootLocation);

        SwerveControllerCommand strafeToShootLocationCommand = new SwerveControllerCommand(
            strafeToShootLocation,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                        AutoConstants.kThetaControllerConstraints),
            m_robotDrive::setModuleStates,
            m_robotDrive
        );
    
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

            // SHOOT 3
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(3),
            //new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootSeqCommandAuto(m_sequence).withTimeout(3),

            new WaitCommand(getSecondaryDelaySeconds()),
            
            // BACKWARD TO BUDDY (UP TO) 5
            backwardToBuddy5Command,
            new IntakeSeqCommand(m_intake, m_sequence),

            strafeToShootLocationCommand,

            // SHOOT
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(3),
            //new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootSeqCommandAuto(m_sequence).withTimeout(3)

        );

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));        
    }
}