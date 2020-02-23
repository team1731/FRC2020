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
import frc.robot.subsystems.LedStringSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.util.Utils;

public class L1_EnemyPair_Front3 extends _DelayableStrafingAutoMode {
    public L1_EnemyPair_Front3(LedStringSubsystem m_ledstring, DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
            ShootClimbSubsystem m_shootclimb, JevoisVisionSubsystem m_vision, TargetingSubsystem m_targeting) {
                
        TrajectoryConfig config =
            new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(true);
    
        // BACKWARD TO ENEMY PAIR
        Trajectory backwardToEnemyPair = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI/4)),
                //  BACKWARD TO ENEMY PAIR
                List.of(
                    new Translation2d(-2,-3.11),
                    new Translation2d(-3.93,-3.11)
            ),
            new Pose2d(-7.06,-3.01, new Rotation2d(0)),
            config
        );

        backwardToEnemyPair = new Trajectory(unrotateTrajectory(backwardToEnemyPair.getStates(), 90)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": backwardToEnemyPair", backwardToEnemyPair);
    
        SwerveControllerCommand backwardToEnemyPairCommand = new SwerveControllerCommand(
            backwardToEnemyPair,
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
    
        // BACKWARD TO PICKUP FRONT 3
        Trajectory backwardToPickupFront3 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI/4)),
                //  BACKWARD TO PICKUP FRONT 3
                List.of(
                    new Translation2d(-2,-3.11),
                    new Translation2d(-3.93,-3.11)
            ),
            new Pose2d(-7.06,-3.01, new Rotation2d(0)),
            config
        );
    
        backwardToPickupFront3 = new Trajectory(unrotateTrajectory(backwardToPickupFront3.getStates(), 90)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": backwardToPickupFront3", backwardToPickupFront3);

        SwerveControllerCommand backwardToPickupFront3Command = new SwerveControllerCommand(
            backwardToPickupFront3,
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

            // ENEMY PAIR
            backwardToEnemyPairCommand,
            new IntakeSeqCommand(m_intake, m_sequence),

            strafeToShootLocationCommand,

            // SHOOT
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(3),
            //new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootSeqCommandAuto(m_ledstring, m_shootclimb, m_sequence).withTimeout(3),

            new WaitCommand(getSecondaryDelaySeconds()),

            // FRONT 3
            backwardToPickupFront3Command,
            new IntakeSeqCommand(m_intake, m_sequence),

            // SHOOT
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(3),
            //new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootSeqCommandAuto(m_ledstring, m_shootclimb, m_sequence).withTimeout(3)

        );

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}