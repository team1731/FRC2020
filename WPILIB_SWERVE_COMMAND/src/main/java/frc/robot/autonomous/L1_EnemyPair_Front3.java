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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
                
        TrajectoryConfig configBackward =
            new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(true);

        TrajectoryConfig configForward =
            new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(false);

                /*
        TrajectoryConfig configForward = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(false);
                */
    
        // BACKWARD TO ENEMY PAIR
        Trajectory backwardToEnemyPair = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),

            //  BACKWARD TO ENEMY PAIR
            List.of(
                new Translation2d(-1.06, -0.2),
                new Translation2d(-1.9, -0.2)
            ),

            new Pose2d(-2.2, -0.2, new Rotation2d(0)),
            configBackward
        );

        backwardToEnemyPair = new Trajectory(unrotateTrajectory(backwardToEnemyPair.getStates(), 15)); // make it pure strafe
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
        Trajectory strafeToShootLocation1_1 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(-2.2, -0.2, new Rotation2d(-50)),

            //  STRAFE TO SHOOT LOCATION
            List.of(
                //new Translation2d(-1, -0.1),
                //new Translation2d(-2, -2.43)
            ),

            new Pose2d(-0.54, -4.74, new Rotation2d(Math.toRadians(-50))),
            configForward
        );
    
        strafeToShootLocation1_1 = new Trajectory(unrotateTrajectory(strafeToShootLocation1_1.getStates(), -20)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": strafeToShootLocation", strafeToShootLocation1_1);

        SwerveControllerCommand strafeToShootLocation1_1Command = new SwerveControllerCommand(
            strafeToShootLocation1_1,
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
        Trajectory strafeToShootLocation1_2 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, -0.32, new Rotation2d(Math.toRadians(0))),

            //  STRAFE TO SHOOT LOCATION
            List.of(
                new Translation2d(-0.16, -2.43)
            ),

            new Pose2d(-0.54, -4.74, new Rotation2d(Math.toRadians(-12))),
            configBackward
        );
    
        strafeToShootLocation1_1 = new Trajectory(unrotateTrajectory(strafeToShootLocation1_2.getStates(), -10)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": strafeToShootLocation", strafeToShootLocation1_2);

        SwerveControllerCommand strafeToShootLocation1_2Command = new SwerveControllerCommand(
            strafeToShootLocation1_2,
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
        Trajectory backwardToPickupFront3_1 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(-0.54, -4.74, new Rotation2d(Math.toRadians(-12))),
                //  BACKWARD TO PICKUP FRONT 3
                List.of(
                    new Translation2d(-0.93, -3.45),
                    new Translation2d(-1.81, -3.3),
                    new Translation2d(-1.96, -3.55)

            ),
            new Pose2d(-2.14, -3.4, new Rotation2d(Math.toRadians(65))),
            configBackward
        );
    
        backwardToPickupFront3_1 = new Trajectory(unrotateTrajectory(backwardToPickupFront3_1.getStates(), 65)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": backwardToPickupFront3", backwardToPickupFront3_1);

        SwerveControllerCommand backwardToPickupFront3_1Command = new SwerveControllerCommand(
            backwardToPickupFront3_1,
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
        Trajectory backwardToPickupFront3_2 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(-2.14, -3.4, new Rotation2d(Math.toRadians(65))),

            //  BACKWARD TO PICKUP FRONT 3
            List.of(
                new Translation2d(-2.11, -3.53)
            ),

            new Pose2d(-1.79, -4.08, new Rotation2d(Math.toRadians(70))),
            configBackward
        );
    
        backwardToPickupFront3_2 = new Trajectory(maintainTrajectory(backwardToPickupFront3_2.getStates(), 65)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": backwardToPickupFront3", backwardToPickupFront3_2);

        SwerveControllerCommand backwardToPickupFront3_2Command = new SwerveControllerCommand(
            backwardToPickupFront3_2,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                      AutoConstants.kThetaControllerConstraints),
            m_robotDrive::setModuleStates,
            m_robotDrive
        );
        
        //#region STRAFE TO SHOOT LOCATION 2
            //#region Path

            // STRAFE TO SHOOT LOCATION
            Trajectory strafeToShootLocation2 = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(-1.79, -4.08, new Rotation2d(70)),

                //  STRAFE TO SHOOT LOCATION
                List.of(
                    new Translation2d(-0.89, -4.24)
                ),

                new Pose2d(-0.59, -4.92, new Rotation2d(-7)),
                configBackward
            );

            //#endregion
        
            //#region Command
        //#endregion

        strafeToShootLocation2 = new Trajectory(unrotateTrajectory(strafeToShootLocation2.getStates(), 70)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": strafeToShootLocation", strafeToShootLocation2);

        SwerveControllerCommand strafeToShootLocation2Command = new SwerveControllerCommand(
            strafeToShootLocation2,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                        AutoConstants.kThetaControllerConstraints),
            m_robotDrive::setModuleStates,
            m_robotDrive
        );

        //#endregion

        //#region Sequence
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

            // ENEMY PAIR
            new ParallelCommandGroup(
                backwardToEnemyPairCommand,
                new IntakeSeqCommand(m_intake, m_sequence).withTimeout(4)
            ),

            new ParallelCommandGroup(
                new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),
                strafeToShootLocation1_1Command
            ),

            /*
            new ParallelCommandGroup(
                new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(3),
                strafeToShootLocation1_2Command
            ),
            */
            //new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)),
            new WaitCommand(1),
            //new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootSeqCommandAuto(m_ledstring, m_shootclimb, m_sequence).withTimeout(1),

            new WaitCommand(getSecondaryDelaySeconds())

            /*
            // FRONT 3
            backwardToPickupFront3_1Command,

            new ParallelCommandGroup(
                backwardToPickupFront3_2Command,
                new IntakeSeqCommand(m_intake, m_sequence).withTimeout(5)
            ),

            strafeToShootLocation2Command,
            
            // SHOOT
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(3),
            //new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootSeqCommandAuto(m_ledstring, m_shootclimb, m_sequence).withTimeout(3)
*/
        );
        //#endregion

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}