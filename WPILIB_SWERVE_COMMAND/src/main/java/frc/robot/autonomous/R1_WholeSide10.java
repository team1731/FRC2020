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

public class R1_WholeSide10 extends _DelayableStrafingAutoMode {
    public R1_WholeSide10(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
            ShootClimbSubsystem m_shootclimb, JevoisVisionSubsystem m_vision, TargetingSubsystem m_targeting) {

        TrajectoryConfig config =
            new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(true);

        // BACKWARD TO RAIL 2
        Trajectory backwardToRail2 = TrajectoryGenerator.generateTrajectory(
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

        backwardToRail2 = new Trajectory(unrotateTrajectory(backwardToRail2.getStates(), 90)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": backwardToRail2", backwardToRail2);

        SwerveControllerCommand backwardToRail2Command = new SwerveControllerCommand(
            backwardToRail2,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                        AutoConstants.kThetaControllerConstraints),
            m_robotDrive::setModuleStates,
            m_robotDrive
        );


        // STRAFE TO SHOOT LOCATION 1
        Trajectory strafeToShootLocation1 = TrajectoryGenerator.generateTrajectory(
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

        strafeToShootLocation1 = new Trajectory(unrotateTrajectory(strafeToShootLocation1.getStates(), 90)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": strafeToShootLocation1", strafeToShootLocation1);

        SwerveControllerCommand strafeToShootLocation1Command = new SwerveControllerCommand(
            strafeToShootLocation1,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                        AutoConstants.kThetaControllerConstraints),
            m_robotDrive::setModuleStates,
            m_robotDrive
        );

        // BACKWARD TO PICKUP TRENCH 5
        Trajectory backwardToPickupTrench5 = TrajectoryGenerator.generateTrajectory(
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

        backwardToPickupTrench5 = new Trajectory(unrotateTrajectory(backwardToPickupTrench5.getStates(), 90)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": backwardToPickupTrench5", backwardToPickupTrench5);

        SwerveControllerCommand backwardToPickupTrench5Command = new SwerveControllerCommand(
            backwardToPickupTrench5,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                        AutoConstants.kThetaControllerConstraints),
            m_robotDrive::setModuleStates,
            m_robotDrive
        );

        // STRAFE TO SHOOT LOCATION 2
        Trajectory strafeToShootLocation2 = TrajectoryGenerator.generateTrajectory(
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

        strafeToShootLocation2 = new Trajectory(unrotateTrajectory(strafeToShootLocation2.getStates(), 90)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": strafeToShootLocation2", strafeToShootLocation2);

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


        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

            // GET 2 OFF RAIL
            backwardToRail2Command,
            new IntakeSeqCommand(m_intake, m_sequence),

            strafeToShootLocation1Command,

            // SHOOT 5
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(3),
            //new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(3),

            new WaitCommand(getSecondaryDelaySeconds()),

            // TRENCH 5
            backwardToPickupTrench5Command,
            new IntakeSeqCommand(m_intake, m_sequence),

            strafeToShootLocation2Command,

            // SHOOT 5
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(3),
            //new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(3)

        );

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}