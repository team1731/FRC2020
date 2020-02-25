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
import frc.robot.util.Utils;

public class M1_Shoot3_Front3_Shoot3 extends _DelayableStrafingAutoMode {
    public M1_Shoot3_Front3_Shoot3(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
            ShootClimbSubsystem m_shootclimb, JevoisVisionSubsystem m_vision) {

        TrajectoryConfig config =
            new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(true);

        // 1ST PATH--
        Trajectory backwardToFront3 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI/4)),
                //  BACKWARD TO FRONT 3
                List.of(
                    new Translation2d(-2,-3.11),
                    new Translation2d(-3.93,-3.11)
            ),
            new Pose2d(-7.06,-3.01, new Rotation2d(0)),
            config
        );

        backwardToFront3 = new Trajectory(unrotateTrajectory(backwardToFront3.getStates(), 90)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": backwardToFront3", backwardToFront3);

        SwerveControllerCommand backwardToFront3Command = new SwerveControllerCommand(
            backwardToFront3,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                      AutoConstants.kThetaControllerConstraints),
            m_robotDrive::setModuleStates,
            m_robotDrive
        );


        // 2ND PATH--
        Trajectory forwardToShootLocation = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI/4)),
                //  FORWARD TO SHOOT LOCATION
                List.of(
                    new Translation2d(-2,-3.11),
                    new Translation2d(-3.93,-3.11)
            ),
            new Pose2d(-7.06,-3.01, new Rotation2d(0)),
            config
        );

        forwardToShootLocation = new Trajectory(unrotateTrajectory(forwardToShootLocation.getStates(), 90)); // make it pure strafe
        Utils.printTrajectory(this.getClass().getSimpleName() + ": forwardToShootLocation", forwardToShootLocation);

        SwerveControllerCommand forwardToShootLocationCommand = new SwerveControllerCommand(
            forwardToShootLocation,
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
            //new Aim(m_robotDrive, m_vision)),
            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(3),

            new WaitCommand(getSecondaryDelaySeconds()),

            // FRONT 3
            backwardToFront3Command,
            new IntakeSeqCommand(m_intake, m_sequence),
            forwardToShootLocationCommand,

            // SHOOT 3
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(3),
            //new Aim(m_robotDrive, m_vision)),
            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(3)
        );

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}