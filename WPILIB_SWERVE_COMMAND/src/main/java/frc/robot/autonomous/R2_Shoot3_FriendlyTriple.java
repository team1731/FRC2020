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
import frc.robot.commands.IntakeSeqCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.LedStringSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.util.Utils;

public class R2_Shoot3_FriendlyTriple extends _DelayableStrafingAutoMode {
    public R2_Shoot3_FriendlyTriple(LedStringSubsystem m_ledstring, DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, 
            SequencerSubsystem m_sequence, ShootClimbSubsystem m_shootclimb, JevoisVisionSubsystem m_vision,
            TargetingSubsystem m_targeting) {

    // Create config for trajectory
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

    // An example trajectory to follow.  All units in meters.
    Trajectory moveToTrench = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        
        List.of(
          //new Translation2d(, 0)
        ),
        
      new Pose2d(-1.56, -0.2, new Rotation2d(Math.toRadians(1))),
      configBackward
    );

    moveToTrench = new Trajectory(unrotateTrajectory(moveToTrench.getStates(), 1)); // make it pure strafe
    Utils.printTrajectory(this.getClass().getSimpleName() + ": moveToTrench", moveToTrench);
    
    SwerveControllerCommand moveToTrenchCommand = new SwerveControllerCommand(
        moveToTrench,
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

    // An example trajectory to follow.  All units in meters.
    Trajectory pickupTriple = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(-1.56, -0.2, new Rotation2d(Math.toRadians(1))),
        
        List.of(
          //new Translation2d(, 0)
        ),
        
      new Pose2d(-4.59, -0.2, new Rotation2d(Math.toRadians(1))),
      configBackward
    );

    pickupTriple = new Trajectory(unrotateTrajectory(pickupTriple.getStates(), 1)); // make it pure strafe
    Utils.printTrajectory(this.getClass().getSimpleName() + ": pickupTriple", pickupTriple);
    
    SwerveControllerCommand pickupTripleCommand = new SwerveControllerCommand(
        pickupTriple,
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

    // An example trajectory to follow.  All units in meters.
    Trajectory goHome = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(-4.59, -0.2, new Rotation2d(Math.toRadians(1))),
        
        List.of(
          //new Translation2d(, 0)
        ),
        
      new Pose2d(0, 0, new Rotation2d(0)),
      configForward
    );

    goHome = new Trajectory(unrotateTrajectory(goHome.getStates(), 0)); // make it pure strafe
    Utils.printTrajectory(this.getClass().getSimpleName() + ": goHome", goHome);
    
    SwerveControllerCommand goHomeCommand = new SwerveControllerCommand(
        pickupTriple,
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

    //#region Sequence
    SequentialCommandGroup commandGroup = new SequentialCommandGroup(
        new WaitCommand(getInitialDelaySeconds()),


        new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),

        new WaitCommand(3),

        //new Aim(m_robotDrive, m_vision, m_targeting)),
        new ShootSeqCommandAuto(m_ledstring, m_shootclimb, m_sequence).withTimeout(2),

        new WaitCommand(getSecondaryDelaySeconds()),

        moveToTrenchCommand,

        new ParallelCommandGroup(
            new IntakeSeqCommand(m_intake, m_sequence).withTimeout(4),
            pickupTripleCommand
        ),

        new ParallelCommandGroup(
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),
            goHomeCommand
        ),

        //new Aim(m_robotDrive, m_vision, m_targeting)),
        new ShootSeqCommandAuto(m_ledstring, m_shootclimb, m_sequence).withTimeout(2)
        
        
        );

        //command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> m_shootclimb.stopShooting());
    }
    //#endregion
}