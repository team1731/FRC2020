package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Aim;
import frc.robot.commands.ShootSeqCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.TargetingSubsystem;

public class T4_AimAndShoot extends _DelayableStrafingAutoMode {  
    public T4_AimAndShoot(DriveSubsystem m_robotDrive, SequencerSubsystem m_sequence, ShootClimbSubsystem m_shootclimb, JevoisVisionSubsystem m_vision, TargetingSubsystem m_targeting) {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(3),
            //new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootSeqCommand(m_shootclimb, m_sequence).withTimeout(3));
        CommandScheduler.getInstance().onCommandExecute(command -> SmartDashboard.putString(
            "T4_AimAndShoot", "RUNNING"));
        CommandScheduler.getInstance().onCommandFinish(command -> SmartDashboard.putString(
            "T4_AimAndShoot", "FINISHED"));
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}