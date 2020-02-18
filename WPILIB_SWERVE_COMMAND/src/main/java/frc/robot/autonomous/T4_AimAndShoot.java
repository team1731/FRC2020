package frc.robot.autonomous;

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
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb),
            //new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootSeqCommand(m_shootclimb, m_sequence));
        //command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> commandGroup.cancel());
    }
}