package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Aim;
import frc.robot.commands.ShootAllBalls;
import frc.robot.commands.SpinUpShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.TargetingSubsystem;

public class _3_AimAndShoot extends SequentialCommandGroup {
    public Command getCommand(DriveSubsystem m_robotDrive,
                              SequencerSubsystem m_seq, 
                              ShootClimbSubsystem m_shootclimb,
                              JevoisVisionSubsystem m_vision,
                              TargetingSubsystem m_targeting){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SpinUpShooter(m_shootclimb),
                new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootAllBalls(m_shootclimb, m_seq));
        return commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}