package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Aim;
import frc.robot.commands.ShootAllBalls;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.StartIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

public class T3_BwdPickup2BallsAndShoot extends _DelayableStrafingAutoMode {
    public T3_BwdPickup2BallsAndShoot(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence, ShootClimbSubsystem m_shootclimb, JevoisVisionSubsystem m_vision) {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new T2_BwdPickup2Balls(m_robotDrive).getCommand(),
            new ParallelCommandGroup(
                new StartIntake(m_intake, m_sequence),
                new SpinUpShooter(m_shootclimb),
                new Aim(m_robotDrive, m_vision)),
            new ShootAllBalls(m_shootclimb, m_sequence));
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}