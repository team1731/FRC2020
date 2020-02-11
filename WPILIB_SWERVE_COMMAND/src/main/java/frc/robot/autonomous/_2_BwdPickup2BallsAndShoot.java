package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootAllBalls;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.StartIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class _2_BwdPickup2BallsAndShoot extends SequentialCommandGroup  {
    public Command getCommand(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, ShooterSubsystem m_shooter){
        return new SequentialCommandGroup(
            new _1_BwdPickup2Balls().getCommand(m_robotDrive),
            new ParallelCommandGroup(
                new StartIntake(m_intake),
                new SpinUpShooter(m_shooter)),
            new ShootAllBalls(m_shooter));
    }
}