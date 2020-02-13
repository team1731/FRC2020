package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Aim;
import frc.robot.commands.ShootAllBalls;
import frc.robot.commands.SpinUpShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class _3_AimAndShoot extends SequentialCommandGroup {
    public Command getCommand(DriveSubsystem m_robotDrive, 
                              ShooterSubsystem m_shooter,
                              VisionSubsystem m_vision,
                              TargetingSubsystem m_targeting){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SpinUpShooter(m_shooter),
                new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootAllBalls(m_shooter));
        return commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}