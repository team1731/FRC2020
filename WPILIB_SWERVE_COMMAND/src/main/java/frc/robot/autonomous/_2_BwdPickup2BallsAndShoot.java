package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Aim;
import frc.robot.commands.ShootAllBalls;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.StartIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class _2_BwdPickup2BallsAndShoot extends SequentialCommandGroup {
    public Command getCommand(DriveSubsystem m_robotDrive, 
                              IntakeSubsystem m_intake, 
                              SequencerSubsystem m_seq, 
                              ShootClimbSubsystem m_shooter,
                              VisionSubsystem m_vision,
                              TargetingSubsystem m_targeting){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new _1_BwdPickup2Balls().getCommand(m_robotDrive),
            new ParallelCommandGroup(
                new StartIntake(m_intake, m_seq),
                new SpinUpShooter(m_shooter),
                new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootAllBalls(m_shooter, m_seq));
        return commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}