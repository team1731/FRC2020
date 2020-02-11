package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootAllBalls;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.StartIntake;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

public class _2_BwdPickup2BallsAndShoot extends SequentialCommandGroup  {
    public Command getCommand(IntakeSubsystem m_intake, SequencerSubsystem m_seq, ShootClimbSubsystem m_shooter){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new StartIntake(m_intake, m_seq),
                new SpinUpShooter(m_shooter)),
            new ShootAllBalls(m_shooter, m_seq));
    }
}
