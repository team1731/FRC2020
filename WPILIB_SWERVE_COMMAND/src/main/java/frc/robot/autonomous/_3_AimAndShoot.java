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

public class _3_AimAndShoot extends DelayableAutoMode {
    private DriveSubsystem m_robotDrive;
    private SequencerSubsystem m_sequence;
    private ShootClimbSubsystem m_shootclimb;
    private JevoisVisionSubsystem m_vision;
    private TargetingSubsystem m_targeting;
  
    public _3_AimAndShoot(DriveSubsystem m_robotDrive, SequencerSubsystem m_sequence, ShootClimbSubsystem m_shootclimb, JevoisVisionSubsystem m_vision, TargetingSubsystem m_targeting) {
      this.m_robotDrive = m_robotDrive;
      this.m_sequence = m_sequence;
      this.m_shootclimb = m_shootclimb;
      this.m_vision = m_vision;
      this.m_targeting = m_targeting;
    }
  
  @Override
  public Command getCommand() {
          SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SpinUpShooter(m_shootclimb),
                new Aim(m_robotDrive, m_vision, m_targeting)),
            new ShootAllBalls(m_shootclimb, m_sequence));
        return commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}