package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Aim extends CommandBase {
    public Aim(DriveSubsystem m_robotDrive, VisionSubsystem m_vision, TargetingSubsystem m_targeting){
        
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  
}
