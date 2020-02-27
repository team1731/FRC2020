package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;

public class Aim extends CommandBase {
    public int count;

    public Aim(DriveSubsystem m_robotDrive, JevoisVisionSubsystem m_vision){
		count = (int)((Math.random() + 1) * 100);  
        //System.out.println("Aim : " + count);     
    }

    @Override
    public boolean isFinished() {
		  if(--count > 0) return false;
		  //System.out.println("Aim : DONE!");
		  return true;
   }
  
}
