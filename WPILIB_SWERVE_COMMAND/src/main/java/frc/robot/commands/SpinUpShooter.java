package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooter extends CommandBase {
	public int count;

	public SpinUpShooter(ShooterSubsystem m_shooter) {
		count = (int)((Math.random() + 1) * 100);  
		System.out.println("SpinUpShooter : " + count);  
	}

    @Override
    public boolean isFinished() {
		if(--count > 0) return false;
		System.out.println("SpinUpShooter : DONE!");
		return true;
    }

}
