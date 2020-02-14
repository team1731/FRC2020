package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAllBalls extends CommandBase {
	public int count;

	public ShootAllBalls(ShooterSubsystem m_shooter) {
		count = (int)((Math.random() + 1) * 100);  
		System.out.println("ShootAllBalls : " + count);  
	}

    @Override
    public boolean isFinished() {
		if(--count > 0) return false;
		System.out.println("ShootAllBalls : DONE!");
		return true;
    }

}
