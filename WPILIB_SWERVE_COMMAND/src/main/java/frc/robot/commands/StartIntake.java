package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntake extends CommandBase {
	public int count;

	public StartIntake(IntakeSubsystem m_intake) {
		count = (int)((Math.random() + 1) * 100);  
		System.out.println("StartIntake : " + count);  
	}

    @Override
    public boolean isFinished() {
		if(--count > 0) return false;
		System.out.println("Startintake : DONE!");
		return true;
    }

}
