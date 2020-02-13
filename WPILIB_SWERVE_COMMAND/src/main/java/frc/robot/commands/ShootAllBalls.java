package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAllBalls extends CommandBase {

	public ShootAllBalls(ShooterSubsystem m_shooter) {
	}

    @Override
    public boolean isFinished() {
      return false;
    }

}
