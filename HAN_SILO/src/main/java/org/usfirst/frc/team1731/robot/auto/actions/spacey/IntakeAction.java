package org.usfirst.frc.team1731.robot.auto.actions.spacey;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1731.robot.auto.actions.Action;
import org.usfirst.frc.team1731.robot.subsystems.PowerCell;

/**
 * Controls LEDs, can take an LED.WantedState
 */
public class IntakeAction implements Action {

    private PowerCell mPowerCell = PowerCell.getInstance();
    private double mStartTime;
    private PowerCell.WantedState myDesiredState;

    public IntakeAction() {
        myDesiredState = PowerCell.WantedState.INTAKE;
    }

    public IntakeAction(PowerCell.WantedState desiredState) {
        myDesiredState = desiredState;
        
    }

    @Override
    public boolean isFinished() {
        return false; //mIntake.hasCargo();
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        mPowerCell.setWantedState(PowerCell.WantedState.IDLE);

    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mPowerCell.setWantedState(myDesiredState);

    }
}
