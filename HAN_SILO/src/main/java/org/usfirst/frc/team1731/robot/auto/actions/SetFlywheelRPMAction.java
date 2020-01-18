package org.usfirst.frc.team1731.robot.auto.actions;


/**
 * Spins up the flywheel to a specified RPM in advance in order to save time later.
 * 
 * @see Action
 * @see RunOnceAction
 */
public class SetFlywheelRPMAction extends RunOnceAction {

    double rpm;

    public SetFlywheelRPMAction(double s) {
        rpm = s;
    }

    @Override
    public synchronized void runOnce() {
   //     Shooter.getInstance().setSpinUp(rpm);
    }
}
