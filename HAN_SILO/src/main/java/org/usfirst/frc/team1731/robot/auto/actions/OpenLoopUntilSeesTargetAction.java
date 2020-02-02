package org.usfirst.frc.team1731.robot.auto.actions;

import java.util.Optional;

import org.usfirst.frc.team1731.lib.util.DriveSignal;
import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.ShooterAimingParameters;
//import org.usfirst.frc.team1731.robot.subsystems.LED;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

/**
 * Runs the drivebase in open loop until the robot sees the boiler.
 *
 * @see Action
 */
public class OpenLoopUntilSeesTargetAction implements Action {

    RobotState mState = RobotState.getInstance();
    double left;
    double right;

    public OpenLoopUntilSeesTargetAction(double left, double right) {
        this.left = left;
        this.right = right;
    }

    public boolean isFinished() {
        double now = Timer.getFPGATimestamp();
        Optional<ShooterAimingParameters> aimParams = mState.getAimingParameters();
        if (aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5) {
            return true;
        }
        return false;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub

    }

    @Override
    public void done() {
        // TODO Auto-generated method stub

    }

    @Override
    public void start() throws RuntimeException {
        System.err.println("OpenLoopUntilSeesTargetAction does not do anything");
        //LED.getInstance().setWantedState(LED.WantedState.FIND_RANGE);
        //Robot.m_swerve.setOpenLoop(new DriveSignal(left, right));
        //Drive.getInstance().setOpenLoop(new DriveSignal(left, right));
    }

}
