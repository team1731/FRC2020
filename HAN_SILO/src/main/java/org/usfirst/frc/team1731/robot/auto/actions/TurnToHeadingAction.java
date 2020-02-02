package org.usfirst.frc.team1731.robot.auto.actions;

import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.robot.subsystems.Drive;

import frc.robot.Robot;
import swervebot.Drivetrain;

/**
 * Turns the robot to a specified heading
 * 
 * @see Action
 */
public class TurnToHeadingAction implements Action {

    private Rotation2d mTargetHeading;
    private Drive mDrive = Drive.getInstance();
    private Drivetrain mSwerveDrive = Robot.m_swerve;

    public TurnToHeadingAction(Rotation2d heading) {
        mTargetHeading = heading;
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithTurn();
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
    }

    @Override
    public void start() throws RuntimeException {
        //mDrive.setWantTurnToHeading(mTargetHeading);
        mSwerveDrive.setWantTurnToHeading(mTargetHeading);
    }
}
