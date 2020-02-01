package org.usfirst.frc.team1731.robot.auto.actions;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.robot.paths.PathContainer;
import org.usfirst.frc.team1731.robot.subsystems.Drive;

import frc.robot.Robot;
import swervebot.Drivetrain;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 * 
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DrivePathAction implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private Drive mDrive = Drive.getInstance();
    private Drivetrain mSwerveDrive = Robot.m_swerve;

    public DrivePathAction(PathContainer p) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
    }

    @Override
    public boolean isFinished() {
        return (mDrive.isDoneWithPath() || mDrive.isTBFinished());
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
        // TODO: Perhaps set wheel velocity to 0?
    }

    @Override
    public void start() throws RuntimeException {
        mDrive.resetTractorBeam();
        throw new java.lang.RuntimeException("Not implemented");
        //mSwerveDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
        //mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
    }
}
