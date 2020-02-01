package org.usfirst.frc.team1731.robot.auto.actions;

import org.usfirst.frc.team1731.robot.subsystems.Drive;

import frc.robot.Robot;

/**
 * Forces the current path the robot is driving on to end early
 * 
 * @see DrivePathAction
 * @see Action
 * @see RunOnceAction
 */
public class ForceEndPathAction extends RunOnceAction {

    @Override
    public synchronized void runOnce() throws RuntimeException {
        throw new RuntimeException("Not implemented");
        //Robot.m_swerve.forceDoneWithPath();
    }
}
