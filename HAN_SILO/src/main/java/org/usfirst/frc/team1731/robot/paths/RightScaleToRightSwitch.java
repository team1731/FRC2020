package org.usfirst.frc.team1731.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class RightScaleToRightSwitch implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(285,87,0,0));
        sWaypoints.add(new Waypoint(239,89,0,50));
        sWaypoints.add(new Waypoint(230,90,0,20));  // changed 234 to 230

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(285, 87), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":280,"y":70},"speed":0,"radius":0,"comment":""},{"position":{"x":220,"y":85},"speed":30,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightScaleToRightSwitch

	public static void main(String[] args) {
		
	}
}