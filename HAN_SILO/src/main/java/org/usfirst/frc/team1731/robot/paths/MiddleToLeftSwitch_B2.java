package org.usfirst.frc.team1731.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class MiddleToLeftSwitch_B2 implements PathContainer {  
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        
        sWaypoints.add(new Waypoint(122, 222,  0, 0));
        sWaypoints.add(new Waypoint(100, 222, 10, 80));
        sWaypoints.add(new Waypoint( 80, 202,  0, 80));
        sWaypoints.add(new Waypoint( 60, 182, 10, 80));
        sWaypoints.add(new Waypoint( 30, 182,  0, 80));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(122, 222), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":280,"y":70},"speed":0,"radius":0,"comment":""},{"position":{"x":220,"y":85},"speed":30,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightScaleToRightSwitch
}