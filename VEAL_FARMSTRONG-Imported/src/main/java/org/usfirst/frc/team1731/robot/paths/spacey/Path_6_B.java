package org.usfirst.frc.team1731.robot.paths.spacey;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class Path_6_B implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(297,71,0,60));
        sWaypoints.add(new Waypoint(150,71,20,60));
        sWaypoints.add(new Waypoint(150,31,20,60));
        sWaypoints.add(new Waypoint(17,31,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(297, 71), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":297,"y":71},"speed":60,"radius":0,"comment":""},{"position":{"x":150,"y":71},"speed":60,"radius":20,"comment":""},{"position":{"x":150,"y":31},"speed":60,"radius":20,"comment":""},{"position":{"x":17,"y":31},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Path_6_B
}