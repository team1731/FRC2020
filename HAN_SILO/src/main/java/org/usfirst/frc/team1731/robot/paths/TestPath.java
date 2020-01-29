package org.usfirst.frc.team1731.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

/*
You may notice several comments in all caps surrounded by brackets. Please do not change/remove these. They are used by
CheesyPath to import the path into the CheesyPath editor. Also, ensure the spacing and tabs are the same as the editor also
assumes the same spacing when importing. If these are altered, CheesyPath may attempt to import using the JSON string at the
bottom of the path.

NOTICE: At the time of this path's creation, this feature was not implemented yet.
*/

//[TITLE]
public class TestPath implements PathContainer {
    
    @Override
    public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		//[PATHINIT]
        sWaypoints.add(new Waypoint(50,50,0,0,0));
        sWaypoints.add(new Waypoint(60,50,0,60,0));
        //sWaypoints.add(new Waypoint(60,60,0,60,0));
        //sWaypoints.add(new Waypoint(100,50,0,60,0));
        //sWaypoints.add(new Waypoint(100,100,0,60,90));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
		//[STARTPOINT]
        return new RigidTransform2d(new Translation2d(50, 50), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
		//[ISREVERSED]
        return false; 
	}
	
	// WAYPOINT_DATA: [{"position":{"x":50,"y":50},"speed":0,"radius":0,"azimuth":0,"comment":""},{"position":{"x":100,"y":50},"speed":60,"radius":0,"azimuth":0,"comment":""},{"position":{"x":100,"y":100},"speed":60,"radius":0,"azimuth":90,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: TestPath
}