package org.usfirst.frc.team1731.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class LeftCargo1ToFeederStationPath2 extends MirrorablePath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(287, getY(248), 0,   0));
        sWaypoints.add(new Waypoint(230, getY(240), 0, 120));
        sWaypoints.add(new Waypoint( 130, getY(295), 0, 110,"StartTractorBeam"));
        sWaypoints.add(new Waypoint( 90, getY(295), 0, 90));

     //   sWaypoints.add(new Waypoint( 19, getY(295), 0,  80));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(287,  getY(248)), Rotation2d.fromDegrees(getAngle(28))); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":19,"y":295},"speed":60,"radius":0,"comment":""},{"position":{"x":55,"y":295},"speed":0,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Path_1
}