package org.usfirst.frc.team1731.robot.auto.modes;

import java.util.Arrays;

import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1731.robot.auto.actions.DrivePathAction;
import org.usfirst.frc.team1731.robot.auto.actions.ParallelAction;
import org.usfirst.frc.team1731.robot.auto.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamEjectHatchAction;
import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamParallelPickupHatchAction;
//import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamPickupHatchAction;
import org.usfirst.frc.team1731.robot.auto.actions.TurnToHeadingAction;
import org.usfirst.frc.team1731.robot.auto.actions.WaitAction;
//import org.usfirst.frc.team1731.robot.paths.LeftCargo1ToFeederStationPath1;
import org.usfirst.frc.team1731.robot.paths.LeftFeedStationToRocketFrontPath1;
import org.usfirst.frc.team1731.robot.paths.LeftLevel1ToRocketRearPath1;
//import org.usfirst.frc.team1731.robot.paths.LeftRocketFrontToFeedStationPath1;
//import org.usfirst.frc.team1731.robot.paths.LeftRocketFrontToFeedStationPath2;
import org.usfirst.frc.team1731.robot.paths.LeftRocketRearToFeedStationPath1;
import org.usfirst.frc.team1731.robot.paths.LeftRocketRearToFeedStationPath2;
import org.usfirst.frc.team1731.robot.paths.PathContainer;
import org.usfirst.frc.team1731.robot.subsystems.Drive;
import org.usfirst.frc.team1731.robot.auto.actions.Action;


/**
 * Scores the preload gear onto the boiler-side peg then deploys the hopper and shoots all 60 balls (10 preload + 50
 * hopper).
 * 
 * This was the primary autonomous mode used at SVR, St. Louis Champs, and FOC.
 * 
 * @see AutoModeBase
 */
public class LeftLevel1ToRocketRearMode extends MirrorableMode {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Executing LeftLevel1ToRocketRearMode");

        Drive.getInstance().setGyroAngle( Rotation2d.fromDegrees(180.0));
        PathContainer Path = new LeftLevel1ToRocketRearPath1();
        runAction(new ResetPoseFromPathAction(Path));
        runAction(new DrivePathAction(Path));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(getAngle(150.0))));
        runAction(new TractorBeamEjectHatchAction());
        runAction(new WaitAction(0.5));
        Path = new LeftRocketRearToFeedStationPath1();
        runAction(new ResetPoseFromPathAction(Path));
        runAction(new DrivePathAction(Path));
        Path = new LeftRocketRearToFeedStationPath2();
        
        runAction(new ParallelAction(Arrays.asList(new Action[] {
            new TractorBeamParallelPickupHatchAction(), 
            new DrivePathAction(Path)
        })));
       
       // runAction(new DrivePathAction(Path));
      //  runAction(new TractorBeamPickupHatchAction());

        runAction(new WaitAction(0.5));
        Path = new LeftFeedStationToRocketFrontPath1();
        runAction(new ResetPoseFromPathAction(Path));
        runAction(new DrivePathAction(Path));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(getAngle(30.0))));
        runAction(new TractorBeamEjectHatchAction());

      //  runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0.0)));
      //  Path = new LeftFeedStationToRocketFrontPath2();
      //  runAction(new DrivePathAction(Path));

    }
}
