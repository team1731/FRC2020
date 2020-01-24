package org.usfirst.frc.team1731.robot.auto.modes;

import java.util.Arrays;

import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1731.robot.auto.actions.*;
import org.usfirst.frc.team1731.robot.paths.*;

import edu.wpi.first.wpilibj.Timer;

/**
 * Scores the preload gear onto the boiler-side peg then deploys the hopper and shoots all 60 balls (10 preload + 50
 * hopper).
 * 
 * This was the primary autonomous mode used at SVR, St. Louis Champs, and FOC.
 * 
 * @see AutoModeBase
 */
public class MiddleRightPutCubeOnRightSwitch extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
    	System.out.println("Executing MiddleRightPutCubeOnRightSwitch ");
    	PathContainer Path = new MiddleToRightSwitch();
    	runAction(new ResetPoseFromPathAction(Path));
        runAction(new ParallelAction(Arrays.asList(new Action[] {
        		new ElevatorHome(),
        		new RotateIntakeActionUp(false),
        		new DrivePathAction(Path),
        })));

    	runAction(new SpitAction());
    	
//    	PathContainer Path2 = new MiddleToRightSwitch_B();
//    	runAction(new DrivePathAction(Path2));
//    	
//    	PathContainer Path3 = new MiddleToRightSwitch_C();
//    	runAction(new ParallelAction(Arrays.asList(new Action[] {
//    			 new DrivePathAction(Path3),
//    			 new PickUpAction()
//    	 })));
//    	runAction(new ElevatorDown());
//    	
//    	PathContainer Path4 = new MiddleToRightSwitch_D();
//    	runAction(new DrivePathAction(Path4));
//    	
//    	runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180.0)));
//    	
//    	PathContainer Path5 = new MiddleToRightSwitch_E();
//    	runAction(new DrivePathAction(Path5));
//    	
//    	runAction(new SpitAction());
    }
}