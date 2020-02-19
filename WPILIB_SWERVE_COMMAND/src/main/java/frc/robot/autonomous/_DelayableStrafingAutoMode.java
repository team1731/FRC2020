package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class _DelayableStrafingAutoMode {
    private int initialDelaySeconds;
    private int secondaryDelaySeconds;
    Command command;
  
    public _DelayableStrafingAutoMode(int initialDelaySeconds, int secondaryDelaySeconds) {
        this.initialDelaySeconds = initialDelaySeconds;
        this.secondaryDelaySeconds = secondaryDelaySeconds;
    }

    public _DelayableStrafingAutoMode() {
        this(0, 0);
    }

    public void setInitialDelaySeconds(int initialDelaySeconds){
        this.initialDelaySeconds = initialDelaySeconds;
    }
  
    public void setSecondaryDelaySeconds(int secondaryDelaySeconds){
        this.secondaryDelaySeconds = secondaryDelaySeconds;
    }

    public int getInitialDelaySeconds(){
        return initialDelaySeconds;
    }
 
    public int getSecondaryDelaySeconds(){
        return secondaryDelaySeconds;
    }

    public Command getCommand(){
        return command;
    }

    List<Trajectory.State> unrotateTrajectory(List<Trajectory.State> oldStates, double finalRotationDegrees){
        List<Trajectory.State> newStates = new ArrayList<Trajectory.State>();
        int i = 0;
        for(Trajectory.State state : oldStates){
          //Rotation2d newRot = state.poseMeters.getRotation().rotateBy(new Rotation2d(-state.poseMeters.getRotation().getRadians()));
          Pose2d newPose = i++ == oldStates.size()-1 ? 
          new Pose2d(state.poseMeters.getTranslation(), new Rotation2d(Math.toRadians(finalRotationDegrees))) : 
          new Pose2d(state.poseMeters.getTranslation(), new Rotation2d(0));

          newStates.add(new Trajectory.State(state.timeSeconds, 
                                            state.velocityMetersPerSecond, 
                                            state.accelerationMetersPerSecondSq, 
                                            newPose, 
                                            state.curvatureRadPerMeter));
        }
        return newStates;
    }
}
