package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Utils;

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

    /**
     * @param oldStates -- original list of states from the (calculated) trajectory
     * @param finalRotationDegrees -- desired final pose rotation -- to assign to last state in list
     * @return -- list of fixed-up (unrotated) states (except for the last one in the list)
     */
    List<Trajectory.State> unrotateTrajectory(List<Trajectory.State> oldStates, double finalRotationDegrees){
        List<Trajectory.State> newStates = new ArrayList<Trajectory.State>();
        int i = 0;
        for(Trajectory.State state : oldStates){
          //instead of rotating the pose by its inverse (dumb)...
          //  Rotation2d newRot = state.poseMeters.getRotation().rotateBy(new Rotation2d(-state.poseMeters.getRotation().getRadians()));
          //simply assign a new Rotation having 0 degrees...
          double rotationDegrees = i++ == oldStates.size()-1 ? finalRotationDegrees : 0;
          Pose2d newPose = new Pose2d(state.poseMeters.getTranslation(), new Rotation2d(Math.toRadians(rotationDegrees)));
          newStates.add(new Trajectory.State(state.timeSeconds, 
                                            state.velocityMetersPerSecond, 
                                            state.accelerationMetersPerSecondSq, 
                                            newPose, 
                                            state.curvatureRadPerMeter));
        }
        return newStates;
    }

    /**
     * @param oldStates -- original list of states from the (calculated) trajectory
     * @param rotationDegrees -- desired final pose rotation -- to assign to last state in list
     * @return -- list of fixed-up (unrotated) states (except for the last one in the list)
     */
    List<Trajectory.State> maintainTrajectory(List<Trajectory.State> oldStates, double rotationDegrees){
        List<Trajectory.State> newStates = new ArrayList<Trajectory.State>();
        for(Trajectory.State state : oldStates){
          //simply assign a new Rotation having rotationDegrees degrees...
          Pose2d newPose = new Pose2d(state.poseMeters.getTranslation(), new Rotation2d(Math.toRadians(rotationDegrees)));
          newStates.add(new Trajectory.State(state.timeSeconds, 
                                            state.velocityMetersPerSecond, 
                                            state.accelerationMetersPerSecondSq, 
                                            newPose, 
                                            state.curvatureRadPerMeter));
        }
        return newStates;
    }

    enum TrajectoryDirection {
        FWD,
        REV
    }
    
    public TrajectoryConfig createTrajectoryConfig(TrajectoryDirection dir){
        switch(dir){
            case FWD: return new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(false);
            case REV: return new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(true);
        }
        return null;
    }

    enum TrajectoryHeading {
        UNROTATE,
        MAINTAIN,
        DO_NOTHING
    }

    public Trajectory createTrajectory(String name, TrajectoryDirection dir, TrajectoryHeading mode, double value, double[][] points){
        // waypoints
        List<Translation2d> waypoints = new ArrayList<Translation2d>();
        if(points.length > 2){
            for(int i=1; i<points.length-1; i++){
                waypoints.add(new Translation2d(points[i][0], points[i][1]));
            }
        }

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // initial pose
            new Pose2d(points[0][0], points[0][1], new Rotation2d(Math.toRadians(points[0][2]))),
            waypoints,
            // ending pose
            new Pose2d(points[points.length-1][0], points[points.length-1][1], new Rotation2d(Math.toRadians(points[points.length-1][2]))),
            createTrajectoryConfig(dir));

        switch(mode){
            case UNROTATE: trajectory = new Trajectory(unrotateTrajectory(trajectory.getStates(), value)); break;
            case MAINTAIN: trajectory = new Trajectory(maintainTrajectory(trajectory.getStates(), value)); break;
            case DO_NOTHING: // do not alter trajectory
        }
        Utils.printTrajectory(this.getClass().getSimpleName() + ": " + name, trajectory);
        return trajectory;
    }

    public _InstrumentedSwerveControllerCommand createSwerveCommand(DriveSubsystem m_robotDrive, String name, TrajectoryDirection dir, TrajectoryHeading mode, double value, double[][] points){
        return new _InstrumentedSwerveControllerCommand(
            m_robotDrive.getCSVWriter(),
            createTrajectory(name, dir, mode, value, points),
            m_robotDrive::getPose, //Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            //Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                    AutoConstants.kThetaControllerConstraints),

            m_robotDrive::setModuleStates,
            m_robotDrive
        )
        {
            @Override
            public void end(boolean interrupted) {
              super.end(interrupted);
              System.out.println("at end of swerve command, interrupted=" + interrupted);
            }
        };
    }

}
