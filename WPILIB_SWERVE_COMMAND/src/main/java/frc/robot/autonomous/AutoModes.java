package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

@Deprecated() //"something goofy about this class -- DO NOT USE!!"
public class AutoModes {
    //Position controllers
    static PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    static PIDController yController = new PIDController(-AutoConstants.kPYController, 0, 0);
    static ProfiledPIDController thetaController = new ProfiledPIDController(
                                 AutoConstants.kPThetaController, 0, 0,
                                 AutoConstants.kThetaControllerConstraints);
    
    // Create config for trajectory
    // The reversed property simply represents whether the robot is traveling backward.
    // If you specify four waypoints, a, b, c, and d, the robot will still travel in the
    // same order through the waypoints when the reversed flag is set to true. This also
    // means that you must account for the direction of the robot when providing the
    // waypoints. For example, if your robot is facing your alliance station wall and
    // travels backwards to some field element, the starting waypoint should have a
    // rotation of 180 degrees.

    public static TrajectoryConfig forwardConfig = 
    new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // robot is going backwards for this auto
        .setReversed(false)
        // if we need to constrain the trajectory at some point(s)
        //.addConstraint(new AutoTrajectoryConstraint())
    ;

    public static TrajectoryConfig backwardConfig = 
    new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // robot is going backwards for this auto
        .setReversed(false)
        // if we need to constrain the trajectory at some point(s)
        //.addConstraint(new AutoTrajectoryConstraint())
    ;



    // An example trajectory to follow.  All units in meters.
    //
    public static Trajectory _2MetersFwdTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the -X direction (robot facing alliance station wall)
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
            // Pass through this interior waypoint
            List.of(
              //new Translation2d(1, 0.5)
            ),
            // End 2 meters straight ahead of where we started, facing forward
            new Pose2d(2, 1, new Rotation2d(Units.degreesToRadians(0))),
            forwardConfig
        );
    
    public static Trajectory _3MetersBwdSineTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the -X direction (robot facing alliance station wall)
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
              new Translation2d(1, 1),
              new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(Units.degreesToRadians(0))),
            backwardConfig
        );

    public static Trajectory  _2BallPickupBwdTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the -X direction (robot facing alliance station wall)
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
            // Pass through these 3 interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1.48,-2.28),
                new Translation2d(3.77,-3.12),
                new Translation2d(-4.14,2.54)
            ),
            new Pose2d(-6.41,2.79, new Rotation2d(0)),
            backwardConfig
        );

    public static Trajectory  _2BallPickupBwdTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the -X direction (robot facing alliance station wall)
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these 3 interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(-1.59,-0.063),
                new Translation2d(-3.01,2.39)
            ),
            new Pose2d(-7.65,2.8, new Rotation2d(0)),
            backwardConfig
        );

    public static TrajectoryString[] trajectoryStrings = {
        new TrajectoryString(null,                      "Do Nothing"), 
        new TrajectoryString(_2MetersFwdTrajectory,     "2 Meters Fwd"), 
        new TrajectoryString(_3MetersBwdSineTrajectory, "3 Meters Bwd Sine"), 
        new TrajectoryString(_2BallPickupBwdTrajectory2, "2-Ball Pickup Bwd")
    };

    public static class TrajectoryString {
        Trajectory t;
        String s;
        public TrajectoryString(Trajectory t, String s){
            this.t = t;
            this.s = s;
        }
    }

    public static Trajectory getTrajectory(int autoNum){
        if(autoNum >= trajectoryStrings.length){
            return null;
        }
        return trajectoryStrings[autoNum].t;
    }

    public static String getName(int autoNum){
        if(autoNum >= trajectoryStrings.length){
            return "UNDEFINED";
        }
        return trajectoryStrings[autoNum].s;
    }

    public static SwerveControllerCommand getCommand(DriveSubsystem m_robotDrive, int autoNum) { 
        Trajectory trajectory = getTrajectory(autoNum);
        return trajectory == null ? null : new SwerveControllerCommand(
            trajectory, 
            m_robotDrive::getPose, //Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            xController, yController, thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
    }
}