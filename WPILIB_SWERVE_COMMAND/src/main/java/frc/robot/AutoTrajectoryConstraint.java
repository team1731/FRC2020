package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

public class AutoTrajectoryConstraint implements TrajectoryConstraint {

    @Override
    public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
            double velocityMetersPerSecond) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter,
            double velocityMetersPerSecond) {
        // TODO Auto-generated method stub
        return null;
    }

}
