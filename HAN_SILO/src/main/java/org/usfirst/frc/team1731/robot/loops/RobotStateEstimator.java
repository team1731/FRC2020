package org.usfirst.frc.team1731.robot.loops;

import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Twist2d;
import org.usfirst.frc.team1731.robot.Kinematics;
import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.subsystems.Drive;

import frc.robot.Robot;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    RobotState robot_state_ = RobotState.getInstance();
    Drive drive_ = Drive.getInstance();
    SwerveDrive swerveDrive_ = Robot.DRIVE.getSwerveInstance();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;

    @Override
    public synchronized void onStart(double timestamp) {
        left_encoder_prev_distance_ = swerveDrive_.getLeftDistanceInches();
        right_encoder_prev_distance_ = swerveDrive_.getRightDistanceInches();
    }

   // private ControlBoardInterface mControlBoard = GamepadControlBoard.getInstance();

    @Override
    public synchronized void onLoop(double timestamp) {
        //#region Original Estimator Code
        final double left_distance = swerveDrive_.getLeftDistanceInches();
        final double right_distance = swerveDrive_.getRightDistanceInches();
        final Rotation2d gyro_angle = swerveDrive_.getGyroAngle();
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(swerveDrive_.getLeftVelocityInchesPerSec(),
                swerveDrive_.getRightVelocityInchesPerSec());
        robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;

        //#endregion
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

}
