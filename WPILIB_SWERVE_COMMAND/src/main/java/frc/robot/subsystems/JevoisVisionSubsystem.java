package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.vision.GoalTracker;
import frc.robot.vision.JevoisVisionServer;
import frc.robot.vision.JevoisVisionUpdate;
import frc.robot.vision.TargetInfo;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This function adds vision updates (from the Nexus smartphone) to a list in RobotState. This helps keep track of goals
 * detected by the vision system. The code to determine the best goal to shoot at and prune old Goal tracks is in
 * GoalTracker.java
 * 
 * @see GoalTracker.java
 */
public class JevoisVisionSubsystem extends SubsystemBase {
    private static JevoisVisionSubsystem instance_ = new JevoisVisionSubsystem();
    private JevoisVisionUpdate update_ = null;
    private JevoisVisionServer m_VisionServer = JevoisVisionServer.getInstance();

    private GoalTracker goal_tracker_;
    //RobotState robot_state_ = RobotState.getInstance();

    public static JevoisVisionSubsystem getInstance() {
        return instance_;
    }

    public JevoisVisionSubsystem() {
        goal_tracker_ = new GoalTracker();
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getFPGATimestamp();
        JevoisVisionUpdate update;
        synchronized (this) {
            if (update_ == null) {
                return;
            }
            update = update_;
            update_ = null;
        }

     //   SmartDashboard.putString("JevoisVisionProcessorUpdate", "update was captured at "+update.getCapturedAtTimestamp());
        //robot_state_.addVisionUpdate(update.getCapturedAtTimestamp(), update.getTargets());
    }
    int updateCounter = 0;
   
    public synchronized void gotUpdate(JevoisVisionUpdate update) {
        updateCounter++;
     //   SmartDashboard.putString("JevoisVisionProcessorGotUpdate", "Got Update: "+updateCounter);
        update_ = update;
    }

    /**
     * From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
     * Normalizing forces us to re-scale the sin and cos to reset rounding errors.
     */
    public Rotation2d normalize(Rotation2d rotation) {
        //cosangle sinangle
        double cos_angle_ = rotation.getCos();
        double sin_angle_ = rotation.getSin();

        double magnitude = Math.hypot(cos_angle_, sin_angle_);
        if (magnitude > VisionConstants.kEpsilon) {
            sin_angle_ /= magnitude;
            cos_angle_ /= magnitude;
        } else {
            sin_angle_ = 0;
            cos_angle_ = 1;
        }

        return new Rotation2d(cos_angle_, sin_angle_);
    }

    public void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {
        List<Translation2d> field_to_goals = new ArrayList<>();
        Pose2d field_to_camera = null;
        Rotation2d camera_pitch_correction_ = Rotation2d.fromDegrees(-VisionConstants.kCameraPitchAngleDegrees);;
        Rotation2d camera_yaw_correction_ = Rotation2d.fromDegrees(-VisionConstants.kCameraYawAngleDegrees);;
        double differential_height_ = VisionConstants.kBoilerTargetTopHeight - VisionConstants.kCameraZOffset;

        //RigidTransform2d field_to_camera = getFieldToCamera(timestamp);
        if (!(vision_update == null || vision_update.isEmpty())) {
            for (TargetInfo target : vision_update) {
                
                double ydeadband = (target.getY() > -VisionConstants.kCameraDeadband
                        && target.getY() < VisionConstants.kCameraDeadband) ? 0.0 : target.getY();

                // Compensate for camera yaw
                double xyaw = target.getX() * camera_yaw_correction_.getCos() + ydeadband * camera_yaw_correction_.getSin();
                double yyaw = ydeadband * camera_yaw_correction_.getCos() - target.getX() * camera_yaw_correction_.getSin();
                double zyaw = -target.getZ();

                // Compensate for camera pitch
                double xr = zyaw * camera_pitch_correction_.getSin() + xyaw * camera_pitch_correction_.getCos();
                double yr = yyaw;
                double zr = zyaw * camera_pitch_correction_.getCos() - xyaw * camera_pitch_correction_.getSin();

                // find intersection with the goal
                if (zr > 0) {
                    double scaling = differential_height_ / zr;
                 //   System.out.println("zr: "+zr);
                    double distance = Math.hypot(xr, yr) * scaling;
                    Rotation2d angle = normalize(new Rotation2d(xr, yr));
                    SmartDashboard.putString("RobotState_distance/angle", "Distance: "+distance+" angle: "+angle);
    //                System.out.println("RobotState_distance/angle Distance: "+distance+" angle: "+angle);
                    angle = angle.rotateBy(Rotation2d.fromDegrees(-1.5));
                   
                    

                   field_to_camera = field_to_camera.transformBy(new Transform2d(new Translation2d(distance * angle.getCos(), distance * angle.getSin()), new Rotation2d()));
                   
                   field_to_goals.add(field_to_camera.getTranslation());
                }
            }
        }
        synchronized (this) {
            goal_tracker_.update(timestamp, field_to_goals);
        }
    }

    /**
     * Writes "SEND" to the vision camera. 
     * This should start the flood of JSON strings determining the center point of the target of interest
     * @return A boolean determining the success of the write
     */
    public boolean StartCameraDataStream(){
        if(m_VisionServer.getVisionCam() != null){
            m_VisionServer.getVisionCam().writeString("SEND");
            return true;
        }
        
        return false;
    }

    /**
     * Writes "STOP" to the vision camera.
     * This should stop the flood of serial outputs coming from the vision camera
     * @return A boolean determining the success of the write
     */
    public boolean StopCameraDataStream(){
        if(m_VisionServer.getVisionCam() != null){
            m_VisionServer.getVisionCam().writeString("STOP");
            return true;
        }

        return false;
    }

}
