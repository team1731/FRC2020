package frc.robot.subsystems;

import frc.robot.vision.JevoisVisionUpdate;
import edu.wpi.first.wpilibj.Timer;
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
    static JevoisVisionSubsystem instance_ = new JevoisVisionSubsystem();
    JevoisVisionUpdate update_ = null;
    //RobotState robot_state_ = RobotState.getInstance();

    public static JevoisVisionSubsystem getInstance() {
        return instance_;
    }

    public JevoisVisionSubsystem() {
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

}
