package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.vision.GoalTracker;
import frc.robot.vision.JevoisVisionServer;
import frc.robot.vision.JevoisVisionUpdate;
import frc.robot.vision.LimeTargetInfo;
import frc.robot.vision.ShooterAimingParameters;
import frc.robot.vision.TargetInfo;
import frc.robot.vision.GoalTracker.TrackReport;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Example code for limelight
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableEntry;
    import edu.wpi.first.networktables.NetworkTableInstance;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
*/

/**
 * This function adds vision updates (from the Nexus smartphone) to a list in RobotState. This helps keep track of goals
 * detected by the vision system. The code to determine the best goal to shoot at and prune old Goal tracks is in
 * GoalTracker.java
 * 
 * @see GoalTracker.java
 */
public class LimeLightSubsystem extends SubsystemBase {

    private NetworkTable limeTable;
    private NetworkTableEntry limeTX;
    private NetworkTableEntry limeTY;
    private NetworkTableEntry limeValidTargets;
    private NetworkTableEntry limeLED;

    private LimeTargetInfo lastTarget;

    public LimeLightSubsystem() {
        limeTable = NetworkTableInstance.getDefault().getTable("limelight");
        limeTX = limeTable.getEntry("tx");
        limeTY = limeTable.getEntry("ty");
        limeValidTargets = limeTable.getEntry("tv");
        limeLED = limeTable.getEntry("ledMode");

        disableLED();
    }

    @Override
    public void periodic() {
        if(hasTarget()){
            lastTarget = new LimeTargetInfo(limeTX.getDouble(0), limeTY.getDouble(0), Timer.getFPGATimestamp());
        }
    }

    public LimeTargetInfo getLastTarget(){
        return lastTarget;
    }

    public boolean hasTarget(){
        return limeValidTargets.getDouble(0) > 0;
    }

    public void enableLED(){
        limeLED.setNumber(3);
    }

    public void disableLED(){
        limeLED.setNumber(1);
    }
}
