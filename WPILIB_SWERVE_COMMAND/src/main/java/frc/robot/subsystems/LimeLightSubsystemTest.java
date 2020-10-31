package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightSubsystemTest extends SubsystemBase {

    NetworkTable limeTable;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    /*

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


    public LimeLightSubsystemTest(){
        limeTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limeTable.getEntry("tx");
        ty = limeTable.getEntry("ty");
        ta = limeTable.getEntry("ta");

        SmartDashboard.putNumber("LimeX", 0);
        SmartDashboard.putNumber("LimeY", 0);
        SmartDashboard.putNumber("LimeArea", 0);
    }

    @Override
    public void periodic() {
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        SmartDashboard.putNumber("LimeX", x);
        SmartDashboard.putNumber("LimeY", y);
        SmartDashboard.putNumber("LimeArea", area);
    }

}