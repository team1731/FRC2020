package frc.robot.vision;

public class LimeTargetInfo {

    private double x = 1.0;
    private double y;
    private double z;
    private double timestamp;

    public LimeTargetInfo(double y, double z, double timestamp){
        this.y = y;
        this.z = z;
        this.timestamp = timestamp;
    }

    /**
     * Returns the X of the target in robot coordinates (Z in target coordinates).
     * This is locked at 1.
     * @return
     */
    public double getX(){
        return x;
    }

    /**
     * Returns the Y of the target in robot coordinates (X in target coordinates).
     * @return
     */
    public double getY(){
        return y;
    }

    /**
     * Returns the Z of the target in robot coordinates (Y in target coordinates).
     * @return
     */
    public double getZ(){
        return z;
    }

    /**
     * Returns the timestamp when this data was captured
     * @return
     */
    public double getTimeCaptured(){
        return timestamp;
    }

}