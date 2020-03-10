package frc.robot.util;

  public class AutoSwerveDebug {
    public double curTime;
    public double desiredX;
    public double desiredY;
    public double desiredTheta;
    public double actualX;
    public double actualY;
    public double actualTheta;

  public AutoSwerveDebug(
    double curTime,
    double desiredX,
    double desiredY,
    double desiredTheta,
    double actualX,
    double actualY,
    double actualTheta){
        this.curTime = curTime;
        this.desiredX = desiredX;
        this.desiredY = desiredY;
        this.desiredTheta = desiredTheta;
        this.actualX = actualX;
        this.actualY = actualY;
        this.actualTheta = actualTheta;
    }
}
