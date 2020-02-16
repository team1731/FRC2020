package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class DelayableAutoMode {
    private int initialDelaySeconds;
    private int secondaryDelaySeconds;
  
    public DelayableAutoMode(int initialDelaySeconds, int secondaryDelaySeconds) {
        this.initialDelaySeconds = initialDelaySeconds;
        this.secondaryDelaySeconds = secondaryDelaySeconds;
    }

    public DelayableAutoMode() {
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

    public abstract Command getCommand();
}
