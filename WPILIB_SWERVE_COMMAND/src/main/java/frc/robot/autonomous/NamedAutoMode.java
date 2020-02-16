package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

public class NamedAutoMode {
    public String name;
    public DelayableAutoMode delayableAutoMode;
    
    public NamedAutoMode(String name, DelayableAutoMode delayableAutoMode){
        this.name = name;
        this.delayableAutoMode = delayableAutoMode;
    }

    public Command getCommand(){
        return delayableAutoMode.getCommand();
    }
}
