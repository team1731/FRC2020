package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

public class _NamedAutoMode {
    public String name;
    public _DelayableStrafingAutoMode delayableStrafingAutoMode;
    
    public _NamedAutoMode(String name, _DelayableStrafingAutoMode delayableStrafingAutoMode) {
        this.name = name;
        this.delayableStrafingAutoMode = delayableStrafingAutoMode;
    }

    public Command getCommand(){
        return delayableStrafingAutoMode.getCommand();
    }
}
