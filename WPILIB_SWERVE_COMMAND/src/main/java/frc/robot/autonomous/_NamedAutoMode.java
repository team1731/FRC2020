package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

public class _NamedAutoMode {
    public String name = "UNKNOWN";
    public _DelayableStrafingAutoMode delayableStrafingAutoMode;
    
    public _NamedAutoMode(_DelayableStrafingAutoMode delayableStrafingAutoMode)
            throws _NotImplementedProperlyException {
        if(delayableStrafingAutoMode != null){
            if(delayableStrafingAutoMode instanceof _DelayableStrafingAutoMode){
                name = delayableStrafingAutoMode.getClass().getSimpleName();
                if(name.length() < 3){
                    throw new _NotImplementedProperlyException();
                }
                if(!Character.isAlphabetic(name.charAt(0)) || !Character.isUpperCase(name.charAt(0))){
                    throw new _NotImplementedProperlyException();
                }
                if(!Character.isDigit(name.charAt(1))){
                    throw new _NotImplementedProperlyException();
                }
                if(name.charAt(2) != '_'){
                    throw new _NotImplementedProperlyException();
                }
            }
            else {
                throw new _NotImplementedProperlyException("You must supply a _DelayableStrafingAutoMode object!");
            }
        }
        else{
            throw new _NotImplementedProperlyException("You must supply a non-null _DelayableStrafingAutoMode!");
        }
        this.delayableStrafingAutoMode = delayableStrafingAutoMode;
    }

    public Command getCommand(){
        return delayableStrafingAutoMode.getCommand();
    }
}
