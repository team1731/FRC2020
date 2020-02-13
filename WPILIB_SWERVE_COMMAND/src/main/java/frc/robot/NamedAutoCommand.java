package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class NamedAutoCommand {
    public String name;
    public Command command;
    
    public NamedAutoCommand(String name, Command command){
        this.name = name;
        this.command = command;
    }
}
