package frc.robot.commands;

import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

public class ShootSeqCommandAuto extends ShootSeqCommand {
    private SequencerSubsystem sequenceSubsystem;

    public ShootSeqCommandAuto(ShootClimbSubsystem shootClimbSubsystem, SequencerSubsystem sequenceSubsystem) {
        super(shootClimbSubsystem, sequenceSubsystem);
        this.sequenceSubsystem = sequenceSubsystem;
    }

    @Override
    public boolean isFinished(){
        return sequenceSubsystem.getPowerCellCount() == 0;
    }
}