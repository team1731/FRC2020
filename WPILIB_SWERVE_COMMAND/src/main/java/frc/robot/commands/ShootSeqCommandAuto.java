package frc.robot.commands;

import frc.robot.subsystems.SequencerSubsystem;

public class ShootSeqCommandAuto extends ShootSeqCommand {
    private SequencerSubsystem sequenceSubsystem;

    public ShootSeqCommandAuto(SequencerSubsystem sequenceSubsystem) {
        super(sequenceSubsystem);
        this.sequenceSubsystem = sequenceSubsystem;
    }

    @Override
    public boolean isFinished(){
        return sequenceSubsystem.getPowerCellCount() == 0;
    }
}