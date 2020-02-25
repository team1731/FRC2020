package frc.robot.commands;

import frc.robot.subsystems.LedStringSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

public class ShootSeqCommandAuto extends ShootSeqCommand {
    private ShootClimbSubsystem shootSubsystem;
    private SequencerSubsystem seqSubsystem;

    public ShootSeqCommandAuto(LedStringSubsystem m_ledstring, ShootClimbSubsystem shoot, SequencerSubsystem sequence) {
        super(m_ledstring, shoot, sequence);
        this.shootSubsystem = shoot;
        this.seqSubsystem = sequence;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        seqSubsystem.stop();
        //shootSubsystem.stopShooting();
        //shootSubsystem.hoodRetract();
    }

    @Override
    public boolean isFinished(){
        return seqSubsystem.getPowerCellCount() == 0;
    }
}