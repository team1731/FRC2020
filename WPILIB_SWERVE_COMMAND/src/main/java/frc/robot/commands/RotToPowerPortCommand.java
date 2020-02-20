package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.vision.ShooterAimingParameters;

public class RotToPowerPortCommand extends CommandBase {

    private final JevoisVisionSubsystem m_vision;
    private final DriveSubsystem m_drive;
    private final XboxController m_driverController;

    public RotToPowerPortCommand(JevoisVisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem, XboxController driveController){
        m_vision = visionSubsystem;
        m_drive = driveSubsystem;
        m_driverController = driveController;
    }

    @Override
    public void initialize(){
        //Doing this from teleopInit and autonomousInit. May want to do this here instead...
        //m_vision.StartCameraDataStream();
        m_drive.setStickControlledHeading(false);
    }

    @Override
    public void execute(){
        
        ShooterAimingParameters aimParams = m_vision.getAimingParameters().get();

        if(aimParams != null){
            m_drive.setHeadingControllerGoal(m_drive.getHeading()+aimParams.getRobotToGoal().getDegrees());
        }
    }

    @Override
    public void end(boolean interrupted){
        //Doing this from disabledInit. May want to do this here instead...
        //m_vision.StopCameraDataStream();
        m_drive.setStickControlledHeading(true);
    }

    @Override
    public boolean isFinished(){
        //When the right bumper is released, stop the command
        return !m_driverController.getRawButton(6);
    } 

}