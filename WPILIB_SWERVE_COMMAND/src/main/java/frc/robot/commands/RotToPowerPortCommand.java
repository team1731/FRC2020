package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.XboxConstants;
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
        m_drive.setStickControlledHeading(false);
        m_vision.StartCameraDataStream();
    }

    @Override
    public void execute(){
        
        Optional<ShooterAimingParameters> aimParams = m_vision.getAimingParameters();

        if(!aimParams.isEmpty() && aimParams != null){
            m_drive.setHeadingControllerGoal(m_drive.getHeading()+aimParams.get().getRobotToGoal().getDegrees());
        }
    }

    @Override
    public void end(boolean interrupted){
        m_drive.setStickControlledHeading(true);
        m_vision.StopCameraDataStream();
    }

    @Override
    public boolean isFinished(){
        //When the right bumper is released, stop the command
        return !m_driverController.getRawButton(XboxConstants.kRBumper);
    } 

}