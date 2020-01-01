package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.control.Controls;
import frc.robot.subsystem.DriveSubsystem;
import java.util.Date;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class Robot extends TimedRobot {
  private static final Logger logger = LoggerFactory.getLogger(Robot.class);
  public static final DriveSubsystem DRIVE = new DriveSubsystem();

  // Controls initialize Commands so this should be instantiated last to prevent
  // NullPointerExceptions in commands that require() Subsystems above.
  public static final Controls CONTROLS = new Controls();

  private CANSparkMax m_motor;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private static final int deviceID = 1;

  @Override
  public void robotInit() {
    System.out.println("Today is " + new Date().toString());
    DRIVE.zeroAzimuthEncoders();
    DRIVE.zeroGyro();

    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.disable();
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }
}
