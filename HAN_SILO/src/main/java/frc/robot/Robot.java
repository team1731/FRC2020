/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

import org.strykeforce.thirdcoast.swerve.Wheel;






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
  private Wheel[] wheelObjects;

  @Override
  public void robotInit() {
    //logger.info("<b>Robot</b>: robotInit Started");
    System.out.println("Today is " + new Date().toString());
    DRIVE.zeroAzimuthEncoders();
    DRIVE.zeroGyro();
    //logger.info("<b>Robot</b>: robotInit Finished");
    //just some example lines i used to prove that code completion still works!
    //m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    //m_motor.disable();
  }

  @Override
  public void teleopPeriodic() {
    //logger.info("<b>Robot</b>: teleopPeriodic started");
    Scheduler.getInstance().run();
    //logger.info("<b>Robot</b>: teleopPeriodic finished");
  }

  @Override
  public void disabledPeriodic(){
    if(wheelObjects == null){
      wheelObjects = DRIVE.getWheelObjects();
    } else if(wheelObjects.length == 4) {
      for (int i = 0; i < wheelObjects.length; i++) {
        Wheel wheel = wheelObjects[i];
        if(wheel.m_encoder != null){
          SmartDashboard.putNumber("Wheel"+wheel.wheelID+" Encoder Pos", wheel.m_encoder.getPosition());
          SmartDashboard.putNumber("Wheel"+wheel.wheelID+" Encoder Get Abs", wheel.getAzimuthAbsolutePosition());
        }
      }
    }
  }
}
