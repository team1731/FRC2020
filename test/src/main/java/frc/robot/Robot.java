/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
public class Robot extends TimedRobot {
  private final PWMVictorSPX m_leftMotor = new PWMVictorSPX(0);
  private final PWMVictorSPX m_rightMotor = new PWMVictorSPX(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);

  @Override
  public void robotInit() {
    CameraServer camServer = CameraServer.getInstance();
    camServer.startAutomaticCapture();
  }
  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    double x = m_stick.getX();
    double y = m_stick.getY();
    double NewY = (-y/2) + 0.5;
    double OrdY = -y;


    SmartDashboard.putNumber("David X", x);
    SmartDashboard.putNumber("David y", y);
  
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
    System.out.println(x + ", " + NewY);
    if (0.5 <= x && x <= 1 && 0.5 <= OrdY && 1 >= OrdY) {
      System.out.println("Correct movement.");
    }
  }
}
