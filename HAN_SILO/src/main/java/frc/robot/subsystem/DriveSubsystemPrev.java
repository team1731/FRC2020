package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.command.TeleOpDriveCommand;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.usfirst.frc.team1731.lib.util.drivers.NavX;
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig;
import org.strykeforce.thirdcoast.swerve.Wheel;

//import org.slf4j.Logger;
//import org.slf4j.LoggerFactory;

public class DriveSubsystemPrev extends Subsystem {

  //private static final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
  private static final double DRIVE_SETPOINT_MAX = 0.0;
  //private static final double ROBOT_LENGTH = 1.0;
  //private static final double ROBOT_WIDTH = 1.0;
  private static Wheel[] wheelObjects;

  private final SwerveDrive swerve = getSwerve();

  public DriveSubsystemPrev() {}

  @Override
  protected void initDefaultCommand() {
    //logger.info("<b>DriveSubsystem</b>: initDefaultCommand started");
    setDefaultCommand(new TeleOpDriveCommand());
    //logger.info("<b>DriveSubsystem</b>: initDefaultCommand finished");
  }

  public void setDriveMode(DriveMode mode) {
    //logger.info("<b>DriveSubsystem</b>: setDriveMode started");
    swerve.setDriveMode(mode);
    //logger.info("<b>DriveSubsystem</b>: setDriveMode finished");
  }

  public void zeroAzimuthEncoders() {
    //logger.info("<b>DriveSubsystem</b>: zeroAzimuthEncoders started");
    if (RobotBase.isReal()) {
      swerve.zeroAzimuthEncoders();
    }
    //logger.info("<b>DriveSubsystem</b>: zeroAzimuthEncoders finished");
  }

  public void drive(double forward, double strafe, double azimuth) {
    //logger.info("<b>DriveSubsystem</b>: drive started");
    swerve.drive(forward, strafe, azimuth);
    //logger.info("<b>DriveSubsystem</b>: drive finished");
  }

  public void zeroGyro() {
    if (RobotBase.isReal()) {
      //logger.info("<b>DriveSubsystem</b>: zeroGyro started");
      AHRS gyro = swerve.getGyro();
      gyro.setAngleAdjustment(0);
      double adj = gyro.getAngle() % 360;
      gyro.setAngleAdjustment(-adj);
      //logger.info("<b>DriveSubsystem</b>: zeroGyro finished");
    }      
  }

  // Swerve configuration

  private SwerveDrive getSwerve() {
    //logger.info("<b>DriveSubsystem</b>: getSwerve started");
    SwerveDriveConfig config = new SwerveDriveConfig();
    config.wheels = getWheels();
    if (RobotBase.isReal()) {
      config.gyro = NavX.getAHRS();
    }
    //config.length = ROBOT_LENGTH;
    //config.width = ROBOT_WIDTH;
    config.gyroLoggingEnabled = true;
    config.summarizeTalonErrors = false;

    //logger.info("<b>DriveSubsystem</b>: getSwerve returning");
    return new SwerveDrive(config);
  }

  private Wheel[] getWheels() {
    //logger.info("<b>DriveSubsystem</b>: getWheels starting");
    Wheel[] wheels = new Wheel[4];
    int smartMotionSlot = 0;
    for (int i = 1; i < 5; i++) {
      if (RobotBase.isReal()) {
        CANSparkMax azimuthSpark = new CANSparkMax(i + 10, MotorType.kBrushless);
        azimuthSpark.restoreFactoryDefaults();
        CANPIDController azimuth_pidController = azimuthSpark.getPIDController();
        CANEncoder azimuth_encoder = azimuthSpark.getEncoder();
        azimuthSpark.setInverted(true);
        azimuth_pidController.setP(5e-5);
        azimuth_pidController.setI(1e-6);
        azimuth_pidController.setD(0);
        azimuth_pidController.setIZone(0);
        azimuth_pidController.setFF(0.000156);
        azimuth_pidController.setOutputRange(-1, 1);
   
        azimuth_pidController.setSmartMotionMaxVelocity(2000, smartMotionSlot);
        azimuth_pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        azimuth_pidController.setSmartMotionMaxAccel(1500, smartMotionSlot);
        azimuth_pidController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
  
        CANSparkMax driveSpark = new CANSparkMax(i, MotorType.kBrushless);
        driveSpark.restoreFactoryDefaults();
        CANPIDController drive_pidController = driveSpark.getPIDController();
        CANEncoder drive_encoder = driveSpark.getEncoder();
        drive_pidController.setP(5e-5);
        drive_pidController.setI(1e-6);
        drive_pidController.setD(0);
        drive_pidController.setFF(0.000156);
        drive_pidController.setOutputRange(-1, 1);
        drive_pidController.setSmartMotionMaxVelocity(2000, smartMotionSlot);
        drive_pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        drive_pidController.setSmartMotionMaxAccel(1500, smartMotionSlot);
        drive_pidController.setSmartMotionAllowedClosedLoopError(50, smartMotionSlot);
  
        Wheel wheel = new Wheel(azimuthSpark, driveSpark, DRIVE_SETPOINT_MAX);
        wheel.wheelID = i-1;
  
        wheels[i-1] = wheel;
        }

    }
     

    //logger.info("<b>DriveSubsystem</b>: getWheels returning");
    wheelObjects = wheels;
    return wheels;
  }

  public Wheel[] getWheelObjects(){
      return wheelObjects;
  }
}
