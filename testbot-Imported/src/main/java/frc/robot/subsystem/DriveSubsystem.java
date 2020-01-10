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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.command.TeleOpDriveCommand;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig;
import org.strykeforce.thirdcoast.swerve.Wheel;

public class DriveSubsystem extends Subsystem {

  private static final double DRIVE_SETPOINT_MAX = 0.0;
  private static final double ROBOT_LENGTH = 1.0;
  private static final double ROBOT_WIDTH = 1.0;

  private final SwerveDrive swerve = getSwerve();

  public DriveSubsystem() {}

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new TeleOpDriveCommand());
  }

  public void setDriveMode(DriveMode mode) {
    swerve.setDriveMode(mode);
  }

  public void zeroAzimuthEncoders() {
    swerve.zeroAzimuthEncoders();
  }

  public void drive(double forward, double strafe, double azimuth) {
    swerve.drive(forward, strafe, azimuth);
  }

  public void zeroGyro() {
    AHRS gyro = swerve.getGyro();
    gyro.setAngleAdjustment(0);
    double adj = gyro.getAngle() % 360;
    gyro.setAngleAdjustment(-adj);
  }

  // Swerve configuration

  private SwerveDrive getSwerve() {
    SwerveDriveConfig config = new SwerveDriveConfig();
    config.wheels = getWheels();
    config.gyro = new AHRS(SPI.Port.kMXP);
    config.length = ROBOT_LENGTH;
    config.width = ROBOT_WIDTH;
    config.gyroLoggingEnabled = true;
    config.summarizeTalonErrors = false;

    return new SwerveDrive(config);
  }

  private Wheel[] getWheels() {
     
    Wheel[] wheels = new Wheel[4];
    int smartMotionSlot = 0;
    for (int i = 1; i < 5; i++) {

      CANSparkMax azimuthSpark = new CANSparkMax(i, MotorType.kBrushless);
      azimuthSpark.restoreFactoryDefaults();
      CANPIDController azimuth_pidController = azimuthSpark.getPIDController();
      CANEncoder azimuth_encoder = azimuthSpark.getEncoder();
      azimuth_pidController.setP(5e-5);
      azimuth_pidController.setI(1e-6);
      azimuth_pidController.setD(0);
      azimuth_pidController.setFF(0.000156);
      azimuth_pidController.setOutputRange(-1, 1);
 
      azimuth_pidController.setSmartMotionMaxVelocity(2000, smartMotionSlot);
      azimuth_pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
      azimuth_pidController.setSmartMotionMaxAccel(1500, smartMotionSlot);
      azimuth_pidController.setSmartMotionAllowedClosedLoopError(50, smartMotionSlot);

      CANSparkMax driveSpark = new CANSparkMax(i + 10, MotorType.kBrushless);
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

      wheels[i-1] = wheel;

    }
     


    return wheels;
  }
}
