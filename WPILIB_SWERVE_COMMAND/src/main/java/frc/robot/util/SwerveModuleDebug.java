package frc.robot.util;

import frc.robot.subsystems.SwerveModule;

public class SwerveModuleDebug {
    public double time;

    public double drive1AppliedOutput;
    public double drive2AppliedOutput;
    public double drive3AppliedOutput;
    public double drive4AppliedOutput;

    public double drive1Velocity;
    public double drive2Velocity;
    public double drive3Velocity;
    public double drive4Velocity;

    public double turn1AppliedOutput;
    public double turn2AppliedOutput;
    public double turn3AppliedOutput;
    public double turn4AppliedOutput;

    public double turn1Velocity;
    public double turn2Velocity;
    public double turn3Velocity;
    public double turn4Velocity;

    public SwerveModuleDebug(double time, 
                            SwerveModule.DebugValues debugValues1, 
                            SwerveModule.DebugValues debugValues2, 
                            SwerveModule.DebugValues debugValues3, 
                            SwerveModule.DebugValues debugValues4){
      update(time, debugValues1, debugValues2, debugValues3, debugValues4);
    }

    public void update(double time,
                       SwerveModule.DebugValues debugValues1, 
                       SwerveModule.DebugValues debugValues2, 
                       SwerveModule.DebugValues debugValues3, 
                       SwerveModule.DebugValues debugValues4){
      this.time = time;
      drive1AppliedOutput = debugValues1.driveAppliedOutput;
      drive1Velocity = debugValues1.driveVelocity;
      turn1AppliedOutput = debugValues1.turnAppliedOutput;
      turn1Velocity = debugValues1.turnVelocity;

      drive2AppliedOutput = debugValues2.driveAppliedOutput;
      drive2Velocity = debugValues2.driveVelocity;
      turn2AppliedOutput = debugValues2.turnAppliedOutput;
      turn2Velocity = debugValues2.turnVelocity;

      drive3AppliedOutput = debugValues3.driveAppliedOutput;
      drive3Velocity = debugValues3.driveVelocity;
      turn3AppliedOutput = debugValues3.turnAppliedOutput;
      turn3Velocity = debugValues3.turnVelocity;

      drive4AppliedOutput = debugValues4.driveAppliedOutput;
      drive4Velocity = debugValues4.driveVelocity;
      turn4AppliedOutput = debugValues4.turnAppliedOutput;
      turn4Velocity = debugValues4.turnVelocity;
    }

  }
