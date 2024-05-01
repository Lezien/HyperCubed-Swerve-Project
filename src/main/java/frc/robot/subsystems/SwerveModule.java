// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
   public CANSparkMax driveMotor;
   public CANSparkMax angleMotor;
   //public static AbsoluteEncoder m_absoluteEncoder;
   public static int kCPR = 4096;
   public boolean absoluteEncoderReversed;
   public double absoluteEncoderOffsetRad;

   public RelativeEncoder m_driveEncoder;
   public RelativeEncoder m_angleEncoder;

   public PIDController m_anglePidController;

  public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean angleMotorReversed,
    double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    
      this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      
      driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
      angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

      driveMotor.setInverted(driveMotorReversed);
      angleMotor.setInverted(angleMotorReversed);

      m_driveEncoder = driveMotor.getEncoder();
      m_angleEncoder = angleMotor.getEncoder();
      //m_absoluteEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

      m_driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveEncoderRot2Meter);
      m_driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveEncoderRPM2MeterPerSec);
      m_angleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleEncoderRot2Rad);
      m_angleEncoder.setVelocityConversionFactor(Constants.SwerveConstants.angleEncoderRPM2RadPerSec);

      m_anglePidController = new PIDController(Constants.SwerveConstants.kpAngle, 0, 0);
      m_anglePidController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
  }

public double getDrivePosition() {
 
  return m_driveEncoder.getPosition();

}

public double getAnglePosition() {

  return m_angleEncoder.getPosition();
} 

public double getDriveVelocity() {

  return m_driveEncoder.getVelocity();
}

public double getAngleVelocity() {

  return m_angleEncoder.getVelocity();

}

/*public double getAbsolutePosition() {

  m_absoluteEncoder.setZeroOffset(absoluteEncoderOffsetRad);
  
  return m_absoluteEncoder.getPosition() * (absoluteEncoderReversed ? -1 : 1);
}
*/
public void resetEncoders() {
  m_driveEncoder.setPosition(0);

  m_angleEncoder.setPosition(0);
}

public SwerveModuleState getState() {

  return new SwerveModuleState( getDriveVelocity(), new Rotation2d(getAnglePosition()));
}

public SwerveModulePosition getPosition() {
  return new SwerveModulePosition( getDrivePosition(), new Rotation2d(getAnglePosition()));
}
public void setDesiredState( SwerveModuleState state) {

 /*  if(Math.abs(state.speedMetersPerSecond) < 0.001) {
    stop();
    return;
  }*/

  state = SwerveModuleState.optimize(state, getState().angle);
  driveMotor.set(state.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed);
  angleMotor.set(m_anglePidController.calculate(getAnglePosition(), state.angle.getRadians()));
  
}

public void stop() {
  driveMotor.set(0);
  angleMotor.set(0);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RPMVelocityConversion", m_driveEncoder.getVelocityConversionFactor());
    SmartDashboard.putNumber("VelocityConversionFactor", m_angleEncoder.getVelocityConversionFactor());
  }
}
