// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.Pigeon2;
public class SwerveDriveTrain extends SubsystemBase {
  /** Creates a new SwerveDriveTrain. */

  public static SwerveModule FrontLeft = new SwerveModule(
    Constants.FrontLeftModule.drivePort, Constants.FrontLeftModule.anglePort,
    Constants.FrontLeftModule.driveMotorReversed, Constants.FrontLeftModule.angleMotorReversed,
    Constants.FrontLeftModule.absoluteEncoderOffset, Constants.FrontLeftModule.absoluteEncoderReversed);

  public static SwerveModule FrontRight = new SwerveModule(
    Constants.FrontRightModule.drivePort, Constants.FrontRightModule.anglePort,
    Constants.FrontRightModule.driveMotorReversed, Constants.FrontRightModule.angleMotorReversed,
    Constants.FrontRightModule.absoluteEncoderOffset, Constants.FrontRightModule.absoluteEncoderReversed);

  public static SwerveModule BackLeft = new SwerveModule(
    Constants.BackLeftModule.drivePort, Constants.BackLeftModule.anglePort,
    Constants.BackLeftModule.driveMotorReversed, Constants.BackLeftModule.angleMotorReversed,
    Constants.BackLeftModule.absoluteEncoderOffset, Constants.BackLeftModule.absoluteEncoderReversed);

  public static SwerveModule BackRight = new SwerveModule(
    Constants.BackRightModule.drivePort, Constants.BackRightModule.anglePort,
    Constants.BackRightModule.driveMotorReversed, Constants.BackRightModule.angleMotorReversed,
    Constants.BackRightModule.absoluteEncoderOffset, Constants.BackRightModule.absoluteEncoderReversed);
  

  public static Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.gyroPort);
  public static SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.SwerveConstants.DriveKinematics,
            new Rotation2d(0), new SwerveModulePosition[] {
              FrontLeft.getPosition(), FrontRight.getPosition(),
              BackLeft.getPosition(), BackRight.getPosition()
            });

  public SwerveDriveTrain() {
        new Thread( () -> {
            try {
              Thread.sleep(1000);
              zeroHeading();
            } catch (Exception e) {
            }
        }).start();
  }

  public void zeroHeading() {
    gyro.setYaw(0);
  }

  public double getHeading() {

     double heading = gyro.getYaw().getValueAsDouble();

    return Math.IEEEremainder(heading, 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    FrontLeft.stop();
    FrontRight.stop();
    BackLeft.stop();
    BackRight.stop();
  }

  public void setModuleStates( SwerveModuleState[] desiredStates) {
    //SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
    FrontLeft.setDesiredState(desiredStates[0]);
    FrontRight.setDesiredState(desiredStates[1]);
    BackLeft.setDesiredState(desiredStates[2]);
    BackRight.setDesiredState(desiredStates[3]);
    
  }
  public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition( getRotation2d(), new SwerveModulePosition[] {
              FrontLeft.getPosition(), FrontRight.getPosition(),
              BackLeft.getPosition(), BackRight.getPosition()
            }, pose);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
              FrontLeft.getPosition(), FrontRight.getPosition(),
              BackLeft.getPosition(), BackRight.getPosition()
            }
          );
    SmartDashboard.putNumber( "Robot Heading", getHeading());
    //SmartDashboard.putNumber("driveEncoderRPM2MeterPerSec", Constants.SwerveConstants.driveEncoderRPM2MeterPerSec);
    //SmartDashboard.putNumber("angleEncoderRPM2RadPerSec", Constants.SwerveConstants.angleEncoderRPM2RadPerSec);
  }
}
