// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class SwerveConstants {
    
    public static final double neoRPM = 5820;
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveMotorGearRatio = ( 1 / 8.14);
    public static final double angleMotorGearRatio = ( 1 / (150 / 7) );
    public static final double driveEncoderRot2Meter = driveMotorGearRatio * Math.PI *wheelDiameter;
    public static final double angleEncoderRot2Rad = angleMotorGearRatio * 2 * Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter / 60;
    public static final double angleEncoderRPM2RadPerSec = angleEncoderRot2Rad / 60;
    public static final double kpAngle = 0.01;
    public static final double maxSpeed = neoRPM * driveMotorGearRatio * (Math.PI*wheelDiameter) * (1/60);

    public static final double DriveMaxAccelerationUnitsPerSecond = 3;
    public static final double DriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final int gyroPort = 5;

     public static final double TrackWidth = Units.inchesToMeters(23.875);
        // Distance between right and left wheels
        public static final double WheelBase = Units.inchesToMeters(23.875);
        // Distance between front and back wheels
        public static Translation2d m_frontLeft = new Translation2d(WheelBase / 2, TrackWidth / 2);
        public static Translation2d m_frontRight = new Translation2d(WheelBase / 2, -TrackWidth / 2);
        public static Translation2d m_backLeft = new Translation2d(-WheelBase / 2, TrackWidth / 2);
        public static Translation2d m_backRight = new Translation2d(-WheelBase / 2, -TrackWidth / 2);
        public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
                m_frontLeft, m_frontRight, m_backLeft, m_backRight);
       
  }
        
  
  public static class FrontLeftModule {
    public static final int drivePort = 10;
    public static final int anglePort = 9;
    public static final boolean driveMotorReversed = false;
    public static final boolean angleMotorReversed = false;
    public static final boolean absoluteEncoderReversed = false;
    public static final double absoluteEncoderOffset = 0;

  }

  public static class FrontRightModule {
    public static final int drivePort = 3;
    public static final int anglePort = 2;
    public static final boolean driveMotorReversed = false;
    public static final boolean angleMotorReversed = false;
    public static final boolean absoluteEncoderReversed = false;
    public static final double absoluteEncoderOffset = 0;
  }

  public static class BackLeftModule {
    public static final int drivePort = 8;
    public static final int anglePort = 7;
    public static final boolean driveMotorReversed = false;
    public static final boolean angleMotorReversed = false;
    public static final boolean absoluteEncoderReversed = false;
    public static final double absoluteEncoderOffset = 0;
  }

  public static class BackRightModule {
    public static final int drivePort = 6;
    public static final int anglePort = 4;
    public static final boolean driveMotorReversed = false;
    public static final boolean angleMotorReversed = false;
    public static final boolean absoluteEncoderReversed = false;
    public static final double absoluteEncoderOffset = 0;
  }

  public static class OIConstants {
    public static final double deadband = 0.05;
    }
  }

