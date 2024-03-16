// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverPartnerControllerPort = 1;
  }

  public static class Shooter {
    public static final double SHOOTER_SPEED = 0.5;
  }

  public static class Drivetrain {
    public static final double ksVolts = 0;
    public static final double ksVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kPDriveVel = 0;
  
    public static final double kTrackWidthMeters = Units.inchesToMeters(22.5);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kGearRatio = 8.46;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3);

    public static final double kLinearDistanceConversionFactor = 0.12;//(Units.inchesToMeters(1 / (kGearRatio * 2 * Math.PI * kWheelRadiusMeters) * 10));

    public static final double DRIVETRAIN_SPEED = 0.8;

    
  }
}
