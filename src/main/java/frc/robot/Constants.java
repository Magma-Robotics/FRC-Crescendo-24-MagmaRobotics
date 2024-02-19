// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class POVButton {
    public static final int kUP = 0;
    public static final int kDOWN = 180;
    public static final int kLEFT = 90;
    public static final int kRIGHT = 270;
  }

  public static class Shooter {
    public static final double SHOOTER_SPEED = 0.5;
  }

  public static class Drivetrain {
    public static final double WHEEL_DIAMETER_IN_METERS = Units.inchesToMeters(6);
    public static final double TRACK_WIDTH_IN_METERS = Units.inchesToMeters(22.5);
    public static final double DRIVETRAIN_SPEED = 0.8;

    public static class LeftWheels {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
    }

    public static class RightWheels {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  }
}
