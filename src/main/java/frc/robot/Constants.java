// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kOperatorControllerPort = 1;

    // Joystick Deadbands
    public static final double  DEADBAND           = 0.07;

    // Scales for movement and rotation (1 is full speed)
    public static final double  TRANSLATION_SCALE  = 0.88;
    public static final double  ROTATION_SCALE     = 0.88;
  }

  public static final   double  MAX_SPEED = 2.5 ;

  public static class Pose {
    public static final double feildFlip = 17.5;
    public static final double feildFlipy = 8;
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = 12;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 60  ;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_SPEED_DOWN = -0.15;
    public static final double ARM_SPEED_UP = 0.2;
    public static final double ARM_HOLD_DOWN = -0.0;
    public static final double ARM_HOLD_UP = 0.0;
    public static final double ARM_UP_OFFSET = -1;
    public static final double ARM_DOWN_OFFSET = -17;
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 13;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_ALGAE_IN = -0.2;
    public static final double ROLLER_ALGAE_OUT = 0.2;
    public static final double ROLLER_ALGAE_UP = -0.1;
    public static final double ROLLER_CORAL_OUT = -.17;
    public static final double ROLLER_CORAL_STACK = -0.22;
  }
}
