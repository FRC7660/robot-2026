// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.*;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //  public static final class AutonConstants
  //  {
  //
  //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  //  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.15;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class NeutralToBallPickupAutoConstants {
    public static final String DEFAULT_OUTBOUND_TRAJECTORY_NAME = "NeutralRush";
    public static final String DEFAULT_RETURN_TRAJECTORY_NAME = "Pig";

    // Timing
    public static final double DEFAULT_PICKUP_TIMEOUT_SEC = 5.0;
    public static final double POST_TIMEOUT_FINISH_GRACE_SEC = 1.5;

    // Search/approach strategy
    public static final double SEARCH_ROTATE_STEP_DEG = 15.0;
    public static final double OBJECT_PICKUP_AREA_PERCENT_THRESHOLD = 8.0;
    public static final double OBJECT_APPROACH_MIN_FWD_MPS = 0.10;
    public static final double OBJECT_AREA_TO_FWD_GAIN = 0.08;
    public static final double OBJECT_YAW_TO_ROT_GAIN = 2.0;

    // Max speeds by mode (currently same by request)
    public static final double MAX_MODE_FORWARD_MPS = 0.35;
    public static final double MAX_MODE_ROTATION_RAD_PER_SEC = 0.35;
    public static final double DEBUG_MODE_FORWARD_MPS = 0.35;
    public static final double DEBUG_MODE_ROTATION_RAD_PER_SEC = 0.35;

    public static final String PICKUP_PROXY_COUNT_DASHBOARD_KEY =
        "Auto/NeutralToBallPickup/ProxyPickups";
  }

  // Index Constants
  public static final class Index {
    public static final int INDEX_MOTOR_ID = 16;
    public static final double INDEX_SPEED = 0.5;
    public static final int FUNNEL_MOTOR_ID = 19;
    public static final double FUNNEL_SPEED = 0.5;
  }

  // Intake Constants
  public static final class Intake {
    public static final int LIFT_MOTOR_ID = 15;
    public static final double LIFT_SPEED = 0.5;
    public static final int LIFT_CURRENT_LIMIT = 30; // amps
    public static final int ROLLER_MOTOR_ID = 17;
    public static final double ROLLER_SPEED = 0.5;
    public static final int ROLLER_CURRENT_LIMIT = 25; // amps
  }

  // Launch Constants
  public static final class Launch {
    public static final int MOTOR1_ID = 31;
    public static final int MOTOR2_ID = 37;
  }

  // Turret Constants
  public static final class Turret {
    public static final int MOTOR_ID = 18;
    public static final double TURRET_SPEED = 0.5;
  }
}
