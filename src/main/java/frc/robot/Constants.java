// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (110) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.05; // s, 20ms + 110ms spark max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(19.8);
  public static final double ALIGN_KP = 0.005;
  public static final boolean isCompetition = true;

  public static final Transform2d leftbranchrobotoffset = new Transform2d(Units.inchesToMeters(-10),
      Units.inchesToMeters(6), Rotation2d.kZero);
  public static final Transform2d rightbranchrobotoffset = new Transform2d(Units.inchesToMeters(-10),
      Units.inchesToMeters(-6), Rotation2d.kZero);

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double slow_multiplier = 0.33;
    public static final double normal_multiplier = 0.8;

  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.035;
    public static final double TURN_CONSTANT = 6;
  }

  public static class ClimbConstants {
    public static final int climb_motor_id = 62;
    public static final double climb_climbing_limit = 1.55;
    public static final double climb_deployed_position = 3.5;
    public static final double climb_stow_position = 0.4 + 0.2;
    public static final double climb_backstop = 1.55;
    public static final double climb_power = -1;
    public static final double climb_slot0_kP = 0.8999999761581421;
    public static final double climb_slot0_kI = 0.00004999999873689376;
    public static final double climb_slot0_kD = 0;
    public static final double climb_sensor_to_mechanism_ratio = 56.076900482177734;
    public static final double forward_soft_limit = 4.7;
    public static final double reverse_soft_limit = 0.2;

  }

  public static class BallIntakeConstants { // bi is Ball Intake
    // PIDs
    public static final double bi_slot0_kP = 0.009999999776482582;
    public static final double bi_slot0_kI = 0;
    public static final double bi_slot0_kD = 0;
    public static final double bi_slot0_kS = 0;
    public static final double bi_slot0_kV = 0;
    public static final double bi_slot0_kA = 0;
    // Motor IDs
    public static final int bi_roller_motor_id = 61;
    public static final int bi_pivot_motor_id = 60;
    // Sensor IDs
    public static final int bi_range_id = 57;
    // Encoder IDs
    public static final int bi_pivot_encoder_id = 56;

    // Positions
    public static final double bi_stow_position = 65; // No Ball
    public static final double bi_algae_intake_position = 132;
    public static final double bi_algae_stow_position = 110;
    public static final double bi_algae_score_position = 80;
    public static final double bi_climb_position = 182;
    // Sensor Thresholds
    public static final double bi_algae_threshhold = 0.7;
    // Power
    public static final double bi_intake_power = 0.4;
    public static final double bi_outtake_power = -0.25;
    public static final double bi_idle_power = 0.04;
    public static final double bi_hold_power = 0.15;
    // Motor Configs
    public static final double bi_absolute_sensor_continuity_point = 1;
    public static final double bi_rotor_to_sensor_ratio = 135;
    public static final double bi_sensor_to_mechanism_ratio = 0.0027777778450399637;
    public static final double bi_fw_soft_limit = 180;
    public static final double bi_rev_soft_limit = 53.25;

  }

  public static class EndevatorConstants {
    /*
     * CAN IDs
     */
    public static final int endeffector_cancoder_id = 50;
    public static final int endeffector_pivot_motor_id = 52;
    public static final int elevator_motor_id = 51;

    /*
     * Motor Configurations
     */
    // Endeffector Pivot
    public static final double endeffector_sensor_to_mechanism_ratio = 0.00571999978274107;
    public static final double endeffector_rotor_to_sensor_ratio = 1;
    // slot0

    public static final double endeffector_slot0_kP = 0.009499999694526196;
    public static final double endeffector_slot0_kI = 0.0006000000284984708;
    public static final double endeffector_slot0_kD = 0.0010000000474974513;
    public static final double endeffector_slot0_kS = 0;
    public static final double endeffector_slot0_kV = 0;
    public static final double endeffector_slot0_kA = 0;
    // slot1
    public static final double endeffector_slot1_kP = 0.02500000037252903;
    public static final double endeffector_slot1_kI = 0.003000000026077032;
    public static final double endeffector_slot1_kD = 0.0010000000474974513;
    public static final double endeffector_slot1_kS = 0;
    public static final double endeffector_slot1_kV = 0;
    public static final double endeffector_slot1_kA = 0;
    // Other
    public static final double endeffector_duty_cycle_closed_loop_ramp = 0.25;

    // Elevator Motor
    public static final double elevator_sensor_to_mechanism_ratio = 0.715;
    public static final double elevator_rotor_to_sensor_ratio = 1; // 1/23.36
    public static final double elevator_mm_cruise_velocity = 3500;
    public static final double elevator_mm_acceleration = 2000;
    public static final double elevator_mm_jerk = 9999;
    public static final double elevator_kP = 0.07;
    public static final double elevator_kI = 0.0;
    public static final double elevator_kD = 0.0;
    public static final double elevator_kS = 0;
    public static final double elevator_kV = 1 / 3;
    public static final double elevator_kA = 1 / 3;

    /*
     * Elevator heights
     */
    // Stow
    public static final double coral_stow_height = 5;
    public static final double algae_stow_height = 9;
    // Floor
    public static final double coral_floor_pickup_height = 1;
    public static final double algae_floor_pickup_height = 11.5;
    // Barge
    public static final double barge_height = 68.5;
    // L4
    public static final double L4_height = 57.5;
    public static final double L4_auto_height = 56;
    public static final double L4_score_height = 42.5;
    public static final double L4_score_auto_height = 41.75;
    public static final double L4_score_auto_lower_height = 30.5;
    // L3
    public static final double L3_height = 45.95;
    public static final double algae_L3_height = 42.5;
    // L2
    public static final double L2_height = 29.88;
    public static final double algae_L2_height = 27;
    // L1
    public static final double L1_height = 19.1 + 1;
    // Coral Station
    public static final double coral_station_auto_height = 24;
    public static final double coral_station_tele_height = 23.5;
    // Processor
    public static final double algae_processor_height = 1.5;
    /*
     * Endeffector Angles
     */
    // Stow
    public static final int auto_coral_stow_angle = 35;
    public static final int teleop_coral_stow_angle = 35;
    public static final int algae_stow_angle = 66;
    // Floor
    public static final int coral_floor_angle = 125;
    public static final int algae_floor_angle = 160;
    // Barge
    public static final int barge_flick_angle = 25;
    // L4
    public static final int auto_L4_angle = 35;
    public static final int teleop_L4_angle = 35;
    // L3
    public static final int coral_L3_angle = 125;
    public static final int algae_L3_angle = 125;
    // L2
    public static final int coral_L2_angle = 125;
    public static final int algae_L2_angle = 125;
    // L1
    public static final int coral_L1_angle = 125;
    // Coral Station
    public static final int auto_coral_station_angle = 73;
    public static final int teleop_coral_station_angle = 69;
    // Processor
    public static final int algae_processor_angle = 125;
    /*
     * Antenna Servo
     */
    public static final double antenna_reef_intake_limit = 0.55;
    public static final double antenna_floor_intake_limit = 0.6;
    public static final double antenna_home = 1;
    /*
     * Antenna Ball Detection
     */
    public static final double antenna_ball_detection_current = 11;
  }

  public static class EndeffectorIntakeConstants {
    /*
     * CAN IDs
     */
    public static final int ei_left_motor_id = 53;
    public static final int ei_top_motor_id = 54;
    public static final int ei_right_motor_id = 55;
    public static final int ei_coral_range_id = 56;
    public static final int ei_algae_range_id = 58;

    /*
     * Powers for Various Tasks
     */
    // Idle
    public static final double ei_idle_power = 0.05;
    // Intaking
    public static final double ei_coral_floor_intake_power = 0.5;
    public static final double ei_coral_coral_station_intake_power = 0.65;
    public static final double ei_algae_intake_power = -0.4; // Takes ball from reef
    public static final double ei_algae_idle_power = -0.125; // Takes ball from reef
    public static final double ei_algae_floor_intake_power = -1;
    public static final double ei_algae_floor_intake_top_roller_power = 0.9;
    // L1 Score
    public static final double ei_L1_score_power = -0.16;
    // L2 and L3 Score
    public static final double ei_L2_L3_score_power = -0.25;
    // L4 Score
    public static final double ei_L4_score_outtake_power = -0.075;
    // Barge Score
    public static final double ei_teleop_barge_score_power = 1; // Outtake ball
    public static final double ei_auto_barge_score_power = 0.75; // Outtake ball
    // Processor Score
    public static final double ei_processor_score_power = 0.4;
    // Detection Threshholds
    public static final double ei_coral_threshhold = 0.09;
    public static final double ei_algae_threshhold = 0.12;

  }

}
