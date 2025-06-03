package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndeffectorIntakeConstants;
import frc.robot.Constants.EndevatorConstants;

public class EndEvatorSubsystem extends SubsystemBase {

    /*
     * Elevator Motor
     */
    static TalonFX elevator_motor = new TalonFX(EndevatorConstants.elevator_motor_id);
    static TalonFXConfiguration elevator_motor_config = new TalonFXConfiguration();
    // static PositionDutyCycle elevator_PositionDutyCycle0 = new
    // PositionDutyCycle(0).withSlot(0);
    static MotionMagicDutyCycle elevator_MotionMagicDutyCycle0 = new MotionMagicDutyCycle(0);

    /*
     * Endeffector pivot
     */
    static TalonFX endeffector_pivot = new TalonFX(EndevatorConstants.endeffector_pivot_motor_id);
    static TalonFXConfiguration endeffector_pivot_config = new TalonFXConfiguration();
    static PositionDutyCycle endeffector_PositionDutyCycle = new PositionDutyCycle(0);

    /*
     * Servo
     */
    static Servo antennaServo = new Servo(1);

    /*
     * Coral CANRange
     */
    static CANrange coral_range = new CANrange(EndeffectorIntakeConstants.ei_coral_range_id);
    static CANrangeConfiguration coral_range_config = new CANrangeConfiguration();
    /*
     * Algae CANRange
     */
    static CANrange algae_range = new CANrange(EndeffectorIntakeConstants.ei_algae_range_id);
    static CANrangeConfiguration algae_range_config = new CANrangeConfiguration();
    /*
     * Coral Intake 
     */
    static TalonFX right_motor_T = new TalonFX(EndeffectorIntakeConstants.ei_right_motor_id);
    static TalonFX top_motor_T = new TalonFX(EndeffectorIntakeConstants.ei_top_motor_id);
    static TalonFX left_motor_T_follower = new TalonFX(EndeffectorIntakeConstants.ei_left_motor_id);

    public double targetElevatorHeight = 0;

    // Required initialization crap
    public void initialize() {
        System.out.println("ElevatorSubsystem Initialized");

    }

    public EndEvatorSubsystem() { // Apply motor configs
        /*
         * Elevator Motor Config
         */
        elevator_motor.getConfigurator().refresh(elevator_motor_config); // Place current config in new config
        elevator_motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevator_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        elevator_motor_config.Slot0.kP = EndevatorConstants.elevator_kP;
        elevator_motor_config.Slot0.kI = EndevatorConstants.elevator_kI;
        elevator_motor_config.Slot0.kD = EndevatorConstants.elevator_kD;
        elevator_motor_config.Slot0.kA = EndevatorConstants.elevator_kA;
        elevator_motor_config.Slot0.kV = EndevatorConstants.elevator_kV;
        elevator_motor_config.Slot0.kS = EndevatorConstants.elevator_kS;
        elevator_motor_config.MotionMagic.MotionMagicCruiseVelocity = EndevatorConstants.elevator_mm_cruise_velocity;
        elevator_motor_config.MotionMagic.MotionMagicAcceleration = EndevatorConstants.elevator_mm_acceleration;
        elevator_motor_config.MotionMagic.MotionMagicJerk = EndevatorConstants.elevator_mm_jerk;
        elevator_motor_config.Feedback.SensorToMechanismRatio = EndevatorConstants.elevator_sensor_to_mechanism_ratio;
        elevator_motor_config.Feedback.RotorToSensorRatio = EndevatorConstants.elevator_rotor_to_sensor_ratio;
        /*
         * Endeffector Motor Config
         */
        endeffector_pivot.getConfigurator().refresh(endeffector_pivot_config);
        endeffector_pivot_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        endeffector_pivot_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Slot 0
        endeffector_pivot_config.Slot0.kP = EndevatorConstants.endeffector_slot0_kP;
        endeffector_pivot_config.Slot0.kI = EndevatorConstants.endeffector_slot0_kI;
        endeffector_pivot_config.Slot0.kD = EndevatorConstants.endeffector_slot0_kD;
        endeffector_pivot_config.Slot0.kA = EndevatorConstants.endeffector_slot0_kA;
        endeffector_pivot_config.Slot0.kV = EndevatorConstants.endeffector_slot0_kV;
        endeffector_pivot_config.Slot0.kS = EndevatorConstants.endeffector_slot0_kS;
        // Slot 1
        endeffector_pivot_config.Slot1.kP = EndevatorConstants.endeffector_slot1_kP;
        endeffector_pivot_config.Slot1.kI = EndevatorConstants.endeffector_slot1_kI;
        endeffector_pivot_config.Slot1.kD = EndevatorConstants.endeffector_slot1_kD;
        endeffector_pivot_config.Slot1.kA = EndevatorConstants.endeffector_slot1_kA;
        endeffector_pivot_config.Slot1.kV = EndevatorConstants.endeffector_slot1_kV;
        endeffector_pivot_config.Slot1.kS = EndevatorConstants.endeffector_slot1_kS;
        // Feedback
        endeffector_pivot_config.Feedback.FeedbackRemoteSensorID = EndevatorConstants.endeffector_cancoder_id;
        endeffector_pivot_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        endeffector_pivot_config.Feedback.RotorToSensorRatio = EndevatorConstants.endeffector_rotor_to_sensor_ratio;
        endeffector_pivot_config.Feedback.SensorToMechanismRatio = EndevatorConstants.endeffector_sensor_to_mechanism_ratio;
        endeffector_pivot_config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = EndevatorConstants.endeffector_duty_cycle_closed_loop_ramp;
        /*
         * Coral CANRange Config
         */
        coral_range_config.ProximityParams.ProximityThreshold = EndeffectorIntakeConstants.ei_coral_threshhold;
        /*
         * Algae CANRange Config
         */
        algae_range_config.ProximityParams.ProximityThreshold = EndeffectorIntakeConstants.ei_algae_threshhold;
        /*
         * Apply Configs
         */
        elevator_motor.getConfigurator().apply(elevator_motor_config);
        endeffector_pivot.getConfigurator().apply(endeffector_pivot_config);
        coral_range.getConfigurator().apply(coral_range_config);
        algae_range.getConfigurator().apply(algae_range_config);
        System.out.println("ElevatorSubsystem Initialized");
        state = EndEvatorState.STARTING;
    }

    public double initial_endeffector_position = 0.0;

    // Initialize ElevatorState Enum, and start at stow
    public enum EndEvatorState {
        STOW,
        STARTING,
        L1,
        L2,
        L3,
        L4,
        L4_Score,
        CORAL_FLOOR_INTAKE,
        ALGAE_FLOOR_INTAKE,
        HIGH_ALGAE_INTAKE,
        LOW_ALGAE_INTAKE,
        BARGE,
        FLICK
    }
    /*
     * All state machine interaction and reading
     */

    public EndEvatorState state;

    /**
     * FUNCTION for setting the elevator state machine to another state.
     * 
     * @param setstateto
     */
    public void setState(EndEvatorState setstateto) {
        initial_endeffector_position = getEndeffectorCurrentPosition();
        state = setstateto;

    }

    /**
     * COMMAND for moving the elevator. Usually accessed by the controller, and
     * eventually sets the state of the state machine.
     * 
     * @param setto
     * @return Command
     */
    public Command setTo(EndEvatorState setto) {
        return runOnce(() -> setState(setto));
    }

    /**
     * Returns current elevator state.
     * 
     * @return EndEvatorState
     */
    public EndEvatorState getCurrentState() {
        return state;
    }

    /**
     * Sets elevator position
     * 
     * @param position
     */
    public void moveElevator(double position) {
        elevator_motor.setControl(elevator_MotionMagicDutyCycle0.withPosition(position));

    }

    /**
     * Sets endeffector angle
     * 
     * @param position
     */
    public void moveEndeffector(double position, int slot) {
        endeffector_pivot.setControl(endeffector_PositionDutyCycle.withPosition(position).withSlot(slot));

    }

    public void moveAntennaServo(double position) {
        antennaServo.set(position);
    }

    /*
     * Various Functions used for retrieving positions of stuff.
     */
    public double getElevatorCurrentPosition() {
        return elevator_motor.getPosition().refresh().getValueAsDouble();
    }

    public double getEndeffectorCurrentPosition() {
        return endeffector_pivot.getPosition().refresh().getValueAsDouble();
    }

    public double getServoPosition() {
        return antennaServo.getPosition();
    }

    public double getElevatorTargetPosition() {
        // return elevator_motor.getClosedLoopOutput().getValueAsDouble();
        return elevator_MotionMagicDutyCycle0.getPositionMeasure().in(Units.Rotations);
    }

    public BooleanSupplier elevatorAtTargetPosition(double position) {
        return () -> (getElevatorCurrentPosition() <= position + 1)
                && (getElevatorCurrentPosition() >= position - 1);
    }

    public BooleanSupplier notatElevatorTargetPosition(double position) {
        return () -> (!elevatorAtTargetPosition(position).getAsBoolean());
    }

    /*
     * Booleans for controlling binds.
     */

    /*
     * Coral
     */
    public BooleanSupplier hasCoralSupplier = () -> hasCoral();
    public BooleanSupplier readyToRaiseWithCoralSupplier = () -> readyToRaiseWithCoral();
    public BooleanSupplier hasNoCoralSupplier = () -> !hasCoral();
    public BooleanSupplier notReadyToRaiseWithCoralSupplier = () -> !readyToRaiseWithCoral();

    public Boolean hasCoral() {
        return coral_range.getIsDetected(true).getValue();
    }

    public Boolean 
    readyToRaiseWithCoral() {
        return coral_range.getIsDetected(true).getValue() && state != EndEvatorState.CORAL_FLOOR_INTAKE;
    }

    /*
     * Algae
     */
    public BooleanSupplier hasAlgaeSupplier = () -> hasAlgae();
    public BooleanSupplier readyToRaiseWithAlgaeSupplier = () -> readyToRaiseWithCoral();
    public BooleanSupplier hasNoAlgaeSupplier = () -> !hasAlgae();
    public BooleanSupplier notReadyToRaiseWithAlgaeSupplier = () -> !readyToRaiseWithCoral();


    public Boolean hasAlgae() { // TODO: Make this serve a real purpose
        return algae_range.getIsDetected(true).getValue();
    }
    
    public Boolean readyToRaiseWithAlgae() {
        return algae_range.getIsDetected(true).getValue() && state != EndEvatorState.ALGAE_FLOOR_INTAKE;
    }

    /*
     * Check for state matches
     */

    public BooleanSupplier isAt(EndEvatorState state) {
        return () -> getCurrentState() == state;
    }

    public BooleanSupplier isNotAt(EndEvatorState state) {
        return () -> getCurrentState() != state;
    }

    // State Machine Garbage
    public void periodic() {

        switch (state) {
            case STARTING -> {
                moveElevator(0);
                moveEndeffector(EndevatorConstants.teleop_coral_stow_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_home);
            }
            case STOW -> {
                if (hasAlgae()) {
                    moveElevator(EndevatorConstants.algae_stow_height);
                    moveEndeffector(EndevatorConstants.algae_stow_angle, 0);
                    moveAntennaServo(EndevatorConstants.antenna_reef_intake_limit);
                } else {
                    moveElevator(EndevatorConstants.coral_stow_height);
                    moveEndeffector(EndevatorConstants.teleop_coral_stow_angle, 0);
                    moveAntennaServo(EndevatorConstants.antenna_home);
                }

            }
            case L1 -> {
                moveElevator(EndevatorConstants.L1_height);
                moveEndeffector(EndevatorConstants.coral_L1_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_home);
            }
            case L2 -> {
                moveElevator(EndevatorConstants.L2_height);
                moveEndeffector(EndevatorConstants.coral_L2_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_home);
            }
            case L3 -> {
                moveElevator(EndevatorConstants.L3_height);
                moveEndeffector(EndevatorConstants.coral_L3_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_home);

            }
            case L4 -> {
                moveElevator(EndevatorConstants.L4_height);
                moveEndeffector(EndevatorConstants.teleop_L4_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_home);

            }
            case L4_Score -> {
                moveElevator(EndevatorConstants.L4_score_height);
                moveEndeffector(EndevatorConstants.teleop_L4_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_home);
            }
            case CORAL_FLOOR_INTAKE -> {
                moveElevator(EndevatorConstants.coral_floor_pickup_height);
                moveEndeffector(EndevatorConstants.coral_floor_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_home);
            }
            case ALGAE_FLOOR_INTAKE -> {
                moveElevator(EndevatorConstants.algae_floor_pickup_height);
                moveEndeffector(EndevatorConstants.algae_floor_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_floor_intake_limit);
            }
            case BARGE -> {
                moveElevator(EndevatorConstants.barge_height);
                moveEndeffector(EndevatorConstants.algae_stow_angle, 0); // TODO: VERIFY
                moveAntennaServo(EndevatorConstants.antenna_reef_intake_limit);
            }
            case HIGH_ALGAE_INTAKE -> {
                moveElevator(EndevatorConstants.algae_L3_height);
                moveEndeffector(EndevatorConstants.algae_L3_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_reef_intake_limit);

            }
            case LOW_ALGAE_INTAKE -> {
                moveElevator(EndevatorConstants.algae_L2_height);
                moveEndeffector(EndevatorConstants.algae_L2_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_reef_intake_limit);
            }
            case FLICK -> {
                moveElevator(EndevatorConstants.barge_height);
                moveEndeffector(EndevatorConstants.barge_flick_angle, 0);
                moveAntennaServo(EndevatorConstants.antenna_reef_intake_limit);
            }
            default -> throw new IllegalArgumentException("Unexpected value: " + state);

        }
        SmartDashboard.putString("EndEvator State", state.toString());
        SmartDashboard.putBoolean("HasAlgae", hasAlgae());
        SmartDashboard.putBoolean("HasCoral", hasCoral());
    }

}