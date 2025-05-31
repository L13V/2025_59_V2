package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
         * Servo Config
         */

        /*
         * Apply Configs
         */
        elevator_motor.getConfigurator().apply(elevator_motor_config);
        endeffector_pivot.getConfigurator().apply(endeffector_pivot_config);
        System.out.println("ElevatorSubsystem Initialized");
    }

    public double initial_endeffector_position = 0.0;

    // Initialize ElevatorState Enum, and start at stow
    public enum EndEvatorState {
        L1,
        L2,
        L3,
        L4,
        L4_Score,
        STOW,
        CORAL_FLOOR_INTAKE,
        ALGAE_FLOOR_INTAKE,
        HIGH_ALGAE_INTAKE,
        LOW_ALGAE_INTAKE,
        BARGE
    }
    /* 
     * All state machine interaction and reading
     */

    public EndEvatorState state = EndEvatorState.STOW;

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

    /*
     * Various Functions used for retrieving positions of stuff.
     */
    public double getElevatorCurrentPosition() {
        return elevator_motor.getPosition().getValueAsDouble();
    }

    public double getEndeffectorCurrentPosition() {
        return endeffector_pivot.getPosition().getValueAsDouble();
    }

    public double getServoPosition() {
        return antennaServo.getPosition();
    }

    /* 
     * Booleans for controlling binds.
     */
    public Boolean readyToStow() { // TODO: Make this serve a real purpose
        return getCurrentState() == (EndEvatorState.L2);
    }

    // State Machine Garbage
    public void periodic() {
        switch (state) {
            case L1 -> {
                moveElevator(EndevatorConstants.L1_height);
                moveEndeffector(EndevatorConstants.coral_L1_angle, 0);
            }
            case L2 -> {
                moveElevator(EndevatorConstants.L2_height);
                moveEndeffector(EndevatorConstants.coral_L2_angle, 0);
            }
            case L3 -> {
                moveElevator(EndevatorConstants.L3_height);
                moveEndeffector(EndevatorConstants.coral_L3_angle, 0);

            }
            case L4 -> {
                moveElevator(EndevatorConstants.L4_height);
                moveEndeffector(EndevatorConstants.teleop_L4_angle, 0);

            }
            case L4_Score -> {
                moveElevator(EndevatorConstants.L4_score_height);
                moveEndeffector(EndevatorConstants.teleop_L4_angle, 0);
            }
            case STOW -> {
                moveElevator(EndevatorConstants.coral_stow_height);
                moveEndeffector(EndevatorConstants.teleop_coral_stow_angle, 0);
            }
            case CORAL_FLOOR_INTAKE -> {
                moveElevator(EndevatorConstants.coral_floor_pickup_height);
                moveEndeffector(EndevatorConstants.coral_floor_angle, 0);
            }
            case ALGAE_FLOOR_INTAKE -> {
                moveElevator(EndevatorConstants.algae_floor_pickup_height);
                moveEndeffector(EndevatorConstants.algae_floor_angle, 0);
            }
            case BARGE -> {
                moveElevator(EndevatorConstants.barge_height);
                moveEndeffector(EndevatorConstants.teleop_L4_angle, 0); //TODO: VERIFY
            }
            case HIGH_ALGAE_INTAKE -> {
                moveElevator(EndevatorConstants.algae_L3_height);
                moveEndeffector(EndevatorConstants.algae_L3_angle, 0);

            }
            case LOW_ALGAE_INTAKE -> {
                moveElevator(EndevatorConstants.algae_L2_height);
                moveEndeffector(EndevatorConstants.algae_L2_angle, 0);
            }

        }
        SmartDashboard.putString("State", state.toString());
        SmartDashboard.putBoolean("ReadyToStow", readyToStow());
    }
}
