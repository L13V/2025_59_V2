package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
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
    /*
     * Endeffector pivot
     */
    static TalonFX endeffector_pivot = new TalonFX(EndevatorConstants.endeffector_pivot_motor_id);
    static TalonFXConfiguration endeffector_pivot_config = new TalonFXConfiguration();

    // Required initialization crap
    public void initialize() {
        System.out.println("ElevatorSubsystem Initialized");

    }

    public EndEvatorSubsystem() { // Motor Configuration Apply-ings ig.\
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
        //Feedback
        endeffector_pivot_config.Feedback.FeedbackRemoteSensorID = EndevatorConstants.endeffector_cancoder_id;
        endeffector_pivot_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        endeffector_pivot_config.Feedback.RotorToSensorRatio = EndevatorConstants.endeffector_rotor_to_sensor_ratio;
        endeffector_pivot_config.Feedback.SensorToMechanismRatio = EndevatorConstants.endeffector_sensor_to_mechanism_ratio;
        endeffector_pivot_config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = EndevatorConstants.endeffector_duty_cycle_closed_loop_ramp;
        /* 
         * Apply Configs
         */
        elevator_motor.getConfigurator().apply(elevator_motor_config);
        endeffector_pivot.getConfigurator().apply(endeffector_pivot_config);
        System.out.println("ElevatorSubsystem Initialized");
    }

    // Initialize ElevatorState Enum, and start at stow
    public enum ElevatorState {
        L1,
        L2,
        L3,
        L4,
        L4_Score,
        STOW,
        CORAL_FLOOR_INTAKE,
        ALGAE_FLOOR_INTAKE,
        BARGE
    }

    public ElevatorState state = ElevatorState.STOW;

    /**
     * Sets the elevator state machine to another state.
     * 
     * @param setto
     */
    public void setElevatorState(ElevatorState setto) {
        state = setto;
    }

    /**
     * Command for moving the elevator. Usually accessed by the controller, and
     * eventually sets the state of the state machine.
     * 
     * @param moveto
     * @return Command
     */
    public Command moveTo(ElevatorState moveto) {
        return runOnce(() -> setElevatorState(moveto));
    }

    // Checking State Machine States
    public ElevatorState getCurrenElevatorState() {
        return state;
    }

    // Booleans
    public Boolean readyToStow() {
        return getCurrenElevatorState() == (ElevatorState.L2);
    }

    // State Machine Garbage
    public void periodic() {
        switch (state) {
            case L1 -> {
            }
            case L2 -> {
            }
            case L3 -> {
            }
            case L4 -> {
            }
            case L4_Score -> {
            }
            case STOW -> {
            }
            case CORAL_FLOOR_INTAKE -> {
            }
            case ALGAE_FLOOR_INTAKE -> {
            }
            case BARGE -> {
            }

        }
        SmartDashboard.putString("State", state.toString());
        SmartDashboard.putBoolean("ReadyToStow", readyToStow());
    }
}
