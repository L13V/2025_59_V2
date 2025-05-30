package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndevatorConstants;

public class EndEvatorSubsystem extends SubsystemBase {
    static TalonFX elevator_motor = new TalonFX(EndevatorConstants.elevator_motor_id);
    static TalonFXConfiguration elevator_motor_config = new TalonFXConfiguration();
    static ExternalFeedbackConfigs elevator_external_feedback = new ExternalFeedbackConfigs();

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
        elevator_motor.getConfigurator().apply(elevator_motor_config);
        /* 
         * Endeffector Motor Config
         */
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
