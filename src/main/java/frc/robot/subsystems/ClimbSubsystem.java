package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    /*
     * Climb Motor
     */
    private final TalonFX climb_motor = new TalonFX(ClimbConstants.climb_motor_id);
    private PositionDutyCycle climb_motor_PositionDutyCycle = new PositionDutyCycle(ClimbConstants.climb_stow_position);
    static TalonFXConfiguration climb_motor_config = new TalonFXConfiguration();

    /*
     * Climb States
     */
    public enum ClimbState {
        STOW,
        DEPLOYED,
        CLIMB,
    }

    /*
     * Initialize ClimbState
     */
    public ClimbState state = ClimbState.STOW;

    /*
     * Initialize ClimbSubsystem
     */
    public void initialize() {
        System.out.println("ClimbSubsystem Initialized");

    }

    public ClimbSubsystem() {
        /*
         * Climb Motor Config
         */
        climb_motor.getConfigurator().refresh(climb_motor_config);
        climb_motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climb_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climb_motor_config.Slot0.kP = ClimbConstants.climb_slot0_kP;
        climb_motor_config.Slot0.kI = ClimbConstants.climb_slot0_kI;
        climb_motor_config.Slot0.kD = ClimbConstants.climb_slot0_kD;
        climb_motor_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climb_motor_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        climb_motor_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.forward_soft_limit;
        climb_motor_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.reverse_soft_limit;
        climb_motor.getConfigurator().apply(climb_motor_config);

    }

    public void periodic() {
        switch (state) {
            case STOW -> {
                moveClimbByPosition(ClimbConstants.climb_stow_position);
            }
            case CLIMB -> {
                moveClimbByPower(ClimbConstants.climb_power);
            }
            case DEPLOYED -> {
                moveClimbByPosition(ClimbConstants.climb_deployed_position);

            }
            default -> {
            }
        }

    }

    /**
     * FUNCTION for setting the climb state machine to another state.
     * 
     * @param setstateto
     */
    public void setState(ClimbState setstateto) {
        state = setstateto;
    }

    /**
     * COMMAND for moving the climb. Usually accessed by the controller, and
     * eventually sets the state of the state machine.
     * 
     * @param setto
     * @return Command
     */
    public Command setTo(ClimbState setto) {
        return runOnce(() -> setState(setto));
    }

    /**
     * Moves climb at the power provided until hitting either the reverse limit or
     * forward limit.
     * 
     * @param power Power to move by
     */
    public void moveClimbByPower(double power) {
        if (power < 0) {
            if (getPosition() >= 0.2) {
                climb_motor.set(power);
            } else {
                climb_motor.set(0);
            }
        } else {
            if (getPosition() <= 4.5) {
                climb_motor.set(power);
            } else {
                climb_motor.set(0);
            }
        }
    }

    /**
     * Moves climb to a certain position provided
     * 
     * @param position Position to move to
     */
    public void moveClimbByPosition(double position) {

        if (getPosition() <= ClimbConstants.climb_deployed_position) {
            climb_motor.setControl(climb_motor_PositionDutyCycle.withPosition(position));
        }
    }

    /*
     * Various Functions used for retrieving positions of stuff.
     */
    public double getPosition() {
        return climb_motor.getPosition().refresh().getValueAsDouble();
    }

    /**
     * Returns current climb state.
     * 
     * @return EndEvatorState
     */
    public ClimbState getCurrentState() {
        return state;
    }

}
