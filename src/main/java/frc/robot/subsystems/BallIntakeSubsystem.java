package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallIntakeConstants;

public class BallIntakeSubsystem extends SubsystemBase {

    // Pivot Motor and PiD
    // ClimbSubsystem m_climb_subsystem = new ClimbSubsystem();
    private final SparkMax PivotMotor = new SparkMax(BallIntakeConstants.bi_pivot_motor_id, MotorType.kBrushless);
    private final SparkClosedLoopController pid = PivotMotor.getClosedLoopController();

    // Roller Motor
    private final TalonFX rollerMotor_right = new TalonFX(BallIntakeConstants.bi_roller_motor_id);

    // Through Bore Encoder
    private final AbsoluteEncoder abs_encoder = PivotMotor.getAbsoluteEncoder();
    SparkMaxConfig config = new SparkMaxConfig();

    //
    static CANrange algae_range = new CANrange(BallIntakeConstants.bi_range_id);
    static CANrangeConfiguration algae_range_config = new CANrangeConfiguration();

    public BallIntakeSubsystem() {
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        PivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        /* 
         * Back Ball Intake Algae CANRange Config
         */
        algae_range.getConfigurator().refresh(algae_range_config);
        algae_range_config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
        algae_range_config.ProximityParams.ProximityThreshold = 0.3;
        algae_range_config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        algae_range.getConfigurator().apply(algae_range_config);
    }

    public enum BallIntakeState {
        STOW,
        HOLD,
        INTAKE,
        SCORE,
        CLIMB
    }

    public BallIntakeState state = BallIntakeState.STOW;

    /**
     * FUNCTION for setting the elevator state machine to another state.
     * 
     * @param setstateto
     */
    public void setState(BallIntakeState setstateto) {
        state = setstateto;

    }

    /**
     * COMMAND for moving the elevator. Usually accessed by the controller, and
     * eventually sets the state of the state machine.
     * 
     * @param setto
     * @return Command
     */
    public Command setTo(BallIntakeState setto) {
        return runOnce(() -> setState(setto));
    }

    public BallIntakeState getCurrentState() {
        return state;
    }

    public boolean hasAlgae() {
        return algae_range.getIsDetected(true).getValue();
    }

    public void moveRollerByPower(double power) {
        if (Math.abs(power) > 1) {
            rollerMotor_right.set(1);
        } else {
            if (power > 0) {
                rollerMotor_right.set(power);

            } else {
                rollerMotor_right.set(power);
            }
        }
    }

    public void moveIntakeToPosition(double position) {
        pid.setReference(position, ControlType.kPosition);
    }

    public double getPosition() {
        return abs_encoder.getPosition();
    }

    public void periodic() {
        switch (state) {
            case STOW -> {
                if (hasAlgae()) {
                    // m_climb_subsystem.ballstop();
                    moveIntakeToPosition(BallIntakeConstants.bi_algae_stow_position);
                    moveRollerByPower(BallIntakeConstants.bi_hold_power);
                } else {
                    moveIntakeToPosition(BallIntakeConstants.bi_stow_position);
                    moveRollerByPower(BallIntakeConstants.bi_idle_power);
                }

            }
            case HOLD -> {
                // moveIntakeToPosition(BallIntakeConstants.bi_algae_stow_position);
                // moveRollerByPower(BallIntakeConstants.bi_hold_power);

            }
            case INTAKE -> {
                moveIntakeToPosition(BallIntakeConstants.bi_algae_intake_position);
                moveRollerByPower(BallIntakeConstants.bi_intake_power);
                // m_climb_subsystem.ballstop();
                if (hasAlgae()) {
                    state = BallIntakeState.STOW;
                }
            }
            case SCORE -> {
                moveIntakeToPosition(BallIntakeConstants.bi_algae_score_position);
                moveRollerByPower(BallIntakeConstants.bi_outtake_power);
            }
            case CLIMB -> {
                moveIntakeToPosition(BallIntakeConstants.bi_climb_position);
                moveRollerByPower(0);
            }

        }
        SmartDashboard.putString("Ball Intake State", state.toString());

    }
}
